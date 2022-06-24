/* WiFiPowerStrip C++ for ESP8266 (Version 0.0.1 - 2022/05)
    <https://github.com/peychart/WiFiPowerStrip>

    Copyright (C) 2020  -  peychart

    This program is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of
    the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty
    of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this program.
    If not, see <http://www.gnu.org/licenses/>.

    Details of this licence are available online under:
                        http://www.gnu.org/licenses/gpl-3.0.html
*/

/* Tower Pro Micro Servo 9g SG90 :
Envoi d'une impulsion haute d’une durée comprise entre 1,3 ms et 1,7 ms toutes les 20 ms,
soit à la fréquence de 50 Hz, pour assurer un bon maintien en angle de l’axe du servomoteur.
const short int minPeriodUs = 800;  //-> 800 us
const short int maxPeriodUs = 2350; //-> 2350 us
const short int totalPeriod = 20;   //-> ms
*/
#define _MAIN_

#define DEBUG
#define ALLOW_TELNET_DEBUG
#include "debug.h"                  //<-- telnet and serial debug traces

#include <Arduino.h>
#include <Timezone.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "croncpp.h"

#include "setting.h"                //<--Can be adjusted according to the project...

#define INFINY                      60000UL
#define DEBOUNCE_DELAY              50UL  //(ms)

#include "untyped.h"                //<-- de/serialization object for JSON communications and backups on SD card

#include <ESP8266HTTPUpdateServer.h>
#include "WiFiManager.h"            //<-- WiFi connection manager
WiFiManager                         myWiFi;
ESP8266WebServer                    ESPWebServer(80);
ESP8266HTTPUpdateServer             httpUpdater;

#include "pins.h"
pinsMap                             myPins;

#ifdef WEBGUI
  #include "webPage.h"                //<-- definition of a web interface
#else
  void setupWebServer ( void ) {};
#endif
typedef long unsigned int           ulong;
typedef short unsigned int          ushort;
volatile bool                       intr(false);
volatile ulong                      rebound_completed(0L);

WiFiUDP                             ntpUDP;
NTPClient                           timeClient(ntpUDP);
Timezone                            *myTZ;
String                              ntpServer(DEFAULTNTPSERVER);
short                               localTimeZone(DEFAULTTIMEZONE);
bool                                daylight(DEFAULTDAYLIGHT);

#ifdef DEFAULT_MQTT_BROKER
  #include "mqtt.h"                 //<-- mqtt input/output manager
  mqtt                              myMqtt;
#endif

#define WiFi_WAKING_TIME            120000UL
unsigned long                       next_state(0UL), next_lightSleep(WiFi_WAKING_TIME);
byte                                currentLockState(-1), nextLockstate(0);
untyped                             calendar;

inline static bool  _isNow( unsigned long v )    {unsigned long ms(millis()); return((v<ms) && (ms-v)<60000UL);};  //<-- Because of millis() rollover.
inline ulong         Now()                       {return( timeClient.isTimeSet() ?myTZ->toLocal(timeClient.getEpochTime()) :(millis()/1000UL) );}


volatile ushort servoTarget[2]={90,90};
const short int minPeriodUs = 800;  //-> 800 us
const short int maxPeriodUs = 2350; //-> 2350 us
const short int totalPeriod = 20;   //-> ms
const short int milliTips   = 5000; //-> 1000 us
//void ICACHE_RAM_ATTR onTimerISR();
void IRAM_ATTR onTimerISR();
void initTimer() {
//https://github.com/esp8266/Arduino/blob/eea9999dc5eaf464a432f77d5b65269f9baf198d/cores/esp8266/Arduino.h
  timer1_attachInterrupt( onTimerISR );
  timer1_enable( TIM_DIV16, TIM_EDGE, TIM_SINGLE ); // 1ms = 5000 tips
  timer1_write( totalPeriod * milliTips );
}

void IRAM_ATTR onTimerISR() {
  static bool nServo[2];
  static std::string servo[2];
  static unsigned int d1(0), d2(0);
  static byte i(1);
  static ushort target[2];

  if(myPins(RELAY).isOn()) switch(++i){
    case 0:
      d2=(target[nServo[1]]>180 ?180 :target[nServo[1]]); d2=(minPeriodUs*milliTips) + (d2*milliTips*(maxPeriodUs-minPeriodUs)/180); d2/=1000;
      d1=(target[nServo[0]]>180 ?180 :target[nServo[0]]); d1=(minPeriodUs*milliTips) + (d1*milliTips*(maxPeriodUs-minPeriodUs)/180); d1/=1000;
      timer1_write( d1 ); d2 -= d1;
      if(servo[nServo[0]][0]) myPins( servo[nServo[0]] ).set(true);
      break;
    case 1:
      timer1_write( d2 ); d2 += d1;
      if(servo[nServo[1]][0]) myPins( servo[nServo[1]] ).set(true);
      break;
    default: i=-1;
      if( (target[0]=servoTarget[0]) <= (target[1]-servoTarget[1]) )
            nServo[0]=!(nServo[1]=1);
      else  nServo[0]=!(nServo[1]=0);
      servo[nServo[0]]=SERVO1;
      #ifdef SINGLE_SERVO
        servo[nServo[1]]="";
      #else
        servo[nServo[1]]=SERVO2;
      #endif

      timer1_write( totalPeriod * milliTips - d2 ); // 20000us/(1/80MHz*TIM_DIV16)) - v -> Freq. = 1/20ms
      myPins( SERVO1 ).set(false);
      #ifdef SERVO2
        myPins( SERVO2 ).set(false);
      #endif
} }

//Gestion des switchs/Switches management
void IRAM_ATTR debouncedInterrupt(){intr = true;}

inline void openDoor()    {if(currentLockState!=0) myPins(RELAY).set(true, 2000UL); servoTarget[0] = servoTarget[1] = 90;   currentLockState=0;}
#ifdef SINGLE_SERVO
inline void closeInput()  {if(currentLockState!=2) myPins(RELAY).set(true, 2000UL); servoTarget[0] = 25; currentLockState=2;}
inline void closeOutput() {if(currentLockState!=1) myPins(RELAY).set(true, 2000UL); servoTarget[0] =155; currentLockState=1;}
inline void closeDoor()   {closeOutput();}
#else
inline void closeInput()  {if(currentLockState!=2) myPins(RELAY).set(true, 2000UL); servoTarget[1]=155;                     currentLockState=2;}
inline void closeOutput() {if(currentLockState!=1) myPins(RELAY).set(true, 2000UL); servoTarget[2]= 25;                     currentLockState=1;}
inline void closeDoor()   {if(currentLockState!=3) myPins(RELAY).set(true, 2000UL); servoTarget[0]= 25; servoTarget[1]=155; currentLockState=3;}
#endif

void reboot() {
  DEBUG_print(F("Restart needed!...\n"));
  myPins.mustRestore(true).saveToSD();
  ESP.restart();
}

void searchPreviousCron(){
  static bool knownLockPosition(false);
  if(currentLockState==-1 || !knownLockPosition){
    knownLockPosition=true;
    /* ... */ 
    openDoor();
} }

void searchNextCron(){
  if(timeClient.isTimeSet()){ time_t next(0); bool isDone(false);
    for( size_t i=0; i<calendar.vectorSize(); i++) try{
DEBUG_print(calendar[i]["date"].c_str()); DEBUG_print(" "); DEBUG_print(calendar[i]["cmd"].c_str()); DEBUG_print("\n");
//      auto cron = cron::make_cron( calendar[i]["date"].c_str() );      // memory pb at the moment...
//      next = cron::cron_next(cron, Now());
      if(next<Now()) continue;
      if(!isDone || !_isNow(next_state - (next - Now()))){
        isDone=true; next_state = next - Now();
        nextLockstate = 0;
        if(calendar[i]["cmd"][0]=='c' || calendar[i]["cmd"][0]=='C'){   // Close the Door:
          if(calendar[i]["cmd"][5]=='I' || calendar[i]["cmd"][5]=='i')      // closeInput
            nextLockstate = 2;
          else if(calendar[i]["cmd"][5]=='O' || calendar[i]["cmd"][5]=='o') // closeOutput
            nextLockstate = 1;
    } } }catch(cron::bad_cronexpr const &ex) {};
  }else{
    if( !myWiFi.connected() ) myWiFi.connect();
} }

void lightSleep(){
  if(myWiFi.staConnected() &&_isNow(next_lightSleep) && !_isNow(next_state-WiFi_WAKING_TIME)){
    next_lightSleep = millis() + WiFi_WAKING_TIME;
    myWiFi.disconnect(0UL);
} }

void onWiFiConnect() {
  next_lightSleep = millis() + WiFi_WAKING_TIME;

}

#ifdef WIFI_MEMORY_LEAKS
  struct tcp_pcb;
  extern struct tcp_pcb* tcp_tw_pcbs;
  extern "C" void tcp_abort (struct tcp_pcb* pcb);
  inline void tcpCleanup(){while (tcp_tw_pcbs != NULL) tcp_abort(tcp_tw_pcbs);}
#endif

void ifWiFiConnected() {
#ifdef WIFI_MEMORY_LEAKS                            //No bug according to the ESP8266WiFi devs... but the device ended up rebooting!
  ulong m=ESP.getFreeHeap();                        //(on lack of free memory, particularly quickly on DNS failures)
  DEBUG_print(F("FreeMem: ")); DEBUG_print(m); DEBUG_print(F("\n"));
  if( m < WIFI_MEMORY_LEAKS ){
    ESPWebServer.stop(); ESPWebServer.close(); myWiFi.disconnect(1UL);
    tcpCleanup();
    ESPWebServer.begin();
    DEBUG_print(F("TCP cleanup -> "));
    DEBUG_print(F("FreeMem: ")); DEBUG_print(ESP.getFreeHeap()); DEBUG_print(F("\n"));
  }
#endif
}

void onStaConnect() {
  timeClient.forceUpdate();
  searchNextCron();
#ifdef WIFI_STA_LED
  myPins(WIFI_STA_LED).set(true);
#endif

#ifdef ALLOW_TELNET_DEBUG
  telnetBegin();
#endif
}

void ifStaConnected() {
  timeClient.update();
#ifdef MQTT_SCHEMA
  static bool configSended(false);
  static byte retry(0);
  if( !configSended ){
    if(!retry--){
      retry=10; configSended=true;
      for(auto &x : myPins)
        if( !(configSended &= myMqtt.send( untyped(MQTT_SCHEMA(x.gpio())).serializeJson(), STR(x.gpio()) + G("/" MQTT_CONFIG_TOPIC))) )
          break;
  } }
#endif

#ifdef ALLOW_TELNET_DEBUG
  telnetManagement();
#endif
}

void onStaDisconnect() {
#ifdef WIFI_STA_LED
  myPins(WIFI_STA_LED).set(false);
#endif

#ifdef ALLOW_TELNET_DEBUG
  telnetEnd();
#endif
}

#ifdef DEFAULT_MQTT_BROKER
std::string Upper( std::string s ) {std::for_each(s.begin(), s.end(), [](char & c){c = ::toupper(c);}); return s;};

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  for(auto &x : myPins)
    if( strcmp( topic, (DEFAULT_MQTT_INTOPIC + STR(x.gpio()) +  G("/" ROUTE_PIN_SWITCH)).c_str() )==0 )
      x.set( Upper(std::string(reinterpret_cast<char*>(payload), length))==Upper(G(PAYLOAD_ON)) );
}
#endif

void setStateFromSwitch(){
  if(intr) {
    intr=0; delay(DEBOUNCE_DELAY); next_state=millis(); nextLockstate=currentLockState+1;
    if( !myWiFi.connected() )  // WEB interface access...
      myWiFi.connect();
} }

void doorPositioning(){
  setStateFromSwitch();
  if( nextLockstate!=currentLockState && _isNow(next_state) ){
    switch(nextLockstate){
      case  1: closeOutput();
        break;
      case  2: closeInput();
        break;
      case  3:
        #ifndef SINGLE_SERVO
          closeDoor();
          break;
        #endif
      default: openDoor();
        #ifdef SINGLE_SERVO
          nextLockstate=0;
        #endif
  }searchNextCron();
} }

void onSwitch( void ) {
  static std::vector<bool> previous;
  if(!previous.size()) {previous.push_back(false);previous.push_back(false);previous.push_back(false);}
  if(previous[0]) return;
  previous[0] = true;
  byte i(0); for(auto &x : myPins){ // Search for the switched switch...
    if(!x.hidden()){
      if(previous.size()<=++i) previous.push_back(false);
      if(previous[i] != x.isOn()){  // It's me!
        previous[i] = x.isOn();

        // Then do it ...
        switch(i){  // nextLockstate: bit1=input, bit0=output
          case 1:   // set CLOSE OUTPUT
            nextLockstate =  x.isOn();
            #ifdef SINGLE_SERVO
              if(x.isOn()) myPins(INPUT_DOOR).set((previous[2]=false));
            #else
              nextLockstate += ((nextLockstate/2)<<1);
            #endif
            break;
          case 2:   // set CLOSE INPPUT
            nextLockstate = (x.isOn()<<1);
            #ifdef SINGLE_SERVO
              if(x.isOn()) myPins(OUTPUT_DOOR).set((previous[1]=false));
            #else
              ; nextLockstate += (nextLockstate%2);
            #endif
            break;
        }next_state=millis();

        //And say it!
        #ifdef DEFAULT_MQTT_BROKER
          myMqtt.send( x.isOn() ?G(PAYLOAD_ON) :G(PAYLOAD_OFF), STR(x.gpio()) + G("/" ROUTE_PIN_STATE) );
        #endif
        
        previous[0] = false;
        return;
  } } }
  previous[0] = false;
}

#ifdef WEBGUI
void onWebServerRequest(){
    next_lightSleep = millis() + WiFi_WAKING_TIME;
}
#endif

// ***********************************************************************************************
// **************************************** SETUP ************************************************
std::vector<std::string> pinFlashDef( String s ){ //Allows pins declaration on Flash...
  std::vector<std::string> v;
  untyped u; u( s.c_str() );
  for(auto &x : u.vector())
    v.push_back( x.serializeJson().c_str() );
  return v;
}

void setup(){
  Serial.begin(115200);
  while(!Serial) {yield();};
#ifdef DEBUG
  delay(2000UL);
  Serial.print( F("\n\nChipID(") ); Serial.print(ESP.getChipId()); Serial.print( F(") says: Hello World!\n\n") );
#endif

  //initialisation du WiFi /WiFi init
  for(ushort i(0); i<2; i++){
    myWiFi.version         ( G(VERSION) )
          .onConnect       ( onWiFiConnect )
          .onStaConnect    ( onStaConnect )
          .ifConnected     ( ifWiFiConnected )
          .ifStaConnected  ( ifStaConnected )
          .onStaDisconnect ( onStaDisconnect )
          .hostname        ( G(DEFAULTHOSTNAME) )
          .setOutputPower  ( 17.5 )
          .restoreFromSD   ();
    if( myWiFi.version() != G(VERSION) ){
      myWiFi.clear();
      LittleFS.begin();
      if( !LittleFS.format() )
        DEBUG_print( F("LittleFS formatting failed!\n") );
      LittleFS.end();
    }else
      break;
  }DEBUG_print( F("WiFi: ") ); DEBUG_print(myWiFi.serializeJson().c_str()); DEBUG_print(F( ", ssidCount=") ); DEBUG_print(myWiFi.ssidCount()); DEBUG_print( F(".\n") );
  //myWiFi.clear().push_back("myWiFiSSID", "myWiFiPassword").saveToSD();  // only for DEBUG...
  myWiFi.saveToSD();
  myWiFi.connect();

  //Pins init:
  #ifdef POWER_LED
    myPins.set( pinFlashDef(POWER_LED_CONFIG) );
  #endif
  #ifdef WIFI_STA_LED
    myPins.set( pinFlashDef(WIFI_LED_CONFIG) );
  #endif
  myPins.set( pinFlashDef(OUTPUT_CONFIG) ).set( pinFlashDef(SERVO1_CONFIG) );
  #ifdef SERVO2
    myPins.set( pinFlashDef(SERVO2_CONFIG) );
  #endif
  myPins.mode(OUTPUT);
  myPins(OUTPUT_DOOR).onStateSettled(onSwitch); myPins(INPUT_DOOR).onStateSettled(onSwitch);
    //additional pin initializations:
  initTimer();  // SERVO config...
    // Input Pin (manual switching of the door):
  myPins.set( pinFlashDef(INPUT_CONFIG) ); myPins("switch").mode(INPUT); attachInterrupt(digitalPinToInterrupt(myPins("switch").gpio()), debouncedInterrupt, FALLING);

#ifdef DEBUG
  for(auto &x : myPins) DEBUG_print((x.serializeJson() + G("\n")).c_str());
#endif

  if( myPins.exist(1) || myPins.exist(3) ){
    DEBUG_print( F("Pins conflict: serial stopped!\n") );
    Serial.end();
  }

  // Servers:
  httpUpdater.setup( &ESPWebServer );  //--> OnTheAir (OTA) updates added...
  setupWebServer();                    //--> Webui interface started...

#ifdef DEFAULT_MQTT_BROKER
//initialisation du MQTT /MQTT init
  myMqtt.broker       ( G(DEFAULT_MQTT_BROKER) )
        .port         ( DEFAULT_MQTT_PORT )
        .ident        ( String(F(DEFAULT_MQTT_IDENT)).length() ?G(DEFAULT_MQTT_IDENT) :String(ESP.getChipId(), DEC).c_str() )
        .user         ( G(DEFAULT_MQTT_USE3
  myMqtt.saveToSD();
  for(auto &x:myPins) myMqtt.subscribe( DEFAULT_MQTT_INTOPIC + STR(x.gpio()) + G("/" ROUTE_PIN_SWITCH) );
  myMqtt.setCallback( mqttCallback );
  DEBUG_print(myMqtt.serializeJson().c_str()); DEBUG_print(F("\n"));
#endif

  //NTP service:
  timeClient.setPoolServerName(ntpServer.c_str());
  timeClient.setUpdateInterval(NTP_UPDATE_INTERVAL_MS*1000UL);
  //timeClient.setTimeOffset(3600 * localTimeZone);
  dstRule.offset = 60 * (localTimeZone - daylight);
  stdRule.offset = 60 * localTimeZone;
  myTZ = new Timezone(dstRule, stdRule);
  //myTZ->setRules(dstRule, stdRule);
  timeClient.begin();

  calendar.deserializeJson( String(CALENDAR).c_str() );
}

// **************************************** LOOP *************************************************
void loop(){
  ESPWebServer.handleClient(); delay(1L);             //WEB server
  myWiFi.loop();                                      //WiFi manager
  myPins.timers();                                    //Pins timeout Management
  doorPositioning();
#ifdef DEFAULT_MQTT_BROKER
  myMqtt.loop();                                      //MQTT manager
#endif
  lightSleep();
}
// ***********************************************************************************************
