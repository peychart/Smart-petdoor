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

//#define DEBUG
//#define ALLOW_TELNET_DEBUG

#define _MAIN_
#include <Arduino.h>
#include <Timezone.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "croncpp.h"

#include "setting.h"                //<--Can be adjusted according to the project...
#include "debug.h"                  //<-- telnet and serial debug traces

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


volatile unsigned short servoTarget(90);
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

//Gestion des switchs/Switches management
void IRAM_ATTR debouncedInterrupt(){intr = true;}

void IRAM_ATTR onTimerISR() {
  static unsigned int v(0);
  if( v ) {
    timer1_write( totalPeriod * milliTips - v ); // 20000us/(1/80MHz*TIM_DIV16)) - v -> Freq. = 1/20ms
    v=0;
  }else{
    v=(servoTarget>180 ?180 :servoTarget); v=(minPeriodUs*milliTips) + (v*milliTips*(maxPeriodUs-minPeriodUs)/180); v/=1000;
    timer1_write( v );
  }myPins( SERVO ).set(v);
}

inline void disableInput()  {myPins(SERVO).set(true, 2000UL); servoTarget= 25; currentLockState=2;}
inline void openDoor()      {myPins(SERVO).set(true, 2000UL); servoTarget= 90; currentLockState=0;}
inline void disableOutput() {myPins(SERVO).set(true, 2000UL); servoTarget=155; currentLockState=1;}

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
  if(timeClient.isTimeSet()){ time_t next; bool isDone(false);
    for( size_t i=0; i<calendar.vectorSize(); i++) try{
      auto cron = cron::make_cron( calendar[i]["date"].c_str() );
      next = cron::cron_next(cron, Now());
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
  }
}

void lightSleep(){
  if(myWiFi.staConnected() &&_isNow(next_lightSleep) && !_isNow(next_state-WiFi_WAKING_TIME)){
    next_lightSleep = millis() + WiFi_WAKING_TIME;
    myWiFi.disconnect(0UL);
  }
}

void onWiFiConnect() {
  next_lightSleep = millis() + WiFi_WAKING_TIME;
}

void onStaConnect() {
  timeClient.forceUpdate();
  searchNextCron();
#ifdef WIFI_STA_LED
  myPins(WIFI_STA_LED).set(true);
#endif
}

void onApConnected(){
  std::cout << "APConnected...\n";
}

void onApDisconnected(){
  std::cout << "APDisconnected...\n";
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
}

void onStaDisconnect() {
#ifdef WIFI_STA_LED
  myPins(WIFI_STA_LED).set(false);
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

void onSwitch( void ) {
  static std::vector<bool> previous;
  byte i(0); for(auto &x : myPins){ // Search for the switched switch...
    if(x.gpio()<0){                 // I'm virtual...
      if(previous[i] != x.isOn()){  // It's me!
        if(previous.size()<=i) previous.push_back(false);
        previous[i] = x.isOn();

        // Then do it ...
        switch(i){  // nextLockstate: bit1=input, bit0=output
          case 0:   // set CLOSE OUTPUT
            nextLockstate =  x.isOn()  + 2*(nextLockstate>1);
            break;
          case 1:   // set CLOSE INPPUT
            nextLockstate = 2*x.isOn() + (nextLockstate%2);
            break;
        }next_state=millis();

        //And say it!
        #ifdef DEFAULT_MQTT_BROKER
          myMqtt.send( x.isOn() ?G(PAYLOAD_ON) :G(PAYLOAD_OFF), STR(x.gpio()) + G("/" ROUTE_PIN_STATE) );
        #endif
        return;
      }else i++;
} } }

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
      case  1: disableOutput(); break;
      case  2: disableInput();  break;
      default: openDoor();      nextLockstate=0;  //Single-engine rules (only output XOR input can be disabled)...
  }searchNextCron();
} }

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
  while(!Serial);
#ifdef DEBUG
  delay(1000UL);
  Serial.print(F("\n\nChipID(")); Serial.print(ESP.getChipId()); Serial.print(F(") says: Hello World!\n\n"));
#endif

  //initialisation du WiFi /WiFi init
  for(ushort i(0); i<2; i++){
    myWiFi.version         ( G(VERSION) )
          .onConnect       ( onWiFiConnect )
          .onStaConnect    ( onStaConnect )
          .onApConnect     ( onApConnected )
          .ifStaConnected  ( ifStaConnected )
          .onStaDisconnect ( onStaDisconnect )
          .onApDisconnect  ( onApDisconnected )
          .hostname        ( G(DEFAULTHOSTNAME) )
          .setOutputPower  ( 17.5 )
          .restoreFromSD   ();
    if( myWiFi.version() != G(VERSION) ){
      if( !LittleFS.format() )
        DEBUG_print(F("LittleFS format failed!\n"));
    }else
      break;
  }DEBUG_print(F("WiFi: ")); DEBUG_print(myWiFi.serializeJson().c_str()); DEBUG_print(F("\n"));
  //myWiFi.clear().push_back("hello world", "password").saveToSD();  // only for DEBUG...
  myWiFi.saveToSD();
  myWiFi.connect();

//Pins init:
  myPins.set( pinFlashDef(OUTPUT_CONFIG) )
        .mode(OUTPUT)
        .restoreFromSD(F("out-gpio-"));
  (myPins.mustRestore() ?myPins.set() :myPins.set(false)).mustRestore(false).saveToSD();
  //additional pin initializations:
  myPins(OUTPUT_DOOR).onPinChange(onSwitch); myPins(INPUT_DOOR).onPinChange(onSwitch);
  initTimer();  // SERVO
  #ifdef POWER_LED
    myPins(POWER_LED).set(true);
  #endif
  #ifdef DEBUG
    for(auto &x : myPins) DEBUG_print((x.serializeJson() + G("\n")).c_str());
  #endif

  // Input Pin:
  myPins.set( pinFlashDef(INPUT_CONFIG) ); attachInterrupt(digitalPinToInterrupt(myPins(14).gpio()), debouncedInterrupt, FALLING);

  if( myPins.exist(1) || myPins.exist(3) ){
    DEBUG_print("Pins conflict: serial stopped!\n");
    Serial.end();
  }

   // Servers:
  setupWebServer();                    //--> Webui interface started...
  httpUpdater.setup( &ESPWebServer );  //--> OnTheAir (OTA) updates added...

#ifdef DEFAULT_MQTT_BROKER
//initialisation du MQTT /MQTT init
  myMqtt.broker       ( G(DEFAULT_MQTT_BROKER) )
        .port         ( DEFAULT_MQTT_PORT )
        .ident        ( String(F(DEFAULT_MQTT_IDENT)).length() ?G(DEFAULT_MQTT_IDENT) :String(ESP.getChipId(), DEC).c_str() )
        .user         ( G(DEFAULT_MQTT_USE3
  myMqtt.saveToSD();
  for(auto x:myPins) myMqtt.subscribe( DEFAULT_MQTT_INTOPIC + STR(x.gpio()) + G("/" ROUTE_PIN_SWITCH) );
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
  searchPreviousCron();
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
