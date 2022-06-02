/* WiFiPowerStrip C++ for ESP8266 (Version 3.0.0 - 2020/07)
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
//Reference: https://www.arduino.cc/en/Reference/HomePage
//See: http://esp8266.github.io/Arduino/versions/2.1.0-rc1/doc/libraries.html
//Librairies et cartes ESP8266 sur IDE Arduino: http://arduino.esp8266.com/stable/package_esp8266com_index.json
//http://arduino-esp8266.readthedocs.io/en/latest/

/*
Envoi d'une impulsion haute d’une durée comprise entre 1,3 ms et 1,7 ms toutes les 20 ms,
soit à la fréquence de 50 Hz, pour assurer un bon maintien en angle de l’axe du servomoteur.
const short int minPeriodUs = 800;  //-> 800 us
const short int maxPeriodUs = 2350; //-> 2350 us
const short int totalPeriod = 20;   //-> ms
*/

#define _MAIN_
#include <Arduino.h>
#include <Timezone.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "croncpp.h"

#include "setting.h"                //<--Can be adjusted according to the project...
#include "debug.h"                  //<-- telnet and serial debug traces

#define INFINY                      60000UL

#ifdef WIFI
#include "untyped.h"                //<-- de/serialization object for JSON communications and backups on SD card
#include <ESP8266HTTPUpdateServer.h>
#include "WiFiManager.h"            //<-- WiFi connection manager
WiFiManager                         myWiFi;
ESP8266WebServer                    ESPWebServer(80);
ESP8266HTTPUpdateServer             httpUpdater;

#include "pins.h"
pinsMap                             myPins;

#include "webPage.h"                //<-- definition of a web interface

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

#endif

untyped                             calendar;
unsigned long                       next_set;
short int                           next_state;

inline static bool  _isNow( unsigned long v ) {unsigned long ms(millis()); return(v && (v<ms) && (ms-v)<60000UL);};  //<-- Because of millis() rollover.
inline ulong         Now() {return( timeClient.isTimeSet() ?myTZ->toLocal(timeClient.getEpochTime()) :(millis()/1000UL) );}
inline bool          isSynchronizedTime(ulong t) {return(t>-1UL/10UL);}
inline bool          isSynchronizedTime(void)   {return(Now()>-1UL/10UL);}


#define SERVO                       2
#define RELAY                       0
bool servoReverse(true), relayReverse(false); //according hardware...
const unsigned long delta = 5L;
volatile unsigned short angle(90-delta);
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
  pinMode( SERVO, OUTPUT );
}

void reboot() {
  DEBUG_print(F("Restart needed!...\n"));
  myPins.mustRestore(true).saveToSD();
  ESP.restart();
}

void IRAM_ATTR onTimerISR() {
  static unsigned int v(0);
  if( v ) {
    timer1_write( totalPeriod * milliTips - v ); // 20000us/(1/80MHz*TIM_DIV16)) - v -> Freq. = 1/20ms
    v = 0;
  }else{
    v = ( angle > 180 ?180 :angle );
    v = ( minPeriodUs * milliTips) + ( v * milliTips * (maxPeriodUs-minPeriodUs) / 180 ); v /= 1000;
    timer1_write( v );
  }digitalWrite( SERVO, (servoReverse ?!v :v) );
}

void onWiFiConnect() {
}

void onStaConnect() {
#ifdef WIFI_STA_LED
  myPins(WIFI_STA_LED_NAME).set(true);
#endif
}

void ifStaConnected() {
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

void ifWiFiConnected() {
  MDNS.update();
  timeClient.update();
}

void onStaDisconnect() {
#ifdef WIFI_STA_LED
  myPins(WIFI_STA_LED_NAME).set(false);
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
  byte i(0); for(auto &x : myPins){ // Search for switch switched...
    if(previous.size()<=i)
      previous.push_back(false);
    if(previous[i] != x.isOn()){    // it's me:
      previous[i] = x.isOn();
#ifdef DEFAULT_MQTT_BROKER
      myMqtt.send( x.isOn() ?G(PAYLOAD_ON) :G(PAYLOAD_OFF), STR(x.gpio()) + G("/" ROUTE_PIN_STATE) );
#endif
      switch(i) {
        case 0: // set OUTPUT
          next_state = (x.isOn() ?-1 :-2); break;
        case 1: // set INPPUT
          next_state = (x.isOn() ? 1 : 2); break;
      }  next_set=millis();
      return;
  }i++;}
}

void setRelay(bool newVal=false){
  static unsigned long next_relayStatus=0L;
  if(newVal==true){
    digitalWrite( RELAY, (relayReverse ?HIGH: LOW) ); //Relay on
    next_relayStatus=millis() + 1000L;
  }else if(_isNow(next_relayStatus)){
    next_relayStatus=0L;
    digitalWrite( RELAY, (relayReverse ?LOW: HIGH) ); //Relay off
}}inline void setRelayOn() {setRelay(true);};


void ajustAngle(bool set=true, volatile unsigned short Angle=0){
  static unsigned long next_angle=0L; volatile unsigned short target=90;
  if( angle==target) {
    next_angle=0L;
    return;
  } setRelayOn();

  if (set &&_isNow(next_angle)){
    if ( _isNow( ((unsigned long)(target-angle)*delta) + next_angle ) )
      angle += (angle>target ?1 :-1);
  }else{
    target=angle;
    next_angle = millis();
}}
inline void openDoor()      {ajustAngle(false, 90);}
inline void enableOutput()  {openDoor();}             //Single-engine rules (door is open)...
inline void disableOutput() {ajustAngle(false, 75);}  //Single-engine rules (only the output can be disabled)...
inline void enableInput()   {openDoor();}             //Single-engine rules (door is open)...
inline void disableInput()  {ajustAngle(false, 25);}  //Single-engine rules (only the input can be disabled)...

//Gestion des switchs/Switches management
void IRAM_ATTR debouncedInterrupt(){if(!intr){intr=true; rebound_completed = millis() + DEBOUNCE_TIME;}}

// ***********************************************************************************************
// **************************************** SETUP ************************************************
std::vector<std::string> pinFlashDef( String s ){ //Allows pins declaration on Flash...
  std::vector<std::string> v;
  untyped u; u( s.c_str() );
  for(auto &x : u.vector())
    v.push_back( x.serializeJson().c_str() );
  return v;
}

void setup() {
  //initialisation des broches /pins init
  pinMode( RELAY, OUTPUT );
  initTimer();

  //Serial.begin(115200);
  Serial.begin(9600);
  while(!Serial);
  Serial.print(F("\n\nChipID(")); Serial.print(ESP.getChipId()); Serial.print(F(") says: Hello World!\n\n"));

  //initialisation des broches /pins init
  pinMode( RELAY, OUTPUT );
  initTimer();
  if(RELAY==3 || RELAY==1 || SERVO==3 || SERVO==1) Serial.end();

  //attachInterrupt(digitalPinToInterrupt(COUNTERPIN), counterInterrupt, FALLING);

  //initialisation du WiFi /WiFi init
#ifdef WIFI
  for(ushort i(0); i<2; i++){
    myWiFi.version        ( G(VERSION) )
          .onConnect      ( onWiFiConnect )
          .onStaConnect   ( onStaConnect )
          .ifStaConnected ( ifStaConnected )
          .ifConnected    ( ifWiFiConnected )
          .onStaDisconnect( onStaDisconnect )
          .hostname       ( G(DEFAULTHOSTNAME) )
          .setOutputPower ( 17.5 )
          .restoreFromSD  ();
    if( myWiFi.version() != G(VERSION) ){
#ifndef DEBUG
      myWiFi.clear();
#endif
      if( !LittleFS.format() )
        DEBUG_print(F("LittleFS format failed!\n"));
    }else
      break;
  }DEBUG_print(F("WiFi: ")); DEBUG_print(myWiFi.serializeJson().c_str()); DEBUG_print(F("\n"));
  //myWiFi.clear().push_back("hello world", "password").saveToSD();  // only for DEBUG...
  myWiFi.saveToSD();
  myWiFi.connect();

  myPins.set( pinFlashDef(OUTPUT_CONFIG) )
  #ifdef POWER_LED
        .set( pinFlashDef(POWER_LED) )
  #endif
  #ifdef WIFI_STA_LED
        .set( pinFlashDef(WIFI_STA_LED) )
  #endif
        .mode(OUTPUT)
        .onPinChange( onSwitch )
        .restoreFromSD(F("out-gpio-"));
  (myPins.mustRestore() ?myPins.set() :myPins.set(false)).mustRestore(false).saveToSD();
  if( myPins.exist(1) || myPins.exist(3) ) Serial.end();
  #ifdef DEBUG
    for(auto &x : myPins) DEBUG_print((x.serializeJson() + G("\n")).c_str());
  #endif

   // Servers:
  setupWebServer();                    //--> Webui interface started...
  httpUpdater.setup( &ESPWebServer );  //--> OnTheAir (OTA) updates added...
#endif

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

  //MDNS service:
  MDNS.begin(myWiFi.hostname().c_str());
  MDNS.addService("http", "tcp", 80);

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
  next_set=1UL; next_state=0;
}

void nextSet(){
  if( _isNow(next_set) ){
    switch(next_state){
      case -2: disableOutput(); break;
      case -1: enableOutput();  break;
      case  1: enableInput();   break;
      case  2: disableInput();  break;
      default: openDoor();
    }

    if( isSynchronizedTime() ){
      std::time_t nextSet(-1UL), next;
      for( size_t i=0; i<calendar.vectorSize(); i++) try{
        auto cron = cron::make_cron( calendar[i]["date"].c_str() );
        next = cron::cron_next(cron, Now());
        if( next < nextSet ){
          nextSet = next;
          // Set the Door:
          next_state =  0;
          if(calendar[i]["cmd"][0]=='o'){         // Open the Door:
            if (calendar[i]["cmd"][4]=='I')       // openInput
              next_state =  1;
            else if (calendar[i]["cmd"][4]=='O')  // openOutput
              next_state = -1;
          }else if(calendar[i]["cmd"][0]=='c'){   // Close the Door:
            if(calendar[i]["cmd"][5]=='I')        // closeInput
              next_state =  2;
            else  if (calendar[i]["cmd"][4]=='O') // closeOutput
              next_state = -2;
        } }
      } catch(cron::bad_cronexpr const &ex) {};
      next_set = ((nextSet != -1UL) ?nextSet :0);
} } }

// **************************************** LOOP *************************************************
void loop() {
#ifdef WIFI
  ESPWebServer.handleClient(); delay(1L);             //WEB server
  myWiFi.loop();                                      //WiFi manager
#ifdef DEFAULT_MQTT_BROKER
  myMqtt.loop();                                      //MQTT manager
#endif
#endif
  nextSet();
  ajustAngle();
  setRelay();
}
// ***********************************************************************************************

