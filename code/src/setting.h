/* WiFiPowerStrip C++ for ESP8266 (Version 3.0.0 - 2020/07)
    <https://github.com/peychart/WiFiPowerStrip-cpp>

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
#ifndef HEADER_FB360A2A9706121
#define HEADER_FB360A2A9706121

//Ajust the following:

#define VERSION                  "0.0.1"              //Change this value to reset current config on the next boot...
#define DEFAULTHOSTNAME          "ESP8266"
//NOTA: no SSID declared (in web interface) will qualify me as a slave candidate...
#define DEFAULTWIFIPASS          "defaultPassword"
#define WIFISTADELAYRETRY       30000UL
#define MAXWIFIRETRY            2
#define WIFIAPDELAYRETRY        300000UL
#define MEMORYLEAKS             5000L
#define SSIDCount()             3
#define WEBGUI

#define ACCESS_CONTROL_ALLOW_ORIGIN "*"
#define EXCLUDED_IPV4_FROM_TUNE 192,168,0,253       //Update requests from this (IP & MASK) (HAProxy server?) are prohibited...
#define EXCLUDED_MASK_FROM_TUNE 255,255,255,255     //Mask to exclude prohibited IPs (warning: ',' not '.')

#define WIFI_MEMORY_LEAKS         16800UL
#define DEBOUNCE_TIME             25UL                //(ms) <- One switches treatments.

#define DEFAULTTIMEZONE           -10
#define DEFAULTNTPSERVER         "pool.ntp.org"
#define NTP_UPDATE_INTERVAL_MS    1800                //(s)
#define DEFAULTDAYLIGHT           false
#ifdef _MAIN_
TimeChangeRule dstRule = {"CEST", Last, Sun, Mar, 1, (DEFAULTTIMEZONE - 1) * 60};
TimeChangeRule stdRule = {"CET",  Last, Sun, Oct, 1,  DEFAULTTIMEZONE * 60};
#endif

// Cron syntaxe: s m  h dom mon down year
#define CALENDAR F("[ \
    {\"date\": \"? 30 17 * 05-10 * ?\", \"cmd\": \"openOutPut\"}, \
    {\"date\": \"? 30 18 * 11-04 * ?\", \"cmd\": \"closeOutPut\"}, \
    {\"date\": \"? 00 06 *   *   * ?\", \"cmd\": \"openDoor\",} \
]")

//Outsourcing of the user interface to 'EXTERN_WEBUI':
//#define EXTERN_WEBUI         "http://webui-domaine-name/"

//HTML SCHEMA:
#define ROUTE_VERSION            "version"
#define ROUTE_HOSTNAME           "hostname"
#define ROUTE__DEFAULT_HOSTNAME  "defHName"
#define ROUTE__DEFAULT_PASSWORD  "defWFPwd"
#define ROUTE_CHIP_IDENT         "ident"
#define ROUTE_UPTIME             "uptime"
#define ROUTE_MAC_ADDR           "macAddr"
#define ROUTE_IP_ADDR            "ip"
#define ROUTE_PIN_GPIO           "gpio"
#define ROUTE_PIN_NAME           "name"
#define ROUTE_PIN_STATE          "state"
#define ROUTE_PIN_SWITCH         "switch"
#define ROUTE_PIN_VALUE          "timeout"
#define ROUTE_PIN_ENABLED        "enabled"
#define ROUTE_PIN_MODE           "mode"
#define ROUTE_PIN_REVERSE        "reverse"
#define ROUTE_PIN_HIDDEN         "hidden"
#define ROUTE_PIN_BLINKING       "blinking"
#define ROUTE_PIN_BLINKING_UP    "blinkingUp"
#define ROUTE_PIN_BLINKING_DOWN  "blinkingDown"
#define ROUTE_WIFI_SSID          "ssid"
#define ROUTE_WIFI_PWD           "pwd"
#define ROUTE_NTP_SOURCE         "ntpSource"
#define ROUTE_NTP_ZONE           "ntpZone"
#define ROUTE_NTP_DAYLIGHT       "ntpDayLight"
#define ROUTE_MQTT_BROKER        "mqttBroker"
#define ROUTE_MQTT_PORT          "mqttPort"
#define ROUTE_MQTT_IDENT         "mqttIdent"
#define ROUTE_MQTT_USER          "mqttUser"
#define ROUTE_MQTT_PWD           "mqttPwd"
#define ROUTE_MQTT_OUTOPIC       "mqttOuTopic"
#define ROUTE_MQTT_SCHEMA        "mqttSchema"
#define ROUTE_HTML_CODE          "html"
#define ROUTE_RESTART            "restart"
#define ROUTE_RESTORE            "restoreStateOnBoot"

#define STR(i)                    std::string(String(i,DEC).c_str())
#define G(n)                      String(F(n)).c_str()
// MQTT SCHEMA:
/*
//#define DEFAULT_MQTT_BROKER      "192.168.0.254"
#define DEFAULT_MQTT_BROKER      "mosquitto"
*/
#ifdef  DEFAULT_MQTT_BROKER
  #define DEFAULT_MQTT_PORT       1883
  #define DEFAULT_MQTT_IDENT     ""    // WARNING: must be different between devices on a same broker...
  #define DEFAULT_MQTT_USER      ""
  #define DEFAULT_MQTT_PWD       ""
  #define DEFAULT_MQTT_OUTOPIC   (String(F("home-assistant/")) + G(ROUTE_PIN_SWITCH "/") + String(ESP.getChipId(),DEC) + G("/")).c_str()
  #define DEFAULT_MQTT_INTOPIC   (String(ESP.getChipId(),DEC)  + G("/")).c_str()
  #define MQTT_CONFIG_TOPIC      "config"
  #define PAYLOAD_ON             "on"
  #define PAYLOAD_OFF            "off"

/*
  #define MQTT_SCHEMA(i)          std::map<std::string,untyped>{                                                 \
                                    {"state_topic"  , DEFAULT_MQTT_OUTOPIC + STR(i) + G("/" ROUTE_PIN_STATE)  }  \
                                   ,{"command_topic", myMqtt.ident() + STR(i) + G("/" ROUTE_PIN_SWITCH) }        \
                                   ,{"payload_on"   , G(PAYLOAD_ON)  }                                           \
                                   ,{"payload_off"  , G(PAYLOAD_OFF) }                                           \
                                   ,{"mame"         , myPins(i).name() }                                         \
                                   ,{"retain"       , false }                                                    \
                                   ,{"optimistic"   , false }                                                    \
                                   ,{"device_class" , G(ROUTE_PIN_SWITCH) }                                      \
                                  }
*/
#endif

#define BACKGROUND_IMAGE       "https://static.mycity.travel/manage/uploads/7/36/12705/989bd67a1aad43055bd0322e9694f8dd8fab2b43_1080.jpg"


#ifdef _MAIN_

#define OUTPUT_DOOR       "output"
#define INPUT_DOOR        "input"
#define RELAY             "relay"
#define SERVO             "servo"
#define RELAYPIN_REVERSE  "false"  //according hardware...
#define SERVOPIN_REVERSE  "true"   //according hardware...
//#define POWER_LED         "powerLed"
//#define WIFI_STA_LED      "wifiLed"

#ifdef POWER_LED
#define POWER_LED F("[\
  {/*D0*/ \"16\": {\"" ROUTE_PIN_NAME "\": \"" POWER_LED "\", \"" ROUTE_PIN_HIDDEN "\": true, \"" ROUTE_PIN_BLINKING "\": true, \"" ROUTE_PIN_BLINKING_UP "\": 1000, \"" ROUTE_PIN_BLINKING_DOWN "\": 0, \"" ROUTE_PIN_STATE "\": true }}\
]")
#endif

#ifdef WIFI_STA_LED
#define WIFI_STA_LED F("[\
  {/*D1*/ \"5\": {\"" ROUTE_PIN_NAME "\": \"" WIFI_STA_LED "\", \"" ROUTE_PIN_HIDDEN "\": true, \"" ROUTE_PIN_BLINKING "\": true, \"" ROUTE_PIN_BLINKING_UP "\": 5000, \"" ROUTE_PIN_BLINKING_DOWN "\": 250, \"" ROUTE_PIN_STATE "\": true }}\
]")
#endif

#define OUTPUT_CONFIG F("[\
/*virtual*/ {\"-1\": {\"" ROUTE_PIN_NAME "\": \"" OUTPUT_DOOR "\", \"" ROUTE_PIN_REVERSE "\": false,                \"" ROUTE_PIN_VALUE "\": -1, \"" ROUTE_PIN_STATE "\": false}},\
/*virtual*/ {\"-2\": {\"" ROUTE_PIN_NAME "\": \"" INPUT_DOOR  "\", \"" ROUTE_PIN_REVERSE "\": false,                \"" ROUTE_PIN_VALUE "\": -1, \"" ROUTE_PIN_STATE "\": false}},\
/*D2*/      { \"4\": {\"" ROUTE_PIN_NAME "\": \"" RELAY       "\", \"" ROUTE_PIN_REVERSE "\": " RELAYPIN_REVERSE ", \"" ROUTE_PIN_VALUE "\": -1, \"" ROUTE_PIN_STATE "\": false, \"" ROUTE_PIN_HIDDEN "\": true}},\
/*D4*/      { \"2\": {\"" ROUTE_PIN_NAME "\": \"" SERVO       "\", \"" ROUTE_PIN_REVERSE "\": " SERVOPIN_REVERSE ", \"" ROUTE_PIN_VALUE "\": -1, \"" ROUTE_PIN_STATE "\": false, \"" ROUTE_PIN_HIDDEN "\": true}}\
]")

#define INPUT_CONFIG F("[\
/*D5*/      {\"14\": {\"" ROUTE_PIN_NAME "\": \"switch\"}}\
]")

#endif

#endif
