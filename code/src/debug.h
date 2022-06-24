
#ifndef HEADER_A43236928626356
#define HEADER_A43236928626356

#include <Arduino.h>
//#include "setting.h"

#define Serial_print(m)          {if(Serial) Serial.print (m);}
#define Serial_printf(m,n)       {if(Serial) Serial.printf(m,n);}

#ifdef DEBUG

#ifdef ALLOW_TELNET_DEBUG
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

    #ifdef _MAIN_
      WiFiServer                  telnetServer(23);
      WiFiClient                  telnetClient;
    #else
      extern WiFiServer           telnetServer;
      extern WiFiClient           telnetClient;
    #endif

    #define DEBUG_print(m)       {if(telnetClient && telnetClient.connected()) telnetClient.print (m);    Serial_print (m);  }
    #define DEBUG_printf(m,n)    {if(telnetClient && telnetClient.connected()) telnetClient.printf(m,n);  Serial_printf(m,n);}

    #ifdef _MAIN_
      void telnetBegin(){       // Called on STA connection
        telnetServer.begin();
       telnetServer.setNoDelay(true);
      }

      void telnetManagement(){  // Called periodically on connected STA.
        if(telnetServer.hasClient()){  //Telnet client connection:
         if (!telnetClient || !telnetClient.connected()){
            if(telnetClient){
              telnetClient.stop();
              DEBUG_print("Telnet Client Stop\n");
            }telnetClient=telnetServer.available();
            telnetClient.flush();
            DEBUG_print("Telnet client connected...\n");
      } } }

      void telnetEnd(){         // Called on STA disconnection
        telnetServer.stop();
      }
    #endif

  #else
    #define DEBUG_print(m)        Serial_print(m)
    #define DEBUG_printf(m,n)     Serial_printf(m,n)
  #endif
#else
  #define DEBUG_print(m)          ;
  #define DEBUG_printf(m,n)       ;
#endif

#endif
