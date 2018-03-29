
/*
 * Pinger class: send a ping message (in OSC format) to the network every n milliseconds
 */

#ifndef Pinger_h
#define Pinger_h

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

class Pinger {
  
  public:

    /*
    *   Constructor
    */
  
    Pinger( IPAddress outIp,          //  destination IP
            int port,                 //  destination PORT
            unsigned long timeDelay,  //  time between the pings
            WiFiUDP udp );            //  WiFiUDP instance        

    /*
    *   sendPing() send a /ping message to the network
    */
    
    void  sendPing();
  
  private:
    IPAddress     _outIp;
    int           _outPort;
    unsigned long _timeDelay;
    unsigned long _pTime;
    WiFiUDP       _udp;
};

#endif Pinger_h
