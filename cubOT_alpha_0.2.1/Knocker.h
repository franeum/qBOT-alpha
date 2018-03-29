/*
 * Knocker class: listen for a knock on piezo disc and send a trigger to the network
 */

#ifndef Knocker_h
#define Knocker_h

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

class Knocker {
  
  public:

//  ====  CONSTRUCTOR =============================================================================
  
    Knocker(  IPAddress outIp,          //  destination IP 
              int port,                 //  destination PORT
              unsigned long timeDelay,  //  delay after a knock is received
              WiFiUDP udp,              //  WiFiUDP instance
              uint16_t thresh,          //  threshold for piezo sensitivity
              int knockSensor );        //  analog PIN (tipically A0)

    
//  ====  LISTEN FOR A KNOCK FROM THE PIEZO DISK ==================================================  
    
    void      readKnock();


//  ==== SEND A TRIGGER WHEN KNOCK IS RECEIVED ====================================================
       
    void      sendKnock();


//  ==== SET THRESHOLD FOR PIEZO SENSIBILITY ======================================================
    
    void      setKnockThreshold(uint16_t thresh);


//  ==== SET DELAY TIME AFTER A KNOCK IS RECEIVED =================================================
    
    void      setKnockDelay(uint16_t timeDel);

    
  private:
    IPAddress     _outIp;
    int           _outPort;
    unsigned long _timeDelay;
    unsigned long _pTime;
    WiFiUDP       _udp;
    byte          _state;
    byte          _pState;
    uint16_t      _thresh;
    int           _knockSensor;
};

#endif Knocker_h
