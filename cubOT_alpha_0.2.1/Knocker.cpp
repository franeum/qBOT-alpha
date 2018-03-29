
#include "Arduino.h"
#include "Knocker.h"

Knocker::Knocker( IPAddress     outIp,
                  int           outPort, 
                  unsigned long timeDelay, 
                  WiFiUDP       udp, 
                  uint16_t      thresh, 
                  int           knockSensor ) 
{
  _outIp        = outIp;
  _outPort      = outPort;  
  _timeDelay    = timeDelay;
  _udp          = udp;
  _pTime        = 0;
  _state        = 0;
  _pState       = 1;
  _thresh       = thresh;
  _knockSensor  = knockSensor;
}

void Knocker::readKnock() 
{
  uint16_t sRead = analogRead(_knockSensor);
  
  if ((sRead >= _thresh)  && ((millis() - _pTime) >= _timeDelay)) {
    if (_state != _pState) {  
      _pState = _state = 1;
      _pTime = millis();
      Serial.println("Knock!");
      sendKnock();
    }
  } else _state = 0;
}

void Knocker::sendKnock() 
{  
  OSCMessage msg("/knock");
  msg.add(1);
  _udp.beginPacket(_outIp, _outPort);
  msg.send(_udp);
  _udp.endPacket();
  msg.empty();  
}

void Knocker::setKnockThreshold(uint16_t thresh) 
{
  _thresh = thresh;
}

void Knocker::setKnockDelay(uint16_t timeDel) 
{
  _timeDelay =  timeDel; 
}

