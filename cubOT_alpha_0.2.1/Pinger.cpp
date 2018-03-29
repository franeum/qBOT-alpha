#include "Arduino.h"
#include "Pinger.h"

Pinger::Pinger( IPAddress outIp,
                int outPort, 
                unsigned long timeDelay, 
                WiFiUDP udp ) 
{
  _outIp      = outIp;
  _outPort    = outPort;  
  _timeDelay  = timeDelay;
  _udp        = udp;
  _pTime      = 0;

}

void Pinger::sendPing() 
{  
  if ((millis() - _pTime) >= _timeDelay) {

    OSCMessage msg("/ping");
    _udp.beginPacket(_outIp, _outPort);
    msg.send(_udp);
    _udp.endPacket();
    msg.empty();  
    _pTime = millis();
  }
}
