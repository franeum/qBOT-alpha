/*
 *  qBOT alpha 0.2.1
 *  created Tue 2018 - 03 - 29 15.34
 */


/*
 * 2018-02-01 bugfix in ledPWMFunc function: now it can accept also floating point number 
 * 2018-02-02 change analogWriteRange = 255 (Arduino-like)
 * 2018-02-02 re-added (and fixed) ledfunc() function (enery saving)
 * 2018-02-15 created separated Config.h file
 * 2018-02-15 added sendPING() function (and relative /ping OSC message)
 * 2018-02-16 added ledrange() function (and relative /ledrange OSC message)
 * 2018-02-16 added configdata() function (and relative /configdata OSC message)
 * 2018-02-17 fixed threshFunc() and timeDelayFunc() to accept also float (for Pure Data compatibility)
 * 2018-03-24 added imu software
 * 2018-03-24 added face recognition (funcion getFace())
 * 2018-03-24 added OSC face sender function
 * 2018-03-29 inserted imudelay, to set IMU messages sender delay
 * 2018-03-29 added OSC imudelay() setter function
 * 2018-03-29 added OSC port sending messages for imu messages (9000 + NUMBER)
 * 
 */

 /*
 * OSC messages accepted:
 * 
 * /LED               turn on/off the led
 * /LEDPWM            turn on/off the led with PWM
 * /knockthresh       piezo sensitivity
 * /knockdelay        piezo delay time after knocking
 * /sendip            send IP to server
 * /ping              send ping response to server
 * /ledrange          set pwm resolution
 * /configdata        send threshold and delay time of piezo
 * 
 * 
 * OSC messages send:
 * 
 * /knock            for every knock on the piezo (followed by 1)
 * /ip               when connected, when asked for ip and for ping request
 * /dataconf         respond to message /configdata
 * 
 */


#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "Pinger.h"
#include "Knocker.h"
#include "Config.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9255.h"

#define COMPARE(x,y,z,a,b,c)  ((x) == (a)) && ((y) == (b)) && ((z) == (c))
#define DEGREES(a)            ((((a) + 32768) / 65536.0) * 360 - 180)


WiFiUDP Udp;                                  // A UDP instance to let us send and receive packets over UDP

const IPAddress outIp(remoteip[0], remoteip[1], remoteip[2], remoteip[3]);           // remote IP of your computer


//  IP ADDRESS LOCALE (assegnato staticamente)
//  Impostazione automatica  

const IPAddress localIp(remoteip[0], remoteip[1], remoteip[2], 100 + NUMBER);

const IPAddress netMask(255,255,255,0);


//  PORTA DI USCITA DEI MESSAGGI OSC (7001 per il primo qBOT, 7002 per il secondo e così via) 

const uint16_t  outPort       =   7000 + NUMBER;  // remote port to receive OSC (LE PORTE DEVONO ESSERE DIVERSE PER OGNI qBOT)
const uint16_t  imuOutPort    =   9000 + NUMBER;  // remote port to receive OSC (LE PORTE DEVONO ESSERE DIVERSE PER OGNI qBOT)



//  PORTA DI ENTRATA DEI MESSAGGI OSC (uguale per tutti i qBOT)
  
const uint16_t  localPort     =   8000;           // local port to listen for OSC packets (actually not used for sending)


// get 4 IP Address octets and store them

uint16_t        octet1;
uint16_t        octet2;
uint16_t        octet3;
uint16_t        octet4;


//  ================================================================================================
//  ====  LED PIN ==================================================================================
//  ================================================================================================
 
const int             LEDPIN            =   D3;


//  ================================================================================================
//  ====  KNOCK DATA  ==============================================================================
//  ================================================================================================

const int             knockSensor       =   A0; 
uint16_t              knockThresh       =   (uint16_t)KNOCKTHRESH; 
unsigned long         timeKnockDelay    =   (unsigned long)KNOCKDELAY;
Knocker               knocker(outIp, outPort, timeKnockDelay, Udp, knockThresh, knockSensor);


//  ================================================================================================
//  ====  PING DATA ================================================================================
//  ================================================================================================

const unsigned long   pingTime          =   (unsigned long)PINGDELAY;
Pinger                pinger(outIp, outPort, pingTime, Udp);


//  ================================================================================================
//  ====  CONFIG IMU ===============================================================================
//  ================================================================================================

MPU9150 mpu9255_AM(0x68);
const int MPU9255_AM = 0x68;  // I2C address of the MPU-6050
int16_t   ax, ay, az;
int16_t   gx, gy, gz;
int16_t   mx, my, mz;
int16_t   AcX,AcY,AcZ,GyX,GyY,GyZ, MaX, MaY, MaZ;
int16_t   vals[9];
byte      face = 0;
byte      prevFace = 0;

unsigned long   previousMillis  = 0;
unsigned long   imudelay        = (unsigned long)IMUDELAY;

//  ================================================================================================
//  ====  SETUP FUNCTION  ==========================================================================
//  ================================================================================================

void setup() {

  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);    //  light off LED_BUILTIN
 


  //  ================================================================================================
  //  ====  WIFI CONNECTION ==========================================================================
  //  ================================================================================================
  
  //  For static IP
  //  If you erase this line, your ESP8266 will get a dynamic IP address
  //  Wifi.config(Local address, remote address, netmask);
  WiFi.config(localIp, outIp, netMask); 

  if (Serial) {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }
  WiFi.begin(ssid, pass);


  //  ==== Blinking LED while connecting ===========================================================
    
  while (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LEDPIN, HIGH);
      delay(50);
      digitalWrite(LEDPIN, LOW);
      delay(250);       
      if (Serial) Serial.print(".");
  }

  if (Serial) {
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP()); 
    Serial.println("Starting UDP");
  }
  
  Udp.begin(localPort);

  if (Serial) {
    
    Serial.print("Local port: ");
    Serial.println(Udp.localPort());
  }


  //  ==== send local IP to server ================================================================  
 
  octet1    = WiFi.localIP().operator[](0);
  octet2    = WiFi.localIP().operator[](1);
  octet3    = WiFi.localIP().operator[](2);
  octet4    = WiFi.localIP().operator[](3);

  sendLocalIP();
  
  //  ==== 2 secs led ON when connected ============================================================
      
  digitalWrite(LEDPIN, HIGH);
  delay(2000);
  digitalWrite(LEDPIN, LOW);


  // ==== set led range ============================================================================
 
  analogWriteRange((uint16_t)pow(2,constrain(LEDRANGE,2,10)) - 1);
  analogWriteFreq(1000);

  // ==== initialize imu ===========================================================================

  Wire.begin();
  delay(100);
  I2Cscan();// look for I2C devices on the bus
  
  Wire.beginTransmission(MPU9255_AM);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Initializing I2C devices...");
  mpu9255_AM.initialize();

}


void loop() {
  
  pinger.sendPing();
  knocker.readKnock();
  incOSC();

  unsigned long currentMillis = millis();
  
  createData();

  if ((currentMillis - previousMillis) >= imudelay) {

    previousMillis = currentMillis;
    
    OSCMessage msg("/wek/inputs");
    msg.add((float)vals[0]);
    msg.add((float)vals[1]);
    msg.add((float)vals[2]);
    msg.add((float)vals[3]);
    msg.add((float)vals[4]);
    msg.add((float)vals[5]);
    msg.add((float)vals[6]);
    msg.add((float)vals[7]);
    msg.add((float)vals[8]);
    Udp.beginPacket(outIp, imuOutPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
  
    getFace(&vals[0], &vals[1], &vals[2]);
    if (face != prevFace) {
      //Serial.println(face);
      sendFace();
      prevFace = face;
    }
  }
 
  //delay(2);
}

//  ==== incOSC() takes incoming OSC messages and route them by label 

void incOSC() {
  
  OSCMessage msgIN;
  int sizet;
  
  if((sizet = Udp.parsePacket()) > 0) {
    
    while(sizet--) msgIN.fill(Udp.read());
    
    if(!msgIN.hasError()){
      
      msgIN.route("/led", ledFunc);
      msgIN.route("/ledpwm", ledPWMFunc);
      msgIN.route("/knockthresh", threshFunc);
      msgIN.route("/knockdelay", timeDelayFunc);
      msgIN.route("/sendip", sendIP);
      msgIN.route("/ping", sendPING);
      msgIN.route("/ledrange", ledrange);
      msgIN.route("/configdata", configdata);
      msgIN.route("/imudelay", imudelayset);
    }
  }  
}


//  ==== ledFunc() pilota il led con 0 (LOW) e 1 (HIGH)


void ledFunc(OSCMessage &msg, int addrOffset) {
  
  byte ledState = msg.getInt(0);

  if (msg.getType(0) == 'i') {
    ledState = msg.getInt(0) ? 1 : 0;
  } else ledState = (byte)msg.getFloat(0) ? 1 : 0;
  
  digitalWrite(LEDPIN, ledState);
}



//  ==== ledPWMFunc() drives the led with PWM ============================================================

void ledPWMFunc(OSCMessage &msg, int addrOffset) {

  uint16_t ledState;
  
  if (msg.getType(0) == 'i') {
    ledState = msg.getInt(0);
  } else ledState = (uint16_t)msg.getFloat(0);

  analogWrite(LEDPIN, ledState);
}

//  ==== imudelayset() set imu delay =====================================================================

void imudelayset(OSCMessage &msg, int addrOffset) {

  uint16_t idelay;
  
  if (msg.getType(0) == 'i') {
    idelay = msg.getInt(0);
  } else idelay = (uint16_t)msg.getFloat(0);

  //analogWrite(LEDPIN, ledState);
  imudelay = (unsigned long)idelay;
}

//  ==== threshFunc() calibrates the threshold of piezo sensitivity ======================================

void threshFunc(OSCMessage &msg, int addrOffset) {

  if (msg.getType(0) == 'i') {
    knockThresh = msg.getInt(0);
  } else knockThresh = (uint16_t)msg.getFloat(0);
  
  knocker.setKnockThreshold(knockThresh);
}

//  ==== timeDelayFunc() set time delay after a knock is occurred ========================================

void timeDelayFunc(OSCMessage &msg, int addrOffset) {


  if (msg.getType(0) == 'i') {
    timeKnockDelay = msg.getInt(0);
  } else timeKnockDelay = (uint16_t)msg.getFloat(0);
  
  knocker.setKnockDelay(timeKnockDelay);
}

//  ==== send local IP to the server =====================================================================

void sendIP(OSCMessage &msg, int addrOffset) {
  
  sendLocalIP();
}

inline void sendLocalIP() {

  OSCMessage smsg("/ip");
  smsg.add("\t");
  smsg.add(octet1);
  smsg.add(".");
  smsg.add(octet2);
  smsg.add(".");
  smsg.add(octet3);
  smsg.add(".");
  smsg.add(octet4);
  smsg.add("\t");
  smsg.add("\t");
  smsg.add("\tCONNECTED");
  Udp.beginPacket(outIp, outPort);
  smsg.send(Udp);
  Udp.endPacket();
  smsg.empty();   
}

//  ==== send PING response to the server =================================================================

void sendPING(OSCMessage &msg, int addrOffset) {
  
  sendPingResponse();
}


inline void sendPingResponse() {

  OSCMessage smsg("/ip");
  smsg.add("\t");
  smsg.add(octet1);
  smsg.add(".");
  smsg.add(octet2);
  smsg.add(".");
  smsg.add(octet3);
  smsg.add(".");
  smsg.add(octet4);
  smsg.add("\t");
  smsg.add("\t");
  smsg.add("\tPINGED");
  Udp.beginPacket(outIp, outPort);
  smsg.send(Udp);
  Udp.endPacket();
  smsg.empty();   
}

//  ==== ledrange() set the resolution of PWM. Max is 10, then max value is 2^10 - 1 = 1023
//range of pwm for the led (max: 2^10 - 1: 1023)


void ledrange(OSCMessage &msg, int addrOffset) {

  uint16_t ledState;
  
  if (msg.getType(0) == 'i') {
    ledState = msg.getInt(0);
  } else ledState = (uint16_t)msg.getFloat(0);

  ledState = constrain(ledState, 2, 10);
  
  analogWriteRange((uint16_t)pow(2,ledState) - 1);
}


//  ==== send DATA CONFIG to the server =================================================================


void configdata(OSCMessage &msg, int addrOffset) {
  
  sendConfigData();
}


inline void sendConfigData() {

  OSCMessage smsg("/dataconf");
  smsg.add("\t");
  smsg.add("knockthresh:\t");
  smsg.add(knockThresh);
  smsg.add("knockdelay:\t");
  smsg.add((uint16_t)timeKnockDelay);
  Udp.beginPacket(outIp, outPort);
  smsg.send(Udp);
  Udp.endPacket();
  smsg.empty();   
}

inline void createData() {

  mpu9255_AM.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MaX, &MaY, &MaZ);

  //vals[0] = (int16_t)(((AcX + 32768) / 65536.0) * 360 - 180);
  vals[0] = (int16_t)DEGREES(AcX);
  vals[1] = (int16_t)DEGREES(AcY);
  vals[2] = (int16_t)DEGREES(AcZ);
  vals[3] = (int16_t)DEGREES(GyX);
  vals[4] = (int16_t)DEGREES(GyY);
  vals[5] = (int16_t)DEGREES(GyZ);
  vals[6] = (int16_t)DEGREES(MaX);
  vals[7] = (int16_t)DEGREES(MaY);
  vals[8] = (int16_t)DEGREES(MaZ);

}


// I2C scan function
void I2Cscan() {
  
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
    //test_connection = false;
  }
  else
  {
    Serial.println("done\n");
    //test_connection = true;
  }    
}

void getFace(int16_t *x, int16_t *y, int16_t *z) {
  
  int16_t X = *x;
  int16_t Y = *y;
  int16_t Z = *z;

  byte *f = &face;

  
  X = ((X > -15) && (X < 15))   ? 0   :   X;
  X = ((X > 75) && (X < 105))   ? 90  :   X; 
  X = ((X > -105) && (X < -75)) ? -90 :   X;  
  Y = ((Y > -15) && (Y < 15))   ? 0   :   Y;
  Y = ((Y > 75) && (Y < 105))   ? 90  :   Y;
  Y = ((Y > -105) && (Y < -75)) ? -90 :   Y;
  Z = ((Z > -15) && (Z < 15))   ? 0   :   Z;
  Z = ((Z > 75) && (Z < 105))   ? 90  :   Z;
  Z = ((Z > -105) && (Z < -75)) ? -90 :   Z; 
  

  if (COMPARE(X,Y,Z,0,0,90))
    *f = 1;
  else if (COMPARE(X,Y,Z,0,0,-90)) 
    *f = 2;
  else if (COMPARE(X,Y,Z,0,-90,0)) 
    *f = 3;
  else if (COMPARE(X,Y,Z,0,90,0))
    *f = 4;
  else if (COMPARE(X,Y,Z,-90,0,0))
    *f = 5;
  else if (COMPARE(X,Y,Z,90,0,0))
    *f = 6;
}

inline void sendFace() {

  OSCMessage smsg("/face");
  smsg.add((float)face);
  Udp.beginPacket(outIp, outPort);
  smsg.send(Udp);
  Udp.endPacket();
  smsg.empty();   
}

