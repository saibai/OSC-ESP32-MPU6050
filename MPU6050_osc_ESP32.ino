//Diego Parbuono ©OSC Project//
//_Enjoy_//
#include <ESP8266WebServer.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>  //ESP8266 Core WiFi Library         
#else
#include <WiFi.h>      //ESP32 Core WiFi Library    
#endif

#include <WiFiUDP.h>
#include <OSCMessage.h> /// https://github.com/CNMAT/OSC
#include <OSCBundle.h> /// https://github.com/CNMAT/OSC
#include <Wire.h>
#include <ESPmDNS.h>
#include <OSCData.h>
#include <WebServer.h> //Local DNS Server used for redirecting all requests to the configuration portal (  https://github.com/zhouhan0126/DNSServer---esp32  )
#include <DNSServer.h> //Local WebServer used to serve the configuration portal (  https://github.com/zhouhan0126/DNSServer---esp32  )
#include <WiFiManager.h> // WiFi Configuration Magic (  https://github.com/zhouhan0126/DNSServer---esp32  ) >>  https://github.com/zhouhan0126/DNSServer---esp32  (ORIGINAL)

  
void configModeCallback (WiFiManager *myWiFiManager) {  
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP()); //IP  AP
  Serial.println(myWiFiManager->getConfigPortalSSID()); //SSID 
    }
void saveConfigCallback () {
  Serial.println("Should save config");
 Serial.println(WiFi.softAPIP()); 
}

char *espname = "ESP-Accel";
 

WiFiUDP Udp; 
IPAddress destIp;

#define destPort 8000  //EDIT osc input port
#define localPort 9000 //EDIT osc output port (only used at startup for announcement)



#define MPU6050_ADDR         0x69 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b

char T [1] ;
String F ;
double offsetX = 0, offsetY = 0, offsetZ = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

void culcRotation();
//I2c
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

//i2C
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}



unsigned int ledState = LOW;              // LOW means led is *on*

OSCErrorCode error;

#ifndef BUILTIN_LED
#ifdef LED_BUILTIN
#define BUILTIN_LED LED_BUILTIN
#else
#define BUILTIN_LED 5
#endif
#endif



void setup() {

Serial.begin(115200);
   
WiFiManager wifiManager;


//wifiManager.resetSettings();
wifiManager.setAPCallback(configModeCallback); 
wifiManager.setSaveConfigCallback(saveConfigCallback); 
 
wifiManager.autoConnect("ESP-Accel", "12345678"); 
   
delay(100);
 
MDNS.begin(espname);
MDNS.addService("_osc", "_udp", localPort);

destIp = WiFi.localIP();
  Udp.begin(destPort );
  OSCMessage msg("/ready"); //announcement
  msg.add(espname);
  msg.add(int(destIp[0]));
  msg.add(int(destIp[1]));
  msg.add(int(destIp[2]));
  msg.add(int(destIp[3]));
  msg.add(destPort );
  destIp[3] = 255;  //use broadcast ip x.x.x.255
Serial.print("Accel IP: ");
Serial.println(destIp);
Serial.println("");
Serial.println("WiFi connected");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());

Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
#ifdef ESP32
    Serial.println(localPort);
#else
    Serial.println(Udp.localPort());
#endif

    // btnInput + LED Output

   pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, ledState);    // turn *on* led
   
Wire.begin(21, 22);
 
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  

  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }

  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro


  Serial.print("Calculate Calibration");
  for(int i = 0; i < 3000; i++){
    
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
  
    raw_acc_x = Wire.read() << 8 | Wire.read();
    raw_acc_y = Wire.read() << 8 | Wire.read();
    raw_acc_z = Wire.read() << 8 | Wire.read();
    raw_t = Wire.read() << 8 | Wire.read();
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();
    dpsX = ((float)raw_gyro_x) / 131;
    dpsY = ((float)raw_gyro_y) / 131;
    dpsZ = ((float)raw_gyro_z) / 131;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;
    if(i % 1000 == 0){
      Serial.print(".");
    }
  }
  Serial.println();

  offsetX /= 3000;
  offsetY /= 3000;
  offsetZ /= 3000;

  Serial.print("offsetX : ");
  Serial.println(offsetX);
  Serial.print("offsetY : ");
  Serial.println(offsetY);
  Serial.print("offsetZ : ");
  Serial.println(offsetZ);

  digitalWrite(BUILTIN_LED, ledState);    // turn *on* led
}

void OSCMsgReceive(void){
OSCMessage msg;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      msg.fill(Udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch("/led", led);
    } else {
      error = msg.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }
}
void loop()
{
  WiFiManager wifiManager;
  
  OSCMsgReceive();

    acXOSC();
    acYOSC();
    acZOSC();
    anZOSC();
    anYOSC();
    anXOSC();
    led;
    
     
}

void acXOSC(){
   calcRotation();
    // read btnInput and send OSC
    OSCMessage msgOut("/ax");
     msgOut.add(acc_x);
    
        Serial.print("AccX: ");
        Serial.println(acc_x);
        

    Udp.beginPacket(destIp, destPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
   // delay(100);
}
void acYOSC(){
   calcRotation();
    // read btnInput and send OSC
    OSCMessage msgOut("/ay");
     
    
     msgOut.add(acc_y);
     
        
    Udp.beginPacket(destIp, destPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
   // delay(100);
}

void acZOSC(){
   calcRotation();
    // read btnInput and send OSC
    OSCMessage msgOut("/az");
     msgOut.add(acc_z);
   
        Serial.print("AccZ: ");
        Serial.println(acc_z);
        
        
    

    Udp.beginPacket(destIp, destPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
   // delay(100);
}
void anXOSC(){
   calcRotation();
    // read btnInput and send OSC
    OSCMessage msgOut("/gx");
    
     msgOut.add(angleX);

        Serial.print("GX: ");
        Serial.println(angleY);
    
        
    

    Udp.beginPacket(destIp, destPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
   // delay(100);
}
void anYOSC(){
   calcRotation();
    // read btnInput and send OSC
    OSCMessage msgOut("/gy");
     msgOut.add(angleY);

        Serial.print("GY: ");
        Serial.println(angleX);
       
        
    

    Udp.beginPacket(destIp, destPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
  //  delay(100);
}
void anZOSC(){
   calcRotation();
    // read btnInput and send OSC
    OSCMessage msgOut("/gz");
     msgOut.add(angleZ);
     
        Serial.print("GZ: ");
        Serial.println(angleZ);
        
    

    Udp.beginPacket(destIp, destPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
  //  delay(100);
}



void led(OSCMessage &msg) {
  if(msg.isFloat(0)){
 ledState = msg.getFloat(0);
}
if(msg.isInt(0)){
 ledState = msg.getInt(0);
 }



  digitalWrite(BUILTIN_LED, ledState);
  Serial.print("/led: ");
  Serial.println(ledState);
}
 


void calcRotation(){

  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
 
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
  
  
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;
  
  
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  dpsX = ((float)raw_gyro_x) / 131; // LSB sensitivity: 131 LSB/dps (±250 dps), 65.5 LSB/dps(±500 dps), 32.8 LSB/dps (±1000 dps), and 16.4 LSB (±2000 dps)
  dpsY = ((float)raw_gyro_y) / 131;
  dpsZ = ((float)raw_gyro_z) / 131;
  
  
  interval = millis() - preInterval;
  preInterval = millis();
  
  
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);
  
  
  angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
  angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
  angleZ = gyro_angle_z;
  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;
}
