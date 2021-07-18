#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "credentials.h"

//IMU sensor I2C address
#define MPU6050 0x68

//Conversion ratios
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250

//Radians to degrees conversion 180/PI
#define RAD_TO_DEG 57.295779

//MPU-6050 returns values in 16-bit integers
//Raw values
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
int16_t tempRaw;

//Accelerometer, gyroscope and angles arrays
float Acc[2];
float Gy[3];
float Angle[3];
float temp;

String values;

long prev_time;
float dt;


//Credentials to setup wifi
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
const int port = WEB_SERVER_PORT;

ESP8266WebServer webServer(port);

void setup()
{
  Wire.begin(4,5); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);
  //Setup network and server
  setupWiFi();
  setupWebServer();
}

void loop()
{
  // Update IMU sensor values
  //Read accelerometer values
   Wire.beginTransmission(MPU6050);
   Wire.write(0x3B); //Register 0x3B - AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU6050,6,true);   //From 0x3B, request 6 registers
   AcX=Wire.read()<<8|Wire.read(); //Each value occupies 2 registers
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //With accelerometer values, calculate angles
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Read gyroscope values
   Wire.beginTransmission(MPU6050);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU6050,6,true);   //From 0x43, request 6 registers
   GyX=Wire.read()<<8|Wire.read(); //Each value occupies 2 registers
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Get gyroscoper angle
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

  //Read temperature sensor values
  Wire.beginTransmission(MPU6050);                 
  Wire.write(0x41);      //pointing Temp_Out_High Reg
  Wire.endTransmission(false); 
  
  Wire.requestFrom(MPU6050, 2); //two-byte temp data from Temp_H and Temp_L
  tempRaw = Wire.read()<<8|Wire.read(); 
  temp = (float)((tempRaw/340.0) + 36.53);  //formula from data sheets

  dt = (millis() - prev_time) / 1000.0;
  prev_time = millis();
 
  //Apply Complementary Filter
  Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
  Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

  //Integration to get Yaw value
  Angle[2] = Angle[2]+Gy[2]*dt;
   
  webServer.handleClient();
  delay(10);
}

void handleRoot()
{
  Serial.println("[WebServer] Request: /");

  // Read value of light sensor
  float value = analogRead(A0);

  // Build response
  String response = "";
  response += "[{";
  response += "\"illuminance\":[{";
  response += "\"value\":";
  response += value;
  response += "}],";
  response += "\"temperature\":[{";
  response += "\"celsius\":";
  response += temp;
  response += "}],";
  response += "\"angles\":[{";
  response += "\"roll\":";
  response += Angle[0];
  response += ",\"pitch\":";
  response += Angle[1];
  response += ",\"yaw\":";
  response += Angle[2];
  response += "}]";
  response += "}]";

  // Send response
  webServer.send(200, "application/json", response);
}

void setupWiFi()
{
  Serial.println("[WiFi] Setup");
  Serial.print("[WiFi] Connecting to: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("[WiFi] Connected!");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());
}

void setupWebServer()
{
  Serial.println("[WebServer] Setup");
  webServer.on("/", handleRoot);

  Serial.println("[WebServer] Starting..");
  webServer.begin();
  Serial.println("[WebServer] Running!");
}
