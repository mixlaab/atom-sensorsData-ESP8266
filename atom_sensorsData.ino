#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "credentials.h"

#define I2C_ADDR_MPU6050 0x68

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
const int port = WEB_SERVER_PORT;

ESP8266WebServer webServer(port);

void setup()
{
  // Setup I2C
  Wire.begin(D2, D1);

  // Setup Serial
  Serial.begin(9600);
  Serial.println();
  setupWiFi();
  setupWebServer();
}

void loop()
{
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
  response += "\"visible\":";
  response += 100;
  response += ",\"full\":";
  response += 100;
  response += ",\"value\":";
  response += value;
  response += "}],";
  response += "\"acceleration\":[{";
  response += "\"roll\":";
  response += 0;
  response += ",\"pitch\":";
  response += 0;
  response += ",\"yaw\":";
  response += 0;
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

/*
void setupSensorMPU6050(){
  Serial.println("[MPU6050] Setup");
  Serial.print("[MPU6050] Connecting..");

  while (!mpu6050.begin())
  {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("[MPU6050] Connected!");
}
*/
