/*
  Example of APM_BMP085 (absolute pressure sensor) library.
  Code by Jordi Mu?oz and Jose Julio. DIYDrones.com
*/

#include <Wire.h>
#include <APM_BMP085.h> // ArduPilot Mega BMP085 Library

unsigned long timer;

void setup()
{  
  Serial.begin(115200);
  Serial.println("ArduPilot Mega BMP085 library test");
  delay(1000);
  Serial.println("Initializing BMA085...");
  APM_BMP085.Init();   // APM ADC initialization
  Serial.println("Init done. Current altitude assumed zero");
  timer = millis();
}

void loop()
{
  int ch;
  float tmp_float;
  float Altitude;
  
  if((millis()- timer) > 100)
    {
    timer=millis();
    APM_BMP085.Read();
    Serial.print("Press:");
    Serial.print(APM_BMP085.Press);
    Serial.print(" Temp:");
    Serial.print(APM_BMP085.Temp/10.0);
    Serial.print(" Alt:");
    Serial.print(APM_BMP085.GetAltitude()/100.0);
    Serial.println();
    }
}