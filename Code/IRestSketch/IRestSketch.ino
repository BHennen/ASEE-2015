#include <Sensors.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <eeprom.h>
#include <EEPromAnything.h>
#include <L3G.h>

const char IRPort = A0;
float stopVoltage = 3.8;
VisualSensor* pixy;

void setup()
{
	Serial.begin(9600);
	pixy = new VisualSensor(IRPort, stopVoltage);

}

void loop()
{
	Serial.println(pixy->readProximity());

}
