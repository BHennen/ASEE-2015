#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>
#include <FishManager.h>

/*******************************
 * Test Sketch for Bin Dumping *
 *******************************/

const byte binServoPin = 13;

byte conveyorPowerDownward = 120;
byte conveyorPowerUpward = 120;

unsigned long binDumpingTime = 2000UL;
unsigned long previousTime = 0;

byte binServoStop = 90;
byte binServoForward = 60;

Bins *bins;

void setup() {
  Serial.begin(9600);
  bins = new Bins(binServoPin, binServoStop, binServoForward, binDumpingTime);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - previousTime >= 3000UL && bins->dumpNextBin(currentTime)) //Dump bins until we've dumped all 3
  {
	  Serial.println("dumped bin");
	  previousTime = currentTime;
  }
}
