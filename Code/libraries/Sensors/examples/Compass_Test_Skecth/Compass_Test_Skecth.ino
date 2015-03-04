#include <SPI.h>
#include <Pixy.h>
#include <Sensors.h>



/************************************
 * Test Sketch for Compass Sesnor *
 ************************************
 * Tests the compass to see if the values it 
 * puts out or what we think it should  be.
 */
 
 
//Constants for sensor class
const char CompassPort1 = A1; //Port DataIN for compass
const char CompassPort2 = A2; //Port DataOUT for compass

 //Pointers to robot objects

Compass *compass;

void setup() {
   Serial.begin(9600);

  compass = new Compass(CompassPort1, CompassPort2);
  
  Serial.println( "Printing Initial Degree heading");
   Serial.println( (*compass).getInitDegrees());
   
   Serial.println("Printing current Degree ");
}

void loop() {

  Serial.println((*compass).getDegrees());
}
