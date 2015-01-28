#include <SPI.h>
#include <Pixy.h>
#include <VisualSensor.h>
#include <Drivetrain.h>

/**************************************
 * Test Sketch for Drivetrain and VisualSensor Libraries *
 **************************************
 * Tests all of the functions of the drivetrain and visualsensor libraries.
 * Should do the following, in order:
 * 1) move towards the fish until it is close. (tests isClose() and getBlock() methods, as well as goToFish() method
 * 2) if the robot is now close to the fish, it will turn right for 6 seconds, then stop (tests isClose() method as well as 
 *         rotate() and stopMotors()
 * 3) the robot should wait 1 second, and then turn left for 2 seconds (if this happens but step 2 does not, then there is a 
           problem with the isClose() method
 * 4) the robot should now stop for 2 seconds
 */

//Pins for motors
byte leftMotorForward = 3;
byte leftMotorBackward = 10;
byte rightMotorForward = 5;
byte rightMotorBackward = 6;

//values for VisualSensor object
char iRPort =  1   ; //IR port value, 1 chosen at random
float stopVoltage = 2.8;//maxium value is 3.2


int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
int deadZone = 20; //How big the "center" of the robot is. Smaller values will cause robot to wiggle more.
int power = 160; //How much power for wheel motors. Valid values are 0 - 255.
int stepTimes[3] = {6000, 3000, 2000}; //An array where each element is how much time in milliseconds should be spent at each step of rotation.

Drivetrain *wheels;
VisualSensor *eyes;


void setup()
{
  Serial.begin(9600);
  wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward, center, deadZone, power, stepTimes);
  eyes = new VisualSensor(iRPort, stopVoltage);
}


void loop()
{
  //If there is a block and it is not close, the robot should move towards it
  while(! (*eyes).isClose())
  {
    Block targetBlock = (*eyes).getBlock();
    (*wheels).goToFish(targetBlock);
  }

  //Once the robot is close to the target it will turn right for 6 seconds, then the motors will stop 
  if((*eyes).isClose())
  {
    (*wheels).rotate(1);
    (*wheels).stopMotors();
  }

  delay(1000);
  
  (*wheels).rotate(3);
  
  delay(2000);

}
