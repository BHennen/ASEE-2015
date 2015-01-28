#include <SPI.h>
#include <Pixy.h>
#include <Drivetrain.h>

/**************************************
 * Test Sketch for Drivetrain Library *
 **************************************
 * Tests all of the functions of the drivetrain library.
 * Should do the following, in order:
 * 1) Both wheels should power for 1 sec. (test goToFish() method, as well as the goStraight() method)
 * 2) Right wheel should power for 1 sec. (test goToFish() method, as well as the turnLeft() method)
 * 3) Left wheel should power for 1 sec. (test goToFish() method, as well as the turnRight() method)
 * 4) Left wheel should power for 1 sec. (test the rotate() method)
 * 5) Right wheel should power for 1 sec. (test the rotate() method)
 * 6) Right wheel should power for 1 sec. (test the rotate() method)
 */
 
//Pins for motors
const byte leftMotorForward = 2;
const byte leftMotorBackward = 3;
const byte rightMotorForward = 4;
const byte rightMotorBackward = 5;

int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
int deadZone = 20; //How big the "center" of the robot is. Smaller values will cause robot to wiggle more.
int power = 160; //How much power for wheel motors. Valid values are 0 - 255.
int stepTimes[3] = {1000, 3000, 1000}; //An array where each element is how much time in milliseconds should be spent at each step of rotation.

Drivetrain *wheels;
Block testCenterBlock;
Block testRightBlock;
Block testLeftBlock;

void setup() 
{
  Serial.begin(9600);
  wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward, center, deadZone, power, stepTimes);  
  testCenterBlock.x = center;
  testRightBlock.x = center + deadZone + 1;
  testLeftBlock.x = center - deadZone - 1;
}

void loop() 
{
  //Tests as if there was a block in the center. Both wheels should power for 1 sec.
  (*wheels).goToFish(testCenterBlock);
  delay(1000);
  (*wheels).stopMotors();
  delay(1000);
  
  //Tests as if there was a block in the left. Right wheel should power for 1 sec.
  (*wheels).goToFish(testLeftBlock);
  delay(1000);
  (*wheels).stopMotors();
  delay(1000);
  
  //Tests as if there was a block in the right. Left wheel should power for 1 sec.
  (*wheels).goToFish(testRightBlock);
  delay(1000);
  (*wheels).stopMotors();
  delay(1000);
  
  //Tests the first rotation. Left wheel should power for 1 sec.
  (*wheels).rotate(1);
  delay(1000);
  
  //Tests the second rotation. Right wheel should power for 1 sec.
  (*wheels).rotate(2);
  delay(1000);
  
  //Tests the third rotation. Right wheel should power for 1 sec.
  (*wheels).rotate(3);
  delay(1000);
}
