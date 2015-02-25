#include <SPI.h>
#include <Pixy.h>
#include <Sensors.h>
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
byte leftMotorForward = 2;
byte leftMotorBackward = 3;
byte rightMotorForward = 4;
byte rightMotorBackward = 5;

//Constants for motors
int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
byte power = 100; //How much power for wheel motors. Valid values are 0 - 255.

//Constant for turning
int stepTimes[3] = {1000, 3000, 1000}; //An array where each element is how much time in milliseconds should be spent at each step of rotation.
byte turnDeadzone = 2;

//Constants for PID controller
float kp = 0.0; //proportional
float ki = 0.0; //integral
float kd = 0.0; //derivative

//Constants for visual sensor
const char IRPort = A0; //Port for IR sensor
float stopVoltage = 2.6; //Voltage to stop the robot

//Pointers to robot objects
Drivetrain *wheels;
Compass *compass; //We dont actually use compass here
Block testCenterBlock;
Block testRightBlock;
Block testLeftBlock;

void setup() 
{
  Serial.begin(9600);
  wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward,
                          center, power,
                          kp, ki, kd,
                          compass, stepTimes, turnDeadzone);
  testCenterBlock.x = center;
  testRightBlock.x = center + 1;
  testLeftBlock.x = center - 1;
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
