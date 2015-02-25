#include <SPI.h>
#include <Pixy.h>
#include <Sensors.h>
#include <Drivetrain.h>

/**********************************
 * Test Sketch for PID Controller *
 **********************************
 * Tests the PID Controller of the Drivetrain Library.
 * Goes toward a fish using PID control.
 */

//Pins for motors
byte leftMotorForward = 2;
byte leftMotorBackward = 3;
byte rightMotorForward = 4;
byte rightMotorBackward = 5;

//Constants for motors
int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
byte power = 80; //How much power for wheel motors. Valid values are 0 - 255.

//Constant for turning
int stepDegrees[] = {45};
byte turnDeadzone = 2;

//Constants for PID controller
float kp = 0.25; //proportional
float ki = 0.025; //integral
float kd = 0.07; //derivative

//Constants for visual sensor
const char IRPort = A0; //Port for IR sensor
float stopVoltage = 2.6; //Voltage to stop the robot

//Pointers to robot objects
VisualSensor *eyes;
Drivetrain *wheels;
Compass *compass;

void setup()
{
  Serial.begin(9600);
  
  //Create objects
  eyes = new VisualSensor(IRPort, stopVoltage);
  compass = new Compass();
  wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward,
                          center, power,
                          kp, ki, kd,
                          compass, stepDegrees, turnDeadzone);
}

void loop()
{
  unsigned long currentTime = millis();

  //See if there is a block in front of robot
  if (!(*eyes).isClose())
  {
     //Move toward the closest fish
    Block targetBlock = (*eyes).getBlock(); //Get closest fish
    
    //Get block returns a bad block if no blocks were found, check if the block is the bad block
    if (targetBlock.signature != (*eyes).badBlock.signature)
    {
      (*wheels).goToFishPID(targetBlock, currentTime); //Block is good, Move toward it
    }
    else
    {
      (*wheels).stopMotors();
    }
  }
  else //there is a block in front
  {
    (*wheels).stopMotors();
  }
}
