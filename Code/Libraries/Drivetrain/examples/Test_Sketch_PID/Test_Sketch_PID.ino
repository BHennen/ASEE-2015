#include <SPI.h>
#include <Pixy.h>
#include <VisualSensor.h>
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

int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
byte power = 80; //How much power for wheel motors. Valid values are 0 - 255.
int stepTimes[] = {1000};

//Constants for PID controller
float kp = 0.4; //proportional
float ki = 0.0; //integral
float kd = 0.0; //derivative

VisualSensor *eyes;
Drivetrain *wheels;

const char IRPort = A0; //Port for IR sensor
float stopVoltage = 2.6; //Voltage to stop the robot

void setup()
{
  Serial.begin(9600);
  eyes = new VisualSensor(IRPort, stopVoltage);
  wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward, center, power, stepTimes, kp, ki, kd);
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
