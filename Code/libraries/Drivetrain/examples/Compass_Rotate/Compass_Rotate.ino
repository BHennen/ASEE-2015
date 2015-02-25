#include <SPI.h>
#include <Pixy.h>
#include <Sensors.h>
#include <Drivetrain.h>

/************************************
 * Test Sketch for Compass Rotation *
 ************************************
 * Tests the compass rotate method of the Drivetrain Library.
 * Simulates the rotating part of the fish collection route.
 * At the end, it should face 45 degrees to the left of the original orientation.
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
int stepDegrees[] = {-45, //At fish 1, turn RIGHT to fish 2
                     90, //At fish 2, turn LEFT to fish 3
                     90, //At fish 3, turn LEFT to fish 4

                     //Turn to face the outer ring of fish
                     -45, //At fish 4, turn RIGHT to fish 5
                     135, //At fish 5, turn LEFT to fish 6
                     45, //At fish 6, turn LEFT to fish 7
                     45, //At fish 7, turn LEFT to fish 8
                     45, //At fish 8, turn LEFT to fish 9
                     45, //At fish 9, turn LEFT to fish 10
                     45, //At fish 10, turn LEFT to fish 11
                     45, //At fish 11, turn LEFT to fish 12

                     //End of fish collection route
                     -45, //At fish 12, turn RIGHT to face bin 1
                     90, //At bin 1, reposition for dumping
                     45, //At bin 1, face bin 2
                     45, //At bin 2, reposition for dumping
                     45, //At bin 2, face bin 3
                     45, //At bin 3, reposition for dumping
                     45, //At bin 3, face bin 4
                     45, //At bin 4, reposition for dumping
                     };
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

byte stepNum;

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
                          
  stepNum = 1;
}

void loop()
{
  //Rotates through 12 steps in stepDegrees (This rotates through the fish collection route and ends facing the first bin).
  if(stepNum < 13 && wheels->rotateDegrees(stepNum, power)) //rotateDegrees returns TRUE when it has finished rotating
  {
    //Robot has rotated fully
    stepNum++; //Increment step
    delay(1000); //Wait 1 sec
  }
}
