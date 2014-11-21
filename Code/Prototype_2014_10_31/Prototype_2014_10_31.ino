/**
 * Include header files
 */
#include <SPI.h>
#include <Pixy.h>
#include "Definitions.h"

/**
 * Global variables
 */
Pixy pixy;
float stopVoltage;
int speed;


/**
 * Initialization of variables; runs only once
 */
void setup()
{
  speed = 160; //Default speed of our motors.

  stopVoltage = 2.8; //3.2 volts is the maximum, so we have a dead zone of 0.4 volts

  pixy.init(); //Initialize pixy

  Serial.begin(9600);
  Serial.print("Starting...\n");

  //Set the pinmode of the motor ports to be output.
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
}

/**
 * The main function, runs indefinitely
 */
void loop()
{
  //Get IRVoltage from IR sensor
  float IRVoltage = getIRVoltage();
  //If IR detects object that is close, then stop the motors. 
  if (IRVoltage > stopVoltage)
  {
    Serial.println("fish too close");
    stopMotors();
  }
  else //The IR didnt detect a close object; look for a fish
  {
    int numBlocks = pixy.getBlocks(); //Get all the blocks(detected objects) from the pixy
    int numGoodBlocks = 0; //Sets the number of "good blocks" to be 0 initially.
    //Loop throgh each block
    for (int j = 0; j < numBlocks; j++)
    {
      //Filter out really small blocks(which is currently our only filter for blocks) and move the robot.
      if (pixy.blocks[j].width > 30 && pixy.blocks[j].width < 50)
      {
        numGoodBlocks++; //We now have at least one good block, increment number of good blocks

        //See where the block we detected is. Right now, our code does this for every block we see and doesn't choose ONLY one fish to go to.
        //If the center of the block is on the left side of the camera
        if (pixy.blocks[j].x < 150)
        {
          Serial.print("turning left\n");
          go(speed, left);
        }
        //If the center of the block is on the right side of the camera
        else if (pixy.blocks[j].x > 170)
        {
          Serial.print("turning right\n");
          go(speed, right);
        }
        //The center of the block is in the center of the camera
        else //Dead zone
        {
          Serial.print("not turning: Dead Zone\n");
          go(speed, straight);
        }
      }
      //If  there was NO blocks detected or if there is no GOOD blocks, we dont move. This is only temporary for our prototype
      if (numBlocks == 0 || numGoodBlocks == 0)
      {
        if (numBlocks == 0)Serial.print("not turning: No Blocks\n");
        if (numGoodBlocks == 0)Serial.print("not turning: No Good Blocks\n");
        stopMotors();
      }
    }
  }
}
