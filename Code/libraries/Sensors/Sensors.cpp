#include <Arduino.h>
#include "Sensors.h"

/**
 * Constructor. Intitialize the following variables
 * badBlock: Initialize all values of the block to -1.
 * _pixy: Initialize the pixy camera.
 * _IRPort: Set the IRPort.
 * _stopVoltage: Set the stop voltage; The robot should stop whenever the 
 *               input voltage from the IR sensor is greater than this voltage.
 */
VisualSensor::VisualSensor(const char IRPort, const float stopVoltage)
{
  //Set all the badBlock's values to -1
  badBlock.signature = -1;
  badBlock.x = -1;
  badBlock.y = -1;
  badBlock.width = -1;
  badBlock.height = -1;

  //Initialize pixy
  _pixy.init();

  //Set IR Port
  _IRPort = IRPort;
  _stopVoltage = stopVoltage;
}

/**
 * Destructor
 */
VisualSensor::~VisualSensor()
{
}

/**
 * Find the correct block to go to. 
 * Currently finds the lowest one in the Pixy's view (lower = closer).
 * Returns null if there are no blocks found.
 */
Block VisualSensor::getBlock()
{
  //Get the number of blocks(detected objects) from the pixy
  int numBlocks = _pixy.getBlocks();

  if (numBlocks == 0)
  {
	  return badBlock;
  }

  //Serial.println( "--------" );

  //Set the initial maximum to be the first block found
  int maxY = _pixy.blocks[0].y;
  Block block; //Declare block to be returned
  //Loop through each block
  for (int j = 0; j < numBlocks; j++)
  {
	// _pixy.blocks[ j ].print();
    
	//Find the lowest block in the frame (which should be the closest block)
    //higher y value means lower in the frame
    if (_pixy.blocks[j].y >= maxY)
    {
      maxY = _pixy.blocks[j].y;
      block = _pixy.blocks[j];
    }
  }
  return block;
}

/**
 * Reads the input from the IRSensor port. This number is from 0-1023,
 * so we convert this number into a float from 0.0 to 5.0 volts. Return true if it is 
 * in the voltage range we want.
 */
boolean VisualSensor::isClose()
{
  float voltage = analogRead(_IRPort) * (5.0 / 1023.0);
  if (voltage > _stopVoltage)
  {
   return true;
  }
  return false;
}

/**
* Constructor. Set the initial heading whenever the program starts.
*/
Compass::Compass()
{
	_initDegrees = getDegrees();
}

Compass::~Compass()
{

}

/**
* Returns the current heading of the robot in degrees.
*/
int Compass::getDegrees()
{
	//TODO
	return -1;
}

/**
* Returns the initial heading when the program was first started in degrees.
*/
int Compass::getInitDegrees()
{
	return _initDegrees;
}
