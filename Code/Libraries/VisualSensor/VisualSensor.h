#ifndef VISUALSENSOR_H
#define VISUALSENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>

class VisualSensor
{
  public:
	/**
	 * Constructor. Intitialize the following variables
	 * _pixy: Initialize the pixy camera.
	 * _IRPort: Set the IRPort.
	 * _stopVoltage: Set the stop voltage; The robot should stop whenever the
	 *               input voltage from the IR sensor is greater than this voltage.
	 */
    VisualSensor(const char IRPort, float stopVoltage);

	/**
	 * Destructor
	 */
    ~VisualSensor();

	/**
	 * Find the correct block to go to.
	 * Currently finds the lowest one in the Pixy's view (lower = closer).
	 */
    Block getBlock();

	/**
	 * Reads the input from the IRSensor port. This number is from 0-1023,
	 * so we convert this number into a float from 0.0 to 5.0 volts. Return true if it is
	 * in the voltage range we want.
	 */
    boolean isClose();

	Block badBlock; //Variable that is a "bad block", used when we find no good blocks
  private:
    Pixy _pixy; //Variable for pixy camera
    float _stopVoltage; //The robot should stop whenever the input voltage from the IR sensor is greater than this voltage.
    char _IRPort; //The port for the IR sensor
};

#endif
    


