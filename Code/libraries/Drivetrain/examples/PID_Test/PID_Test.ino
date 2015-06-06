#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>
#include <Sensors.h>
#include <Drivetrain.h>
#include <Wire.h>
#include <eeprom.h>
#include <EEPROMAnything.h>
#include <L3G.h>

/**********************************
 * Test Sketch for PID Controller *
 **********************************
 * Tests the PID Controller of the Drivetrain Library.
 * Goes toward a fish using PID control.
 */

//Pins for motors
byte leftMotorForward   = 3;
byte leftMotorBackward  = 2;
byte rightMotorForward  = 5;
byte rightMotorBackward = 4;

//Constants for motors
int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
byte power = 100; //How much power for wheel motors. Valid values are 0 - 255.

//Constant for turning
int stepDegrees[] = { 45 };
byte turnDeadzone = 2;

//Constants for PID controller
float kp = 0.5f; //0.5;  //proportional
float ki = 0.1f; //0.1; //integral
float kd = 0.07f; //0.07;  //derivative

//Constants for visual sensor
const char IRPort = A0; //Port for IR sensor
float stopVoltage = 2.6; //Voltage to stop the robot

//Pointers to robot objects
VisualSensor *eyes;
Drivetrain *wheels;
Gyro *gyro;

void setup()
{
	Serial.begin(9600);

	//Create objects
	eyes = new VisualSensor(IRPort, stopVoltage);
	gyro = new Gyro();
	wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward,
		center, power,
		kp, ki, kd,
		gyro, stepDegrees, turnDeadzone);
}

void loop()
{
	unsigned long currentTime = millis();

	//See if there is a block in front of robot
	if (!eyes->isClose())
	{
		//Move toward the closest fish
		Block targetBlock = eyes->getBlock(); //Get closest fish
		float targetValue = (float)targetBlock.x;
		//Get block returns a bad block if no blocks were found, check if the block is the bad block
		if (targetBlock.signature != eyes->badBlock.signature)
		{
			wheels->goToFishPID(targetValue, currentTime); //Block is good, Move toward it
		}
		else
		{
			wheels->resetIntegral();
			wheels->stopMotors();
		}
	}
	else //there is a block in front
	{
		wheels->resetIntegral();
		wheels->stopMotors();
	}
}
