#ifndef Drivetrain_h
#define Drivetrain_h

#include "Arduino.h"
#include "Sensors.h"
#include <Pixy.h>
#include <SPI.h>

/**
 * The drivetrain for the robot. Controls the motors driving where the robot goes.
 */
class Drivetrain
{
public:
	/**
	* Constructor. Sets the pins for all the motors.
	* center: Where the robot aims when it detects a block. Valid values are 0 - 319.
	* power: How much power for wheel motors. Valid values are 0 - 255.
	* *stepDegrees: An array where each element is how much degrees from initial heading at each step of rotation.
	*/
	Drivetrain(const byte leftMotorForward, const byte leftMotorBackward, const byte rightMotorForward, const byte rightMotorBackward,
			   int center, byte power,
			   float kp, float ki, float kd,
			   Compass* compass, int *stepDegrees, byte turnDeadzone);

	/**
	 * Destructor
	 */
	~Drivetrain();

	/**
	 * Gives power to motors, keeping the center of the block aligned with the requested center
	 * set in the Drivetrain constructor.
	 */
	void goToFishPID(Block block, unsigned long currentTime);

	/**
	* Rotates an amount based on what step we're on. StepNum > 0
	* Returns false if the robot has not rotated the correct amount, otherwise
	* returns true when the robot has rotated the required amount.
	*/
	boolean rotateDegrees(byte stepNum, byte power);

	/**
	* Brakes the motors.
	*/
	void stopMotors();

	/**
	* Give the right forward motor power.
	*/
	void turnLeft(byte power);

	/**
	* Give the left forward motor power.
	*/
	void turnRight(byte power);

	//Old methods

	/**
	* Gives all forward motors power. Since our motors move at different rates given the same power, it will inevitably turn.
	* Need encoders or a reference (like the pixy) to go in a straight line.
	*/
	void goStraight(byte power);

	/**
	* Gives power to motors, keeping the center of the block aligned with the requested center
	* set in the Drivetrain constructor.
	*/
	void goToFish(Block block);

	/**
	* Rotates an amount based on what step we're on. StepNum > 0
	* Returns false if the robot has not rotated the correct amount, otherwise
	* returns true when the robot has rotated the required amount.
	*/
	void rotate(byte stepNum);

private:
	byte _leftMotorForward; //Pin for left motor forward.
	byte _rightMotorForward; //Pin for right motor forward.
	byte _leftMotorBackward; //Pin for left motor backward.
	byte _rightMotorBackward; //Pin for right motor backward.

	//Motor variables
	int _center; //Where the robot aims when it detects a block. Valid values are 0 - 319.
	byte _power; //How much power for wheel motors. Valid values are 0 - 255.

	//PID controller variables
	unsigned long _previousTime;
	int _previousError;
	float _integral;
	float _kp;
	float _ki;
	float _kd;

	//Variables for rotate method
	int* _stepDegrees; //An array where each element is how much degrees from initial heading at each step of rotation.
	Compass* _compass; //Allow drivetrain access to a compass
	boolean _isRotating; //Boolean to keep track if the robot is in the rotate method
	//Keep track of values needed to turn correctly
	boolean _turnRight; //Keep track of turning right or left
	int _desiredDegrees;
	int _leftDegrees;
	int _rightDegrees;
	byte _turnDeadzone;
};

#endif
