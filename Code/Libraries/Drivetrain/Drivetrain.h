#ifndef Drivetrain_h
#define Drivetrain_h

#include "Arduino.h"
#include "Sensors.h"
#include <Pixy.h>
#include <SPI.h>

struct Rotation
{
	float angleFromVertical;
	float length;
	byte rotationType; //0 = stationary, 1 = sweep
	boolean direction; //0 = left; 1 = right
};

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
		Gyro* gyro, Rotation *rotations, byte turnDeadzone, double robotStopDist, double robotTurnRadius, double robotCenter);

	/**
	 * Destructor
	 */
	~Drivetrain();

	void resetIntegral();

	/**
	 * Uses PID control to go forward, trying to keep the robot aligned with the desired value passed into the function.
	 * Returns the error.
	 */
	float goUsingPID(float currentValue, float desiredValue, float* PIDconsts, unsigned long currentTime);

	/**
	 * Sets the _center of the robot for use in the PID controller.
	 */
	void setCenter(float desiredCenter);

	/**
	 * Gets the _center value of the robot.
	 */
	float getCenter();

	/**
	* Rotates an amount based on what step we're on. StepNum > 0
	* Returns false if the robot has not rotated the correct amount, otherwise
	* returns true when the robot has rotated the required amount.
	*/
	boolean rotateDegrees(byte stepNum, byte power);

	/**
	* Rotates an amount based on what step we're on.
	* Returns false if the robot has not rotated the correct amount, otherwise
	* returns true when the robot has rotated the required amount. Internally the step
	* is updated so the next time the function is called it will rotate to the desired degrees.
	* Uses default power.
	*/
	boolean rotateToNextPosition();

	/**
	* Drives to the next position using the PID based on the gyroscope
	*/
	void driveToNextPosition(unsigned long currentTime);

	/**
	 * TODO
	 */
	double determineNextAngle(Rotation nextFishValues, float currentDegrees);

	/**
	* Brakes the motors.
	*/
	void stopMotors();

	/**
	* Give both wheels power to rotate on a dime.
	*/
	void turnStationary(byte power, boolean right);

	/**
	* Give a single motor power to sweep out an area.
	*/
	void turnSweep(byte power, boolean right, boolean backwards);

	boolean _isRotating; //Boolean to keep track if the robot is in the rotate method
	float drivingDegrees;

	Gyro* gyro; //Allow drivetrain access to a gyro
	int _stepNum;
	byte _power; //How much power for wheel motors. Valid values are 0 - 255.
	byte _turnDeadzone;

private:
	byte _leftMotorForward; //Pin for left motor forward.
	byte _rightMotorForward; //Pin for right motor forward.
	byte _leftMotorBackward; //Pin for left motor backward.
	byte _rightMotorBackward; //Pin for right motor backward.

	//Motor variables
	float _center; //Where the robot aims for the PID control.

	//PID controller variables
	unsigned long _previousTime;
	float _previousError;
	float _integral;

	//Variables for rotate method
	Rotation *_rotations; //An array where each element is how much degrees from initial heading at each step of rotation.
	//Keep track of values needed to turn correctly
	boolean _turnRight; //Keep track of turning right or left
	float _desiredDegrees;
	float _leftDegrees;
	float _rightDegrees;

	double _robotStopDist;
	double _robotTurnRadius;
	double _robotCenter;

};

#endif
