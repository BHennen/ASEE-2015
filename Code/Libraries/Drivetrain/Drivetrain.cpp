#include "Arduino.h"
#include "Drivetrain.h"

/**
 * Constructor. Sets the pins for all the motors.
 * center: Where the robot aims when it detects a block. Valid values are 0 - 319.
 * power: How much power for wheel motors. Valid values are 0 - 255.
 * *stepDegrees: An array where each element is how much degrees from initial heading at each step of rotation.
 * kp, ki, kd: Constants for proportional, integral, derivative components used to tune the PID controller.
 */
Drivetrain::Drivetrain(const byte leftMotorForward, const byte leftMotorBackward, const byte rightMotorForward, const byte rightMotorBackward,
					   int center, byte power,
					   float kp, float ki, float kd,
					   Gyro* gyro, int *stepDegrees, byte turnDeadzone)
{
	//Set the pinmode of the motor ports to be output.
	pinMode(leftMotorForward, OUTPUT);
	pinMode(leftMotorBackward, OUTPUT);
	pinMode(rightMotorForward, OUTPUT);
	pinMode(rightMotorBackward, OUTPUT);

	//Set ports
	_leftMotorForward = leftMotorForward;
	_leftMotorBackward = leftMotorBackward;
	_rightMotorForward = rightMotorForward;
	_rightMotorBackward = rightMotorBackward;

	//Set motor variables
	_center = center;
	_power = power;

	//Variables for rotate method
	_stepDegrees = stepDegrees;
	_isRotating = false; //Boolean to keep track if the robot is in the rotate method
	//Keep track of values needed to turn correctly
	this->gyro = gyro;
	_turnRight = false;
	_desiredDegrees = 0; //Set initial desired degrees to be the initial degrees read in
	_leftDegrees = 0.0;
	_rightDegrees = 0.0;
	_turnDeadzone = turnDeadzone;//+- degrees acceptable

	//Set PID variables
	_previousTime = 0;
	_previousError = 0;
	_integral = 0;
	_kp = kp;
	_ki = ki;
	_kd = kd;
}

/**
 * Destructor
 */
Drivetrain::~Drivetrain()
{
}

/******************
 * Public Methods *
 ******************/

/**
* Uses PID control to go forward, trying to keep the robot aligned with _center given the currentValue passed into the function.
*/
void Drivetrain::goToFishPID(float currentValue, unsigned long currentTime)
{
	//Determine PID output
	int dt = currentTime - _previousTime; //Find how long has passed since the last adjustment.
	_previousTime = currentTime;
	//Serial.print("Dt: ");
	//Serial.println(dt);
	
	//Determine error; how far off the robot is from _center
	float error = _center - currentValue;
	//Serial.print("block.x:");
	//Serial.println(block.x);
	//Serial.print("Error: ");
	//Serial.println(error);

	//Determine integral; sum of all errors
	_integral += error*dt / 1000.0f; //Divide by 1000 because dt is milliseconds, adjust for seconds
	//Serial.print("Integral: ");
	//Serial.println(_integral);

	//Determine derivative; rate of change of errors
	float derivative = 1000.0f*(error - _previousError) / dt; //Multiply by 1000 because dt is milliseconds, adjust for seconds
	//Serial.print("Derivative: ");
	//Serial.println(derivative);
	
	//Determine output
	int output = (int) (_kp*error + _ki*_integral + _kd*derivative);
	//Serial.print("Output: ");
	//Serial.println(output);

	_previousError = error;

	//Go to the fish with the adjusted power values.
	//Before adjustment for PWM limits
	int rightPower = _power + output;
	int leftPower = _power - output;

	//After adjustment for PWM limits
	if(rightPower < 0)
	{
		rightPower = 0;
	}
	else if(rightPower > 255)
	{
		rightPower = 255;
	}
	if(leftPower < 0)
	{
		leftPower = 0;
	}
	else if(leftPower > 255)
	{
		leftPower = 255;
	}
	//Serial.print("Right power: ");
	//Serial.println(rightPower);
	//Serial.print("Left Power: ");
	//Serial.println(leftPower);

	//Go with new adjustments
	analogWrite(_rightMotorForward, rightPower);
	analogWrite(_rightMotorBackward, 0);
	analogWrite(_leftMotorForward, leftPower);
	analogWrite(_leftMotorBackward, 0);
}

/**
 * Sets the _center of the robot for use in the PID controller.
 */
void Drivetrain::setCenter(float desiredCenter)
{
	_center = desiredCenter;
}

/**
* Gets the _center value of the robot.
*/
float Drivetrain::getCenter()
{
	return _center;
}

/**
* Rotates an amount based on what step we're on. StepNum > 0
* Returns false if the robot has not rotated the correct amount, otherwise
* returns true when the robot has rotated the required amount.
*/
boolean Drivetrain::rotateDegrees(byte stepNum, byte power)
{
	if(!_isRotating) //If the robot is not currently rotating and this method is called, determine the values needed for the upcoming rotation
	{
		//Set the robots required degrees based on the initial degrees and the degrees required by the step
		//Increments desired degrees by what step we're on. So if we turn right 45 deg and left 45 deg, it will be back at the initial heading(which is what we want)
		_desiredDegrees += _stepDegrees[stepNum - 1]; 
		//Set desiredDegrees so that it is <180 and >=0
		if(_desiredDegrees >= 360)
		{
			_desiredDegrees -= 360;
		}
		else if(_desiredDegrees < 0)
		{
			_desiredDegrees += 360;
		}
	}
	else
	{
		//Robot is currently rotating, values not needed to be computed
	}

	//Turn the robot until the heading measured by the gyro is the correct heading determined by degrees
	float currentDegrees = gyro->getDegrees();
	//Check if robot has turned far enough
	//Calculate the angle between the desired and current degrees.
	//After corrections, negative diff means the robot should turn left. positive means it should turn right.
	float diff = _desiredDegrees - currentDegrees;
	if(diff > 180.0f) diff -= 360;
	if(diff < -180.0f) diff += 360;
	if(abs(diff) < _turnDeadzone)
	{
		//Serial.println("Done rotating!");
		//Robot has rotated the correct amount
		stopMotors();
		_isRotating = false;
		return true;
	}
	else //Robot has not rotated the correct amount, continue rotating
	{
		//Serial.println("--- Not rotated far enough: ---");		
		if(diff < 0)
		{
			//Serial.println("turn left");
			turnLeft(power);
		}
		else
		{
			//Serial.println("turn right");
			turnRight(power);
		}
		_isRotating = true;
		return false;
	}
}

/**
* Brakes the motors.
*/
void Drivetrain::stopMotors()
{
	analogWrite(_rightMotorForward, 100);
	analogWrite(_rightMotorBackward, 100);
	analogWrite(_leftMotorForward, 100);
	analogWrite(_leftMotorBackward, 100);
}

/**
 * Give the left forward motor power.
 */
void Drivetrain::turnRight(byte power)
{
	analogWrite(_rightMotorForward, 0);
	analogWrite(_rightMotorBackward, 0);
	analogWrite(_leftMotorForward, power);
	analogWrite(_leftMotorBackward, 0);
}

/**
 * Give the right forward motor power.
 */
void Drivetrain::turnLeft(byte power)
{
	analogWrite(_rightMotorForward, power);
	analogWrite(_rightMotorBackward, 0);
	analogWrite(_leftMotorForward, 0);
	analogWrite(_leftMotorBackward, 0);
}

//old methods

/**
* Gives all forward motors power. Since our motors move at different rates given the same power, it will inevitably turn.
* Need encoders or a reference (like the pixy) to go in a straight line.
*/
void Drivetrain::goStraight(byte power)
{
	analogWrite(_rightMotorForward, power);
	analogWrite(_rightMotorBackward, 0);
	analogWrite(_leftMotorForward, power);
	analogWrite(_leftMotorBackward, 0);
}

/**
* Gives power to motors, keeping the center of the block aligned with the requested center
* set in the Drivetrain constructor.
*/
void Drivetrain::goToFish(Block block)
{
	//If the center of the block is on the left side of the camera
	if(block.x < _center)
	{
		turnLeft(_power);
	}
	//If the center of the block is on the right side of the camera
	else if(block.x > _center)
	{
		turnRight(_power);
	}
	//The center of the block is in the center of the camera
	else
	{
		goStraight(_power);
	}
}

/**
* Rotates an amount based on what step we're on. StepNum > 0
*/
void Drivetrain::rotate(byte stepNum)
{
	int time = _stepDegrees[stepNum - 1];

	switch(stepNum)
	{
		case 1: turnRight(_power); //At fish 1, turn to fish 2
			delay(time);
			stopMotors();
			break;
		case 2: turnLeft(_power); //At fish 2, turn to fish 3
			delay(time);
			stopMotors();
			break;
		case 3: turnLeft(_power); //At fish 3, turn to fish 4
			delay(time);
			stopMotors();
			break;

			//Turn to face the outer ring of fish
		case 4: turnRight(_power); //At fish 4, turn to fish 5
			delay(time);
			stopMotors();
			break;
		case 5: turnLeft(_power); //At fish 5, turn to fish 6
			delay(time);
			stopMotors();
			break;
		case 6: turnLeft(_power); //At fish 6, turn to fish 7
			delay(time);
			stopMotors();
			break;
		case 7: turnLeft(_power); //At fish 7, turn to fish 8
			delay(time);
			stopMotors();
			break;
		case 8: turnLeft(_power); //At fish 8, turn to fish 9
			delay(time);
			stopMotors();
			break;
		case 9: turnLeft(_power); //At fish 9, turn to fish 10
			delay(time);
			stopMotors();
			break;
		case 10: turnLeft(_power); //At fish 10, turn to fish 11
			delay(time);
			stopMotors();
			break;
		case 11: turnLeft(_power); //At fish 11, turn to fish 12
			delay(time);
			stopMotors();
			break;

			//End of fish collection route
		case 12: turnRight(_power); //At fish 12, face bin 1
			delay(time);
			stopMotors();
			break;
		case 13: turnLeft(_power); //At bin 1, reposition for dumping
			delay(time);
			stopMotors();
			break;
		case 14: turnLeft(_power); //At bin 1, face bin 2
			delay(time);
			stopMotors();
			break;
		case 15: turnLeft(_power); //At bin 2, reposition for dumping
			delay(time);
			stopMotors();
			break;
		case 16: turnLeft(_power); //At bin 2, face bin 3
			delay(time);
			stopMotors();
			break;
		case 17: turnLeft(_power); //At bin 3, reposition for dumping
			delay(time);
			stopMotors();
			break;
		case 18: turnLeft(_power); //At bin 3, face bin 4
			delay(time);
			stopMotors();
			break;
		case 19: turnLeft(_power); //At bin 4, reposition for dumping
			delay(time);
			stopMotors();
			break;
	}
}


