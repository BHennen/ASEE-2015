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
	Gyro* gyro, Rotation* rotations, byte turnDeadzone, double robotStopDist, double robotTurnRadius, double robotCenter, double lengthToFirstFish)
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

	_rotations = rotations;
	_isRotating = false; //Boolean to keep track if the robot is in the rotate method
	_stepNum = 0;
	//Keep track of values needed to turn correctly
	this->gyro = gyro;
	_turnRight = false;
	_desiredDegrees = 0; //Set initial desired degrees to be the initial degrees read in
	drivingDegrees = 0; //Set driving degrees to be initially 0
	_leftDegrees = 0.0;
	_rightDegrees = 0.0;
	_turnDeadzone = turnDeadzone;//+- degrees acceptable
	_robotStopDist = robotStopDist;
	_robotTurnRadius = robotTurnRadius;
	_robotCenter = robotCenter;
	_lengthToPrevFish = lengthToFirstFish;
	//Set PID variables
	_previousTime = 0;
	_previousError = 0;
	_integral = 0;
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
* Make sure everything is good to go before we start
*/
boolean Drivetrain::setup(unsigned long currentTime)
{
	return true; //wheels need no setup prior to moving
}

void Drivetrain::resetIntegral()
{
	_integral = 0;
}

/**
* Uses PID control to go forward, trying to keep the robot aligned with the desired value passed into the function.
* Returns the error.
*/
float Drivetrain::goUsingPID(float currentValue, float desiredValue, float* PIDconsts, unsigned long currentTime, bool stationary, bool forwards)
{
	//Determine PID output
	static int dt = 0;
	(currentTime != _previousTime) ? dt = currentTime - _previousTime : 1;//Find how long has passed since the last adjustment.
	_previousTime = currentTime;
	//Serial.print("Dt: ");
	//Serial.print(dt);

	//Determine error; how far off the robot is from desired value
	float error = currentValue - desiredValue;
	if (!forwards) error = -error;
	//Serial.print("Error: ");
	//Serial.print(error);

	//Determine integral; sum of all errors
	_integral += error*dt / 1000.0f; //Divide by 1000 because dt is milliseconds, adjust for seconds
	//Serial.print("\tIntegral: ");
	//Serial.print(_integral);

	//Determine derivative; rate of change of errors
	float derivative = 1000.0f*(error - _previousError) / dt; //Multiply by 1000 because dt is milliseconds, adjust for seconds
	//Serial.print("\tDerivative: ");
	//Serial.print(derivative);

	//Determine output
	int output = (int)(PIDconsts[0] * error + PIDconsts[1] * _integral + PIDconsts[2] * derivative);
	//Serial.print("\tOutput: ");
	//Serial.print(output);

	_previousError = error;

	//Go to the fish with the adjusted power values.
	//Before adjustment for PWM limits
	int rightPower = _power + output;
	int leftPower = _power - output;

	//After adjustment for PWM limits
	if (rightPower < 0)
	{
		rightPower = 0;
	}
	else if (rightPower > 255)
	{
		rightPower = 255;
	}
	if (leftPower < 0)
	{
		leftPower = 0;
	}
	else if (leftPower > 255)
	{
		leftPower = 255;
	}
	//Serial.print("\t");
	//Serial.print("Rpower: ");
	//Serial.print(rightPower);
	//Serial.print("\t");
	//Serial.print("LPower: ");
	//Serial.println(leftPower);

	//Go with new adjustments
	if (!stationary)
	{
		if (forwards)
		{
			analogWrite(_rightMotorForward, rightPower);
			analogWrite(_rightMotorBackward, 0);
			analogWrite(_leftMotorForward, leftPower);
			analogWrite(_leftMotorBackward, 0);
		}
		else
		{
			analogWrite(_rightMotorForward, 0);
			analogWrite(_rightMotorBackward, rightPower);
			analogWrite(_leftMotorForward, 0);
			analogWrite(_leftMotorBackward, leftPower);
		}
	}
	else
	{

		leftPower = _power * 0.4 - output;
		rightPower = _power * 0.4 + output;
		if (output < 0) //rotate right
		{
			analogWrite(_rightMotorForward, 0);
			analogWrite(_rightMotorBackward, leftPower);
			analogWrite(_leftMotorForward, leftPower);
			analogWrite(_leftMotorBackward, 0);
		}
		else //rotate left
		{
			analogWrite(_rightMotorForward, rightPower);
			analogWrite(_rightMotorBackward, 0);
			analogWrite(_leftMotorForward, 0);
			analogWrite(_leftMotorBackward, rightPower);
		}
	}

	return error;
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
boolean Drivetrain::rotateDegrees(byte stepNum, byte power, unsigned long currentTime)
{
	float currentDegrees = gyro->getDegrees();

	if (!_isRotating) //If the robot is not currently rotating and this method is called, determine the values needed for the upcoming rotation
	{
		Serial.println(stepNum);
		Serial.println(_desiredDegrees);

		//Set the robots required degrees based on the initial degrees and the degrees required by the step
		//Increments desired degrees by what step we're on. So if we turn right 45 deg and left 45 deg, it will be back at the initial heading(which is what we want)
		_desiredDegrees = determineNextAngle(_rotations[stepNum - 1], currentTime);
		drivingDegrees = _desiredDegrees; //Update driving degrees when we rotate
		//Set desiredDegrees so that it is <180 and >=0
		if (_desiredDegrees >= 360)
		{
			_desiredDegrees -= 360;
		}
		else if (_desiredDegrees < 0)
		{
			_desiredDegrees += 360;
		}
	}
	else
	{
		//Robot is currently rotating, values not needed to be computed
	}

	//Turn the robot until the heading measured by the gyro is the correct heading determined by degrees
	//Serial.print(currentDegrees);
	//Serial.print("\t");
	//Serial.println(_desiredDegrees);
	//Check if robot has turned far enough
	//Calculate the angle between the desired and current degrees.
	//After corrections, negative diff means the robot should turn left. positive means it should turn right.
	float diff = _desiredDegrees - currentDegrees;
	if (diff > 180.0f) diff -= 360;
	if (diff < -180.0f) diff += 360;
	if (abs(diff) < _turnDeadzone)
	{
		//Robot has rotated the correct amount
		stopMotors();
		_isRotating = false;
		return true;
	}
	else //Robot has not rotated the correct amount, continue rotating
	{
		if (_rotations[stepNum - 1].rotationType == 0) //stationary rotation
		{
			turnStationary(_power, diff < 0); //turn based on the difference
		}
		else//Sweep
		{
			turnSweep(_power, _rotations[stepNum - 1].direction, diff > 0);
		}
		_isRotating = true;
		return false;
	}
}

/**
* Rotates an amount based on what step we're on.
* Returns false if the robot has not rotated the correct amount, otherwise
* returns true when the robot has rotated the required amount. Internally the step
* is updated so the next time the function is called it will rotate to the desired degrees.
*/
boolean Drivetrain::rotateToNextPosition(unsigned long currentTime)
{
	static boolean incStepNum = true;

	if (incStepNum)
	{
		_stepNum++;
		incStepNum = false;
	}
	if (rotateDegrees(_stepNum, _power, currentTime)) //Rotate to the angle provided by current step using default power
	{
		//when we've reached the correct angle
		incStepNum = true;
		return true;
	}
	return false;
}

/**
 * Drives to the next position using the PID based on the gyroscope
 */
void Drivetrain::driveToNextPosition(unsigned long currentTime, boolean forwards)
{
	//get current gyro readings
	float currentDegrees = gyro->getDegrees();
	//Serial.print(currentDegrees);
	//Serial.print("\t");
	//Serial.print(drivingDegrees);

	//Calculate the angle between the driving degrees and current degrees.
	//After corrections, negative diff means the robot should turn left. positive means it should turn right.
	float diff = drivingDegrees - currentDegrees;
	//Serial.print(drivingDegrees);
	//Serial.print("\t");
	//Serial.print(currentDegrees);
	//Serial.print("\t");
	//Serial.println(diff);

	if (diff > 180.0f) diff -= 360;
	if (diff < -180.0f) diff += 360;

	//go in a straight line based on gyro
	goUsingPID(diff, 0, gyro->_PIDconsts, currentTime, false, forwards);
}

/**
 * Rotations should have:
 *	-angleFromVertical
 *	-length
 */
double Drivetrain::determineNextAngle(Rotation nextFishValues, unsigned long currentTime)
{
	static unsigned long previousTime = currentTime;
	unsigned long sampleTime = currentTime - previousTime; //Measure how long it's been since we last rotated
	previousTime = currentTime;

	static double _expectedAngle = 0;
	static double _alignmentRadius = _robotTurnRadius - _robotCenter;
	Serial.println("-------------");
	Serial.print("Expected Angle: \t");
	Serial.println(_expectedAngle);
	Serial.print("Current Angle: \t");
	Serial.println(gyro->getDegrees());
	//Determine the offset (how far we are off from the expected center)
	double diffExpected = gyro->getDegrees() - _expectedAngle;
	if (diffExpected > 180.0f) diffExpected -= 360;
	if (diffExpected < -180.0f) diffExpected += 360;
	double diffAngle = 180 - diffExpected;
	double angleOffDeg = abs(diffAngle);
	double angleOffRad = angleOffDeg / 180 * M_PI;

	double diffAnglePerDist = diffExpected / _lengthToPrevFish;
	Serial.print("diff angle per dist: \t");
	Serial.println(diffAnglePerDist);
	double diffAnglePerTime = (sampleTime > 0) ? diffExpected / sampleTime : 0; //Measure how far off the gyro was per millisecond of travel.
	static int numSamples = 0;
	numSamples++;
	gyro->averageTimedBias = (gyro->averageTimedBias*(numSamples - 1) + diffAnglePerTime) / numSamples;
	static double avgDiffAnglePerDist = 0;
	avgDiffAnglePerDist = (avgDiffAnglePerDist*(numSamples - 1) + diffAnglePerDist) / numSamples;
	Serial.print("Avg timed bias: \t");
	Serial.println(gyro->averageTimedBias);
	Serial.print("Avg diff angle per dist: \t");
	Serial.println(avgDiffAnglePerDist);

	Serial.print("Prev Fish Length: \t");
	Serial.println(_lengthToPrevFish);
	double offSetAngleRad = asin(sin(angleOffRad) / _lengthToPrevFish * _robotStopDist);
	double offSetAngleDeg = offSetAngleRad * 180 / M_PI;
	Serial.print("Offset Angle: \t");
	Serial.println(offSetAngleDeg);
	gyro->_offSetAngle = ((gyro->_offSetAngle) * (numSamples - 1) + ((diffAngle > 0) ? -offSetAngleDeg : offSetAngleDeg)) / numSamples;
	//Set previous fish length
	_lengthToPrevFish = nextFishValues.length;

	//Find the next angle
	//Get current new degrees with the offset
	double currentDegrees = gyro->getDegrees();
	Serial.print("New Degrees: \t");
	Serial.println(currentDegrees);
	//Find angle: robot_old fish_new fish
	double diff1 = nextFishValues.angleFromVertical - currentDegrees;
	if (diff1 > 180) diff1 -= 360;
	if (diff1 < -180) diff1 += 360;
	bool flipped1 = false;
	if (diff1 < 0) flipped1 = true;
	double angle1Deg = 180 - abs(diff1);
	double angle1Rad = angle1Deg / 180.0 * M_PI;

	//Find distance: robot_new fish
	double distToFish = sqrt(pow(nextFishValues.length, 2) + pow(_robotStopDist, 2) - 2 * nextFishValues.length * _robotStopDist * cos(angle1Rad));

	//find angle: old fish_robot_new fish
	double angle2Rad = acos((pow(_robotStopDist, 2) + pow(distToFish, 2) - pow(nextFishValues.length, 2)) / (2 * _robotStopDist * distToFish));
	double angle2Deg = angle2Rad * 180 / M_PI;

	//find angle from vertical to distToFish
	double newHeading1 = (flipped1) ? currentDegrees - angle2Deg : currentDegrees + angle2Deg;
	//newHeading1 -= avgDiffAnglePerDist * nextFishValues.length;
	if (newHeading1 > 360) newHeading1 -= 360;
	if (newHeading1 < 0) newHeading1 += 360;
	if (nextFishValues.rotationType == 1)
	{
		//Find the second heading(for rotating about one wheel)
		//find small chord angle
		double diff2 = angle2Deg - 90;
		bool flipped2 = false;
		if (diff2 < 0) flipped2 = true;
		double smallChordAngleDeg = abs(diff2);
		double smallChordAngleRad = smallChordAngleDeg / 180 * M_PI;

		//find large chord angle
		double largeChordAngleDeg = 180 - smallChordAngleDeg * 2;
		double largeChordAngleRad = largeChordAngleDeg / 180 * M_PI;

		//find chord length
		double chordLength = sqrt(2 * pow(_alignmentRadius, 2) * (1 - cos(largeChordAngleRad)));

		//Find smallest path to fish
		double smallDistToFish = (flipped1) ? distToFish - chordLength : distToFish;
		//find tangent line length
		double tangentLength = sqrt(smallDistToFish*(smallDistToFish + chordLength));

		//calculate intersection points of circles
		struct Point
		{
			double x;
			double y;
			Point operator +(Point p)
			{
				return Point{ x + p.x, y + p.y };
			}
			Point operator -(Point p)
			{
				return Point{ x - p.x, y - p.x };
			}
			Point operator *(double scalar)
			{
				return Point{ x * scalar, y * scalar };
			}
			double distance(Point p)
			{
				return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
			}
		};
		struct Circle
		{
			Point center;
			double r;
			void intersect(Circle c, Point* intersection1, Point* intersection2)
			{
				Point P0(center);
				Point P1(c.center);
				double d, a, h;
				d = P0.distance(P1);
				a = (r*r - c.r*c.r + d*d) / (2 * d);
				h = sqrt(r*r - a*a);
				Point P2 = P0 + (P1 - P0) * (a / d);
				double x3, y3, x4, y4;
				x3 = P2.x + h*(P1.y - P0.y) / d;
				y3 = P2.y - h*(P1.x - P0.x) / d;
				x4 = P2.x - h*(P1.y - P0.y) / d;
				y4 = P2.y + h*(P1.x - P0.x) / d;

				*intersection1 = Point{ x3, y3 };
				*intersection2 = Point{ x4, y4 };
			}
		};
		Point center1{ 0, 0 };
		Point center2{ -tangentLength, _alignmentRadius };
		Circle circle1{ center1, smallDistToFish };
		Circle circle2{ center2, _alignmentRadius };

		//Get pair of intersections
		Point intersection1;
		Point intersection2;
		circle1.intersect(circle2, &intersection1, &intersection2);

		//Get desired point
		Point desiredPoint;
		if (flipped2)
		{
			desiredPoint = intersection2;
		}
		else
		{
			desiredPoint = intersection1;
		}

		//Get angle of tangent line of turn radius to new fish between the line from the robot to the new fish
		double angle3Rad = atan(desiredPoint.y / desiredPoint.x);
		double angle3Deg = angle3Rad * 180 / M_PI;

		//get new heading 2
		double newHeading2 = newHeading1 + angle3Deg;
		if (newHeading2 > 360) newHeading2 -= 360;
		if (newHeading2 < 0) newHeading2 += 360;
		_expectedAngle = newHeading2;

		Serial.print("New Heading: \t");
		Serial.println(newHeading2);
		return newHeading2;
	}
	else
	{
		_expectedAngle = newHeading1;

		Serial.print("New Heading: \t");
		Serial.println(newHeading1);
		return newHeading1;
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
* Give both wheels power to rotate on a dime.
*/
void Drivetrain::turnStationary(byte power, boolean right)
{
	if (right) //rotate right
	{
		analogWrite(_rightMotorForward, 0);
		analogWrite(_rightMotorBackward, power);
		analogWrite(_leftMotorForward, power);
		analogWrite(_leftMotorBackward, 0);
	}
	else //rotate left
	{
		analogWrite(_rightMotorForward, power);
		analogWrite(_rightMotorBackward, 0);
		analogWrite(_leftMotorForward, 0);
		analogWrite(_leftMotorBackward, power);
	}
}

/**
* Give a single motor power to sweep out an area.
*/
void Drivetrain::turnSweep(byte power, boolean right, boolean backwards)
{
	if (!right)//Sweep right
	{
		if (backwards) //Go backwards
		{
			analogWrite(_leftMotorForward, 0);
			analogWrite(_leftMotorBackward, power);
		}
		else //Go forward
		{
			analogWrite(_leftMotorForward, power);
			analogWrite(_leftMotorBackward, 0);
		}
		analogWrite(_rightMotorForward, 0);
		analogWrite(_rightMotorBackward, 0);
	}
	else //Sweep left
	{
		if (backwards) //Go backwards
		{
			analogWrite(_rightMotorForward, 0);
			analogWrite(_rightMotorBackward, power);
		}
		else //Go forward
		{
			analogWrite(_rightMotorForward, power);
			analogWrite(_rightMotorBackward, 0);
		}
		analogWrite(_leftMotorForward, 0);
		analogWrite(_leftMotorBackward, 0);
	}
}