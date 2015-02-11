#include "Arduino.h"
#include "Drivetrain.h"

/**
 * Constructor. Sets the pins for all the motors.
 * center: Where the robot aims when it detects a block. Valid values are 0 - 319.
 * power: How much power for wheel motors. Valid values are 0 - 255.
 * *stepTimes: An array where each element is how much time in milliseconds should be spent at each step of rotation.
 * kp, ki, kd: Constants for proportional, integral, derivative components used to tune the PID controller.
 */
Drivetrain::Drivetrain(const byte leftMotorForward, const byte leftMotorBackward,
                       const byte rightMotorForward, const byte rightMotorBackward,
                       int center, byte power, int *stepTimes,
					   float kp, float ki, float kd)
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
  _stepTimes = stepTimes;

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
* Uses PID control to go toward a block. Tries to keep the block aligned with _center.
*/
void Drivetrain::goToFishPID( Block block, unsigned long currentTime )
{
	unsigned long dt = currentTime - _previousTime; //Find how long has passed since the last adjustment.
	_previousTime = currentTime;

	//Determine error; how far off the robot is from center
	int error = _center - block.x; 
	Serial.print("error: ");
	Serial.println(error);

	/*
	//Determine integral; sum of all errors
	_integral += error*dt; 
	//Serial.print("integral: ");
	//Serial.println(_integral);

	//Determine derivative; rate of change of errors
	float derivative = (error - _previousError)/dt; 
	//Serial.print("derivative: ");
	//Serial.println(derivative);
	*/

	//Determine output
	int output = (int)(_kp*error); //+ _ki*_integral + _kd*derivative);
	Serial.print( "output: " );
	Serial.println(output);

	_previousError = error;

	//Go to the fish with the adjusted power values.
	go( _power, output );
}

/**
 * Gives power to motors, keeping the center of the block aligned with the requested center
 * set in the Drivetrain constructor.
 */
void Drivetrain::goToFish(Block block)
{
  //If the center of the block is on the left side of the camera
  if (block.x < _center)
  {
    turnLeft(_power);
  }
  //If the center of the block is on the right side of the camera
  else if (block.x > _center)
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
void Drivetrain::rotate(int stepNum)
{
  int time = _stepTimes[ stepNum - 1 ];
  
  switch (stepNum)
  {
    case 1: turnRight(_power); //At fish 1, turn to fish 2
	  delay(time);	  
      stopMotors();
      break;
	case 2: turnLeft(_power); //At fish 2, turn to fish 3
	  delay( time );
	  stopMotors();
	  break;
    case 3: turnLeft(_power); //At fish 3, turn to fish 4
	  delay( time );
	  stopMotors();
      break;

    //Turn to face the outer ring of fish
    case 4: turnRight(_power); //At fish 4, turn to fish 5
	  delay( time );
	  stopMotors();
      break;
    case 5: turnLeft(_power); //At fish 5, turn to fish 6
	  delay( time );
	  stopMotors();
      break;
	case 6: turnLeft(_power); //At fish 6, turn to fish 7
		delay( time );
		stopMotors();
	  break;
	case 7: turnLeft(_power); //At fish 7, turn to fish 8
		delay( time );
		stopMotors();
	  break;
	case 8: turnLeft(_power); //At fish 8, turn to fish 9
		delay( time );
		stopMotors();
	  break;
	case 9: turnLeft(_power); //At fish 9, turn to fish 10
		delay( time );
		stopMotors();
	  break;
	case 10: turnLeft(_power); //At fish 10, turn to fish 11
		delay( time );
		stopMotors();
	  break;
    case 11: turnLeft(_power); //At fish 11, turn to fish 12
		delay( time );
		stopMotors();
      break;

    //End of fish collection route
    case 12: turnRight(_power); //At fish 12, face bin 1
		delay( time );
		stopMotors();
      break;
    case 13: turnLeft(_power); //At bin 1, reposition for dumping
		delay( time );
		stopMotors();
      break;
	case 14: turnLeft(_power); //At bin 1, face bin 2
		delay( time );
		stopMotors();
	  break;
	case 15: turnLeft(_power); //At bin 2, reposition for dumping
		delay( time );
		stopMotors();
	  break;
	case 16: turnLeft(_power); //At bin 2, face bin 3
		delay( time );
		stopMotors();
	  break;
	case 17: turnLeft(_power); //At bin 3, reposition for dumping
		delay( time );
		stopMotors();
	  break;
	case 18: turnLeft(_power); //At bin 3, face bin 4
		delay( time );
		stopMotors();
	  break;
	case 19: turnLeft(_power); //At bin 4, reposition for dumping
		delay( time );
		stopMotors();
	  break;
  }
}

/**
 * Gives all motors 0 power.
 */
void Drivetrain::stopMotors()
{
  analogWrite(_rightMotorForward, 0);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, 0);
  analogWrite(_leftMotorBackward, 0);
}

/**
 * Gives the motors power based on an adjustment to try to make them go straight. The adjustment value is determined by PID controller.
 */
void Drivetrain::go( int power, int adjustment )
{
	//Before adjustment for PWM limits
	int right = power + adjustment;
	int left = power - adjustment;
	
	Serial.print("right before adj.: ");
	Serial.println(right);
	Serial.print("left before adj.: ");
	Serial.println(left);
	

	//After adjustment for PWM limits
	if((power + adjustment) < 0)
	{
		right = 0;
	}
	else if((power + adjustment) > 255)
	{
		right = 255;
	}
	if((power - adjustment) < 0)
	{
		left = 0;
	}
	else if((power - adjustment) > 255)
	{
		left = 255;
	}
	
	Serial.print( "right final: " );
	Serial.println(right);
	Serial.print( "left final: ");
	Serial.println( left );
	

	//Go with new adjustments
	analogWrite( _rightMotorForward, right);
	analogWrite( _rightMotorBackward, 0 );
	analogWrite( _leftMotorForward, left);
	analogWrite( _leftMotorBackward, 0 );
}

/**
 * Gives all forward motors power. Since our motors move at different rates given the same power, it will inevitably turn. 
 * Need encoders or a reference (like the pixy) to go in a straight line.
 */
void Drivetrain::goStraight(int power)
{
  analogWrite(_rightMotorForward, power);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, power);
  analogWrite(_leftMotorBackward, 0);
}

/**
 * Give the left forward motor power.
 */
void Drivetrain::turnRight(int power)
{
  analogWrite(_rightMotorForward, 0);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, power);
  analogWrite(_leftMotorBackward, 0);
}

/**
 * Give the right forward motor power.
 */
void Drivetrain::turnLeft(int power)
{
  analogWrite(_rightMotorForward, power);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, 0);
  analogWrite(_leftMotorBackward, 0);
}




