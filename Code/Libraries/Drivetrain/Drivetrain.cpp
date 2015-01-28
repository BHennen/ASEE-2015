#include "Arduino.h"
#include "Drivetrain.h"

/**
 * Constructor. Sets the pins for all the motors.
 * center: Where the robot aims when it detects a block. Valid values are 0 - 319.
 * deadZone: How big the "center" of the robot is. Smaller values will cause robot to wiggle more.
 * power: How much power for wheel motors. Valid values are 0 - 255.
 * *stepTimes: An array where each element is how much time in milliseconds should be spent at each step of rotation.
 */
Drivetrain::Drivetrain(const byte leftMotorForward, const byte leftMotorBackward,
                       const byte rightMotorForward, const byte rightMotorBackward,
                       int center, int deadZone, int power,
                       int *stepTimes)
{
  //Set the pinmode of the motor ports to be output.
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);

  _leftMotorForward = leftMotorForward;
  _leftMotorBackward = leftMotorBackward;
  _rightMotorForward = rightMotorForward;
  _rightMotorBackward = rightMotorBackward;
  _center = center;
  _deadZone = deadZone;
  _power = power;
  _stepTimes = stepTimes;
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
 * Gives power to motors, keeping the center of the block aligned with the requested center
 * set in the Drivetrain constructor.
 */
void Drivetrain::goToFish(Block block)
{
  //If the center of the block is on the left side of the camera
  if (block.x < (_center - _deadZone))
  {
	Serial.print("turning left\n");
    turnLeft(_power);
  }
  //If the center of the block is on the right side of the camera
  else if (block.x > (_center + _deadZone))
  {
	Serial.print("turning right\n");
    turnRight(_power);
  }
  //The center of the block is in the center of the camera
  else //Dead zone
  {
	Serial.print("going straight\n");
    goStraight(_power);
  }
}

/**
 * Rotates an amount based on what step we're on. StepNum > 0
 */
void Drivetrain::rotate(int stepNum)
{
  switch (stepNum)
  {
    case 1: turnRight(_power);
      delay(_stepTimes[0]);
      stopMotors();
      break;
    case 2: //Same as case 3
    case 3: turnLeft(_power);
      delay(_stepTimes[2]);
      stopMotors();
      break;
    //Turn to face the outer ring of fish
    case 4: turnRight(_power);
      delay(_stepTimes[3]);
      stopMotors();
      break;
    case 5: turnLeft(_power);
      delay(_stepTimes[4]);
      stopMotors();
      break;
    case 6: //Same as case 11
    case 7: //Same as case 11
    case 8: //Same as case 11
    case 9: //Same as case 11
    case 10: //Same as case 11
    case 11: turnLeft(_power);
      delay(_stepTimes[10]);
      stopMotors();
      break;
    //Turn right to face first bin
    case 12: turnRight(_power);
      delay(_stepTimes[11]);
      stopMotors();
      break;
    //TODO more steps than I first thought, will need to finish the rest;
    // there are two steps for each bin
    case 13: turnLeft(_power);
      delay(_stepTimes[12]);
      stopMotors();
      break;
    case 14: turnRight(_power);
      delay(_stepTimes[13]);
      stopMotors();
      break;
    case 15: turnRight(_power);
      delay(_stepTimes[14]);
      stopMotors();
      break;
    case 16: turnRight(_power);
      delay(_stepTimes[15]);
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




