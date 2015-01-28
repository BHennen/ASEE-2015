#include "Conveyor.h"

Conveyor::Conveyor(int motorSpeed, int closedAngle, int openAngle, byte conveyorMotorForwardPin, byte conveyorMotorBackwardPin, byte clawMotorPin)
{

  _conveyorMotorForwardPin = conveyorMotorForwardPin;
  _conveyorMotorBackwardPin = conveyorMotorBackwardPin;
  _clawMotorPin = clawMotorPin;

  _motorSpeed = motorSpeed;
  
  _clawServo.attach(_clawMotorPin);
  _closedAngle = closedAngle;
  _openAngle = openAngle;

  pinMode(FISH, INPUT);
  pinMode(RED_BIN, INPUT);
  pinMode(YELLOW_BIN, INPUT);
  pinMode(BLUE_BIN, INPUT);
  pinMode(GREEN_BIN, INPUT);

  pinMode(_conveyorMotorForwardPin, OUTPUT);
  pinMode(_conveyorMotorBackwardPin, OUTPUT);
  pinMode(_clawMotorPin, OUTPUT);
}

Conveyor::~Conveyor()
{
  
}

/**
 *  If claw position is fish, the motor should turn backward, otherwise it will turn forward until it hits the bin limit switch. Need to call this method twice to return the claw to
 *  fish position.
 */
void Conveyor::goToPosition(ClawPosition clawPos)
{
  if (clawPosition == FISH)
  {
    //Turn motor on backward
    analogWrite(_conveyorMotorBackwardPin, _motorSpeed);

    //If the limit switch of the requested bin is HIGH, stop motor
	if (digitalRead(clawPos) == HIGH)
    {
      analogWrite(_conveyorMotorBackwardPin, 0);
    }
  }
  else //Move claw to appropriate bin
  {
    //Turn motor on
    analogWrite(_conveyorMotorForwardPin, _motorSpeed);

    //If the limit switch of the requested bin is HIGH, stop motor
	if (digitalRead(clawPos) == HIGH)
    {
      analogWrite(_conveyorMotorForwardPin, 0);
    }
  }
}

/**
 * Close the claw
 */
void Conveyor::closeClaw()
{
  _clawServo.write(_closedAngle);
}

/**
 * Opens the claw
 */
void Conveyor::openClaw()
{
  _clawServo.write(_openAngle);
}
