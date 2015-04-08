#include "Conveyor.h"

const byte conveyorMotorForwardPin = 2;
const byte conveyorMotorBackwardPin = 3;
const byte clawMotorPin = 4;
const byte limitSwitchPin = ;

Conveyor::Conveyor()
{
  motorSpeed = 160;
  
  clawServo.attach(clawMotorPin);
  closedAngle = 0;
  openAngle = 90;
  currentPosition = 0;
  pinMode(conveyorMotorForwardPin, OUTPUT);
  pinMode(conveyorMotorBackwardPin, OUTPUT);
  pinMode(clawMotorPin, OUTPUT);
  pinMode(limitSwitchPin, INPUT);
}

Conveyor::~Conveyor()
{
  
}

/**
 *  If claw position is fish, the motor should turn backward, otherwise it will turn forward until it hits the bin limit switch. Need to call this method twice to return the claw to
 *  fish position.
 */
void Conveyor::goToBin(int binPosition)
{
  if (currentPosition == FISH)
  {
    //Turn motor on backward
    if (currentPosition != binPosition
    {
    analogWrite(conveyorMotorBackwardPin, motorSpeed);
    }

    //delay to allow limit switch to go to LOW
    delay(150);

    while (currentPosition < binPosition)
    { 
        if ( digitalRead(limitSwitchPin) == HIGH )
        { 
          currentPosition ++;
          //delay to allow switch to return to LOW
          delay(150);
        }
    }

    //the claw should now be at the correct bin, so it should stop
    analogWrite(conveyorMotorBackwardPin, 0);
  }
  
  else //returns claw to fish position
  {
    //Turn motor on
    analogWrite(conveyorMotorForwardPin, motorSpeed);

    //delay to allow the limit switch to go to LOW
    delay(150);

    while (currentPosition > FISH)
    {
        if ( digitalRead(limitSwitchPin) == HIGH )
        {
          currentPosition--;
          //delay to allow switch to return to LOW
          delay(150);
        }
    }
    //the claw should now be back at the FISH position
      analogWrite(conveyorMotorForwardPin, 0);
  }
}

/**
 * Close the claw
 */
void Conveyor::closeClaw()
{
  clawServo.write(closedAngle);
}

/**
 * Opens the claw
 */
void Conveyor::openClaw()
{
  clawServo.write(closedAngle);
}
