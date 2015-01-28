#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>
#include <Servo.h>

/**
 * Numbers = pin numbers for limit switches
 */
enum ClawPosition : byte
{
  FISH = 22,
  RED_BIN = 23,
  YELLOW_BIN = 24,
  BLUE_BIN = 25,
  GREEN_BIN = 26
};

class Conveyor
{
  public:
    Conveyor(int motorSpeed, int closedAngle, int openAngle, byte conveyorMotorForwardPin, byte conveyorMotorBackwardPin, byte clawMotorPin);
    ~Conveyor();
    void goToPosition(ClawPosition clawPos);
    void openClaw();
    void closeClaw();
  private:
	byte _conveyorMotorForwardPin;
	byte _conveyorMotorBackwardPin;
	byte _clawMotorPin;
    int _motorSpeed;
    int _closedAngle;
    int _openAngle;
    Servo _clawServo;
};

#endif
    
