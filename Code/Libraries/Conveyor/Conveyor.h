#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>
#include <Servo.h>

enum BinPosition : byte
{
  FISH = 0,
  RED_BIN = 1,
  YELLOW_BIN = 2,
  BLUE_BIN = 3,
  GREEN_BIN = 4
};

class Conveyor
{
  public:
    Conveyor();
    ~Conveyor();
    void goToBin(int binPosition);
    void openClaw();
    void closeClaw();
  private:
    int motorSpeed;
    int closedAngle;
    int openAngle;
	bool previos_state;
	bool current_state;
    Servo clawServo;
    int currentPosition;
};

#endif
    