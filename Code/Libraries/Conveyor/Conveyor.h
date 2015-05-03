#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>
#include <Servo.h>

enum BinPosition : int
{
    FISH = 0,
    BIN1 = 1,
    BIN2 = 2,
    BIN3 = 3,
    BIN4 = 4,
};

class Conveyor
{
  public:
    Conveyor(int openAngle, int closedAngle, byte conveyorPowerForward, byte conveyorPowerBackward, byte conveyorMotorDownwardPin,
          byte conveyorMotorUpwardPin, byte clawMotorPin, byte limitSwitchPin, byte binMotorForwardPin, byte binMotorBackwardPin, byte binMotorSpeed,
          unsigned long binDumpingTime);
    ~Conveyor();
    boolean goToBin(BinPosition binPosition, boolean currentState, unsigned long currentTime);
    void openClaw();
    void closeClaw();
    bool dumpBins(unsigned long startTime);
    boolean correctPosition;
    //int count(int input);

  private:
    byte _conveyorMotorDownwardPin	  ;
    byte _conveyorMotorUpwardPin;
    byte _clawMotorPin				  ;
    byte _limitSwitchPin				  ;
    byte _binMotorForwardPin			  ;
    byte _binMotorBackwardPin		  ;

    int binMotorSpeed;
    int motorSpeedDownward;
    int motorSpeedUpward;
    int _closedAngle;
    int _openAngle;
    Servo clawServo;
    BinPosition currentPosition;
    boolean currentState;
    boolean previousState;
    unsigned long binDumpingTime;
    unsigned long previousTime;
    unsigned long debounceTime;

    boolean wentUp;
};

#endif
    