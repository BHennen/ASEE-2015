#ifndef FISHMANAGER_H
#define FISHMANAGER_H

#include <Arduino.h>
#include <Servo.h>

enum BinPosition : int
{
    BEFORE_FISH = -1,
    FISH = 0,
    BIN1 = 1,
    BIN2 = 2,
    BIN3 = 3,
    BIN4 = 4,
};

class Conveyor
{
public:
	Conveyor(int frontUpwardAngle, int frontDownwardAngle, int backUpwardAngle, int backDownwardAngle,
		byte downwardConveyorPower, byte upwardConveyorPower, byte downwardConveyorMotorPin, byte upwardConveyorMotorPin,
		byte frontClawServoPin, byte backClawServoPin, byte IRPin,
		unsigned long clawMovingTime, unsigned long clawRotatingTime, BinPosition restingPosition, BinPosition startingPosition);
    ~Conveyor();
    boolean goToBin(BinPosition binPosition, unsigned long currentTime);
    /**
     * From the resting position, the conveyor moves to the fish position and picks up the fish.
     * The conveyor then moves back to the resting position and returns true when it gets there.
     * Returns false until it arrives back at the the resting position.
     */
    boolean pickUpFish(unsigned long currentTime);
    /**
     * From the resting position, the conveyor moves to the correct position based on the fish signature and drops fish in bin.
     * The conveyor then moves back to the resting position and returns true when it gets there.
     * Returns false until it arrives back at the the resting position.
     */
    boolean storeFish(byte fishSignature, unsigned long currentTime);
    //Returns whether or not the conveyor has a fish
    boolean hasFish();
    //claw
	/*
	* Rotate the claw up or down, based on the position it was in previously
	*/
	boolean rotateClaw(unsigned long currentTime, boolean rotateUp);
    boolean openClaw(unsigned long currentTime);
    boolean closeClaw(unsigned long currentTime);
    //limit Switch
    void updateSwitch(unsigned long currentTime);
    void stop(); //stops the motors
    boolean _switchChanged;
    boolean _switchPressed;
    BinPosition currentPosition;
private:
    //limit switch
    boolean _debouncedKeyPress; // This holds the debounced state of the key.

    unsigned long lastUpdateTime;
    const static unsigned long CHECK_MSEC = 1; // Read hardware every 5 msec
    const static unsigned long PRESS_MSEC = 2; // Stable time before registering pressed
    const static unsigned long RELEASE_MSEC = 10; // Stable time before registering released

    enum States
    {
        preparingToPickUp,
        goingToBin,
        movingClaw,
        goingToRest
    };

    //pins
	byte _downwardConveyorMotorPin;
	byte _upwardConveyorMotorPin;
    byte _frontClawServoPin;
    byte _backClawServoPin;
    byte _IRPin;

    //motor speeds
    int _downwardMotorSpeed;
    int _upwardMotorSpeed;

    //claw angles
    int _frontUpwardAngle  ;
    int _backUpwardAngle   ;
    int _frontDownwardAngle;
    int _backDownwardAngle ;

    //claw states
    Servo frontClawServo;
    Servo backClawServo;
    boolean clawIsClosed;
    boolean clawIsMoving;
	boolean _clawRotatedUp;
	unsigned long _clawRotatingTime;
    unsigned long _clawMovingTime;

    //conveyor states
    boolean correctPosition;
    BinPosition _restingPosition;
    boolean previousState;
    unsigned long previousTime;
    unsigned long debounceTime;
    boolean wentUp;
    boolean _hasFish; //True if the conveyor has a fish that needs to be dropped off
    byte _nextSig4Bin;
};


class Bins
{
public:
    Bins(byte binServoPin, byte binServoStop, byte binServoForward, unsigned long binDumpingTime);
    ~Bins();
    boolean dumpNextBin(unsigned long currentTime);
    const static byte NUM_BINS = 3;
    int _numDumped;
private:
    byte _binServoPin;

    byte _binServoStop;
    byte _binServoForward;

    Servo binServo;

    unsigned long _binDumpingTime;
    unsigned long previousTime;
    boolean isDumping;
};
#endif
