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
		byte conveyorBeltPower, byte downwardConveyorBeltPin, byte upwardConveyorBeltPin,
		byte conveyorRotatorUpPower, byte conveyorRotatorDownPower, byte conveyorRotatorStopPower,
		byte downwardConveyorRotatorPin, byte upwardConveyorRotatorPin, byte rotatorLimitSwitchUpPin, byte rotatorLimitSwitchDownPin,
		byte frontClawServoPin, byte backClawServoPin, byte IRPin,
		unsigned long clawMovingTime, unsigned long clawRotatingTime, BinPosition restingPosition, BinPosition startingPosition);
    ~Conveyor();

	/**
	* Make sure everything is good to go before we start
	*/
	boolean setup(unsigned long currentTime);

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
	/**
	* Rotate the conveyor upward or downward and return true when it has done so.
	*/
	boolean rotateConveyor(boolean rotateDownwards, unsigned long currentTime);

	void update(unsigned long currentTime);

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
	boolean _conveyorIsDown;
	boolean _conveyorIsRotating;
	boolean _rotatingDown;
	boolean downLimitSwitchPressed();
	boolean upLimitSwitchPressed();
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

	enum Rotation
	{
		upwards,
		downwards
	};
    //pins
	byte _downwardConveyorBeltPin;
	byte _upwardConveyorBeltPin;
	byte _downwardConveyorRotatorPin;
	byte _upwardConveyorRotatorPin;
    byte _frontClawServoPin;
    byte _backClawServoPin;
    byte _IRPin;
	byte _rotatorLimitSwitchUpPin;
	byte _rotatorLimitSwitchDownPin;

    //motor speeds
    byte _conveyorBeltPower;
	byte _conveyorRotatorUpPower;
	byte _conveyorRotatorDownPower;
	byte _conveyorRotatorStopPower;

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
	/**
	* Make sure everything is good to go before we start
	*/
	boolean setup(unsigned long currentTime);

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
