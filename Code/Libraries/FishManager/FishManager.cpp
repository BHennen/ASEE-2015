#include "FishManager.h"

Conveyor::Conveyor(int frontUpwardAngle, int frontDownwardAngle, int backUpwardAngle, int backDownwardAngle,
	byte downwardConveyorPower, byte upwardConveyorPower, byte downwardConveyorMotorPin, byte upwardConveyorMotorPin,
	byte frontClawServoPin, byte backClawServoPin, byte IRPin,
	unsigned long clawMovingTime, unsigned long clawRotatingTime, BinPosition restingPosition, BinPosition startingPosition)
{
	_downwardConveyorMotorPin = downwardConveyorMotorPin;
	_upwardConveyorMotorPin = upwardConveyorMotorPin;
	_frontClawServoPin = frontClawServoPin;
	_backClawServoPin = backClawServoPin;
	_IRPin = IRPin;

	//set Pins
	pinMode(_downwardConveyorMotorPin, OUTPUT);
	pinMode(_upwardConveyorMotorPin, OUTPUT);
	pinMode(_frontClawServoPin, OUTPUT);
	pinMode(_backClawServoPin, OUTPUT);
	pinMode(IRPin, INPUT);
	frontClawServo.attach(_frontClawServoPin);
	backClawServo.attach(_backClawServoPin);


	//Conveyor variables
	//motor
	_downwardMotorSpeed = downwardConveyorPower;
	_upwardMotorSpeed = upwardConveyorPower;
	//claw
	_frontUpwardAngle   = frontUpwardAngle  ;
	_frontDownwardAngle = frontDownwardAngle;
	_backUpwardAngle    = backUpwardAngle   ;
	_backDownwardAngle  = backDownwardAngle ;
	_clawMovingTime = clawMovingTime;
	_clawRotatingTime = clawRotatingTime;
	_restingPosition = restingPosition;
	clawIsClosed = true;
	clawIsMoving = false;

	//limit switch variables
	_debouncedKeyPress = false;
	_switchChanged = false;
	_switchPressed = false;
	lastUpdateTime = 0;
	previousTime = 0;
	debounceTime = 200;

	//initial variables
	_nextSig4Bin = 1;
	currentPosition = startingPosition;
	previousState = HIGH;
	correctPosition = false;
	wentUp = false;
	_hasFish = false;
}

Conveyor::~Conveyor()
{

}

//stops the motors
void Conveyor::stop()
{
	//Stop the conveyor
	analogWrite(_downwardConveyorMotorPin, 0);
	analogWrite(_upwardConveyorMotorPin, 0);
	//Open the claw
	frontClawServo.write(_frontUpwardAngle);
	backClawServo.write(_backDownwardAngle);
}

//Called when we go to the bin. Used to debounce the IR sensor
void Conveyor::updateSwitch(unsigned long currentTime)
{
	_switchChanged = false;
	_switchPressed = _debouncedKeyPress;

	if (currentTime - lastUpdateTime >= CHECK_MSEC)
	{
		boolean rawState = digitalRead(_IRPin);
		rawState = !rawState;
		lastUpdateTime = currentTime;
		static uint8_t count = RELEASE_MSEC / CHECK_MSEC;

		if (rawState == _debouncedKeyPress)
		{
			// Set the timer which allows a change from current state.
			if (_debouncedKeyPress) count = RELEASE_MSEC / CHECK_MSEC;
			else count = PRESS_MSEC / CHECK_MSEC;
		}
		else 
		{
			// Key has changed - wait for new state to become stable.
			if (--count == 0) {
				// Timer expired - accept the change.
				_debouncedKeyPress = rawState;
				_switchChanged = true;
				_switchPressed = _debouncedKeyPress;
				// And reset the timer.
				if (_debouncedKeyPress) count = RELEASE_MSEC / CHECK_MSEC;
				else count = PRESS_MSEC / CHECK_MSEC;
			}
		}
	}
}

/**
 *
 */
boolean Conveyor::goToBin(BinPosition binPosition, unsigned long currentTime)
{
	boolean correctPosition = false;
	static boolean previouslyWentUp = false;
	static boolean adjustPosition = false;

	//Any time we call this method, update the switch values
	updateSwitch(currentTime);

	//Check where the claw is.
	if (binPosition > currentPosition) //The desired position is closer to the back of the claw
	{		
		//We're going up
		//Adjust position if we were last going down
		if (adjustPosition && !previouslyWentUp) //Adjust position if we were last going down
		{
			currentPosition = (BinPosition)((int)(currentPosition)-1);
			adjustPosition = false;
		}
		previouslyWentUp = true;

		//Serial.println("going up!");
		if (_switchChanged && _switchPressed) //check if the switch has been pressed & changed(we hit a bump)
		{
			//increment the current position of the claw
			currentPosition = (BinPosition)((int)(currentPosition)+1);
		}
		analogWrite(_downwardConveyorMotorPin, 0);
		analogWrite(_upwardConveyorMotorPin, _upwardMotorSpeed);
	}
	else if (binPosition < currentPosition) //The desired position is closer to the front of the claw
	{
		//We're going down
		//Adjust position if we were last going up
		if (adjustPosition && previouslyWentUp)
		{
			currentPosition = (BinPosition)((int)(currentPosition)+1);
			adjustPosition = false;
		}
		previouslyWentUp = false;

		//Serial.println("going down!");
		if (_switchChanged && _switchPressed)  //check if the switch has been pressed & changed(we hit a bump)
		{
			//decrement the current position of the claw
			currentPosition = (BinPosition)((int)(currentPosition)-1);
		}
		//Tell the claw to go downward to the front of the conveyor
		analogWrite(_downwardConveyorMotorPin, _downwardMotorSpeed);
		analogWrite(_upwardConveyorMotorPin, 0);
	}
	else if (_switchPressed) //We're at desired position and the switch is pressed; keep going until it is not pressed
	{		
		//Serial.println("Adjusting to correct position");
		if (previouslyWentUp)
		{
			//keep going up
			analogWrite(_downwardConveyorMotorPin, 0);
			analogWrite(_upwardConveyorMotorPin, _upwardMotorSpeed);
		}
		else
		{
			//keep going down
			analogWrite(_downwardConveyorMotorPin, _downwardMotorSpeed);
			analogWrite(_upwardConveyorMotorPin, 0);
		}
		
	}
	else //The switch is now not pressed, and we are in the correct possition. Stop motors
	{
		//Serial.println("Made it to correct position!");
		analogWrite(_downwardConveyorMotorPin, 150);
		analogWrite(_upwardConveyorMotorPin, 150);
		adjustPosition = true;
		correctPosition = true;
	}
	return correctPosition;
}

//Returns whether or not the conveyor has a fish
boolean Conveyor::hasFish()
{
	return _hasFish;
}

/**
* From the resting position, the conveyor moves to the fish position with an open claw and picks up the fish.
* The conveyor then moves back to the resting position and returns true when it gets there.
* Returns false until it arrives back at the the resting position.
*/
boolean Conveyor::pickUpFish(unsigned long currentTime)
{
	static States _pickUpState = preparingToPickUp;

	switch (_pickUpState)
	{
	case preparingToPickUp: //make sure we're ready to pick up a fish
		if (openClaw(currentTime))
		{
			if (rotateClaw(currentTime, false)) //Make sure the claw is rotated downward
			{
				//The claw is now open and rotated downward to pick up another fish
				_pickUpState = goingToBin;
			}
		}
		break;
	case goingToBin: //go to the fish position
		if (goToBin(FISH, currentTime))
		{
			//We made it to fish position; set next state to move the claw so we can pick up the fish
			_pickUpState = movingClaw;
		}
		break;
	case movingClaw: //pick up the fish
		if (closeClaw(currentTime))
		{
			//claw is closed, now rotate it upwards
			if (rotateClaw(currentTime, true))
			{
				//the claw is now closed and rotated upwards, so go the the rest position
				_pickUpState = goingToRest;
			}
		}
		break;
	case goingToRest: //Go back to the resting position
		if (goToBin(_restingPosition, currentTime))
		{
			//the conveyor is now at the resting position and out of the way of the pixy and other sensors.
			_pickUpState = preparingToPickUp; //reset state
			_hasFish = true;
			return true; //The conveyor has the fish and is at the resting position. Return true
		}
		break;
	}
	return false; //The conveyor does not have the fish and is not at the resting position
}

/**
* From the resting position, the conveyor moves to the correct position based on the fish signature and drops fish in bin.
* The conveyor then moves back to the resting position and returns true when it gets there.
* Returns false until it arrives back at the the resting position.
*/
boolean Conveyor::storeFish(byte fishSignature, unsigned long currentTime)
{
	static States _dropOffState = goingToBin;
	static boolean determineCorrectBin = true;
	static BinPosition correctBin;

	//convert signatures to bin positions. (signatures are 1-7, since we have 4 colors of fish we only use 1-4)
	//signatures 1-3 get their own bin. signature 4 goes to bin 1, 2, or 3 depending if we put a signature 4 fish in that bin or not
	if (determineCorrectBin)
	{		
		switch (fishSignature)
		{
		case 1: correctBin = BIN1;
			break;
		case 2: correctBin = BIN2;
			break;
		case 3: correctBin = BIN3;
			break;
		case 4:
			switch (_nextSig4Bin)
			{
			case 1: correctBin = BIN1;
				break;
			case 2: correctBin = BIN2;
				break;
			case 3: correctBin = BIN3;
				break;
			}
			break;
		}
		determineCorrectBin = false;
	}

	switch (_dropOffState)
	{
	case goingToBin: //go to the correct bin based on the fish signature
		if (goToBin(correctBin, currentTime))
		{
			//We made it to the correct bin
			//if we picked up a signature 4 fish, change the next bin that signature 4 will go to by incrementing it
			if (fishSignature == 4)
			{
				(_nextSig4Bin == 3) ? _nextSig4Bin = 1 : _nextSig4Bin++; //spread the sig 4 fish out evenly among 3 bins
			}
			//set next state to move the claw so we can drop off
			_dropOffState = movingClaw;
		}
		break;
	case movingClaw: //drop off the fish
		if (openClaw(currentTime))
		{
			//the claw is now open and the fish is dropped off, so go the the rest position
			_dropOffState = goingToRest;
		}
		break;
	case goingToRest: //Go back to the resting position
		if (goToBin(_restingPosition, currentTime))
		{
			//the conveyor is now at the resting position and out of the way of the pixy and other sensors.
			_dropOffState = goingToBin; //reset state
			_hasFish = false;
			determineCorrectBin = true; //reset bin finding for next time
			return true; //The conveyor has dropped off the fish and is at the resting position. Return true
		}
		break;
	}
	return false; //the function returns false by default until the fish is dropped off and the claw is back at the resting position
}

/*
* Rotate the claw up or down, based on the position it was in previously
*/
boolean Conveyor::rotateClaw(unsigned long currentTime, boolean rotateUp)
{
	
	if (_clawRotatedUp == rotateUp) //The claw is already rotated to the desired position, return true
	{
		return true;
	}
	else if (!clawIsClosed) // the claw is open; no need to rotate it
	{
		_clawRotatedUp = rotateUp; //Set it so the claw is in the desired state
		return true;
	}
	else if (!clawIsMoving) //The claw is closed, is in the incorrect position, and it is not in the process of moving
	{
		//move the servos to the desired rotation
		if (rotateUp)
		{
			//Move the servos to the upward position 
			frontClawServo.write(_frontUpwardAngle); //tell the front claw to go to upward angle
			backClawServo.write(_backUpwardAngle); //tell the back claw to go to upward angle
		}
		else
		{
			//Move the servos to the downward position 
			frontClawServo.write(_frontDownwardAngle); //tell the front claw to go to downward angle
			backClawServo.write(_backDownwardAngle); //tell the back claw to go to downward angle
		}
		previousTime = currentTime; //set the timer to allow them time to rotate
		clawIsMoving = true; //the claw is now in the process of moving
		return false;
	}
	else if (currentTime - previousTime >= _clawRotatingTime) //The claw has been rotating for long enough so it should be fully rotated now
	{
		_clawRotatedUp = rotateUp;
		clawIsMoving = false;
		return true;
	}
	else //The claw is still rotating
	{
		return false;
	}
}

/**
 * Close the claw
 */
boolean Conveyor::closeClaw(unsigned long currentTime)
{
	if (clawIsClosed) //The claw is already closed
	{
		return true;
	}
	else if (!clawIsMoving) //The claw is open and it is not in the process of moving
	{
		if (_clawRotatedUp) //The claw should be in the up position
		{
			backClawServo.write(_backUpwardAngle); //tell the back claw to go to upward angle
		}
		else //The claw should be in the down position
		{
			frontClawServo.write(_frontDownwardAngle); //tell the front claw to go to downward angle
		}
		previousTime = currentTime; //set the timer
		clawIsMoving = !clawIsMoving; //the claw is now in the process of moving
		return false;
	}
	else if (currentTime - previousTime >= _clawMovingTime) //The claw has been closing for long enough so it should be fully closed now
	{
		clawIsClosed = true;
		clawIsMoving = false;
		return true;
	}
	else //The claw is still closing
	{
		return false;
	}
}

/**
 * Opens the claw
 */
boolean Conveyor::openClaw(unsigned long currentTime)
{
	if (!clawIsClosed) //The claw is already open
	{
		return true;
	}
	else if (!clawIsMoving) //The claw is closed and it is not in the process of moving
	{
		if (_clawRotatedUp) //The claw is in the up position
		{
			backClawServo.write(_backDownwardAngle); //tell the back claw to go to downward angle
		}
		else //The claw is in the down position
		{
			frontClawServo.write(_frontUpwardAngle); //tell the front claw to go to upward angle
		}
		previousTime = currentTime; //set the timer 
		clawIsMoving = !clawIsMoving; //the claw is now in the process of moving
		return false;
	}
	else if (currentTime - previousTime >= _clawMovingTime) //The claw has been opening for long enough so it should be fully open now
	{
		clawIsClosed = false;
		clawIsMoving = false;
		return true;
	}
	else //The claw is still opening
	{
		return false;
	}
}

Bins::Bins(byte binServoPin, byte binServoStop, byte binServoForward, unsigned long binDumpingTime)
{
	_binServoPin = binServoPin;
	pinMode(binServoPin, OUTPUT);
	binServo.attach(binServoPin);

	isDumping = false;
	_binServoStop = binServoStop;
	_binServoForward = binServoForward;
	_binDumpingTime = binDumpingTime;
	_numDumped = 0;
}

boolean Bins::dumpNextBin(unsigned long currentTime)
{	
	if (!isDumping && _numDumped < NUM_BINS) //If we haven't started turning the motors (this function was just called) and we have more bins to dump
	{		
		//start dumping 
		Serial.println("start dumping");
		binServo.write(_binServoForward);
		isDumping = true;
		previousTime = currentTime;
		return false;
	}
	else if ((currentTime - previousTime) >= _binDumpingTime) //We have turned the motors long enough
	{
		binServo.write(_binServoStop); //stop motor
		isDumping = false;
		_numDumped++;
		return true;
	}
	else //We are still dumping; continue
	{
		return false;
	}
}
