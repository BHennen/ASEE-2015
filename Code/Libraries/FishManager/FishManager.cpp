#include "FishManager.h"

Conveyor::Conveyor(int frontUpwardAngle, int frontDownwardAngle, int backUpwardAngle, int backDownwardAngle,
	byte conveyorBeltPower, byte downwardConveyorBeltPin, byte upwardConveyorBeltPin,
	byte conveyorRotatorUpPower, byte conveyorRotatorDownPower, byte conveyorRotatorStopPower,
	byte downwardConveyorRotatorPin, byte upwardConveyorRotatorPin, byte rotatorLimitSwitchUpPin, byte rotatorLimitSwitchDownPin,
	byte frontClawServoPin, byte backClawServoPin, byte IRPin,
	unsigned long clawMovingTime, unsigned long clawRotatingTime, BinPosition restingPosition, BinPosition startingPosition)
{
	_downwardConveyorBeltPin = downwardConveyorBeltPin;
	_upwardConveyorBeltPin = upwardConveyorBeltPin;
	_downwardConveyorRotatorPin = downwardConveyorRotatorPin;
	_upwardConveyorRotatorPin = upwardConveyorRotatorPin;
	_frontClawServoPin = frontClawServoPin;
	_backClawServoPin = backClawServoPin;
	_IRPin = IRPin;
	_rotatorLimitSwitchUpPin = rotatorLimitSwitchUpPin;
	_rotatorLimitSwitchDownPin = rotatorLimitSwitchDownPin;
	//set Pins
	pinMode(_downwardConveyorBeltPin, OUTPUT);
	pinMode(_upwardConveyorBeltPin, OUTPUT);
	pinMode(_downwardConveyorRotatorPin, OUTPUT);
	pinMode(_upwardConveyorRotatorPin, OUTPUT);
	pinMode(_frontClawServoPin, OUTPUT);
	pinMode(_backClawServoPin, OUTPUT);
	pinMode(_IRPin, INPUT);
	pinMode(_rotatorLimitSwitchUpPin, INPUT);
	pinMode(_rotatorLimitSwitchDownPin, INPUT);

	frontClawServo.attach(_frontClawServoPin);
	backClawServo.attach(_backClawServoPin);

	//Conveyor variables
	_conveyorIsDown = true; //conveyor starts in downward position;
	_conveyorIsRotating = false;
	_rotatingDown = false;
	//motor
	_conveyorBeltPower = conveyorBeltPower;
	_conveyorRotatorUpPower = conveyorRotatorUpPower;
	_conveyorRotatorDownPower = conveyorRotatorDownPower;
	_conveyorRotatorStopPower = conveyorRotatorStopPower;
	//claw
	_frontUpwardAngle = frontUpwardAngle;
	_frontDownwardAngle = frontDownwardAngle;
	_backUpwardAngle = backUpwardAngle;
	_backDownwardAngle = backDownwardAngle;
	_clawMovingTime = clawMovingTime;
	_clawRotatingTime = clawRotatingTime;
	_restingPosition = restingPosition;
	clawIsClosed = false;
	clawIsMoving = false;
	_clawRotatedUp = false;

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

/**
* Make sure everything is good to go before we start
*/
boolean Conveyor::setup(unsigned long currentTime)
{
	boolean one = rotateConveyor(false, currentTime);
	boolean two = rotateClaw(currentTime,false);
	boolean three = closeClaw(currentTime);

	return one && two && three; //Make sure the claw and conveyor are in the correct position before we go

}

void Conveyor::update(unsigned long currentTime)
{
	updateSwitch(currentTime);

	//Check for a stalled upward motion
	static unsigned long previousUpdateTime = currentTime;
	static unsigned long stallTimer = 700;
	if (currentTime - previousUpdateTime >= 15)
	{
		previousUpdateTime = currentTime;
		static unsigned long previousTime = currentTime;
		if (_conveyorIsRotating) //If the conveyor is rotating upward
		{
			if (currentTime - previousTime > stallTimer) //If the motor is stalled out, increase its power
			{
				stallTimer = 30;
				previousTime = currentTime;
				if (_rotatingDown)
				{
					if (_conveyorRotatorDownPower > 50) _conveyorRotatorDownPower--;
				}
				else
				{
					if (_conveyorRotatorUpPower < 255) _conveyorRotatorUpPower++;
				}
			}
		}
		else
		{
			previousTime = currentTime; //reset timer whenever it is not rotating
			stallTimer = 700;
		}

		//Check if the hold power is not strong enough
		static boolean resetToUp = false;
		if (!_conveyorIsDown && !upLimitSwitchPressed() && !_conveyorIsRotating)
		{
			//If the conveyor is supposed to be stopped at the upper limit switch and it's not
			resetToUp = true;
		}
		if (resetToUp) //check if we want to reset the conveyor to the upper limit switch
		{
			if (_conveyorIsRotating && _rotatingDown) //A function was called where they wanted the conveyor to move down. Let them.
			{
				resetToUp = false;
				if (_conveyorRotatorStopPower < 255) _conveyorRotatorStopPower++; //Still increase the stop power for the future
			}
			else
			{
				if (rotateConveyor(false, currentTime)) //rotate the conveyor to the up position
				{
					if (_conveyorRotatorStopPower < 255) _conveyorRotatorStopPower++; //when it gets there, increase the stop power
					resetToUp = false;
				}
			}
		}
	}
}

//stops the motors
void Conveyor::stop()
{
	//Stop the conveyor
	analogWrite(_downwardConveyorBeltPin, 0);
	analogWrite(_upwardConveyorBeltPin, 0);
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
	//Serial.print(_switchPressed);
	//Serial.print("\t");
	//Serial.print(_switchChanged);
	//Serial.print("\t");
	//Serial.print("current pos: \t");
	//Serial.print(currentPosition);
	//Serial.print("going to: \t");
	//Serial.println(binPosition);
	boolean correctPosition = false;
	static boolean previouslyWentUp = false;
	static boolean adjustPosition = false;

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
		analogWrite(_downwardConveyorBeltPin, 0);
		analogWrite(_upwardConveyorBeltPin, _conveyorBeltPower);
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
		analogWrite(_downwardConveyorBeltPin, _conveyorBeltPower);
		analogWrite(_upwardConveyorBeltPin, 0);
	}
	else if (_switchPressed) //We're at desired position and the switch is pressed; keep going until it is not pressed
	{
		//Serial.println("Adjusting to correct position");
		if (previouslyWentUp)
		{
			//keep going up
			analogWrite(_downwardConveyorBeltPin, 0);
			analogWrite(_upwardConveyorBeltPin, _conveyorBeltPower);
		}
		else
		{
			//keep going down
			analogWrite(_downwardConveyorBeltPin, _conveyorBeltPower);
			analogWrite(_upwardConveyorBeltPin, 0);
		}

	}
	else //The switch is now not pressed, and we are in the correct position. Stop motors
	{
		//Serial.println("Made it to correct position!");
		analogWrite(_downwardConveyorBeltPin, 0);
		analogWrite(_upwardConveyorBeltPin, 0);
		adjustPosition = true;
		correctPosition = true;
	}
	return correctPosition;
}

boolean Conveyor::downLimitSwitchPressed()
{
	boolean downLimitSwitchRaw = digitalRead(_rotatorLimitSwitchDownPin);
	return !downLimitSwitchRaw;
}
boolean Conveyor::upLimitSwitchPressed()
{
	boolean upLimitSwitchRaw = digitalRead(_rotatorLimitSwitchUpPin);
	return !upLimitSwitchRaw;
}

/**
 * Rotate the conveyor upward or downward and return true when it has done so.
 */
boolean Conveyor::rotateConveyor(boolean rotateDownwards, unsigned long currentTime)
{
	if (rotateDownwards) //rotate downwards
	{
		//Check if we're at the bottom
		if (downLimitSwitchPressed()) //The conveyor hit the bottom limit switch; we're in correct position
		{
			//Brake the motor
			analogWrite(_downwardConveyorRotatorPin, 0);
			analogWrite(_upwardConveyorRotatorPin, _conveyorRotatorStopPower);


			_conveyorIsRotating = false;
			_conveyorIsDown = true;
			return true;
		}
		else //move the motor down
		{
			analogWrite(_downwardConveyorRotatorPin, 0);
			analogWrite(_upwardConveyorRotatorPin, _conveyorRotatorDownPower);
			_conveyorIsRotating = true;
			_rotatingDown = true;
		}
	}
	else //rotate upwards
	{
		//Check if we're at the top
		if (upLimitSwitchPressed()) //The conveyor hit the top limit switch; we're in correct position
		{
			//Brake the motor
			analogWrite(_downwardConveyorRotatorPin, 0);
			analogWrite(_upwardConveyorRotatorPin, _conveyorRotatorStopPower);

			_conveyorIsRotating = false;
			_conveyorIsDown = false;
			return true;
		}
		else //move the motor up
		{
			analogWrite(_downwardConveyorRotatorPin, 0);
			analogWrite(_upwardConveyorRotatorPin, _conveyorRotatorUpPower);
			_conveyorIsRotating = true;
			_rotatingDown = false;
		}
	}
	return false;
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
	case preparingToPickUp: //make sure the system is ready to pick up a fish
		if (openClaw(currentTime))
		{
			if (rotateClaw(currentTime, false)) //Make sure the claw is rotated downward
			{
				//The claw is now open and rotated downward to pick up another fish
				_pickUpState = goingToBin;
			}
		}
		break;
	case goingToBin: //move the open claw to the fish position
		if (goToBin(FISH, currentTime))
		{
			//We made it to fish position; set next state to move the claw so we can pick up the fish
			_pickUpState = movingClaw;
		}
		break;
	case movingClaw: //The claw is in the correct position
		rotateConveyor(true, currentTime); //rotate conveyor downward
				
		if (_conveyorIsDown && closeClaw(currentTime)) //make sure the conveyor is down and close the claw on the fish
		{
			//claw is closed, now rotate it upwards
			if (rotateClaw(currentTime, true))
			{
				//the claw is now closed,and rotated upwards, so go the the rest position
				_pickUpState = goingToRest;
			}
		}
		break;
	case goingToRest: //Go back to the resting position
		boolean conveyorReady = false;
		boolean clawReady = false;
		if (rotateConveyor(false, currentTime))//Rotate conveyor upward
		{
			conveyorReady = true;
		}
		if (goToBin(BIN1, currentTime)) //Move the claw back to bin1 (out of the way of the camera)
		{
			clawReady = true;
		}
		if (conveyorReady && clawReady) //the conveyor and claw is now at the resting position
		{
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
	{
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
	}
		break;
	case movingClaw: //drop off the fish
	{
		static unsigned long rotateTimer = currentTime;
		const unsigned long rotateTime = 200;
		if (rotateClaw(currentTime, false))
		{
			static boolean setTimer = true;
			if (setTimer)
			{
				rotateTimer = currentTime;
				setTimer = false;
		
			}
			if (currentTime - rotateTimer >= rotateTime)
			{
				if (openClaw(currentTime))
				{
					if (rotateClaw(currentTime, true))
					{
						//the claw is now open and the fish is dropped off, so go the the rest position
						_dropOffState = goingToRest;
						setTimer = true;
					}
				}
			}
		}
	}
		break;
	case goingToRest: //Go back to the resting position
	{
		closeClaw(currentTime);

		static boolean rotateClawDown = false;
		if (!_switchPressed && currentPosition <= 1)//Rotate claw & close it before getting to fish position
		{
			rotateClawDown = true;
		}
		if (rotateClawDown)
		{
			if (closeClaw(currentTime))	rotateClaw(currentTime, false);
		}

		if (goToBin(_restingPosition, currentTime))
		{
			//the conveyor is now at the resting position and out of the way of the pixy and other sensors.
			rotateClawDown = false;
			_dropOffState = goingToBin; //reset state
			_hasFish = false;
			determineCorrectBin = true; //reset bin finding for next time
			return true; //The conveyor has dropped off the fish and is at the resting position. Return true
		}
		break;
	}
	}
	return false; //the function returns false by default until the fish is dropped off and the claw is back at the resting position
}

/*
* Rotate the claw up or down, based on the position it was in previously
*/
boolean Conveyor::rotateClaw(unsigned long currentTime, boolean rotateUp)
{
	static unsigned long numPartialRotations;
	const unsigned long delayBetweenRotations = 5;
	static int count = 0;
	if (_clawRotatedUp == rotateUp) //The claw is already rotated to the desired position, return true
	{
		count = 0;
		return true;
	}
	else if (!clawIsClosed) // the claw is open; no need to rotate it
	{
		_clawRotatedUp = rotateUp; //Set it so the claw is in the desired state
		count = 0;
		return true;
	}
	else if (!clawIsMoving) //The claw is closed, is in the incorrect position, and it is not in the process of moving
	{
		numPartialRotations = _clawRotatingTime / delayBetweenRotations / 2; //set the number of partial rotations

		previousTime = currentTime; //set the timer to allow them time to rotate
		clawIsMoving = true; //the claw is now in the process of moving
		return false;
	}
	else //The claw is still rotating
	{
		//Move the servos to the upward position , one at a time
		if (currentTime - previousTime >= delayBetweenRotations)
		{
			static boolean rotateBack = rotateUp;
			previousTime = currentTime;

			//move the servos to the desired rotation
			if (rotateUp)
			{
				if (rotateBack)
				{
					count++;
					int deltaAngle = _backUpwardAngle - _backDownwardAngle;
					backClawServo.write(_backDownwardAngle + count*deltaAngle / numPartialRotations); //tell the back claw to go to upward angle
					rotateBack = false;
				}
				else
				{
					int deltaAngle = _frontUpwardAngle - _frontDownwardAngle;
					frontClawServo.write(_frontDownwardAngle + count*deltaAngle / numPartialRotations); //tell the front claw to go to upward angle
					rotateBack = true;
				}

			}
			else
			{
				//Move the servos to the downward position 
				if (!rotateBack)
				{
					//move the front claw first
					count++;
					int deltaAngle = _frontUpwardAngle - _frontDownwardAngle;
					frontClawServo.write(_frontUpwardAngle - count*deltaAngle / numPartialRotations); //tell the front claw to go to downward angle
					rotateBack = true;
				}
				else
				{
					int deltaAngle = _backUpwardAngle - _backDownwardAngle;
					backClawServo.write(_backUpwardAngle - count*deltaAngle / numPartialRotations); //tell the back claw to go to downward angle
					rotateBack = false;
				}
			}
		}
		if (count >= numPartialRotations)
		{
			count = 0;
			_clawRotatedUp = rotateUp;
			clawIsMoving = false;
			return true;

			Serial.println("high count");
		}
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
			frontClawServo.write(_frontUpwardAngle); //tell the front claw to go to upward angle
			backClawServo.write(_backUpwardAngle); //tell the back claw to go to upward angle
		}
		else //The claw should be in the down position
		{
			frontClawServo.write(_frontDownwardAngle); //tell the front claw to go to downward angle
			backClawServo.write(_backDownwardAngle); //tell the back claw to go to downward angle
		}
		previousTime = currentTime; //set the timer
		clawIsMoving = true; //the claw is now in the process of moving
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
		backClawServo.write(_backDownwardAngle); //tell the back claw to go to downward angle
		frontClawServo.write(_frontUpwardAngle); //tell the front claw to go to upward angle

		previousTime = currentTime; //set the timer 
		clawIsMoving = true; //the claw is now in the process of moving
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

/**
* Make sure everything is good to go before we start
*/
boolean Bins::setup(unsigned long currentTime)
{
	return true; //no setup required for bins
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
