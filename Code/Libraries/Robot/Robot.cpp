#include "Robot.h"

/**
* Construct a new asee robot!
* testParam: A parameter for testing various modules of the robot. Enters a different mode based on the value of this parameter and
* which modules are not null pointers.
*/
ASEE2015::ASEE2015(int testParam, Drivetrain* driveTrain, VisualSensor* visualSensor, Gyro* gyro, Conveyor* conveyor, Bins* bins)
{
	_wheels = driveTrain;
	_eyes = visualSensor; 
	_gyro = gyro;
	_conveyor = conveyor;
	_bins = bins;
	mode = testParam;

	_fishSignature = 1;
}

ASEE2015::~ASEE2015()
{

}

void ASEE2015::allStop()
{
	if (_conveyor)
	{
		_conveyor->stop();
	}
	if (_wheels)
	{
		_wheels->stopMotors();
	}
}

/**
* Tell the robot to execute a function based on the value of testParam.
* Returns true when it has finished the test. And then it stops.
*/
boolean ASEE2015::go()
{
	static boolean completed = false;
	static boolean printed = false;
	unsigned long currentTime = millis();

	if (!completed)
	{
		switch (mode)
		{
		case 1:
			if (!printed)
			{
				Serial.println("Final test of the robot! This is the final product. Goes around the track and");
				Serial.println("picks up all the fish, storing them inside. Dumps them all off too.");
				printed = true;
			}

			break;
		case 2:
			if (!printed)
			{
				Serial.println("Test PID. Goes to closest fish seen and stops in front of it. ");
				printed = true;
			}
			_gyro->update(currentTime);
			_eyes->isClose(currentTime);//Update IR sensor for fish
			goToFishAndStop(currentTime);

			break;
		case 3:
			if (!printed)
			{
				Serial.println("Test PID and Conveyor.Goes to closest fish seen, picks it up, and stores it");
				Serial.println("based on its color.");
				printed = true;
			}

			break;
		case 4:
			if (!printed)
			{
				Serial.println("Test Conveyor. Picks up a fish and stores it. Has a pattern:");
				Serial.println("1,2,3,1, 1,2,3,2, 1,2,3,3 ...");
				printed = true;
			}

			testConveyor(currentTime, false);

			break;
		case 5:
			if (!printed)
			{
				Serial.println("Test fishCollection Route. Goes to closest fish seen, picks it up, and stores it");
				Serial.println("based on its color. Then rotates to next fish and repeats for all the fish.");
				printed = true;
			}

			_gyro->update(currentTime); //update gyro
			if (collectFish(currentTime)) //collect fish
			{
				Serial.println("Collected all fish!");
				completed = true;
			}
			break;
		case 6:
			if (!printed)
			{
				Serial.println("Test Conveyor.Picks up a fish, determines color using pixy, and stores it.");
				printed = true;
			}

			testConveyor(currentTime, true);

			break;
		case 7:
			if (!printed)
			{
				Serial.println("Test PID. Goes to closest fish seen and stops in front of it, rotates to next one.");
				printed = true;
			}

			_gyro->update(currentTime);
			if (testPIDRotate(currentTime))
			{
				completed = true;
			}

			break;
		}
	}
	return completed;
}

/**
 * Uses the PID, pixy, and IR sensor to go to the closest fish and stop in front of it. Returns true when 
 * the IR sensor is close to something. If the pixy doesnt see a fish, it will stop and return false.
 */
boolean ASEE2015::goToFishAndStop(unsigned long currentTime)
{
	enum goingStates
	{
		usingGyro,
		usingPixy,
		repositioning,
	};
	static goingStates goingState = usingGyro;
	static boolean lockedOn = false;

	//Check if we're close to a fish
	if (_eyes->_isClose)
	{
		lockedOn = false;
		goingState = usingGyro;
		_wheels->resetIntegral();
		_wheels->stopMotors();
		return true;
	}
	else //We're not close
	{
		boolean isGoodBlock = false;
		Block targetBlock;

		//Check what state we should be in
		if (!lockedOn)
		{
			//Get a block
			targetBlock = _eyes->getBlock(currentTime);
			//If the block is a valid block
			if (_eyes->isGoodBlock(targetBlock))
			{
				isGoodBlock = true;
				
				//We see a good block BUT the good block is also really close to the robot
				if (targetBlock.y > _eyes->_maximumBlockY)
				{
					goingState = repositioning; //try to reposition by turning to it
				}
				else
				{
					goingState = usingPixy; //Go to it using the pixy and pid
				}
			}
		}

		//Depending on what state we're in, use a different sensor and technique to go to the fish
		switch (goingState)
		{
		case usingGyro:
			//Serial.print("Using Gyro. Locked on = "); Serial.println(lockedOn);
			_wheels->driveToNextPosition(currentTime);
			break;
		case usingPixy:
			//Serial.print("Using Pixy");
			//Serial.print("\t");
			//targetBlock.print();
			static unsigned long previousTime = currentTime;

			//Check if the block is a good one a. If so, go to it.
			if (isGoodBlock)
			{
				previousTime = currentTime; //Reset stall timer
				if (targetBlock.y > _eyes->_maximumBlockY)
				{
					//Check if we are centered with the fish. 
					if (abs(_eyes->_center - (int)targetBlock.x) < _eyes->_errorDeadzone)
					{
						static int centerCount = 0;
						static float avgDeg = 0.0;
						const float newValWeight = 0.1;
						avgDeg = (centerCount != 0) ? (1 - newValWeight) * avgDeg + newValWeight * _gyro->getDegrees() : _gyro->getDegrees();
						centerCount++;
						if (centerCount >= 10)
						{
							//We are locked on to a fish, go using the gyro now
							_wheels->drivingDegrees = avgDeg;
							_wheels->resetIntegral();
							centerCount = 0;
							avgDeg = 0.0;
							lockedOn = true;
							goingState = usingGyro;
						}
					}					
				}
				else
				{
					_wheels->goUsingPID((float)targetBlock.x, _eyes->_center, _eyes->_PIDconsts, currentTime);
				}

			}
			else if (currentTime - previousTime > _eyes->_pixyStallTime)
			{
				//The pixy hasn't seen the good block in a while. Use the gyro now
				goingState = usingGyro;
			}
			break;
		case repositioning:
			//Serial.println("Repositioning");

			//Rotate until we are aligned with the fish perfectly. Only used if we are close to the fish and unaligned
			int error = _eyes->_center - (int)targetBlock.x;
			
			//Turn to face the fish
			if (error < -_eyes->_errorDeadzone)//The error is negative
			{
				_wheels->turnRightStationary(_wheels->_power);
			}
			else if (error > _eyes->_errorDeadzone) //The error is positive
			{
				_wheels->turnLeftStationary(_wheels->_power);
			}
			else //There is no error; go forward now using the gyro
			{
				//We are locked on to a fish, go using the gyro now
				_wheels->drivingDegrees = _gyro->getDegrees();
				lockedOn = true;
				_wheels->resetIntegral();
				goingState = usingGyro;
			}
			break;
		}
		return false;
	}
}

/**
 * Travels around the track collecting all the fish.
 * Returns true when it has done so.
 */
boolean ASEE2015::collectFish(unsigned long currentTime)
{
	enum collectFishStates //various states for collecting fish
	{
		drivingToNextFish,
		pickingUpFish,
		rotatingToNextFish,
	};
	static collectFishStates collectFishState = drivingToNextFish;
	static boolean fishSigRecorded = false;
	static byte numFishCollected = 0;
	static boolean printed = false;

	//if the conveyor has a fish that is unstored, store it no matter what state we're in
	if (_conveyor->hasFish())
	{
		_conveyor->storeFish(_fishSignature, currentTime);
	}
	
	_eyes->isClose(currentTime);//Update IR sensor for fish

	//Check what state we're in
	switch (collectFishState)
	{
	case drivingToNextFish: //We're driving to the next fish
		if (!printed)
		{
			printed = true;
			Serial.println("going to fish");
		}

		if (goToFishAndStop(currentTime)) //go to the next fish and stop
		{
			//We've made it to the fish. Now go to the pick up fish state
			collectFishState = pickingUpFish;
			printed = false;
		}
		break;
	case pickingUpFish: //We stopped in front of a fish so we need to pick it up
		if (!printed)
		{
			printed = true;
			Serial.println("picking up fish");
		}

		//if the conveyor does not have a fish
		if (!_conveyor->hasFish())
		{
			if (!fishSigRecorded) //We've stored the previous fish and are ready to accept the new one. Record its signature
			{
				_eyes->getFishSignature(true);				
				_fishSignature = _eyes->_signature;
				fishSigRecorded = true;				
			}			
			if (_conveyor->pickUpFish(currentTime))
			{
				//We've picked up the fish and are ready to dump the fish and rotate and drive to next one			
				fishSigRecorded = false; //Reset fish signature state for next time
				collectFishState = rotatingToNextFish; // now rotate to next fish
				printed = false;
			}			
		}
		break;
	case rotatingToNextFish: //We have now picked up a fish. Rotate to the next position

		if (!printed)
		{
			printed = true;
			Serial.println("rotating to next fish");
		}

		// rotate to next position
		if (_wheels->rotateToNextPosition())
		{
			//We've rotated to the correct position
			collectFishState = drivingToNextFish; //Set state for next time
			numFishCollected++;
			printed = false;
		}
		break;
	}
	if (numFishCollected >= 12)
	{
		return true; //We've collected all twelve fish. Return true
	}
	else
	{
		return false; //We haven't picked up all the fish yet
	}
}

/**
* Tests the conveyor code infinitely. Has the option of using the pixy to determine what bin to store the fish in,
* otherwise it goes in a set pattern.
*/
void ASEE2015::testConveyor(unsigned long currentTime, boolean usePixy)
{	
	static unsigned long previousTime = currentTime;
	unsigned long getFishSigTime = 2000UL;
	static boolean determiningFishSig = true;
	
	if (usePixy)
	{
		if (currentTime - previousTime >= getFishSigTime)
		{
			determiningFishSig = false;
		}
		if (determiningFishSig)
		{
			_eyes->getBlock(currentTime);
		}
	}

	if (!usePixy || !determiningFishSig)
	{
		//Drop a fish off if we have one
		if (_conveyor->hasFish())
		{
			if(_conveyor->storeFish(_fishSignature, currentTime)) //Check when we've stored the fish.
			{
				//If not using pixy, increment fish signature or reset it to 1 if it is 4
				if (!usePixy)
				{
					(_fishSignature == 4) ? _fishSignature = 1 : _fishSignature++;
				}
				else //set fish sig based on what the pixy saw
				{
					determiningFishSig = true;
					previousTime = currentTime;
				}
			}
		}
		else //conveyor doesn't have a fish. Pick it up
		{			
			if (_conveyor->pickUpFish(currentTime) && usePixy)
			{
				_eyes->getFishSignature(true);
				_fishSignature = _eyes->_signature; //get the fish sig and reset counts
			}
		}
	}
}

/**
* Travels around the track stopping in front of all the fish.
* Returns true when it has done so.
*/
boolean ASEE2015::testPIDRotate(unsigned long currentTime)
{
	enum collectFishStates //various states for collecting fish
	{
		drivingToNextFish,
		delaying,
		rotatingToNextFish,
	};

	static collectFishStates collectFishState = drivingToNextFish;
	static byte numFishCollected = 0;
	static boolean printed = false;
	static unsigned long previousTime = currentTime;
	unsigned long delayTime = 2000UL;

	_eyes->isClose(currentTime);//Update IR sensor for fish

	//Check what state we're in
	switch (collectFishState)
	{
	case drivingToNextFish: //We're driving to the next fish
		if (!printed)
		{
			printed = true;
			Serial.println("going to fish");
		}

		if (goToFishAndStop(currentTime)) //go to the next fish and stop
		{
			//We've made it to the fish. Now go to the pick up fish state
			collectFishState = delaying;
			printed = false;
			previousTime = currentTime;
		}
		break;
	case delaying:	
		if (!printed)
		{
			printed = true;
			Serial.println("delaying");
		}
		if (currentTime - previousTime >= delayTime)
		{
			collectFishState = rotatingToNextFish;
			printed = false;
		}
		break;
	case rotatingToNextFish: //We have now picked up a fish. Rotate to the next position
		//Serial.println("im here");
		if (!printed)
		{
			printed = true;
			Serial.println("rotating to next fish");
		}

		// rotate to next position
		if (_wheels->rotateToNextPosition())
		{
			//We've rotated to the correct position
			collectFishState = drivingToNextFish; //Set state for next time
			numFishCollected++;
			printed = false;
		}
		break;
	}
	if (numFishCollected >= 12)
	{
		return true; //We've collected all twelve fish. Return true
	}
	else
	{
		return false; //We haven't picked up all the fish yet
	}
}

/**
 * Goes to the each of the bins, repositions, then dumps the fish.
 * Returns true when it has dumped all the bins.
 */
boolean ASEE2015::dumpFish(unsigned long currentTime)
{
	enum DumpFishStates
	{
		goingToBin,
		rotating1,
		repositioning,
		dumping,
		rotating2
	};
	static DumpFishStates dumpFishState = goingToBin;
	static boolean readyToDump = false;

	_eyes->isClose(currentTime);//Update IR sensor for fish

	//if the conveyor has a fish that is unstored, store it no matter what state we're in
	if (_conveyor->hasFish())
	{
		_conveyor->storeFish(_fishSignature, currentTime);
	}
	else if (!readyToDump && _conveyor->goToBin(FISH, currentTime)) //make sure the conveyor is in the fish position
	{
		readyToDump = true; //conveyor does not have a fish and is in the fish position, we're ready to dump
	}

	switch (dumpFishState)
	{
	case goingToBin:
		_eyes->_stopVoltage = 2.4; // set new stopping point
		if (goToFishAndStop(currentTime)) //drive to bin and stop in front of it
		{
			dumpFishState = rotating1;
		}
		break;
	case rotating1:
		if(_wheels->rotateToNextPosition()) //rotate to align with where we want to be
		{
			dumpFishState = repositioning;
		}
		break;
	case repositioning:
		//perform some repositioning before we dump the bin
		dumpFishState = dumping;
		break;
	case dumping:
		if (readyToDump) //check if we're ready to dump
		{
			if (_bins->dumpNextBin(currentTime)) //dump the next bin
			{
				if (_bins->_numDumped == _bins->NUM_BINS) //Check if we dumped all the bins
				{
					return true;
				}
				dumpFishState = rotating2;
			}
		}
		break;
	case rotating2:
		if (_wheels->rotateToNextPosition()) //rotate to the next bin position
		{
			dumpFishState = goingToBin;
		}
		break;
	}
	return false;
}