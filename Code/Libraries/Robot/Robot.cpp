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
	_readyToGo = false;
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
 * Make sure all the modules are ready to go.
 */
boolean ASEE2015::setup(unsigned long currentTime)
{
	boolean eyesReady = false;
	boolean wheelsReady = false;
	boolean conveyorReady = false;
	boolean gyroReady = false;
	boolean binsReady = false;

	if (_eyes && _conveyor)
	{
		if (_conveyor->closeClaw(currentTime) && _eyes->setup(currentTime))
		{
			eyesReady = true;
			if (_conveyor->setup(currentTime))//Set up pixy before conveyor
			{
				conveyorReady = true;
			}
		}
	}
	else if ((!_eyes || _eyes->setup(currentTime)) && (!_conveyor || _conveyor->setup(currentTime)))
	{
		eyesReady = true;
		conveyorReady = true;
	}
	if (!_wheels || _wheels->setup(currentTime))
	{
		wheelsReady = true;
	}
	if (!_gyro || _gyro->setup(currentTime))
	{
		gyroReady = true;
	}
	if (!_bins || _bins->setup(currentTime))
	{
		binsReady = true;
	}
	if (eyesReady && wheelsReady && conveyorReady && gyroReady && binsReady)
	{
		_readyToGo = true;
		return true;
	}
	else return false;
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
		if (_eyes) _eyes->update(currentTime);
		if (_gyro) _gyro->update(currentTime);

		if (!_readyToGo)
		{
			setup(currentTime);
		}
		else
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

				if (goFinalRun(currentTime))
				{
					completed = true;
				}

				break;
			case 2:
				if (!printed)
				{
					Serial.println("Test PID. Goes to closest fish seen and stops in front of it. ");
					printed = true;
				}
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

				if (testPIDRotate(currentTime))
				{
					completed = true;
				}

				break;
			case 8:
				if (!printed)
				{
					Serial.println("Test Conveyor rotator. Rotates the conveyor up for 5 sec and down for 5 sec. ");
					printed = true;
				}

				testConveyorRotate(currentTime);

				break;
			case 9:
				if (!printed)
				{
					Serial.println("Test bin dumping.");
					printed = true;
				}

				if (testBinDumping(currentTime))
				{
					completed = true;
				}

				break;
			case 10:
				if (!printed)
				{
					Serial.println("Test bin dumping going around the track");
					printed = true;
				}

				if (dumpFish(currentTime))
				{
					completed = true;
				}
				break;
			}
		}
	}
	return completed;
}

boolean ASEE2015::goFinalRun(unsigned long currentTime)
{
	if (collectFish(currentTime))
	{
		if (dumpFish(currentTime))
		{
			return true;
		}
	}
	return false;
}

boolean ASEE2015::goToBinAndStop(unsigned long currentTime)
{
	//Check if we're close to a bin
	if (_eyes->_IRaverage >= _eyes->_stopVoltage)
	{
		_wheels->resetIntegral();
		_wheels->stopMotors();
		return true;
	}
	else //We're not close
	{
		_wheels->driveToNextPosition(currentTime);//use the gyro to go to next bin
		return false;
	}
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
		makingFishFlat,
	};
	static goingStates goingState = makingFishFlat;
	static boolean lockedOn = false;
	boolean isGoodBlock = false;
	static double repositionAngle;
	Block targetBlock;
	static boolean peaked = false;
	static byte	tempPower = _wheels->_power;


	//Check what state we should be in
	//Check if the IR sensor peaked
	if (goingState != makingFishFlat)
	{
		if (!peaked && _eyes->readProximity() > _eyes->_peakVoltage)
		{
			//if (goingState != makingFishFlat) _wheels->stopMotors();
			peaked = true;
			goingState = makingFishFlat;
			lockedOn = false;
		}
		else //We're not close
		{
			if (!lockedOn && goingState != makingFishFlat)
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
						if (_eyes->getFishSignature(false) >= 5 && targetBlock.signature == _eyes->_signature)
						{
							if (goingState != repositioning)
							{
								int blockError = _eyes->_center - (int)targetBlock.x;
								repositionAngle = _gyro->getDegrees() - (blockError*(75.0f / 320.0f));
								if (repositionAngle > 360) repositionAngle -= 360;
								if (repositionAngle < 0) repositionAngle += 360;
								_wheels->resetIntegral();
								goingState = repositioning; //try to reposition by turning to it
							}
						}
					}
					else
					{
						if (goingState != usingPixy && goingState != repositioning)
						{
							_wheels->resetIntegral();
							goingState = usingPixy; //Go to it using the pixy and pid
						}
					}
				}
			}
		}
	}
	//Depending on what state we're in, use a different sensor and technique to go to the fish
	switch (goingState)
	{
	case usingGyro:
	{
		//Serial.print("Using Gyro. Locked on = "); Serial.println(lockedOn);
		_wheels->driveToNextPosition(currentTime);
	}
	break;
	case usingPixy:
	{
		//Serial.println("Using Pixy");
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
					if (centerCount >= 5)
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
				_wheels->goUsingPID((float)targetBlock.x, _eyes->_center, _eyes->_PIDconsts, currentTime, false, true);
			}
		}
		else if (currentTime - previousTime > _eyes->_pixyStallTime)
		{
			//The pixy hasn't seen the good block in a while. Use the gyro now
			goingState = usingGyro;
		}
	}
	break;
	case repositioning:
	{
		//Serial.println("Repositioning");
		static boolean resetting = false;
		float rotationError = repositionAngle - _gyro->getDegrees();
		if (rotationError > 180) rotationError -= 360;
		if (rotationError < -180) rotationError += 360;

		if (abs(rotationError) > 15) //Check if we've rotated past the fish
		{
			resetting = true;
		}

		if (resetting) //Reset back to start position if we've rotated too far from where we started
		{
			if (abs(rotationError) < 1)//Check if we've made it to an acceptable spot
			{
				_wheels->resetIntegral();
				resetting = false;
			}
			else
			{
				_wheels->turnStationary(_wheels->_power, rotationError < 0);
			}
		}
		//Rotate until we are aligned with the fish perfectly. Only used if we are close to the fish and unaligned
		else if (isGoodBlock)//make sure the block is good
		{
			if (_eyes->getFishSignature(false) >= 5 && targetBlock.signature == _eyes->_signature)
			{
				int error = (int)targetBlock.x - _eyes->_center;

				if (abs(error) > _eyes->_errorDeadzone)//Turn to face the fish because the error is too large
				{
					_wheels->goUsingPID(error, 0, _eyes->_pixyRotatePIDconsts, currentTime, true, true);
				}
				else //There is no error; go forward now using the gyro
				{
					//We are locked on to a fish, go using the gyro now
					_wheels->drivingDegrees = _gyro->getDegrees();
					lockedOn = true;
					_wheels->resetIntegral();
					goingState = usingGyro;
				}
			}
		}
	}
	break;
	case makingFishFlat: //make the fish flat somehow
	{
		static boolean setPower = true;
		static boolean reverse = false;

		if (setPower)
		{
			_wheels->_power *= 0.6;
			setPower = false;
		}
		if (!peaked)
		{
			_wheels->driveToNextPosition(currentTime, !reverse);
			if (_eyes->_IRaverage > _eyes->_peakVoltage) peaked = true;
		}
		else //we're past the peak; make the fish flat & back up
		{
			static boolean isFlat = false;
			//Make flat with robot
			if (!isFlat)
			{				
				static unsigned long driveTimer = currentTime;
				const unsigned long driveTime = 300;
				static boolean isConstant = false;
				_wheels->driveToNextPosition(currentTime, true);
				if (!isConstant)
				{
					if (_eyes->_IRisConstant) isConstant = true;
					driveTimer = currentTime;
				}
				else//once we're constant, go forward some more
				{
					if (currentTime - driveTimer <= driveTime)
					{
						_wheels->driveToNextPosition(currentTime, true);
					}
					else
					{
						isFlat = true; 
						isConstant = false;
						peaked = false;
						reverse = true;
					}
				}
			}
			else //fish is flat; back up
			{		
				//Serial.println("flat");
				int error = _eyes->_closeVoltage - _eyes->_IRaverage;
				//Serial.print(_eyes->_IRaverage);
				//Serial.print("\t");
				//Serial.println(error);
				if (abs(error) < _eyes->_errorVoltage)
				{
					_wheels->_power = tempPower;
					setPower = true;
					reverse = false;
					peaked = false;
					isFlat = false;
					goingState = usingGyro;
					_wheels->resetIntegral();
					_wheels->stopMotors();
					_eyes->getFishSignature(true);//Set the fish signature 
					return true;
				}
				else //The robot is aligned with the fish and is far enough away
				{
					_wheels->driveToNextPosition(currentTime, error > 0);					
				}
			}
		}
	}
	break;
	}
	return false;
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
	if (collectFishState != rotatingToNextFish)
	{
		_conveyor->update(currentTime);
		if (_conveyor->hasFish())
		{
			_conveyor->storeFish(_fishSignature, currentTime);
		}
	}
	else
	{
		_conveyor->stop();
	}	

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
			//Serial.println("rotating to next fish");
		}

		// rotate to next position
		if (_wheels->rotateToNextPosition(currentTime))
		{
			//We've rotated to the correct position
			collectFishState = drivingToNextFish; //Set state for next time
			numFishCollected++;
			printed = false;
		}
		break;
	}
	if (numFishCollected >= 5)
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
	_conveyor->updateSwitch(currentTime);
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
			if (_conveyor->storeFish(_fishSignature, currentTime)) //Check when we've stored the fish.
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

	if (collectFishState != rotatingToNextFish)
	{
		_conveyor->update(currentTime);
	}

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
		if (!printed)
		{
			printed = true;
			Serial.println("rotating to next fish");
		}

		// rotate to next position
		if (_wheels->rotateToNextPosition(currentTime))
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
	_conveyor->update(currentTime);

	//if the conveyor has a fish that is unstored, store it no matter what state we're in
	if (_conveyor->hasFish())
	{
		_conveyor->storeFish(_fishSignature, currentTime);
	}
	else if (!readyToDump) //make sure the conveyor is in the fish position
	{
		boolean one = _conveyor->goToBin(FISH, currentTime);
		boolean two = _conveyor->rotateConveyor(false, currentTime);
		boolean three = _conveyor->rotateClaw(currentTime, false);
		boolean four = _conveyor->closeClaw(currentTime);
		if (one && two && three && four) readyToDump = true; //conveyor does not have a fish and is in the fish position, we're ready to dump
	}

	switch (dumpFishState)
	{
	case goingToBin:
		Serial.println("going to bin");
		if (goToBinAndStop(currentTime)) //drive to bin and stop in front of it
		{
			dumpFishState = rotating1;
		}
		break;
	case rotating1:
		Serial.println("repositioning");
		if (_wheels->rotateToNextPosition(currentTime)) //rotate to align with where we want to be
		{
			dumpFishState = dumping;
		}
		break;
	case repositioning:
		//perform some repositioning before we dump the bin
		dumpFishState = dumping;
		break;
	case dumping:
		Serial.println("dumping");
		if (readyToDump) //check if we're ready to dump
		{
			if (_bins->dumpNextBin(currentTime)) //dump the next bin
			{
				Serial.println("bin1 dumped");
				if (_bins->dumpNextBin(currentTime)) //dump the next bin
				{
					Serial.println("bin2 dumped");

					if (_bins->dumpNextBin(currentTime)) //dump the next bin
					{
						Serial.println("bin3 dumped");

						dumpFishState = rotating2;
						return true;
					}
				}
			}
		}

		break;
	case rotating2:
		if (_wheels->rotateToNextPosition(currentTime)) //rotate to the next bin position
		{
			dumpFishState = goingToBin;
		}
		break;
	}
	return false;
}

void ASEE2015::testConveyorRotate(unsigned long currentTime)
{
	static unsigned long previousTime = currentTime;
	static unsigned long timer = 5000; //5 sec for up/down
	static boolean conveyorDown = true;
	static boolean setup = true;
	if (setup)
	{
		Serial.println("Setup...");
		if (_conveyor->rotateConveyor(true, currentTime))
		{
			Serial.println("Done!");
			setup = false;
			conveyorDown = true;
			previousTime = currentTime;
		}
	}
	else
	{
		//rotate up when conveyor has been down for 5 sec
		if (conveyorDown && currentTime - previousTime > timer)
		{
			if (_conveyor->rotateConveyor(false, currentTime))
			{
				conveyorDown = false;
				previousTime = currentTime; //reset time
			}
		}
		//rotate down when conveyor has been up for 5 sec
		else if (!conveyorDown && currentTime - previousTime > timer)
		{
			if (_conveyor->rotateConveyor(true, currentTime))
			{
				conveyorDown = true;
				previousTime = currentTime; //reset time
			}
		}
	}
}

boolean ASEE2015::testBinDumping(unsigned long currentTime)
{
	static unsigned long previousTime = currentTime;
	unsigned long timer = 3000;
	static boolean dumping = true;
	if (dumping) //dump
	{
		if (_bins->dumpNextBin(currentTime))
		{
			dumping = false;
			previousTime = currentTime; //reset timer
		}
	}
	else //wait
	{
		if (currentTime - previousTime > timer)
		{
			dumping = true;
		}
	}

	if (_bins->_numDumped >= _bins->NUM_BINS)
	{
		return true;
	}
	else
	{
		return false;
	}
}