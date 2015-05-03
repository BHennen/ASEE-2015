#include "Conveyor.h"

Conveyor::Conveyor(int openAngle, int closedAngle, byte conveyorPowerDownward, byte conveyorPowerUpward, byte conveyorMotorDownwardPin,
	byte conveyorMotorUpwardPin, byte clawServoPin, byte limitSwitchPin, byte binMotorForwardPin, byte binMotorBackwardPin, byte binMotorSpeed,
	unsigned long binDumpingTime)
{

	_conveyorMotorDownwardPin = conveyorMotorDownwardPin;
	_conveyorMotorUpwardPin = conveyorMotorUpwardPin;
	_clawServoPin				= clawServoPin				;
	_limitSwitchPin				= limitSwitchPin			;
	_binMotorForwardPin			= binMotorForwardPin		;
	_binMotorBackwardPin		= binMotorBackwardPin		;
	//set Pins
	pinMode(conveyorMotorDownwardPin, OUTPUT);
	pinMode(conveyorMotorUpwardPin, OUTPUT);
	pinMode(clawServoPin, OUTPUT);
	pinMode(limitSwitchPin, INPUT);
	pinMode(binMotorForwardPin, OUTPUT);
	pinMode(binMotorBackwardPin, OUTPUT);
	clawServo.attach(clawServoPin);

	//Conveyor variables
	//motor
	motorSpeedDownward = conveyorPowerDownward;
	motorSpeedUpward = conveyorPowerUpward;
	//claw
	_closedAngle = closedAngle;
	_openAngle = openAngle;
	//limit switch variables
	previousTime = 0;
	debounceTime = 200;
	//initial variables
	currentPosition = FISH;
	currentState = HIGH; //HIGH since we start out at the fish pos
	previousState = HIGH;
	correctPosition = false;
	wentUp = false;

	//bin stuff
	binMotorSpeed = binMotorSpeed;
	binDumpingTime = binDumpingTime;
}

Conveyor::~Conveyor()
{

}

/**
 * This method counts using the limit switch and moves the claw backwards or forwards
 * depending on where it must go. This method also return a boolean  based on if the claw is
 * in the correct position.
 */
boolean Conveyor::goToBin(BinPosition binPosition, boolean currentState, unsigned long currentTime)
{	
	correctPosition = false; //We are not in the correct position initially

	//Check where the claw is.
	if (binPosition > currentPosition) //The desired position is closer to the back of the claw
	{
		wentUp = true;
		//Serial.println("Going up!");

		//Check if the limit switch is currently depressed and the previous state was low, and the debounce period has passed.
		if (currentState == HIGH && previousState == LOW && currentTime - previousTime > debounceTime)
		{
			//increment the current position of the claw
			currentPosition = (BinPosition)((int)(currentPosition)+1);

			Serial.print("New state:");
			switch (currentPosition)
			{
			case FISH: Serial.println("fish");
				break;
			case BIN1: Serial.println("bin1");
				break;
			case BIN2: Serial.println("bin2");
				break;
			case BIN3: Serial.println("bin3");
				break;
			}

			previousTime = currentTime;
		}
		//Tell the claw to go upward to the back of the conveyor
		analogWrite(_conveyorMotorDownwardPin, 0);
		analogWrite(_conveyorMotorUpwardPin, motorSpeedUpward);
	}
	else if (binPosition < currentPosition) //The desired position is closer to the front of the claw
	{
		wentUp = false;
		//Serial.println("Going down.");

		//Check if the limit switch is currently depressed and the previous state was low, and the debounce period has passed.
		if (currentState == HIGH && previousState == LOW && currentTime - previousTime > debounceTime)
		{
			//decrement the current position of the claw
			currentPosition = (BinPosition)((int)(currentPosition)-1);

			Serial.print("New state:");
			switch (currentPosition)
			{
			case FISH: Serial.println("fish");
				break;
			case BIN1: Serial.println("bin1");
				break;
			case BIN2: Serial.println("bin2");
				break;
			case BIN3: Serial.println("bin3");
				break;
			}

			previousTime = currentTime;
		}
		//Tell the claw to go downward to the front of the conveyor
		analogWrite(_conveyorMotorDownwardPin, motorSpeedDownward);
		analogWrite(_conveyorMotorUpwardPin, 0);
	}
	else if(currentState == LOW) //we're at correct position, but the conveyor is not on the limit switch. Make adjustments until the pin is high.
	{
		Serial.print("Went too far!");
		//Check which we we past the limit switch.
		if (wentUp) //We went too far upward, go back downward
		{
			Serial.println(" Going back downward");
			analogWrite(_conveyorMotorDownwardPin, motorSpeedDownward);
			analogWrite(_conveyorMotorUpwardPin, 0);
		}
		else// we went too far downward, go back upward
		{
			Serial.println(" Going back upward");
			analogWrite(_conveyorMotorDownwardPin, 0);
			analogWrite(_conveyorMotorUpwardPin, motorSpeedUpward);
		}
		
		if (previousState == HIGH && currentTime - previousTime > debounceTime)
		{
			wentUp = wentUp ? false : true;
		}
		previousTime = currentTime;
	} 
	else if(currentTime - previousTime > debounceTime) //we're at correct position.
	{
		Serial.print("Correct position");
		Serial.println(currentState);
		analogWrite(_conveyorMotorDownwardPin, 250);
		analogWrite(_conveyorMotorUpwardPin, 250);
		correctPosition = true;

		previousTime = currentTime;
	}
	previousState = currentState;
	return correctPosition;
}

/**
 * Close the claw
 */
void Conveyor::closeClaw()
{
	clawServo.write(_closedAngle);
}

/**
 * Opens the claw
 */
void Conveyor::openClaw()
{
	clawServo.write(_openAngle);
}

boolean Conveyor::dumpBins(unsigned long startTime)
{
	unsigned long currentTime = millis();

	if ((currentTime - startTime) >= binDumpingTime)
	{
		analogWrite(_binMotorBackwardPin, 0);
		analogWrite(_binMotorForwardPin, 0);
		return true;
	}
	else
	{
		analogWrite(_binMotorBackwardPin, 0);
		analogWrite(_binMotorForwardPin, binMotorSpeed);
		return false;
	}
}
/*
int Conveyor::count(int input)
{
	Serial.print(" ");
	if (input == HIGH && prevState == LOW &&  millis() - time > debounce) {

		if (currentState == HIGH) {

			currentState = LOW;

			Serial.println("add:" + add);
		}
		else
		{
			currentState = HIGH;

		}
		time = millis();
	}
	prevState = input;
	return add + 1;
}*/