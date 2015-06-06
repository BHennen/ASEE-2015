
#include <Servo.h>
#include <L3G.h>
#include <EEPROMAnything.h>
#include <eeprom.h>
#include <Wire.h>
#include <Pixy.h>
#include <SPI.h>
#include <Sensors.h>

/********
* PINS *
********/
/*** EYES ***/
const byte IRPort = 0; //Port for IR sensor


/*************
* CONSTANTS *
*************/

/*** EYES ***/
const float stopVoltage = 2.8; //Voltage to stop the robot
const unsigned long pixyUpdateTime = 500; //pixy takes 20 ms to get new blocks
byte getFishSigCount = 30;
//These values are the weights used to determine a blocks score
const float distConst = 0.8;
const float centerConst = 1.1;
const float bottomLineConst = 1.0;
float blockScoreConsts[4] = { distConst, centerConst, bottomLineConst };
float minimumBlockScore = -600.0; //lower number is more lenient
float minimumBlockSize = 250.0;
float maximumBlockY = 110.0;

//Constants for PID controller using pixy
const float pixykp = 0.3f;	//0.3;	//proportional
const float pixyki = 0.05f;	//0.05;	//integral
const float pixykd = 0.09f;	//0.07;	//derivative
float pixyPIDconsts[3] = { pixykp, pixyki, pixykd };



/***********
* MODULES *
***********/
VisualSensor* eyes;

void setup()
{
	Serial.begin(9600);

	//Construct objects that are enabled; otherwise set them to nullptrs
	eyes = new VisualSensor(IRPort, stopVoltage, 150, pixyUpdateTime, blockScoreConsts, pixyPIDconsts,
		minimumBlockScore, minimumBlockSize, maximumBlockY, getFishSigCount);
	
}

void loop()
{
	if (eyes->detectIRChange(millis()))
	{
		delay(500);
	}

}
