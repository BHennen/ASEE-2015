#include <Servo.h>
#include <L3G.h>
#include <EEPROMAnything.h>
#include <eeprom.h>
#include <Wire.h>
#include <Pixy.h>
#include <SPI.h>
#include <Drivetrain.h>
#include <Sensors.h>
#include <FishManager.h>
#include <Robot.h>

/**********************
 * TESTING CONDITIONS *
 *****************************************************************************************************************************
 * Enable or disable different modules and set the test parameter to test various modules together and separately.           *
 * The description tells what the test does and the modules required tells what needs to be activated (true). All            *
 * modules not mentioned must be disabled.                                                                                   *
 *****************************************************************************************************************************
 * testParam | Description                                                                  | Modules required               *
 *___________|______________________________________________________________________________|________________________________*
 *     1     | Final test of the robot! This is the final product. Goes around the track and| All                            *
 *           | picks up all the fish, storing them inside. Dumps them all off too.          |                                *
 *___________|______________________________________________________________________________|________________________________*
 *     2     | Test PID and gyro. Goes to closest fish seen and stops in front of it.       | eyes, wheels, gyro             *
 *___________|______________________________________________________________________________|________________________________*
 *     3     | Test PID and Conveyor. Goes to closest fish seen, picks it up, and stores it | eyes, wheels, conveyor         *
 *           | based on its color                                                           |                                *
 *___________|______________________________________________________________________________|________________________________*
 *     4     | Test Conveyor. Picks up a fish and stores it. Has a pattern:                 | conveyor                       *
 *           | 1,2,3,1, 1,2,3,2, 1,2,3,3 ...                                                |                                *
 *___________|______________________________________________________________________________|________________________________*
 *     5     | Test fishCollection. Goes to closest fish seen, picks it up, and stores it   | eyes, wheels, conveyor, gyro   *
 *           | based on its color. Then rotates to next fish and repeats for all the fish.  |                                *
 *___________|______________________________________________________________________________|________________________________*
 *     6     | Test Conveyor. Picks up a fish, determines color using pixy, and stores it.  | conveyor, eyes                 *
 *___________|______________________________________________________________________________|________________________________*
 *     7     | Test PID. Goes to closest fish seen and stops in front of it, rotates to next| eyes, wheels, gyro             *
 *           | fish seen until all 12 fish have been visited.                               |                                *
 *___________|______________________________________________________________________________|________________________________*
 */

const int testParam = 7;

const boolean binsEnabled = false;
const boolean eyesEnabled = true;
const boolean conveyorEnabled = false;
const boolean wheelsEnabled = true;
const boolean gyroEnabled = true;


/********
 * PINS *
 ********/
/*** EYES ***/
const byte IRPort = 0; //Port for IR sensor

/*** WHEELS ***/
const byte DLMotorForward = 2;
const byte DLMotorBackward = 3;
const byte DRMotorForward = 4;
const byte DRMotorBackward = 5;

/*** BINS ***/

const byte binServoPin = 10;

/*** CONVEYOR ***/
const byte downwardCMPin = 6;
const byte upwardCMPin = 7;
const byte frontClawServoPin = 10;
const byte backClawServoPin = 10;
const byte IRPin = A1;

/*************
 * CONSTANTS *
 *************/
/*** GYRO ***/
const float gyrokp = 4.0f;	//proportional
const float gyroki = 0.5f;	//integral
const float gyrokd = 0.5f;	//derivative
float gyroPIDconsts[3] = { gyrokp, gyroki, gyrokd };

/*** EYES ***/
const float stopVoltage = 2.8; //Voltage to stop the robot
const unsigned long pixyStallTime = 300;
byte getFishSigCount = 30;
byte errorDeadzone = 5;
//These values are the weights used to determine a blocks score
const float centerConst = 1.0 / 13.75f; //Divide by these random numbers to convert it to inches
const float bottomLineConst = 0.3 / 2.83;
float blockScoreConsts[2] = { centerConst, bottomLineConst };
float minimumBlockScore = 0.0; //lower number is more lenient
float minimumBlockSize = 250.0;
float maximumBlockY = 80.0;

//Constants for PID controller using pixy
const float pixykp = 0.3f;	//0.3;	//proportional
const float pixyki = 0.05f;	//0.05;	//integral
const float pixykd = 0.09f;	//0.07;	//derivative
float pixyPIDconsts[3] = { pixykp, pixyki, pixykd };

/*** WHEELS ***/
//Constants for motors
const int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
const byte drivetrainPower = 65; //How much power for wheel motors. Valid values are 0 - 255.
const double robotStopDist = 4.0;
const double robotTurnRadius = 8.0;
const double robotCenter = 4.0;
//Constants for turning
//struct Rotation
//{
//	float angleFromVertical;
//	float length;
//	byte rotationType; //0 = stationary, 1 = sweep
//	boolean direction; //0 = left; 1 = right
//};
Rotation smallAnglePath[19] = { { 0,		16,			0,			0 },//At fish 1, go STRAIGHT to fish 2
								{ 0,		32,			0,			0 },//At fish 2, go STRAIGHT to fish 3
								{ 0,		16,			0,			0 },//At fish 3, go STRAIGHT to fish 4
								{ 112.5,	24.492,		1,			0 },//At fish 4, turn SWEEP LEFT to fish 5
								{ 157.5,	24.492,		1,			0 },//At fish 5, turn SWEEP LEFT to fish 6
								{ 270,		16,			0,			0 },//At fish 6, turn RIGHT to fish 7
								{ 270,		32,			0,			0 },//At fish 7, turn go STRAIGHT to fish 8
								{ 270,		16,			0,			0 },//At fish 8, turn go STRAIGHT to fish 9
								{ 22.5,		24.492,		1,			0 },//At fish 9, turn RIGHT to fish 10
								{ 180,		45.255,		0,			0 },//At fish 10, turn RIGHT to fish 11
								{ 90,		45.255,		0,			0 },//At fish 11, turn RIGHT to fish 12
																		//End of fish collection route
								{ 0,		0,			0,			0 },//At fish 12, turn LEFT to face bin 1
								{ 0,		0,			0,			0 },//At bin 1, reposition for dumping
								{ 0,		0,			0,			0 },//At bin 1, face bin 2
								{ 0,		0,			0,			0 },//At bin 2, reposition for dumping
								{ 0,		0,			0,			0 },//At bin 2, face bin 3
								{ 0,		0,			0,			0 },//At bin 3, reposition for dumping
								{ 0,		0,			0,			0 },//At bin 3, face bin 4
								{ 0,		0,			0,			0 },//At bin 4, reposition for dumping
};
const byte turnDeadzone = 1;

/*** CONVEYOR ***/
//Claw
const int frontUpwardAngle = 95;
const int frontDownwardAngle = 20;
const int backUpwardAngle = 95;
const int backDownwardAngle = 20;
const unsigned long clawMovingTime = 200UL; //How long the system should wait before the claw is done opening and closing
const unsigned long clawRotatingTime = 500UL; //How long the system should wait before the claw is done rotating

//Conveyor motor powers
const byte downwardConveyorPower = 150;
const byte upwardConveyorPower = 150;

//Conveyor positions
const BinPosition startingPosition = BIN1; //Where the conveyor is when we start off the robot
const BinPosition restingPosition = BIN1; //Where the conveyor rests when it is out of the way of seeing the fish

/*** BINS ***/
const unsigned long binDumpingTime = 10000UL; // how long the bin servo should turn to dump each bin
const byte binServoStop = 90; //Value for the bin servo to stop moving 
const byte binServoForward = 180; //Value for the bin servo to move

/***********
 * MODULES *
 ***********/
Drivetrain* wheels;
VisualSensor* eyes;
Gyro* gyro;
Conveyor* conveyor;
Bins* bins;
ASEE2015* myRobot;

void setup()
{
	Serial.begin(9600);

	//Construct objects that are enabled; otherwise set them to nullptrs
	eyes = (eyesEnabled) ? new VisualSensor(IRPort, stopVoltage, center, errorDeadzone, pixyStallTime,
		blockScoreConsts, pixyPIDconsts, minimumBlockScore, minimumBlockSize, maximumBlockY,
		getFishSigCount) : 0;
	gyro = (gyroEnabled) ? new Gyro(gyroPIDconsts) : 0;
	wheels = (wheelsEnabled) ? new Drivetrain(DLMotorForward, DLMotorBackward, DRMotorForward, DRMotorBackward,
		center, drivetrainPower, gyro, smallAnglePath, turnDeadzone, robotStopDist, robotTurnRadius, robotCenter) : 0;
	conveyor = (conveyorEnabled) ? new Conveyor(frontUpwardAngle, frontDownwardAngle, backUpwardAngle, backDownwardAngle,
		downwardConveyorPower, upwardConveyorPower, downwardCMPin, upwardCMPin,
		frontClawServoPin, backClawServoPin, IRPin,
		clawMovingTime, clawRotatingTime, restingPosition, startingPosition) : 0;
	bins = (binsEnabled) ? new Bins(binServoPin, binServoStop, binServoForward, binDumpingTime) : 0;

	//Construct the robot from the modules
	myRobot = new ASEE2015(testParam, wheels, eyes, gyro, conveyor, bins);
}

void loop()
{
	//Execute the function specified by the test param.
	if (myRobot->go())
	{
		myRobot->allStop();
	}
}
