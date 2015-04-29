#include <SPI.h>
#include <Pixy.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <Sensors.h>
#include <Drivetrain.h>
#include <Wire.h>
#include <L3G.h>

/***********************************************
 * Test Sketch for a test run around the track *
 ***********************************************
 * Tests what the robot should do when it is placed on the track. Should go around the track according to path#1 on the wiki.
 * Should do the following, in order (according to the state diagram on the wiki):
 * 1) Check if numFishCollected<12: If true the robot is in Pick Up Fish Route:
 *    A) Move towards the closest fish until it is close. (tests isClose() and getBlock() methods, as well as goToFish() method)
 *    B) If the robot is now close to the fish, it will stop for 4 seconds to simulate picking up a fish.
 *    C) The robot then rotates to face the next fish. (tests rotate() method)
 *    D) Go to step 1.
 * 2) numFishCollected = 12; The robot has collected all fish and is now in the Dump Fish Route:
 *    A) Move toward the bin closest to it. (tests getBlock() and goToFish() methods)
 *    B) If the robot is close to the bin, stop motors and rotate to position itself for dumping. (tests isClose() and rotate() methods)
 *    C) Robot stops for 4 seconds to simulate dumping a bin.
 *    D) Robot rotates to face the next bin. (tests rotate() method)
 *    E) Go to step 2)A).
 */


//Pins for motors
byte leftMotorForward = 5;
byte leftMotorBackward = 4;
byte rightMotorForward = 3;
byte rightMotorBackward = 2;

//Constants for motors
float center = 0; //Desired angle starts out at 0
byte power = 80; //How much power for wheel motors. Valid values are 0 - 255.

//Constant for turning
int stepDegrees[19] = {-60, //At fish 1, turn RIGHT to fish 2
                       70, //At fish 2, turn LEFT to fish 3
                       90, //At fish 3, turn LEFT to fish 4
                       
                       //Turn to face the outer ring of fish
                       -30, //At fish 4, turn RIGHT to fish 5
                       135, //At fish 5, turn LEFT to fish 6
                       45, //At fish 6, turn LEFT to fish 7
                       45, //At fish 7, turn LEFT to fish 8
                       45, //At fish 8, turn LEFT to fish 9
                       45, //At fish 9, turn LEFT to fish 10
                       45, //At fish 10, turn LEFT to fish 11
                       45, //At fish 11, turn LEFT to fish 12
                       
                       //End of fish collection route
                       -90, //At fish 12, turn RIGHT to face bin 1
                       90, //At bin 1, reposition for dumping
                       45, //At bin 1, face bin 2
                       45, //At bin 2, reposition for dumping
                       45, //At bin 2, face bin 3
                       45, //At bin 3, reposition for dumping
                       45, //At bin 3, face bin 4
                       45, //At bin 4, reposition for dumping
                       };
byte turnDeadzone = 3;

//Constants for PID controller
float kp = 0.25f; //proportional
float ki = 0.06f; //integral
float kd = 0.05f; //derivative

//Constants for visual sensor
const char IRPort = A0; //Port for IR sensor
float stopVoltage = 2.8; //Voltage to stop the robot

//Pointers to robot objects
VisualSensor *eyes;
Drivetrain *wheels;
Gyro *gyro;

int numFishCollected = 0;
int stepNum = 1;

void setup()
{
    Serial.begin(9600);

    //Create objects
    eyes = new VisualSensor(IRPort, stopVoltage);
    gyro = new Gyro();
    wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward,
                            center, power,
                            kp, ki, kd,
                            gyro, stepDegrees, turnDeadzone);
}

void loop()
{
    unsigned long currentTime = millis();
    gyro->update(currentTime); //Update gyro value
    //Test num fish collected; if less than 12 we are in fish collecting state.
    if(numFishCollected < 12)
    {
        if(!wheels->_isRotating)
        {
            //Check if we are close to a fish, if not:
            if(!(*eyes).isClose())
            {
                //Calculate the angle between the current desired angle and current degrees.
                //After corrections, negative diff means the robot should turn left. positive means it should turn right.
                float diff = wheels->getCenter() - gyro->getDegrees();
                if(diff > 180.0f) diff -= 360;
                if(diff < -180.0f) diff += 360;
                wheels->goToFishPID(diff, currentTime); //
            }
            else //We are close to a fish:
            {
                Serial.println("IR sensor close");
                //Insert conveyor code here
                //Conveyor code done; increment number of fish collected
                numFishCollected++;

                //4 second delay to simulate conveyor time
                (*wheels).stopMotors();
                delay(4000);

                //Rotate toward next fish
                wheels->rotateDegrees(stepNum, power);

                eyes->resetClosestBlock();
            }
        }
        else //We are rotating
        {
            //Serial.print("Rotating to next fish. IR value: \t");
            Serial.println(eyes->readProximity());
            
            if(wheels->rotateDegrees(stepNum, power))
            {
                stepNum++;
            }
        }
    }
    else //We are in dumping fish state
    {
        //Check if we are close to a bin, if not:
        if(!(*eyes).isClose())
        {
            //Move toward the closest bin
            Block targetBlock = (*eyes).getBlock(); //Get closest bin

            //Get block returns a bad block if no blocks were found, check if the block is the bad block
            if(targetBlock.signature != (*eyes).badBlock.signature)
            {
                (*wheels).goToFish(targetBlock); //Block is good, Move toward it
            }
        }
        else //We are close to a bin:
        {
            //Rotate to position for dumping
            stepNum++;
            (*wheels).rotate(stepNum);

            //Insert dumping code here
            //4 second delay to simulate dumping time
            (*wheels).stopMotors();
            delay(4000);

            //Rotate toward next bin
            stepNum++;
            (*wheels).rotate(stepNum);
        }
    }
}
