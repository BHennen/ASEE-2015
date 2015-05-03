#include <SPI.h>
#include <Pixy.h>
#include <Conveyor.h>
#include <Servo.h>
/************************************
 * Test Sketch for Conveyor Library *
 ************************************
* Tests the goToBin(int Position), openClaw(), and closeClaw()
*     methods
*
*/

const byte conveyorMotorDownwardPin = 7;
const byte conveyorMotorUpwardPin = 6;
const byte clawServoPin = 11;
const byte limitSwitchPin = 37;
const byte binMotorForwardPin = 9;
const byte binMotorBackwardPin = 10;

int openAngle = 60;
int closedAngle = 1;
byte conveyorPowerDownward = 120;
byte conveyorPowerUpward = 120;

unsigned long binDumpingTime = 2000UL;
byte binMotorSpeed = 100;
boolean state = false;
boolean goingToBin = true;
boolean grabFish = true;
int fishNum = 0;
Conveyor *claw;

void setup() {
  Serial.begin(9600);
  claw = new Conveyor(openAngle, closedAngle, conveyorPowerDownward, conveyorPowerUpward, conveyorMotorDownwardPin,
      conveyorMotorUpwardPin, clawServoPin, limitSwitchPin, binMotorForwardPin, binMotorBackwardPin, binMotorSpeed,
      binDumpingTime);
}

void loop()
{
    unsigned long currentTime = millis();
    state = digitalRead(limitSwitchPin);

    switch (fishNum) //Later this will be fish color, not fish num
    {
    case 0: 
        //Open and close claw for 1st fish
        if (grabFish)
        {
            claw->openClaw();
            delay(5000);

            claw->closeClaw();
            delay(2000);

            grabFish = false;
        }
        //Go to bin 3
        if (!grabFish && goingToBin && claw->goToBin(BIN1, state, currentTime))
        {
            //drop off fish
            claw->openClaw();
            delay(2000);
            goingToBin = false;
        }
        //go back to fish pos
        else if (!grabFish && !goingToBin && claw->goToBin(FISH, state, currentTime))
        {
            goingToBin = true;
            grabFish = true;
            fishNum++;
        }
        break;
    case 1:
        //Open and close claw for 2nd fish
        if (grabFish)
        {
            claw->openClaw();
            delay(2000);

            claw->closeClaw();
            delay(2000);

            grabFish = false;
        }
        //Go to bin 3
        if (!grabFish && goingToBin && claw->goToBin(BIN2, state, currentTime))
        {
            //drop off fish
            claw->openClaw();
            delay(2000);
            goingToBin = false;
        }
        //go back to fish pos
        else if (!grabFish && !goingToBin && claw->goToBin(FISH, state, currentTime))
        {
            goingToBin = true;
            grabFish = true;
            fishNum++;
        }
        break;
    case 2:
        //Open and close claw for 3RD fish
        if (grabFish)
        {
            claw->openClaw();
            delay(2000);

            claw->closeClaw();
            delay(2000);

            grabFish = false;
        }
        //Go to bin 3
        if (!grabFish && goingToBin && claw->goToBin(BIN3, state, currentTime))
        {
            //drop off fish
            claw->openClaw();
            delay(2000);
            goingToBin = false;
        }
        //go back to fish pos
        else if (!grabFish && !goingToBin && claw->goToBin(FISH, state, currentTime))
        {
            goingToBin = true;
            grabFish = true;
            fishNum++;
        }
        break;
    }
}
