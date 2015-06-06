#include <SPI.h>
#include <Pixy.h>
#include <FishManager.h>
#include <Servo.h>
/**********************************
 * Test Sketch for Conveyor class *
 **********************************
 * Tests the goToBin(int Position) method
 */

const byte conveyorMotorDownwardPin = 7;
const byte conveyorMotorUpwardPin = 6;
const byte clawServoPin = 10;
const byte limitSwitchPin = 27;

int openAngle = 90;
int closedAngle = 27;
unsigned long clawMovingTime = 2000UL;
byte conveyorPowerDownward = 100; //100
byte conveyorPowerUpward = 100; //100

BinPosition startingPosition = BIN1;
BinPosition restingPosition = BIN1;
boolean rawState = false;
boolean pickingUpFish = true;
boolean storingFish = false;
byte fishSignature = 1;
Conveyor *conveyor;

void setup() {
    Serial.begin(9600);
    conveyor = new Conveyor(openAngle, closedAngle, conveyorPowerDownward, conveyorPowerUpward, conveyorMotorDownwardPin,
        conveyorMotorUpwardPin, clawServoPin, limitSwitchPin, clawMovingTime, restingPosition, startingPosition);
}

void loop()
{
    unsigned long currentTime = millis();
    if (pickingUpFish)
    {
        if (conveyor->pickUpFish(currentTime))
        {
            storingFish = true;
            pickingUpFish = false;
        }
    }
    else if (storingFish)
    {
        if (conveyor->storeFish(fishSignature, currentTime))
        {
            storingFish = false;
            pickingUpFish = true;
            (fishSignature == 4) ? fishSignature = 1 : fishSignature++; //increment fish signature or reset it to 1 if it is 4
        }
    }
}
