#include <Servo.h>
#include <FishManager.h>

const byte conveyorMotorDownwardPin = 7;
const byte conveyorMotorUpwardPin = 6;
const byte clawServoPin = 10;
const byte limitSwitchPin = 2;

int openAngle = 70;
int closedAngle = 0;
unsigned long clawMovingTime = 2000UL;
byte conveyorPowerDownward = 120;
byte conveyorPowerUpward = 120;

BinPosition restingPosition = BIN1;
boolean rawState = false;
boolean pickingUpFish = true;
boolean storingFish = false;
byte fishSignature = 1;
Conveyor *conveyor;

void setup() {
	Serial.begin(9600);
	conveyor = new Conveyor(openAngle, closedAngle, conveyorPowerDownward, conveyorPowerUpward, conveyorMotorDownwardPin,
		conveyorMotorUpwardPin, clawServoPin, limitSwitchPin, clawMovingTime, restingPosition);
}

void loop()
{
	unsigned long currentTime = millis();
	rawState = digitalRead(limitSwitchPin);
	conveyor->updateSwitch(rawState, currentTime);

	Serial.print(rawState);
	Serial.print(conveyor->_switchChanged);
	Serial.println(conveyor->_switchPressed);
}
