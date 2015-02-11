#include <Servo.h>
#include <Conveyor.h>
Conveyor *conveyor;
int motorSpeed = 100;
int closedAngle =  0; //actually open angle
int openAngle = 133;//actually closed angle
byte conveyorMotorForwardPin = 33;
byte conveyorMotorBackwardPin = 34;
byte clawMotorPin = 9;
void setup() 
{
 Serial.begin(9600); 
 conveyor = new Conveyor(motorSpeed,closedAngle,openAngle,conveyorMotorForwardPin,conveyorMotorBackwardPin,clawMotorPin);
}

void loop() 
{
  while(true)
  {
   (*conveyor).openClaw();
   delay(3000);
   (*conveyor).closeClaw();
   delay(1000);
  }
  Serial.println("Go to fish to position");
 (*conveyor).goToPosition(FISH);
 delay(250);
 (*conveyor).openClaw();
 delay(250);
 (*conveyor).closeClaw();
 delay(250);
 
  Serial.println("Go to fish to red bin");
 (*conveyor).goToPosition(RED_BIN);
 delay(250);
 (*conveyor).openClaw();
 delay(250);
 (*conveyor).closeClaw();
 delay(250);
 
 Serial.println("Go to fish to yellow bin");
 (*conveyor).goToPosition(YELLOW_BIN);
 delay(250);
 (*conveyor).openClaw();
 delay(250);
 (*conveyor).closeClaw();
 delay(250);
 
 Serial.println("Go to fish to blue bin");
 (*conveyor).goToPosition(BLUE_BIN);
 delay(250);
 (*conveyor).openClaw();
 delay(250);
 (*conveyor).closeClaw();
 delay(250);
 
 Serial.println("Go to fish to green bin");
 (*conveyor).goToPosition(GREEN_BIN);
 delay(250);
 (*conveyor).openClaw();
 delay(250);
 (*conveyor).closeClaw();
 delay(250);
 
 Serial.println("Return to fish position");
 (*conveyor).goToPosition(FISH);
 delay(250);
 (*conveyor).openClaw();
 delay(250);
 (*conveyor).closeClaw();
 delay(250);
 
}
