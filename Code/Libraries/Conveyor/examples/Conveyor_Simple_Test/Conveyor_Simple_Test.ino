#include <SPI.h>
#include <Pixy.h>
#include <Conveyor.h>

/************************************
 * Test Sketch for Conveyor Library *
 ************************************
*Tests the goToBin(int Position), openClaw(), and closeClaw()
*    methods
*
*Should do the following things, in order:
*1) Go to the FISH position, and open and close claws.
*2) Go to the RED_BIN position, and open and close claws.
*3) Go to the FISH position, and open and close claws.
*4) Go to the YELLOW_BIN position, and open and close claws.
*5) Go to the FISH position, and open and close claws.
*6) Go to the BLUE_BIN position, and open and close claws.
*7) Go to the FISH position, and open and close claws.
*8) Go to the GREEN_BIN position, and open and close claws.
*9) Go to the FISH position, and open and close claws.

Conveyor *claw;

void setup() {
  Serial.begin(9600);
  claw = new Conveyor();
}

void loop() {
  // put your main code here, to run repeatedly:
  bool inPosition = false;
  while (!inPosition)
  {
      inPosition = goToBin(FISH);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();
  
  while(!inPosition)
  {
    inPosition = goToBin(RED_BIN);
  }
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();
  
  while (!inPosition)
  {
      inPosition = goToBin(FISH);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go.
  closeClaw();
  
    while (!inPosition)
  {
      inPosition = goToBin(Yellow_BIN);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();
  
    while (!inPosition)
  {
      inPosition = goToBin(FISH);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();
  
    while (!inPosition)
  {
      inPosition = goToBin(BLUE_BIN);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();  
  
  while (!inPosition)
  {
      inPosition = goToBin(FISH);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();
  
    while (!inPosition)
  {
      inPosition = goToBin(GREEN_BIN);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();
  
    while (!inPosition)
  {
      inPosition = goToBin(FISH);
``}
  inPosition = false;
  
  openClaw();
  //location where delay would go
  closeClaw();
