#include <Arm7Bot.h>
Arm7Bot Arm;

void setup() {
  // Open communication port
  ARMPORT.begin(BAUD_RATE);
  // initial 7Bot Arm
  Arm.initialMove();
}

void loop() {
  // run softwareSystem
  Arm.softwareSystem();
}   
