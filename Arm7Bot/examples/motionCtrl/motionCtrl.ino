#include <Arm7Bot.h>
Arm7Bot Arm;

void setup() {
  // initial 7Bot Arm
  Arm.initialMove();
}

void loop() {

  // set motor[0] speed to 100
  Arm.maxSpeed[0] = 100;
  // Move to pose 1
  double angles_1[SERVO_NUM] =  {70, 115, 65, 90, 90, 90, 75};
  Arm.move(angles_1);
  // Move to pose 2
  double angles_2[SERVO_NUM] =  {110, 115, 65, 90, 90, 90, 75};
  Arm.move(angles_2);

  // change speed to 30
  Arm.maxSpeed[0] = 30;
  // Move to pose 3
  double angles_3[SERVO_NUM] =  {70, 135, 65, 90, 90, 90, 75};
  Arm.move(angles_3);
  // Move to pose 4
  double angles_4[SERVO_NUM] =  {110, 135, 65, 90, 90, 90, 75};
  Arm.move(angles_4);
  
}   
