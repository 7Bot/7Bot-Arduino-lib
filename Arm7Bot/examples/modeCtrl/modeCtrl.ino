#include <Arm7Bot.h>
Arm7Bot Arm;

void setup() {
  // initial 7Bot Arm, the default mode is Normal Servo
  Arm.initialMove();
}

void loop() {
  
 /* Test */
  // Move to a lower pose
  double angles_1[SERVO_NUM] =  {90, 140, 65, 90, 90, 90, 75};
  Arm.move(angles_1);
  delay(2000);

  // Set mode to forceless
  Arm.forcelessMode();
  delay(3000);
}
