#include <Arm7Bot.h>
Arm7Bot Arm;

/* If you are a Arduino developer,
 * please read offset values from our calibration software UI
 * and change the following offset values. 
 */
double offsetValues[SERVO_NUM] = {20, 0, 0, 0, 0, 0, -50};

void setup() {

  // Set offset values
  for(int i=0; i<SERVO_NUM; i++)
    Arm.offset[i] = offsetValues[i];

  /* Test */
  // initial 7Bot Arm to check the offset values are work
  Arm.initialMove();
  delay(5000);
  
  // Move to a lower pose
  double angles_1[SERVO_NUM] =  {90, 140, 65, 90, 90, 90, 75};
  Arm.move(angles_1);
  delay(1000);

  // Set 7Bot to forceless mode 
  Arm.forcelessMode();
  delay(1000);
  
}

void loop() {
 
  
}   









