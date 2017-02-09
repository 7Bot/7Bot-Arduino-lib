#include <Arm7Bot.h>
Arm7Bot Arm;

void setup() {
  Arm.initialMove();
  while (!Arm.allConverge()) delay(20);

  Arm.moveIK3( {200, 200, 200} );
  while (!Arm.allConverge()) delay(20);
  delay(2000);

  Arm.moveIK3( {100, 200, 200} );
  while (!Arm.allConverge()) delay(20);
  delay(2000);
        
  Arm.moveIK3( {100, 100, 200} );
  while (!Arm.allConverge()) delay(20);
  delay(2000);

  Arm.moveIK3( {300, 100, 200} );
  while (!Arm.allConverge()) delay(20);
  delay(2000);

  Arm.initialMove();
  while (!Arm.allConverge()) delay(20);
}

void loop() {
 
  
}
