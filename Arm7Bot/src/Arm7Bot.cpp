/****************************************************
/* 7Bot class for Arduino platform
/* Author: Jerry Peng
/* Date: 5 May 2016
/* 
/* Version 1.01
/* www.7bot.cc
/*  
/* Description: 
/* 
/*
/***************************************************/
#include "Arm7Bot.h"



Arm7Bot::Arm7Bot() {
  // intitialize parameters
  for (int i = 0; i < SERVO_NUM; i++) {
    offset[i] = offsetInit[i];
    posG[i] = INITIAL_POS[i];
    isFluent[i] = true;
    fluentRange[i] = fluentRangeInit[i];
    filterData[i] = 0;
  }

  analogWriteResolution(12);  // for Due
  
  // initalize elements
  btAndBuzInit();
  vacuumCupInit();
  getStoreData();
  setStoreData();
}

// Initial vacuum cup
void Arm7Bot::vacuumCupInit() {
  pinMode(valve_pin, OUTPUT);
  pinMode(pump_pin, OUTPUT);
  digitalWrite(valve_pin, LOW);
  digitalWrite(pump_pin, LOW);
}

// Read storage data from flash
void Arm7Bot::getStoreData() {
  int poseCntInit = (int)dueFlashStorage.read(256);
  if (poseCntInit != 255) poseCnt = poseCntInit;
  for (int i = 0; i < SERVO_NUM; i++) {
    int offsetRead = (int)dueFlashStorage.read(128 + i);
    if (offsetRead != 255) {
      int mul = 1;
      if (offsetRead >= 64) mul = -1;
      offset[i] = mul * (offsetRead % 64);
    }
  }
}

// Write some firmware datas to flash (ID:0~254, value:0~127 valid)
void Arm7Bot::setStoreData() {
  dueFlashStorage.write(0, 7);  // 7Bot
  dueFlashStorage.write(1, 10);  // Verion 1.0
}

// Motion initial fuction.
// Read current postion first, and then adjust to the inital postion softly and gradually.
void Arm7Bot::initialMove() {

  // Setting initial speeds to low values
  maxSpeed[0] = 15; 
  maxSpeed[1] = 20; 
  maxSpeed[2] = 20; 
  maxSpeed[3] = 30; 
  maxSpeed[4] = 30; 
  maxSpeed[5] = 30; 
  maxSpeed[6] = 30;
  
  for (int i = 0; i < filterSize; i++) {
    delay(10);
    receiveCom();
    filterAnalogData();
  }
  calculatePosD();
    
  for (int i = 0; i < SERVO_NUM; i++) {
    pos[i] = posS[i] = posD[i]; // Set start position & current position to detected position
    posG[i] = INITIAL_POS[i]; // Set the goal locations to firmware defaults
    Servos[i].attach( 2 + i, 90, 2500);  // attach servos
    isConverge[i] = false;
  }

  // Adjust to the inital postion
  while ( !allConverge() ) {
    moveOneStep();
    delay(10);
    receiveCom();
  }

  // Setting running speeds to proper values. Too high may cause unstable.  
  maxSpeed[0] = 80; 
  maxSpeed[1] = 100; 
  maxSpeed[2] = 100; 
  maxSpeed[3] = 200; 
  maxSpeed[4] = 200; 
  maxSpeed[5] = 200; 
  maxSpeed[6] = 200;
}

void Arm7Bot::servoMode(int mode) {
  forceStatus = constrain(mode, 0, 2);
  
  int forceMicroSeconds = 0;
  if (forceStatus == 0) forceMicroSeconds = 100;
  if (forceStatus == 2) forceMicroSeconds = 300;

  // Re-attach servos if switching from forceless or stop modes.
  if (!Servos[0].attached()) {
    for (int i = 0; i < SERVO_NUM; i++) {
      Servos[i].attach( 2 + i, 90, 2500);  // attach servos
    }
    delay(100);    

    // Stop moving
    for (int i = 0; i < filterSize; i++) {
      // delay(10);
      filterAnalogData();
    }
    calculatePosD();
    
    for (int i = 0; i < SERVO_NUM; i++) {
      pos[i] = posS[i] = posD[i]; // Set start position & current position to detected position
    }
    servoCtrl();
  }
  
  
  // Set the servo mode and detatch
  if (forceStatus != 1) {
    if (Servos[0].attached()) {
      for (int i = 0; i < SERVO_NUM; i++) {
        Servos[i].writeMicroseconds(forceMicroSeconds);
      }
      delay(100);
      for (int i = 0; i < SERVO_NUM; i++) Servos[i].detach();
    } 
  }
}

// Set 7Bot to forceless mode, which you can drag 7Bot by hand easily and also read stable pose feedbacks
void Arm7Bot::forcelessMode() { servoMode(0); }

// Set 7Bot to normal mode, which stops the arm from moving.
void Arm7Bot::normalMode() { servoMode(1); }

// Set 7Bot to stop mode, which you can drag 7Bot by hand but the robot still have some forces.
// This mode is useful for device protection, robot can be moved by external forces, but still have some resistance.
void Arm7Bot::stopMode()  { servoMode(2); }

// 7Bot move fuction
// Given each axis angle(Unit:degrees)
void Arm7Bot::move(double angles[SERVO_NUM]) {
  
  for (int i = 0; i < SERVO_NUM; i++) {
    posG[i] = angles[i];
    isConverge[i] = false;
  }
  
  // move gradually
  while (!allConverge()) {
    moveOneStep();
    delay(20);
  }

  // Compatible with vaccum cap
  if (posG[6] < 30) {
    digitalWrite(valve_pin, LOW);
    digitalWrite(pump_pin, HIGH);
  }
  else {
    digitalWrite(valve_pin, HIGH);
    digitalWrite(pump_pin, LOW);
  }
  
}


// Gradually moving fuction
// Move one step when this function is called.
void Arm7Bot::moveOneStep() {

  if (!Servos[0].attached()) initialMove();

  for (int i = 0; i < SERVO_NUM; i++) {

    // Speed limitation
    if(maxSpeed[i]>250) maxSpeed[i] = 250;

    double maxStp = maxSpeed[i] / 50; // max step change 
    double minStp = maxStp / 10;
    double decDiff = fluentRange[i] * maxStp; // Decelerate Zone when difference btween posG & pos below this value. Unit: Degree
    double accDiff = decDiff / 2;  // Accelerate Zone when difference btween posS & pos below this value. Unit: Degree
    isConverge[i] = false;
    double stp = 0;
    double diff = posG[i] - pos[i];
    double diffS = pos[i] - posS[i];

    // Motion without acceleration & deceleration
    if (!isFluent[i]) {
      if (diff > maxStp) stp = maxStp;
      else if (diff < -maxStp) stp = -maxStp;
      else {
        pos[i] = posG[i];
        isConverge[i] = true;   
        posS[i] = pos[i];
      }
    }
    // Motion with acceleration & deceleration (Linear Speed Change)
    if ( (diff < 0 && diffS > 0) || (diff > 0 && diffS < 0) ) {
      posS[i] = pos[i];
      diffS = pos[i] - posS[i];
    }
    else if (diff > decDiff)
      if (diffS < accDiff)
        stp = max(minStp, diffS / accDiff * maxStp); 
      else
        stp = maxStp;  
    else if (diff < -decDiff)
      if (diffS > -accDiff)
        stp = min(-minStp, diffS / accDiff * maxStp); 
      else
        stp = -maxStp;  
    else {

      if (abs(diff) > minStp) {
        if (diff > 0) stp = max(minStp, (diff / decDiff) * maxStp);
        else if (diff < 0) stp = min(-minStp, (diff / decDiff) * maxStp);
      }
      else {
        stp = 0;
        isConverge[i] = true;  // already converge
        posS[i] = pos[i]; 
      }
    }

    pos[i] += stp;
  }
  servoCtrl();

}

void Arm7Bot::moveIK3(PVector j5) {
  int IK_status = IK3( j5 );
  if (IK_status == 0) {
    for (int i = 0; i < 3; i++) {
      posG[i] = degrees(theta[i]);
    }
    // posG[3] = theta3;
    // posG[4] = theta4;
    // posG[5] = theta5;
    // posG[6] = theta6;
  }
  move(posG);
}

void Arm7Bot::moveIK5(PVector j6, PVector vec56) {
  int IK_status = IK5( j6, vec56 );
  if (IK_status == 0) {
    for (int i = 0; i < 5; i++) {
      posG[i] = degrees(theta[i]);
    }
    posG[5] = theta5;
    posG[6] = theta6;
  }
  move(posG);
}
    
void Arm7Bot::moveIK6(PVector j6, PVector vec56, PVector vec67) {
  int IK_status = IK6( j6, vec56, vec67 );
  if (IK_status == 0) {
    for (int i = 0; i < 6; i++) {
      posG[i] = degrees(theta[i]);
    }
    posG[6] = theta6;
  }
  move(posG);
}    


// Set PWM ctrl signal to servos
void Arm7Bot::servoCtrl() {
  geometryConstrain();
  for (int i = 0; i < SERVO_NUM; i++) {
    double posTmp = pos[i];
    if (reverse[i]) {
      posTmp = 180 - pos[i];
    }
    servoPos[i] = posTmp + offset[i];
    if (servoPos[i] < 0) servoPos[i] = 0;
    else if (servoPos[i] > 180) servoPos[i] = 180;
  }
  if (servoPos[3] < 10) servoPos[3] = 10;
  else if (servoPos[3] > 170) servoPos[3] = 170;

  for (int i = 0; i < 7; i++)
    Servos[i].writeMicroseconds( int(500 + servoPos[i] * 11.111111) );

}


// Geometry constrains
void Arm7Bot::geometryConstrain() {
  if (pos[1] < 0) {
    pos[1] = 0;
  } else if (pos[1] + pos[2] > 227) {
    pos[1] = 227 - pos[2];
  }
  if(pos[6]<0){pos[6] = 0;} else if(pos[6]>80){pos[6] = 80;} // for gripper
}

// anolog feedback preprocessing fuction
// Move one step when this function is called.
void Arm7Bot::filterAnalogData() {
  for (int i = 0; i < SERVO_NUM; i++) {
    filterData[i] = filters[i].filter(analogRead(i));
  }
}

// Servo pose calculation fuction
void Arm7Bot::calculateServoPosD() {
  for (int i = 0; i < SERVO_NUM; i++) {
    servoPosD[i] = (double)((filterData[i] - 114)) * 0.21582734;
    // servo range constrains, eliminate out range detection data (0~180 degree)
    if (servoPosD[i] < 0) {
      servoPosD[i] = 0;
    } else if (servoPosD[i] > 180) {
      servoPosD[i] = 180;
    }
  }
}

// Structure calculation fuction
void Arm7Bot::calculatePosD() {

  calculateServoPosD();
  for (int i = 0; i < SERVO_NUM; i++) {
    posD[i] = servoPosD[i] - offset[i];
    if (reverse[i]) {
      posD[i] = 180 - posD[i];
    }
    // servo range constrains, eliminate out range detection data
    else {
      if (posD[i] < 0) {
        posD[i] = 0;
      }
      else if (posD[i] > 180) {
        posD[i] = 180;
      }
    }
  }
}

// Send servos' current position to PC
void Arm7Bot::sendPosDAndForce() {
  int sendData[SERVO_NUM];
  for (int i = 0; i < SERVO_NUM; i++) {
    sendData[i] = (int)(posD[i] * 50 / 9);
    force[i] = constrain(force[i], -14, 14);
    if (force[i] < 0) sendData[i] += 16384;
    sendData[i] += abs(force[i]) / 2 * 1024;

  }
  // Send data
  ARMPORT.write(0xFE); // Beginning Flag 0xFE
  ARMPORT.write(0xF9);
  // SERVO_NUM tuple (2-bytes each)
  for (int i = 0; i < SERVO_NUM; i++) {
    ARMPORT.write((sendData[i] / 128) & 0x7F);
    ARMPORT.write(sendData[i] & 0x7F);
  }
  // Convergency
  int isAllConverge = 0;
  if (allConverge()) isAllConverge = 1;
  ARMPORT.write(isAllConverge & 0x7F);

}

// Motion converge detection fuction
boolean Arm7Bot::allConverge() {
  return isConverge[0] && isConverge[1] && isConverge[2] && isConverge[3] && isConverge[4] && isConverge[5] && isConverge[6];
}

// External force estimate 
void Arm7Bot::calculateForce() {
  for (int i = 0; i < SERVO_NUM; i++) {
    force[i] = forceFilters[i].filter( (int)(pos[i] - posD[i]) );
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Kinematics */

// Valid range check
// return: [0]-no error, [1]-some angle out of range, [2]-theta1 & theta2 cosntrain
int Arm7Bot::angleRangCheck() {
  for (int i = 0; i < SERVO_NUM; i++) {
    if (theta[i] < thetaMin[i] || theta[i] > thetaMax[i]) return 1;
  }
  if (theta[2] + theta[1] > 3.96189733 || theta[2] + theta[1] < 1.44862327 )  return 2;
  return 0;
}

// Calculate model joints
void Arm7Bot::calcJoints() {
  joint[1] = PVector(joint[0].x, joint[0].y + d, joint[0].z + e);
  joint[2] = PVector(0, -b * cos(theta[2] - 1.134464), b * sin(theta[2] - 1.134464));    joint[2].add(joint[1]);
  joint[3] = PVector(0, a * cos(theta[1]), a * sin(theta[1]));     joint[3].add(joint[1]);
  joint[4] = PVector(0, h * sin(theta[2] - 1.134464), h * cos(theta[2] - 1.134464));     joint[4].add(joint[3]);
  joint[5] = PVector(0, c * cos(theta[2] - 1.134464), -c * sin(theta[2] - 1.134464));    joint[5].add(joint[4]);
  joint[6] = PVector(0, f * sin(theta[2] - 1.134464 + theta[4]), f * cos(theta[2] - 1.134464 + theta[4]));    joint[6].add(joint[5]);
  joint[7] = PVector(0, -g * cos(theta[2] - 1.134464 + theta[4]), g * sin(theta[2] - 1.134464 + theta[4]));   joint[7].add(joint[6]);
  joint[7] = arbitraryRotate(joint[7], joint[6], joint[5], theta[5]); 
  joint[6] = arbitraryRotate(joint[6], joint[5], joint[4], theta[3] - HALF_PI); 
  joint[7] = arbitraryRotate(joint[7], joint[5], joint[4], theta[3] - HALF_PI); 
  joint[8] = PVector(2 * joint[6].x - joint[7].x, 2 * joint[6].y - joint[7].y, 2 * joint[6].z - joint[7].z);
  for (int i = 1; i < 9; i++) {
    joint[i] = zAxiRotate(joint[i], theta[0] - HALF_PI);
  }
}

PVector Arm7Bot::zAxiRotate(PVector point, double _angle) {
  PVector pt;
  pt = PVector( cos(_angle) * point.x - sin(_angle) * point.y, sin(_angle) * point.x + cos(_angle) * point.y, point.z );
  return pt;
}

PVector Arm7Bot::arbitraryRotate(PVector point, PVector pointA, PVector pointB, double _angle) {
  PVector pt = PVector(0, 0, 0);
  double x = point.x, y = point.y, z = point.z;
  double u = pointB.x - pointA.x, v = pointB.y - pointA.y, w = pointB.z - pointA.z;
  double l = sqrt(u * u + v * v + w * w);
  u /= l; v /= l; w /= l;
  double a = pointA.x, b = pointA.y, c = pointA.z;
  double u2 = u * u, v2 = v * v, w2 = w * w;
  double au = a * u, av = a * v, aw = a * w;
  double bu = b * u, bv = b * v, bw = b * w;
  double cu = c * u, cv = c * v, cw = c * w;
  double ux = u * x, uy = u * y, uz = u * z;
  double vx = v * x, vy = v * y, vz = v * z;
  double wx = w * x, wy = w * y, wz = w * z;
  pt.x = (a * (v2 + w2) - u * (bv + cw - ux - vy - wz)) * (1 - cos(_angle)) + x * cos(_angle) + (-cv + bw - wy + vz) * sin(_angle);
  pt.y = (b * (u2 + w2) - v * (au + cw - ux - vy - wz)) * (1 - cos(_angle)) + y * cos(_angle) + (cu - aw + wx - uz) * sin(_angle);
  pt.z = (c * (u2 + v2) - w * (au + bv - ux - vy - wz)) * (1 - cos(_angle)) + z * cos(_angle) + (-bu + av - vx + uy) * sin(_angle);
  return pt;
}

PVector Arm7Bot::calcProjectionPt(PVector pt0, PVector pt1, PVector nVec) {
  PVector n = PVector(nVec.x, nVec.y, nVec.z);
  n.normalize();
  PVector vec10 = PVector(pt0.x - pt1.x, pt0.y - pt1.y, pt0.z - pt1.z);
  double dot = vec10.dot(n);
  PVector projectionPt = PVector(pt0.x - dot * n.x, pt0.y - dot * n.y, pt0.z - dot * n.z);
  return projectionPt;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Invers Kinematics */

// Function: IK, input Joint[5], calculate theta[0]~[2]
// return: [0]no error; [1]out of valid range
// input unit:mm
int Arm7Bot::IK3(PVector pt) {
  double x = pt.x, y = pt.y, z = pt.z;
  int status = 1;
  theta[0] = atan(y / x);
  if (theta[0] < 0) theta[0] = PI + theta[0]; 
  x -= d * cos(theta[0]);
  y -= d * sin(theta[0]);
  z -= e;
  double lengthA = sqrt(x * x + y * y + z * z); 
  double lengthC = sqrt(h * h + c * c); 
  double offsetAngle = atan(h / c);
  double angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
  double angleB = atan( z / sqrt(x * x + y * y) );
  double angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
  theta[1] = angleB + angleC;
  theta[2] = PI - angleA - angleB - angleC + offsetAngle;
  theta[2] += 1.134464;

  // range check
  if (theta[1] > thetaMin[1] && theta[1] < thetaMax[1] &&
      theta[2] > thetaMin[2] && theta[2] < thetaMax[2] &&
      theta[2] - 0.8203047 + theta[1] < PI && theta[2] + theta[1] > 1.44862327) {
    status = 0;
  }
  return status;
}

// Function: IK, input joint[6] & Vector(joint[5] to joint[6] direction), calculate theta[0]~[4]
// return: [0]no error; [1]IK3 out of valid range; [2]IK5-theta4 out of range
int Arm7Bot::IK5(PVector j6, PVector vec56_d) {
  int status = -1;
  PVector vec56_u =  PVector(vec56_d.x, vec56_d.y, vec56_d.z); 
  vec56_u.normalize();
  PVector j5 =  PVector(j6.x - f * vec56_u.x, j6.y - f * vec56_u.y, j6.z - f * vec56_u.z);
  PVector vec56 =  PVector(j6.x - j5.x, j6.y - j5.y, j6.z - j5.z);
  int IK3_status = IK3(j5);
  //println("IK3_status: ", IK3_status);
  if (IK3_status != 0) return IK3_status;
  joint[5] = j5;
  theta[3] = 0.; theta[4] = 0.;
  calcJoints();
  PVector j6_0 = joint[6];
  PVector vec56_0 =  PVector(j6_0.x - j5.x, j6_0.y - j5.y, j6_0.z - j5.z);
  PVector vec45 =  PVector(joint[5].x - joint[4].x, joint[5].y - joint[4].y, joint[5].z - joint[4].z);
  PVector j6p = calcProjectionPt(j6, j5, vec45);
  PVector vec56p =  PVector(j6p.x - j5.x, j6p.y - j5.y, j6p.z - j5.z);
  //ARMPORT.print("vec56p= "); ARMPORT.print( vec56p.x ); ARMPORT.print(" ");ARMPORT.print( vec56p.y ); ARMPORT.print(" ");ARMPORT.println( vec56p.z );
  theta[3] = acos( vec56_0.dot(vec56p) / (j5.dist(j6_0) * j5.dist(j6p)) );
  theta[4] = acos( vec56.dot(vec56p) / (j5.dist(j6) * j5.dist(j6p)) );
  calcJoints();
  double dist = j6.dist(joint[6]);
  if (dist < 1) { 
    return 0;
  }
  theta[3] = PI - theta[3]; 
  theta[4] = PI - theta[4]; 
  calcJoints();
  dist = j6.dist(joint[6]);
  if (dist < 1) { 
    return 0;
  }
  else {
    return 2;
  }
}

// Function: IK, input joint[6], Vector(joint[5] to joint[6] direction) & Vector(joint[6] to joint[7]), calculate theta[0]~[5]
// return: [0]no error; [1]IK3 out of valid range; [2]IK5-theta4 out of range;
int Arm7Bot::IK6(PVector j6, PVector vec56_d, PVector vec67_d) {
  int status = -1;
  int IK5_status = IK5(j6, vec56_d);
  if (IK5_status != 0) return IK5_status;
  PVector vec67_u =  PVector(vec67_d.x, vec67_d.y, vec67_d.z);
  vec67_u.normalize();
  PVector j7 =  PVector(j6.x + g * vec67_u.x, j6.y + g * vec67_u.y, j6.z + g * vec67_u.z);
  PVector j7p = calcProjectionPt(j7, j6, vec56_d);
  theta[5] = 0; calcJoints();
  PVector j7_0 = joint[7];
  PVector vec67_0 =  PVector(j7_0.x - j6.x, j7_0.y - j6.y, j7_0.z - j6.z);
  PVector vec67p =  PVector(j7p.x - j6.x, j7p.y - j6.y, j7p.z - j6.z);
  //(3)- calculate theta[5]
  double thetaTmp5 = acos( vec67_0.dot(vec67p) / (j6.dist(j7_0) * j6.dist(j7p)) );
  theta[5] = -thetaTmp5;
  if (vec67_d.x < 0) theta[5] = -theta[5];
  if (theta[5] < 0) theta[5] = PI + theta[5]; 
  calcJoints();
  return 0;
}




///////////////////////////////////////////////////////////////////////////////////////
/*  UART Communication with Host(Such as PC) */ 

void Arm7Bot::receiveCom() {

  while (ARMPORT.available() > 0) {
    // read data
    int rxBuf = ARMPORT.read();
    if (!beginFlag)
    {
      beginFlag = rxBuf == 0xFE ? true : false; // Beginning Flag 0xFE
    }
    else
    {
      if (instruction == 0) instruction = rxBuf - 240;

      else {
        switch (instruction) {
          case 1:
            dataBuf[cnt++] = rxBuf;
            if (cnt >= 2)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              dataBuf[0] = constrain(dataBuf[0], 0, 127);
              dataBuf[1] = constrain(dataBuf[1], 0, 127);
              dueFlashStorage.write(dataBuf[0], (uint8_t)dataBuf[1]);
            }
            break;

          case 2:
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            F2_id = constrain(rxBuf, 0, 127);
            //
            ARMPORT.write(0xFE);
            ARMPORT.write(0xF2);
            ARMPORT.write(F2_id & 0x7F);
            ARMPORT.write(dueFlashStorage.read(F2_id) & 0x7F);
            break;

          case 3:
            dataBuf[cnt++] = rxBuf;
            if (cnt >= SERVO_NUM)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              for (int i = 0; i < SERVO_NUM; i++) {
                dueFlashStorage.write(128 + i, (uint8_t)dataBuf[i]);
              }
            }
            break;

          case 4:
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            //
            ARMPORT.write(0xFE);
            ARMPORT.write(0xF4);
            for (int i = 0; i < SERVO_NUM; i++) {
              ARMPORT.write(dueFlashStorage.read(128 + i) & 0x7F);
            }
            break;

          case 5:
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            forceStatus = constrain(rxBuf, 0, 2);
            break;

          case 6:
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            //
            ARMPORT.write(0xFE);
            ARMPORT.write(0xF6);
            ARMPORT.write(forceStatus & 0x7F);
            break;

          case 7:
            dataBuf[cnt++] = rxBuf;
            if (cnt >= SERVO_NUM)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              for (int i = 0; i < SERVO_NUM; i++) {
                if (dataBuf[i] >= 64) {
                  isFluent[i] = true;
                  dataBuf[i] -= 64;
                }
                else isFluent[i] = false;
                maxSpeed[i] = dataBuf[i] * 10;
              }
            }
            break;

          case 8:
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            //
            int sendData[SERVO_NUM];
            for (int i = 0; i < SERVO_NUM; i++) {
              sendData[i] = maxSpeed[i] / 10;
              if (isFluent[i]) sendData[i] += 64;
            }
            ARMPORT.write(0xFE);
            ARMPORT.write(0xF8);
            for (int i = 0; i < SERVO_NUM; i++) {
              ARMPORT.write(sendData[i] & 0x7F);
            }
            break;

          case 9:
            dataBuf[cnt++] = rxBuf;
            if (cnt >= SERVO_NUM * 2)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              for (int i = 0; i < SERVO_NUM; i++) {
                double posCode = dataBuf[i * 2] * 128 + dataBuf[i * 2 + 1];
                posG[i] = posCode * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
              }
            }
            break;

          case 10:
            dataBuf[cnt++] = rxBuf;
            if (cnt >= 10 * 2)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              int data[10];
              for (int i = 0; i < 10; i++) {
                data[i] = dataBuf[i * 2] * 128 + dataBuf[i * 2 + 1];
              }
              //
              int mul;
              mul = 1; if (data[0] > 1024) mul = -1; j6.x = data[0] % 1024 * mul;
              mul = 1; if (data[1] > 1024) mul = -1; j6.y = data[1] % 1024 * mul;
              mul = 1; if (data[2] > 1024) mul = -1; j6.z = data[2] % 1024 * mul;
              //
              mul = 1; if (data[3] > 1024) mul = -1; vec56.x = data[3] % 1024 * mul;
              mul = 1; if (data[4] > 1024) mul = -1; vec56.y = data[4] % 1024 * mul;
              mul = 1; if (data[5] > 1024) mul = -1; vec56.z = data[5] % 1024 * mul;
              //
              mul = 1; if (data[6] > 1024) mul = -1; vec67.x = data[6] % 1024 * mul;
              mul = 1; if (data[7] > 1024) mul = -1; vec67.y = data[7] % 1024 * mul;
              mul = 1; if (data[8] > 1024) mul = -1; vec67.z = data[8] % 1024 * mul;
              //
              theta6 = ((double)(data[9])) * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
              //
              F10 = true;
            }
            break;

          case 11:
            dataBuf[cnt++] = rxBuf;
            if (cnt >= 8 * 2)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              int data[8];
              for (int i = 0; i < 8; i++) {
                data[i] = dataBuf[i * 2] * 128 + dataBuf[i * 2 + 1];
              }
              //
              int mul;
              mul = 1; if (data[0] > 1024) mul = -1; j6.x = data[0] % 1024 * mul;
              mul = 1; if (data[1] > 1024) mul = -1; j6.y = data[1] % 1024 * mul;
              mul = 1; if (data[2] > 1024) mul = -1; j6.z = data[2] % 1024 * mul;
              //
              mul = 1; if (data[3] > 1024) mul = -1; vec56.x = data[3] % 1024 * mul;
              mul = 1; if (data[4] > 1024) mul = -1; vec56.y = data[4] % 1024 * mul;
              mul = 1; if (data[5] > 1024) mul = -1; vec56.z = data[5] % 1024 * mul;
              //
              theta5 = ((double)(data[6])) * 9 / 50;
              theta6 = ((double)(data[7])) * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
              //
              F11 = true;
            }
            break;

          case 12:
            dataBuf[cnt++] = rxBuf;
            if (cnt >= 7 * 2)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              int data[7];
              for (int i = 0; i < 7; i++) {
                data[i] = dataBuf[i * 2] * 128 + dataBuf[i * 2 + 1];
              }
              //
              int mul;
              mul = 1; if (data[0] > 1024) mul = -1; j5.x = data[0] % 1024 * mul;
              mul = 1; if (data[1] > 1024) mul = -1; j5.y = data[1] % 1024 * mul;
              mul = 1; if (data[2] > 1024) mul = -1; j5.z = data[2] % 1024 * mul;
              //
              theta3 = ((double)(data[3])) * 9 / 50;
              theta4 = ((double)(data[4])) * 9 / 50;
              theta5 = ((double)(data[5])) * 9 / 50;
              theta6 = ((double)(data[6])) * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
              //
              F12 = true;
            }
            break;

          default:
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            break;
        }
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////
/*  Btn & Buzzer */

//  Initialize 
void Arm7Bot::btAndBuzInit() {
  // Initial Buzzer
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);
  // Initial Buttons
  for (int i = 0; i < BUTTON_NUM; i++) {
    pinMode(button_pin[i], INPUT);
    digitalWrite(button_pin[i], HIGH); //Enable the pullup resistor on the button
    last_reading[i] = ! digitalRead(button_pin[i]);
  }
  // Initial Timers
  time_300ms = millis();
}


/* Press button detection & Buzzer */
void Arm7Bot::btDetectionAndBuz() {

  for (int i = 0; i < BUTTON_NUM; i++) {
    reading[i] = ! digitalRead(button_pin[i]);
    if (last_reading[i] != reading[i]) last_debounce_time[i] = millis();

    if ( (millis() - last_debounce_time[i])  > debounce_delay) {
      bool state = reading[i];
      if (last_state[i] && !state) {
        if (btLongPress) btLongPress = false;
        else {
          shortBuz = true;
          shortBuzBegin = millis();
          digitalWrite(buzzer_pin, HIGH);
          // button[i] pressed once
          btS[i] = true;
        }
      }
      last_state[i] = state;
    }
    last_reading[i] = reading[i];
  }

  // press button 1.8s
  if ( (millis() - time_300ms >= 300) && !btLongPress)  {
    time_300ms = millis();

    for (int i = 0; i < BUTTON_NUM; i++) {
      int press2s = pressFilters[i].filter(digitalRead(button_pin[i]));
      if (press2s == 0) {
        btL[i] = true;
        btLongPress = true;
        longBuz = true;
        longBuzBegin = millis();
        digitalWrite(buzzer_pin, HIGH);
      }
    }

  }

  // short buzzer close
  if ( (millis() - shortBuzBegin >= 100) && shortBuz )  {
    shortBuz = false;
    digitalWrite(buzzer_pin, LOW);
  }

  // long buzzer close
  if ( (millis() - longBuzBegin >= 1000) && longBuz)  {
    longBuz = false;
    digitalWrite(buzzer_pin, LOW);
  }

}





//////////////////////////////////////////////////////////////////////////////////////
/*  software System */

//  FSM 
void Arm7Bot::FSMdetector() {

  // Status 1
  if (FSM_Status == 1) {
    // goto status-3
    if (btS[0]) {
      btS[0] = false;
      FSM_Status = 3;
    }
    // useless
    if (btS[1]) {
      btS[1] = false;
    }
    // goto status-2
    if (btL[0]) {
      btL[0] = false;
      FSM_Status = 2;
    }
    // useless
    if (btL[1]) {
      btL[1] = false;
    }

  }

  // Status 2
  else if (FSM_Status == 2) {
    // goto status-3
    if (btS[0]) {
      btS[0] = false;
      FSM_Status = 3;
    }
    // add a pose
    if (btS[1]) {
      btS[1] = false;
      addPoseFlag = true;
    }
    // Clear poses
    if (btL[0]) {
      btL[0] = false;
      clearPoseFlag = true;
    }
    // add a grab/release pose
    if (btL[1]) {
      btL[1] = false;
      if (!isReleaseFlag) {
        isReleaseFlag = true;
        addGrabPoseFlag = true;
      }
      else {
        isReleaseFlag = false;
        addReleasePoseFlag = true;
      }
    }


  }

  // Status 3
  else if (FSM_Status == 3) {

    // goto status-3
    if (btS[0]) {
      btS[0] = false;
      FSM_Status = 3;
    }
    // useless
    if (btS[1]) {
      btS[1] = false;
    }
    // goto status-2
    if (btL[0]) {
      btL[0] = false;
      FSM_Status = 2;
    }
    // useless
    if (btL[1]) {
      btL[1] = false;
    }

  }

} 


// This fuction is design for general usage and software application developments 
void Arm7Bot::softwareSystem() {
  // UART receiver
  receiveCom();

  // 4ms
  if (millis() - time_4ms >= 4)  {
    time_4ms = millis();
    filterAnalogData();  // Analog Filter
  }

  // 33ms
  if (millis() - time_33ms >= 33)  {
    time_33ms = millis();
    calculatePosD();
    calculateForce();
    sendPosDAndForce();
  }


  // press button detection
  btDetectionAndBuz();
  FSMdetector();


  // status-1
  if (FSM_Status == 1) {

    //
    if (forceStatus == 0) {
      forcelessMode();
    }
    else if (forceStatus == 2) {
      stopMode();
    }
    else if (forceStatus == 1) {

      // IK Command
      if (F10) {
        F10 = false;

        //----
        int IK_status = IK6( j6, vec56, vec67 );
        if (IK_status == 0) {
          for (int i = 0; i < 6; i++) {
            posG[i] = degrees(theta[i]);
          }
          posG[6] = theta6;

        }
        else {
          // send IK error status alarm
          ARMPORT.write(0xFE);
          ARMPORT.write(0xFA);
          ARMPORT.write(0x01);
        }
        //----
      }
      else if (F11) {
        F11 = false;
        int IK_status = IK5( j6, vec56 );
        if (IK_status == 0) {
          for (int i = 0; i < 5; i++) {
            posG[i] = degrees(theta[i]);
          }
          posG[5] = theta5;
          posG[6] = theta6;
        }
        else {
          // send IK error status alarm
          ARMPORT.write(0xFE);
          ARMPORT.write(0xFB);
          ARMPORT.write(0x01);
        }

      }
      else if (F12) {
        F12 = false;
        int IK_status = IK3( j5 );
        if (IK_status == 0) {
          for (int i = 0; i < 3; i++) {
            posG[i] = degrees(theta[i]);
          }
          posG[3] = theta3;
          posG[4] = theta4;
          posG[5] = theta5;
          posG[6] = theta6;
        }
        else {
          // send IK error status alarm
          ARMPORT.write(0xFE);
          ARMPORT.write(0xFC);
          ARMPORT.write(0x01);
        }

      }

      if (millis() - time_20ms >= 20 )  {
        time_20ms = millis();
        moveOneStep();
        // Vacuum Cup
        if (allConverge()) {
          if (posG[6] < 30) {
            digitalWrite(valve_pin, LOW);
            digitalWrite(pump_pin, HIGH);
          } else {
            digitalWrite(valve_pin, HIGH);
            digitalWrite(pump_pin, LOW);
          }
        }
      }

    }

  }
  else if (FSM_Status == 2) {
    // initial robot arm
    if (pre_FSM_Status != 2) forcelessMode();

    // add a normal pose
    if (addPoseFlag) {
      addPoseFlag = false;
      calculatePosD();
      // for vacuum cup
      if (vacuumCupState == 1)   posD[6] = 0;
      else posD[6] = 80;
      // count pose number: MaxNum = 254
      if (poseCnt < 254) poseCnt++; ARMPORT.print("AddRecPose: "); ARMPORT.println(poseCnt);
      dueFlashStorage.write(256, (uint8_t)poseCnt);
      // store pose data
      int storeData[SERVO_NUM];
      for (int i = 0; i < SERVO_NUM; i++) {
        storeData[i] = (int)(posD[i] * 50 / 9);
      }
      for (int i = 0; i < SERVO_NUM; i++) {
        dueFlashStorage.write(256 + poseCnt * SERVO_NUM * 2 + 2 * i, (uint8_t)(storeData[i] / 128));
        dueFlashStorage.write(256 + poseCnt * SERVO_NUM * 2 + 2 * i + 1, (uint8_t)(storeData[i] % 128));
      }

    } // END- add a normal pose


    // add a grab pose
    if (addGrabPoseFlag) {
      addGrabPoseFlag = false;
      calculatePosD();
      Servos[6].attach( 2 + 6, 90, 2500);
      //
      double posTmp = 0;
      if (reverse[6]) {
        posTmp = 180 - posTmp;
      }
      servoPos[6] = posTmp + offset[6];
      Servos[6].writeMicroseconds( int(500 + servoPos[6] * (2500 - 500) / 180) );
      posD[6] = 0;

      if (poseCnt < 254) poseCnt++; ARMPORT.print("AddGrabPose: "); ARMPORT.println(poseCnt);
      dueFlashStorage.write(256, (uint8_t)poseCnt);
      int storeData[SERVO_NUM];
      for (int i = 0; i < SERVO_NUM; i++) {
        storeData[i] = (int)(posD[i] * 50 / 9);
      }
      for (int i = 0; i < SERVO_NUM; i++) {
        dueFlashStorage.write(256 + poseCnt * SERVO_NUM * 2 + 2 * i, (uint8_t)(storeData[i] / 128));
        dueFlashStorage.write(256 + poseCnt * SERVO_NUM * 2 + 2 * i + 1, (uint8_t)(storeData[i] % 128));
      }

      digitalWrite(valve_pin, LOW);
      digitalWrite(pump_pin, HIGH);
      vacuumCupState = 1;

    } // END- add a grab pose


    // add a release pose
    if (addReleasePoseFlag) {
      addReleasePoseFlag = false;
      calculatePosD();
      //
      double posTmp = 75;
      if (reverse[6]) {
        posTmp = 180 - posTmp;
      }
      servoPos[6] = posTmp + offset[6];
      Servos[6].writeMicroseconds( int(500 + servoPos[6] * (2500 - 500) / 180) );
      posD[6] = 0;

      posD[6] = 75;

      if (poseCnt < 254) poseCnt++; ARMPORT.print("AddReleasePose: "); ARMPORT.println(poseCnt);
      dueFlashStorage.write(256, (uint8_t)poseCnt);
      // store pose data
      int storeData[SERVO_NUM];
      for (int i = 0; i < SERVO_NUM; i++) {
        storeData[i] = (int)(posD[i] * 50 / 9);
      }
      for (int i = 0; i < SERVO_NUM; i++) {
        dueFlashStorage.write(256 + poseCnt * SERVO_NUM * 2 + 2 * i, (uint8_t)(storeData[i] / 128));
        dueFlashStorage.write(256 + poseCnt * SERVO_NUM * 2 + 2 * i + 1, (uint8_t)(storeData[i] % 128));
      }

      digitalWrite(valve_pin, HIGH);
      digitalWrite(pump_pin, LOW);
      vacuumCupState = 0;

    } // END- add a release pose

    // clear the poses
    if (clearPoseFlag) {
      clearPoseFlag = false; ARMPORT.println("Clear Poses");
      isReleaseFlag = false;
      poseCnt = 0;
      dueFlashStorage.write(256, (uint8_t)poseCnt);
    }

  }
  else if (FSM_Status == 3) {
    if (pre_FSM_Status != 3) {
      initialMove();
      playCnt = 1;
      time_1000ms = 0;  
    }

    // move one pose per second
    if (millis() - time_1000ms >= 1000)  {
      time_1000ms = millis();
      //
      if (poseCnt != 0) {
        if (playCnt > poseCnt) playCnt = 1; ARMPORT.print("PlayPose: "); ARMPORT.println(playCnt);
        //read stored datas
        int storeData[SERVO_NUM];
        for (int i = 0; i < SERVO_NUM; i++) {
          storeData[i] = (int)dueFlashStorage.read(256 + playCnt * SERVO_NUM * 2 + 2 * i) * 128 +
                         (int)dueFlashStorage.read(256 + playCnt * SERVO_NUM * 2 + 2 * i + 1);
        }

        for (int i = 0; i < SERVO_NUM; i++) {
          posG[i] = (double)storeData[i] * 9 / 50;
          isConverge[i] = false;
        }
        playCnt++;

      }
    }  // END- 1000ms

    if (millis() - time_20ms >= 20 )  {
      time_20ms = millis();
      if (poseCnt != 0) {
        moveOneStep();
        // vacuum cup
        if (allConverge()) {
          if (posG[6] < 30) {
            digitalWrite(valve_pin, LOW);
            digitalWrite(pump_pin, HIGH);
          } else {
            digitalWrite(valve_pin, HIGH);
            digitalWrite(pump_pin, LOW);
          }
        }
      }
    }

  }  // END_FSM-status3

  // If external forces too high for a while: FSM_Status = 4;
  // This protection mechanism should be test for a while... 
  else if (FSM_Status == 4) {
    if (pre_FSM_Status != 4) stopMode();

  }
  
  
  pre_FSM_Status = FSM_Status;

}  

