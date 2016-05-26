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
#ifndef _ARM7BOT_H
#define _ARM7BOT_H

#include "Arduino.h"
#include <Servo.h>
#include <DueFlashStorage.h>

#include "PVector.h"
#include "ForceFilter.h"
#include "MedianFilter.h"
#include "PressFilter.h"

const int BAUD_RATE = 115200;

/* Serial port for communication */
#define ARMPORT Serial

/* Arm7Bot parameters */
#define SERVO_NUM 7
const int INITIAL_POS[SERVO_NUM]= {90, 115, 65, 90, 90, 90, 75};
const int fluentRangeInit[SERVO_NUM] = {5, 5, 5, 5, 5, 5, 5};
const boolean reverse[SERVO_NUM] = {true, false, false, false, false, false, true};
const double offsetInit[SERVO_NUM] = {0, 0, 0, 0, 0, 0, -50.0};  // Unit: Degree
const double thetaMin[SERVO_NUM] = { 0,  0, -1.134464,  0.17453292,  0,  0, 0};
const double thetaMax[SERVO_NUM] = {PI, PI, 2.0071287, 2.9670596, PI, PI, PI/2};
const double a=120.0, b=40.0, c=198.50, d=30.05, e=77.80, f=22.10, g=12.0, h = 29.42;

/* Btn & Buzzer */
#define BUTTON_NUM 2
const int button_pin[BUTTON_NUM] = {71, 70};
const int buzzer_pin = 12;


class Arm7Bot {

  private:
    // Kinematics & IK
    PVector joint[9];
    int angleRangCheck();
    void calcJoints();
    PVector arbitraryRotate(PVector point, PVector pointA, PVector pointB, double _angle);
    PVector zAxiRotate(PVector point, double _angle);
    PVector calcProjectionPt(PVector pt0, PVector pt1, PVector nVec);

    //UART receiver
    int dataBuf[60];
    bool beginFlag = false;
    bool haveInstruction = false;
    int instruction = 0;
    int cnt = 0;

    // Btn & Buzzer
    unsigned long time_300ms = 0;
    bool last_reading[BUTTON_NUM];
    bool reading[BUTTON_NUM];
    unsigned long last_debounce_time[BUTTON_NUM] = {0, 0};
    unsigned long debounce_delay = 50;
    // button state buffers
    bool last_state[BUTTON_NUM];
    // Long press detection
    PressFilter pressFilters[BUTTON_NUM];
    bool btLongPress = false;
    // buzzer delay
    bool shortBuz = false;
    bool longBuz = false;
    unsigned long shortBuzBegin = 0;
    unsigned long longBuzBegin = 0;
    // Press Pressed
    bool btS[BUTTON_NUM];
    bool btL[BUTTON_NUM];
    void btAndBuzInit();
    void btDetectionAndBuz();

    // IK recieve
    PVector j5, j6, vec56, vec67;
    double theta3, theta4, theta5, theta6;


    //////////////////////////////////// FSM
    int FSM_Status = 1;
    int pre_FSM_Status = 1;
    bool addPoseFlag = false;
    bool isReleaseFlag = false;
    bool addGrabPoseFlag = false;
    bool addReleasePoseFlag = false;
    bool clearPoseFlag = false;
    void FSMdetector();

    ///////////////////////////////// softwareSystem
    // timer
    unsigned long time_4ms = 0;
    unsigned long time_20ms = 0;
    unsigned long time_33ms = 0;
    unsigned long time_1000ms = 0;
    // FSM_status3, playCnt
    int playCnt = 1;
    // Vaccum Cup
    int valve_pin = 10;
    int pump_pin = 11;
    int vacuumCupState = 0;  // 0-release, 1-grab
    void vacuumCupInit();

    // Hardware pose record
    int poseCnt = 0;

    // Flash read & writre
    DueFlashStorage dueFlashStorage;
    void getStoreData();
    void setStoreData();
     //
    int F2_id = 0;
    bool F2 = false; bool F4 = false; bool F6 = false; bool F8 = false;bool F10 = false; bool F11 = false; bool F12 = false;


    // Servo objects
    int forceStatus = 1;     // 0-forceless, 1-normal servo, 2-protection
    Servo Servos[SERVO_NUM];
    // Geometrical positions
    double posG[SERVO_NUM];  // Goal position, receive from PC
    double pos[SERVO_NUM];   // Control position
    double posS[SERVO_NUM];  // Start position
    double posD[SERVO_NUM];  // Detected position
    
    // servo motor positions
    double servoPos[SERVO_NUM];  // control position
    double servoPosD[SERVO_NUM]; // Detected position, calculate from analog signal
    // filter
    int filterData[SERVO_NUM];
    MedianFilter filters[SERVO_NUM];
    ForceFilter forceFilters[SERVO_NUM];
    int force[SERVO_NUM];
    void calculateForce();

    void moveOneStep();
    void geometryConstrain();
    void servoCtrl();
    void filterAnalogData();
    void calculateServoPosD();
    void calculatePosD();
    void sendPosDAndForce();
    boolean isConverge[SERVO_NUM];
    int fluentRange[SERVO_NUM];

    // IK
    double theta[SERVO_NUM];  // angles
    int IK3(PVector pt);
    int IK5(PVector j6, PVector vec56_d);
    int IK6(PVector j6, PVector vec56_d, PVector vec67_d);

    // UART
    void receiveCom();

  public:
    // Servo state
    double maxSpeed[SERVO_NUM];    // Unit: degrees/second
    boolean isFluent[SERVO_NUM];
    double offset[SERVO_NUM];      // assembly offsets

    Arm7Bot();

    boolean allConverge();
    void servoMode(int mode);
    void initialMove();
    void forcelessMode();
    void normalMode();
    void stopMode();
    void move(double angles[SERVO_NUM]);
    void moveIK3(PVector j5);
    void moveIK5(PVector j6, PVector vec56_d);
    void moveIK6(PVector j6, PVector vec56_d, PVector vec67_d);
    void softwareSystem();

};
#endif
