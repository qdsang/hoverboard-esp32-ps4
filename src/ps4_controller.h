#ifndef PS4_CONTROLLER_H
#define PS4_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

// PS4 controller state variables
extern uint8_t ctrl_mode;
extern float speedCoef;
extern float steerCoef;

extern unsigned long heartbeat_time;
extern unsigned long lastTimeStamp;


// Function declarations
void setPS4CtrlMode(uint8_t mode);
void notify();
void onConnect();
void onDisConnect();
void initPS4();
void printDeviceAddress();

#endif