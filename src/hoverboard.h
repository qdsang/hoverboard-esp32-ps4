#ifndef HOVERBOARD_H
#define HOVERBOARD_H

#include <Arduino.h>
#include "config.h"

#if CONFIG_IDF_TARGET_ESP32C3
#include <SoftwareSerial.h>
static const int Serial2RXPin = SERIAL2_RX_PIN, Serial2TXPin = SERIAL2_TX_PIN;
SoftwareSerial Serial2(Serial2RXPin, Serial2TXPin);
#endif

#define HoverSerial Serial2             // RX 16, TX 17

// Data structures
typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;

// Global variables
extern SerialCommand Command;
extern SerialFeedback Feedback;
extern SerialFeedback NewFeedback;

// Function declarations
void Send(int16_t uSteer, int16_t uSpeed);
void Receive();

#endif