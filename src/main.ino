// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

#include "config.h"
#include "ps4_controller.h"
#include "hoverboard.h"

// External variables from other modules
extern uint8_t ctrl_mode;
extern unsigned long heartbeat_time;
extern unsigned long lastTimeStamp;
extern SerialCommand Command;

// Heartbeat
unsigned long heartbeat_interval = HEARTBEAT_INTERVAL;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  initPS4();
  printDeviceAddress();
  delay(1500);

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  heartbeat_interval = 200;
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);
}


// ########################## LOOP ##########################

void loop(void)
{ 
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  uint8_t println = 0;
  //Only needed to print the message properly on serial monitor. Else we dont need it.
  if (timeNow - lastTimeStamp > 100)
  {
    // Serial.println(messageString);
    Serial.printf("[STATS] %d mode: %d \t steer: %d \t speed:%d", heartbeat_time, ctrl_mode, Command.steer, Command.speed);   
    lastTimeStamp = millis();
    println = 1;
  }

  if ((millis() - heartbeat_time) >= heartbeat_interval) {
    Send(0, 0);
    Serial.print("[heartbeat] Timeout! ");
    println = 1;
  }

  if (println)
    Serial.println();
  
  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
