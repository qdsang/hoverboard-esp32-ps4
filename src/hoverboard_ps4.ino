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

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define PS4_MAC_ADDRESS  "3c:22:fb:57:55:fc"

// https://github.com/xman4242/PS4-ESP32-Bluetooth
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
#include <iostream>
#include <sstream>

#if CONFIG_IDF_TARGET_ESP32C3
#include <SoftwareSerial.h>
static const int Serial2RXPin = 9, Serial2TXPin = 10;
SoftwareSerial Serial2(Serial2RXPin, Serial2TXPin);
#endif

#define HoverSerial Serial2        // RX 16, TX 17


// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

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
SerialFeedback Feedback;
SerialFeedback NewFeedback;


//Heartbeat
unsigned long heartbeat_interval = -1;
unsigned long heartbeat_time = 0;

unsigned long lastTimeStamp = 0;

void printDeviceAddress()
{ 
  const uint8_t* address = esp_bt_dev_get_address();
  char str[100];
  sprintf(str, "ESP32's Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x", address[0],address[1],address[2],address[3],address[4],address[5]);
  Serial.println(str);

  uint8_t pairedDeviceBtAddr[20][6];  
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  Serial.printf("Bind Count: %d\n", count);
  for(int i = 0; i < count; i++) 
  {
    address = pairedDeviceBtAddr[i];
    sprintf(str, "Bind MAC address is - %02x:%02x:%02x:%02x:%02x:%02x", address[0],address[1],address[2],address[3],address[4],address[5]);
    Serial.println(str);
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

uint8_t ctrl_mode;
float speedCoef = 1.0;
float steerCoef = 1.0;

bool ps4_Touchpad_pressed = false;
bool ps4_Triangle_pressed = false;
bool ps4_Cross_pressed = false;
bool ps4_Circle_pressed = false;
bool ps4_Square_pressed = false;

void setPS4CtrlMode(uint8_t mode) {
  ctrl_mode = mode;
  if (ctrl_mode <= 0 || ctrl_mode > 3) {
    ctrl_mode = 1;
  }
  if (ctrl_mode == 1) {
    PS4.setLed(0xff, 0, 0);
  } else if (ctrl_mode == 2) {
    PS4.setLed(0, 0xff, 0);
  } else if (ctrl_mode == 3) {
    PS4.setLed(0, 0, 0xff);
  } else {
    PS4.setLed(0, 0, 0);
  }
  PS4.sendToController();

  Serial.print("ctrl_mode : ");
  Serial.println(ctrl_mode);
}

// 7ms update loop
void notify()
{
  char messageString[200];
  sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  PS4.LStickX(),
  PS4.LStickY(),
  PS4.RStickX(),
  PS4.RStickY(),
  PS4.Left(),
  PS4.Down(),
  PS4.Right(),
  PS4.Up(),
  PS4.Square(),     // 方块
  PS4.Cross(),      // 叉号
  PS4.Circle(),     // 圆圈
  PS4.Triangle(),   // 三角形
  PS4.L1(),
  PS4.R1(),
  PS4.L2(),
  PS4.R2(),  
  PS4.Share(),
  PS4.Options(),
  PS4.PSButton(),
  PS4.Touchpad(),
  PS4.Charging(),
  PS4.Audio(),
  PS4.Mic(),
  PS4.Battery());

  if (PS4.Touchpad() && !ps4_Touchpad_pressed) {
    ps4_Touchpad_pressed = true;
    setPS4CtrlMode(ctrl_mode+1);
  } else if (!PS4.Touchpad()) {
    ps4_Touchpad_pressed = false;
  }

  if (PS4.Triangle() && !ps4_Triangle_pressed) {
    ps4_Triangle_pressed = true;
    speedCoef += 0.1;
    if (speedCoef > 2) {
      speedCoef = 2;
    }
  } else if (!PS4.Triangle()) {
    ps4_Triangle_pressed = false;
  }

  if (PS4.Cross() && !ps4_Cross_pressed) {
    ps4_Cross_pressed = true;
    speedCoef -= 0.1;
    if (speedCoef < 0.1) {
      speedCoef = 0.1;
    }
  } else if (!PS4.Cross()) {
    ps4_Cross_pressed = false;
  }

  if (PS4.Circle() && !ps4_Circle_pressed) {
    ps4_Circle_pressed = true;
    steerCoef += 0.1;
    if (steerCoef > 2) {
      steerCoef = 2;
    }
  } else if (!PS4.Circle()) {
    ps4_Circle_pressed = false;
  }
  if (PS4.Square() && !ps4_Square_pressed) {
    ps4_Square_pressed = true;
    steerCoef -= 0.1;
    if (steerCoef < 0.1) {
      steerCoef = 0.1;
    }
  } else if (!PS4.Square()) {
    ps4_Square_pressed = false;
  }

  int16_t uSteer;
  int16_t uSpeed;
  if (ctrl_mode == 3) {
    int16_t l2 = PS4.L2Value();
    int16_t r2 = PS4.R2Value();
    uSteer = map(abs(l2 - r2), 0, 255, 0, 500) * ((l2 > r2) ? -1 : 1);
    if (abs(l2) > 10 && abs(r2) > 10) {
      uSpeed = -map(max(l2, r2), 0, 255, 0, 500);
    } else {
      uSpeed = 0;
    }
  } else if (ctrl_mode == 2) {
    uSteer = PS4.LStickX();
    uSpeed = PS4.LStickY();
    uSteer = map(uSteer, -128, 128, -500, 500);
    uSpeed = -map(uSpeed, -128, 128, -500, 500);
  } else {
    uSteer = PS4.RStickX();
    uSpeed = PS4.LStickY();
    uSteer = map(uSteer, -128, 128, -500, 500);
    uSpeed = -map(uSpeed, -128, 128, -500, 500);
  }

  if (abs(uSteer) < 10) {
    uSteer = 0;
  }
  if (abs(uSpeed) < 10) {
    uSpeed = 0;
  }

  if (uSteer == 0 && uSpeed == 0) {
    int16_t l2 = PS4.L2Value();
    int16_t r2 = PS4.R2Value();
    uSteer = map(abs(l2 - r2), 0, 255, 0, 500) * ((l2 > r2) ? -1 : 1);
    if (abs(l2) > 10 && abs(r2) > 10) {
      uSpeed = -map(max(l2, r2), 0, 255, 0, 500);
    } else {
      uSpeed = 0;
    }
  }

  uSteer = uSteer * steerCoef;
  uSpeed = uSpeed * speedCoef;

  Send(uSteer, uSpeed);
}

void onConnect()
{
  Serial.println("Connected!.");
  setPS4CtrlMode(1);
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void initPS4() {
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  // https://github.com/xman4242/PS4-ESP32-Bluetooth#pairing-the-ps4-controller
  // bool isOK = PS4.begin("3c:71:bf:d1:a0:a2");
  bool isOK = PS4.begin(PS4_MAC_ADDRESS);
  Serial.printf("initPS4: %d\n", isOK);
}

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

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));

  heartbeat_time = millis();
  // Serial.printf("[STATS2] %d mode: %d \t steer: %d \t speed:%d\n", heartbeat_time, ctrl_mode, Command.steer, Command.speed);   

}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial  100Hz
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
            // 1: 0 2: -85 3: 0 4: 0 5: 4055 6: 408 7: 0
            // 1: 0 2: -44 3: 10 4: -7 5: 4055 6: 408 7: 0
            // 1: -32 2: -138 3: 0 4: -10 5: 4055 6: 410 7: 0
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
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
