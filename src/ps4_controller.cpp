#include "ps4_controller.h"
#include "hoverboard.h"
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
#include <iostream>
#include <sstream>

// Declare external function from hoverboard.cpp
extern void Send(int16_t uSteer, int16_t uSpeed);

// PS4 controller state variables
uint8_t ctrl_mode = CTRL_MODE_DEFAULT;
float speedCoef = INITIAL_SPEED_COEF;
float steerCoef = INITIAL_STEER_COEF;

bool ps4_Touchpad_pressed = false;
bool ps4_Triangle_pressed = false;
bool ps4_Cross_pressed = false;
bool ps4_Circle_pressed = false;
bool ps4_Square_pressed = false;

// Time tracking variables
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

void setPS4CtrlMode(uint8_t mode) {
  ctrl_mode = mode;
  if (ctrl_mode <= 0 || ctrl_mode > CTRL_MODE_MAX) {
    ctrl_mode = CTRL_MODE_DEFAULT;
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
  // Update heartbeat time
  heartbeat_time = millis();
  
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
    speedCoef += COEF_INCREMENT;
    if (speedCoef > MAX_COEF) {
      speedCoef = MAX_COEF;
    }
  } else if (!PS4.Triangle()) {
    ps4_Triangle_pressed = false;
  }

  if (PS4.Cross() && !ps4_Cross_pressed) {
    ps4_Cross_pressed = true;
    speedCoef -= COEF_INCREMENT;
    if (speedCoef < MIN_COEF) {
      speedCoef = MIN_COEF;
    }
  } else if (!PS4.Cross()) {
    ps4_Cross_pressed = false;
  }

  if (PS4.Circle() && !ps4_Circle_pressed) {
    ps4_Circle_pressed = true;
    steerCoef += COEF_INCREMENT;
    if (steerCoef > MAX_COEF) {
      steerCoef = MAX_COEF;
    }
  } else if (!PS4.Circle()) {
    ps4_Circle_pressed = false;
  }
  if (PS4.Square() && !ps4_Square_pressed) {
    ps4_Square_pressed = true;
    steerCoef -= COEF_INCREMENT;
    if (steerCoef < MIN_COEF) {
      steerCoef = MIN_COEF;
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

  if (abs(uSteer) < JOYSTICK_DEADZONE) {
    uSteer = 0;
  }
  if (abs(uSpeed) < JOYSTICK_DEADZONE) {
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

  if (abs(uSteer) < JOYSTICK_DEADZONE) {
    uSteer = 0;
  }
  if (abs(uSpeed) < JOYSTICK_DEADZONE) {
    uSpeed = 0;
  }
  
  uSteer = uSteer * steerCoef;
  uSpeed = uSpeed * speedCoef;

  Send(uSteer, uSpeed);
}

void onConnect()
{
  Serial.println("Connected!.");
  setPS4CtrlMode(CTRL_MODE_DEFAULT);
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