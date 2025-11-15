#ifndef CONFIG_H
#define CONFIG_H

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD      // [-] Start frame definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step

// PS4 Controller MAC Address
#define PS4_MAC_ADDRESS     "3c:22:fb:57:55:fc"

// Control coefficients
#define INITIAL_SPEED_COEF  1.0         // [-] Initial speed coefficient
#define INITIAL_STEER_COEF  1.0         // [-] Initial steering coefficient
#define COEF_INCREMENT      0.1         // [-] Coefficient increment/decrement step
#define MIN_COEF            0.1         // [-] Minimum coefficient value
#define MAX_COEF            2.0         // [-] Maximum coefficient value

// Deadzone for joystick
#define JOYSTICK_DEADZONE   10          // [-] Deadzone value for joystick inputs

// Heartbeat timeout
#define HEARTBEAT_INTERVAL  200         // [ms] Heartbeat interval for communication timeout

// Control modes
#define CTRL_MODE_DEFAULT   1           // [-] Default control mode
#define CTRL_MODE_MAX       3           // [-] Maximum control mode number

// Pin definitions
#if CONFIG_IDF_TARGET_ESP32C3
#define SERIAL2_RX_PIN      9           // [-] Serial2 RX pin for ESP32-C3
#define SERIAL2_TX_PIN      10          // [-] Serial2 TX pin for ESP32-C3
#endif

// Debug options
// #define DEBUG_RX                     // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#endif