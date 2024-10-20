#ifndef CONFIG_HPP
#define CONFIG_HPP

// Pin Definitions
#define SPEED_PWM_PIN 9
#define BRAKE_RPWM 10
#define BRAKE_LPWM 11
#define BRAKE_R_EN 22
#define BRAKE_L_EN 23
#define POT_PIN A0

// GPS Serial pins (for Arduino Mega)
#define GPS_RX_PIN 19
#define GPS_TX_PIN 18

// Buffer Lengths
#define MAX_BUFF_LEN 255

// Other Constants
#define SERIAL_BAUD_RATE 115200
#define GPS_BAUD_RATE 9600

// Task Frequencies (in milliseconds)
#define SERIAL_READ_FREQUENCY 5
#define CONTROL_TASK_FREQUENCY 10
#define SERIAL_OUTPUT_FREQUENCY 100
#define GPS_UPDATE_FREQUENCY 50

// Task Stack Sizes
#define SERIAL_TASK_STACK_SIZE 256
#define CONTROL_TASK_STACK_SIZE 256
#define OUTPUT_TASK_STACK_SIZE 256
#define GPS_TASK_STACK_SIZE 512

// Task Priorities
#define SERIAL_TASK_PRIORITY 3
#define CONTROL_TASK_PRIORITY 2
#define OUTPUT_TASK_PRIORITY 1
#define GPS_TASK_PRIORITY 1

#endif // CONFIG_HPP