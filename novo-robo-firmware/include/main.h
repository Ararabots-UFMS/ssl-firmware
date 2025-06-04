#ifndef __MAIN_H
#define __MAIN_H

#include <Arduino.h>
#include <SimpleFOC.h>

#include <math.h>

////////////////////////////////// Robot Configuration //////////////////////////////////
// Most changes should be made in these defines.

// Robot names should start with a capital adn unique letter.
inline constexpr const char *ROBOT_NAMES[2] = {"Jare", "Kenzo"};
// Index of the robot name in the ROBOT_NAMES array
#define ROBOT_INDEX 0
// The first letter of the name is used to identify the intended reciever of radio messages.
#define ROBOT_NAME ROBOT_NAMES[ROBOT_INDEX][0] // DO NOT CHANGE THIS LINE

// Select the stm you're flashing
// has_radio = 0 STM with kicker
// has_radio = 1 STM with radio
#define HAS_RADIO 0

////////////////////////////////// Base STM Variables //////////////////////////////////
#define LED_PIN PC13

#define RPM_TO_RADS 0.10472

// These values should only change when structure is changed
#define WHEEL_RADIUS 0.034 // Wheel radius in meters
#define ROBOT_RADIUS 0.072 // Robot radius in meters
inline constexpr float WHEEL_ANGLES[4] = {M_PI * (1.0 / 6.0), M_PI * (5.0 / 6.0), M_PI * (5.0 / 4.0), M_PI * (7.0 / 4.0)};

// These values should only change when the motor is changed
#define POLE_PAIR_COUNT 11      // Motor pole pair count
#define PHASE_RESISTANCE 11     // Phase resistance in ohms
#define KV_RATING 60            // Motor KV rating in rpm/V
#define VOLTAGE_POWER_SUPPLY 20 // Motor power supply voltage in volts
#define MAX_RPM 600             // Maximum motor RPM

#define SERIAL_BUFFER_SIZE 9     // Size of the serial buffer
#define SERIAL_BAUDRATE 230400   // Serial baud rate
#define SERIAL_CONFIG SERIAL_8E1 // Serial configuration
#define SERIAL_RX PB11           // Serial RX pin
#define SERIAL_TX PB10           // Serial TX pin

////////////////////////////////// Radio STM Variables //////////////////////////////////
#define ADDRESS 0xFAB10FAB10LL // Address for the radio communication

#define RADIO_CE PB0  // CE pin for the radio module
#define RADIO_CSN PA4 // CSN pin for the radio module

#define RSTM_M1_A PA10  // Motor 1 A pin
#define RSTM_M1_B PA9   // Motor 1 B pin
#define RSTM_M1_C PA8   // Motor 1 C pin
#define RSTM_M1_EN PB15 // Motor 1 enable pin

#define RSTM_M2_A PB9  // Motor 2 A pin
#define RSTM_M2_B PB8  // Motor 2 B pin
#define RSTM_M2_C PB7  // Motor 2 C pin
#define RSTM_M2_EN PB6 // Motor 2 enable pin

////////////////////////////////// Kicker STM Variables //////////////////////////////////
#define INFRARED_PIN PB0

#define KICKER_PIN PB12
#define KICKER__GPIO_PORT GPIOB
#define KICKER_GPIO_PIN GPIO_PIN_12

#define KICK_DURATION 5
#define KICK_COOLDOWN 500

#define KSTM_M1_A PA10  // Motor 1 - phase A pin
#define KSTM_M1_B PA9   // Motor 1 - phase B pin
#define KSTM_M1_C PA8   // Motor 1 - phase C pin
#define KSTM_M1_EN PB15 // Motor 1 - enable pin

#define KSTM_M2_A PB9  // Motor 2 - phase A pin
#define KSTM_M2_B PB8  // Motor 2 - phase B pin
#define KSTM_M2_C PB7  // Motor 2 - phase C pin
#define KSTM_M2_EN PB6 // Motor 2 - enable pin

#endif