#ifndef __MAIN_H
#define __MAIN_H

#include <Arduino.h>
#include <SimpleFOC.h>

// Variable declarations:
#define LED_PIN PC13

#define INFRARED_PIN PB0

#define KICKER_PIN PB12
#define KICKER__GPIO_PORT GPIOB
#define KICKER_GPIO_PIN GPIO_PIN_12

#define KICK_DURATION 5
#define KICK_COOLDOWN 500

// Motor definition:
#define POLE_PAIR_COUNT 11
#define PHASE_RESISTANCE 11
#define KV_RATING 60
#define VOLTAGE_POWER_SUPPLY 20

#endif