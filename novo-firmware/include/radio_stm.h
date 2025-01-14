#ifndef __RADIOSTM_H
#define __RADIOSTM_H

#include <math.h>
#include <RF24.h>

#include "main.h"
#include "base_stm.h"

#define WHEEL_RADIUS 0.034

#define ROBOT_RADIUS 0.072

class RadioSTM : public BaseSTM
{
private:
    // Wheel angles are fixed and should only change when structure is changed
    float wheel_angles[4] = {M_PI * (1.0 / 6.0), M_PI * (5.0 / 6.0), M_PI * (5.0 / 4.0), M_PI * (7.0 / 4.0)};
    float jacobian[4][3];

    RF24 *radio;

    float vx = 0.0;
    float vy = 0.0;
    float vt = 0.0;
    uint8_t kik_sig = 0;

    const uint64_t address = 0xFAB10FAB10LL;

public:
    RadioSTM(HardwareSerial *s, RF24 *r, BLDCDriver3PWM *d1, uint8_t en1, BLDCDriver3PWM *d2, uint8_t en2);
    ~RadioSTM();
    void readRadio();
    void getWheelSpeeds();
    void sendSerialMsg();
    void move();
    void run();
};

#endif