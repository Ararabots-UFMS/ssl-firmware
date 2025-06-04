#ifndef __BASESTM_H
#define __BASESTM_H

#include "main.h"

class BaseSTM
{
private:
public:
    HardwareSerial *serial;

    float *result;

    union
    {
        float floatingP;
        uint8_t binary[4];
    } binaryFloat;

    BLDCMotor *motor1 = new BLDCMotor(POLE_PAIR_COUNT, PHASE_RESISTANCE, KV_RATING);
    BLDCMotor *motor2 = new BLDCMotor(POLE_PAIR_COUNT, PHASE_RESISTANCE, KV_RATING);

    BLDCDriver3PWM *driver1;
    BLDCDriver3PWM *driver2;

    uint8_t en1;
    uint8_t en2;

    BaseSTM(/* args */);
    virtual ~BaseSTM();
    virtual void run();
};

#endif