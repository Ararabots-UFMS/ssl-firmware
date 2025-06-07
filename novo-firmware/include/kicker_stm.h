#ifndef __KICKERSTM_H
#define __KICKERSTM_H

#include "base_stm.h"
#include "main.h"

#include <math.h>
#include <RF24.h>

class KickerSTM : public BaseSTM
{
private:
    float jacobian[4][3];

    RF24 *radio;

    float vx = 0.0;
    float vy = 1.0;
    float vt = 0.0;
    uint8_t kik_sig = 1;

public:
    KickerSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2, uint8_t kp, uint8_t irp);
    ~KickerSTM();
    void getWheelSpeeds();
    void move();
    void run();
};

#endif