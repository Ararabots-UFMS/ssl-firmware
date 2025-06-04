#ifndef __RADIOSTM_H
#define __RADIOSTM_H

#include <math.h>
#include <RF24.h>

#include "main.h"
#include "base_stm.h"

class RadioSTM : public BaseSTM
{
private:
    float jacobian[4][3];

    RF24 *radio;

    float vx = 0.0;
    float vy = 1.0;
    float vt = 0.0;
    uint8_t kik_sig = 1;

    HardwareSerial *uart;

public:
    RadioSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t en1, BLDCDriver3PWM *d2, uint8_t en2);
    ~RadioSTM();
    void readRadio();
    void getWheelSpeeds();
    void sendSerialMsg();
    void move();
    void run();
};

#endif