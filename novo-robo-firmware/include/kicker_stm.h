#ifndef __KICKERSTM_H
#define __KICKERSTM_H

#include "base_stm.h"

class KickerSTM : public BaseSTM
{
private:
    uint8_t kik_sig = 0;

    unsigned long last_kick_time = 0;
    uint8_t kicker_activated = 0;

    // HardwareSerial *uart;

    uint32_t lastTime;

public:
    KickerSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2);
    ~KickerSTM();
    void readSerialMsg();
    void move();
    void kick();
    void deactivateKick();
    void run();
};

#endif