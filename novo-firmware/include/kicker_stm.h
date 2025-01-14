#ifndef __KICKERSTM_H
#define __KICKERSTM_H

#include "base_stm.h"

#define RPM_TO_RADS 0.10472

class KickerSTM : public BaseSTM
{
private:
    uint8_t kik_sig = 0;

    uint8_t kickerPin;
    uint8_t infraRedPin;

    unsigned long last_kick_time = 0;
    uint8_t kicker_activated = 0;

public:
    KickerSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2, uint8_t kp, uint8_t irp);
    ~KickerSTM();
    void readSerialMsg();
    void move();
    void kick();
    void deactivateKick();
    void run();
};

#endif