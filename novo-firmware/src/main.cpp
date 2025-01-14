#include "main.h"
#include "base_stm.h"
#include "radio_stm.h"
#include "kicker_stm.h"

// Select the stm you're flashing
// has_radio = 0 STM with kicker
// has_radio = 1 STM with radio
#define has_radio 0

BaseSTM *stmObject = nullptr;

BLDCDriver3PWM *driver1 = nullptr;
BLDCDriver3PWM *driver2 = nullptr;
uint8_t en1, en2;

// Function declarations:

void setup()
{
  if (has_radio)
  {
    HardwareSerial *stmSerial = new HardwareSerial(PB11, PB10);

    RF24 *radio = new RF24(PB0, PA4);

    driver1 = new BLDCDriver3PWM(PA10, PA9, PA8);
    driver2 = new BLDCDriver3PWM(PB9, PB8, PB7);
    en1 = PB15;
    en2 = PB6;

    stmObject = new RadioSTM(stmSerial, radio, driver1, en1, driver2, en2);
  }
  else
  {
    HardwareSerial *stmSerial = new HardwareSerial(PB11, PB10);

    driver1 = new BLDCDriver3PWM(PA10, PA9, PA8);
    driver2 = new BLDCDriver3PWM(PB9, PB8, PB7);
    en1 = PB15;
    en2 = PB6;

    stmObject = new KickerSTM(stmSerial, driver1, en1, driver2, en2, KICKER_PIN, INFRARED_PIN);
  }
}

void loop()
{
  stmObject->run();
}