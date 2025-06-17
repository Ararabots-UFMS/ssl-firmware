#include "main.h"
#include "base_stm.h"
#include "radio_stm.h"
#include "kicker_stm.h"

BaseSTM *stmObject = nullptr;
BLDCDriver3PWM *driver1 = nullptr;
BLDCDriver3PWM *driver2 = nullptr;
uint8_t en1, en2;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Even if you're just flashing the firmware                                                    //
// there are probably some changes that need to be made in main.h                               //
//////////////////////////////////////////////////////////////////////////////////////////////////
// Check the robot name                                                                         //
// Check the robot index                                                                        //
// Select the correct STM type (with or without radio).                                         //
//////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  if (HAS_RADIO == 1)
  {
    HardwareSerial *stmSerial = new HardwareSerial(SERIAL_RX, SERIAL_TX); // RX, TX

    driver1 = new BLDCDriver3PWM(RSTM_M1_A, RSTM_M1_B, RSTM_M1_C);
    driver2 = new BLDCDriver3PWM(RSTM_M2_A, RSTM_M2_B, RSTM_M2_C);
    en1 = RSTM_M1_EN;
    en2 = RSTM_M2_EN;

    stmObject = new RadioSTM(stmSerial, driver1, en1, driver2, en2);
  }
  else // HAS KICKER
  {
    HardwareSerial *stmSerial = new HardwareSerial(SERIAL_RX, SERIAL_TX); // RX, TX

    driver1 = new BLDCDriver3PWM(KSTM_M1_A, KSTM_M1_B, KSTM_M1_C);
    driver2 = new BLDCDriver3PWM(KSTM_M2_A, KSTM_M2_B, KSTM_M2_C);
    en1 = KSTM_M1_EN;
    en2 = KSTM_M2_EN;

    stmObject = new KickerSTM(stmSerial, driver1, en1, driver2, en2);
  }
}

void loop()
{
  stmObject->run();
}