#include "kicker_stm.h"

KickerSTM::KickerSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2, uint8_t kp, uint8_t irp)
{
    serial = s;
    driver1 = d1;
    en1 = e1;
    driver2 = d2;
    en2 = e2;

    kickerPin = kp;
    infraRedPin = irp;

    result = (float *)malloc(2 * sizeof(float));

    serial->begin(115200);

    driver1->voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driver1->init();
    motor1->linkDriver(driver1);

    motor1->controller = MotionControlType::velocity_openloop;
    motor1->voltage_limit = 0; // Volts
    motor1->init();

    pinMode(en1, OUTPUT);
    digitalWrite(en1, HIGH);

    driver2->voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driver2->init();
    motor2->linkDriver(driver2);

    motor2->controller = MotionControlType::velocity_openloop;
    motor2->voltage_limit = 0; // Volts
    motor2->init();

    pinMode(en2, OUTPUT);
    digitalWrite(en2, HIGH);

    pinMode(kickerPin, OUTPUT_OPEN_DRAIN);
    HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET);

    _delay(1000);
}

KickerSTM::~KickerSTM()
{
}

void KickerSTM::readSerialMsg()
{
    char buffer[10];
    while (serial->available()) // Looking for the data.
    {
        serial->readBytesUntil('\0', buffer, sizeof(buffer));
        serial->flush();
        if (buffer[0] == '<')
        {

            binaryFloat.binary[0] = buffer[1];
            binaryFloat.binary[1] = buffer[2];
            binaryFloat.binary[2] = buffer[3];
            binaryFloat.binary[3] = buffer[4];
            result[0] = binaryFloat.floatingP;

            binaryFloat.binary[0] = buffer[5];
            binaryFloat.binary[1] = buffer[6];
            binaryFloat.binary[2] = buffer[7];
            binaryFloat.binary[3] = buffer[8];
            result[1] = binaryFloat.floatingP;

            kik_sig = buffer[9];
        }
    }
}

void KickerSTM::move()
{
    ///////////////////////////////////////////////////////////
    // Voltage limit is defined by the following calculation //
    //         Max Voltage --------------- Max Rad/s         //
    //     Desired Voltage --------------- Desired Rad/s     //
    //  (((Max Voltage)/2) * |Desired Rad/s|) / (Max Rad/s)  //
    ///////////////////////////////////////////////////////////
    motor1->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[0])) / (600 * RPM_TO_RADS) + 2; // volts

    motor1->loopFOC();
    motor1->move(result[0]);

    motor2->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[1])) / (600 * RPM_TO_RADS) + 2; // volts

    motor2->loopFOC();
    motor2->move(result[1]);
}

void KickerSTM::kick()
{
    if (kik_sig && !digitalRead(infraRedPin))
    {
        if ((millis() - last_kick_time) > KICK_COOLDOWN)
        {
            HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_SET);
            last_kick_time = millis();
            kicker_activated = 1;
        }
    }
}

void KickerSTM::deactivateKick()
{
    if (kicker_activated == 1)
    {
        if ((millis() - last_kick_time) > KICK_DURATION)
        {
            HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET);
            kicker_activated = 0;
        }
    }
}

void KickerSTM::run()
{
    readSerialMsg();
    deactivateKick();
    kick();
    move();
    deactivateKick();
}