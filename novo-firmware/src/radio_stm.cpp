#include "radio_stm.h"

RadioSTM::RadioSTM(HardwareSerial *s, RF24 *r, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2)
{
    serial = s;
    driver1 = d1;
    en1 = e1;
    driver2 = d2;
    en2 = e2;

    radio = r;

    result = (float *)malloc(4 * sizeof(float));

    for (int i = 0; i < 4; i++)
    {
        jacobian[i][0] = cos(wheel_angles[i]);
        jacobian[i][1] = sin(wheel_angles[i]);
        jacobian[i][2] = ROBOT_RADIUS;
    }

    serial->begin(115200);

    radio->begin();
    radio->openReadingPipe(0, address); // Setting the address at which we will receive the data
    radio->setPALevel(RF24_PA_MIN);     // You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
    radio->setDataRate(RF24_250KBPS);
    radio->setChannel(11);
    radio->startListening(); // This sets the module as receiver

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
    _delay(1000);
}

RadioSTM::~RadioSTM()
{
}

void RadioSTM::readRadio()
{
    if (radio->available()) // Looking for the data.
    {

        char buffer[14];
        radio->read(&buffer, sizeof(buffer));
        // for (uint i = 0; i < sizeof(buffer); i++) {
        //   Serial2.printf("%d, ", buffer[i]);
        // }
        // Serial2.printf("\n");

        binaryFloat.binary[0] = buffer[1];
        binaryFloat.binary[1] = buffer[2];
        binaryFloat.binary[2] = buffer[3];
        binaryFloat.binary[3] = buffer[4];
        vx = binaryFloat.floatingP;

        binaryFloat.binary[0] = buffer[5];
        binaryFloat.binary[1] = buffer[6];
        binaryFloat.binary[2] = buffer[7];
        binaryFloat.binary[3] = buffer[8];
        vy = binaryFloat.floatingP;

        binaryFloat.binary[0] = buffer[9];
        binaryFloat.binary[1] = buffer[10];
        binaryFloat.binary[2] = buffer[11];
        binaryFloat.binary[3] = buffer[12];
        vt = binaryFloat.floatingP;

        kik_sig = buffer[13];
    }
}

void RadioSTM::getWheelSpeeds()
{
    for (int i = 0; i < 4; i++)
    {
        result[i] = 0;
        result[i] = result[i] + jacobian[i][0] * vx;
        result[i] = result[i] + jacobian[i][1] * vy;
        result[i] = result[i] + jacobian[i][2] * vt;
        result[i] = 1 / WHEEL_RADIUS * result[i];
    }
}

void RadioSTM::sendSerialMsg()
{
    uint8_t stm[11];
    stm[0] = '<';

    binaryFloat.floatingP = result[2];
    stm[1] = binaryFloat.binary[0];
    stm[2] = binaryFloat.binary[1];
    stm[3] = binaryFloat.binary[2];
    stm[4] = binaryFloat.binary[3];

    binaryFloat.floatingP = result[3];
    stm[5] = binaryFloat.binary[0];
    stm[6] = binaryFloat.binary[1];
    stm[7] = binaryFloat.binary[2];
    stm[8] = binaryFloat.binary[3];

    stm[9] = kik_sig;

    stm[10] = '\0';

    serial->write(stm, 10);
}

void RadioSTM::move()
{
    motor1->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[0])) / (600 * 0.10472) + 2; // volts

    motor1->loopFOC();
    motor1->move(result[0]);

    motor2->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[1])) / (600 * 0.10472) + 2; // volts

    motor2->loopFOC();
    motor2->move(result[1]);
}

void RadioSTM::run()
{
    readRadio();
    getWheelSpeeds();
    sendSerialMsg();
    move();
}