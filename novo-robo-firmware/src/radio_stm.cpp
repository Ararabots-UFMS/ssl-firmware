#include "radio_stm.h"

RadioSTM::RadioSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2)
{
    // turn led on
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    uart = new HardwareSerial(PA3, PA2); // RX, TX
    uart->begin(115200);

    serial = s;
    driver1 = d1;
    en1 = e1;
    driver2 = d2;
    en2 = e2;

    radio = new RF24(RADIO_CE, RADIO_CSN); // CE, CSN

    result = (float *)malloc(4 * sizeof(float));

    for (int i = 0; i < 4; i++)
    {
        jacobian[i][0] = sin(WHEEL_ANGLES[i]);
        jacobian[i][1] = cos(WHEEL_ANGLES[i]);
        jacobian[i][2] = ROBOT_RADIUS;
    }

    serial->begin(SERIAL_BAUDRATE, SERIAL_CONFIG);

    radio->begin();
    digitalWrite(LED_PIN, LOW);
    radio->openReadingPipe(0, ADDRESS); // Setting the address at which we will receive the data
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
    delay(1000);
}

RadioSTM::~RadioSTM()
{
    free(result);
}

void RadioSTM::readRadio()
{
    // if (!radio->available()) // Check if there is data available to read
    //     return;

    char buffer[14];
    radio->read(&buffer, sizeof(buffer));

    if (buffer[0] != ROBOT_NAME) // Check if the first byte matches the robot name
        return;
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

void RadioSTM::getWheelSpeeds()
{
    for (int i = 0; i < 4; i++)
    {
        result[i] = 0.0;
        result[i] = result[i] + jacobian[i][0] * vx;
        result[i] = result[i] + jacobian[i][1] * vy;
        result[i] = result[i] + jacobian[i][2] * vt;
        result[i] = (1.0 / WHEEL_RADIUS) * result[i];
    }
}

void RadioSTM::sendSerialMsg()
{
    serial->read();
    uint8_t stm[SERIAL_BUFFER_SIZE] = {0}; // Buffer to store the serial data.

    uint8_t index = 0;

    // stm[index++] = 0xAA; // The first byte is the robot name.

    binaryFloat.floatingP = result[2];
    stm[index++] = binaryFloat.binary[0];
    stm[index++] = binaryFloat.binary[1];
    stm[index++] = binaryFloat.binary[2];
    stm[index++] = binaryFloat.binary[3];

    binaryFloat.floatingP = result[3];
    stm[index++] = binaryFloat.binary[0];
    stm[index++] = binaryFloat.binary[1];
    stm[index++] = binaryFloat.binary[2];
    stm[index++] = binaryFloat.binary[3];

    stm[index] = kik_sig;

    serial->write(stm, SERIAL_BUFFER_SIZE); // Sending the data to the serial port.
    serial->flush();                        // Ensuring that the data is sent immediately.
}

void RadioSTM::move()
{
    ///////////////////////////////////////////////////////////
    // Voltage limit is defined by the following calculation //
    //         Max Voltage --------------- Max Rad/s         //
    //     Desired Voltage --------------- Desired Rad/s     //
    //  (((Max Voltage)/2) * |Desired Rad/s|) / (Max Rad/s)  //
    ///////////////////////////////////////////////////////////
    motor1->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[0])) / (MAX_RPM * RPM_TO_RADS) + 2; // volts

    motor1->loopFOC();
    motor1->move(result[0]);
    // uart->println("> Motor 1: " + String(result[0]));

    motor2->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[1])) / (MAX_RPM * RPM_TO_RADS) + 2; // volts

    motor2->loopFOC();
    motor2->move(result[1]);
    // uart->println("> Motor 2: " + String(result[1]));
}

void RadioSTM::run()
{
    readRadio();
    getWheelSpeeds();
    sendSerialMsg();
    move();

    uart->println("> Serial Data Sent:" + String(result[2]) + ", " + String(result[3]));
}