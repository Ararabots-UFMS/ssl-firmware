#include "kicker_stm.h"
// #include "stm32f1xx_hal.h"

KickerSTM::KickerSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2)
{
    // turn led on
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // uart = new HardwareSerial(PA3, PA2); // RX, TX
    // uart->begin(115200);

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

KickerSTM::~KickerSTM()
{
    free(result);
}

void KickerSTM::getWheelSpeeds()
{
    vx = 0.00;
    vy = 0.50;
    vt = 1.00;
    for (int i = 0; i < 4; i++)
    {
        result[i] = 0.0;
        result[i] = result[i] + jacobian[i][0] * vx;
        result[i] = result[i] + jacobian[i][1] * vy;
        result[i] = result[i] + jacobian[i][2] * vt;
        result[i] = (1.0 / WHEEL_RADIUS) * result[i];
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
    motor1->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[0])) / (MAX_RPM * RPM_TO_RADS) + 2; // volts

    motor1->loopFOC();
    motor1->move(result[2]);

    motor2->voltage_limit = ((VOLTAGE_POWER_SUPPLY / 2) * abs(result[1])) / (MAX_RPM * RPM_TO_RADS) + 2; // volts

    motor2->loopFOC();
    motor2->move(result[3]);

    // uart->println("> Kik Sig: " + String(kik_sig));
}


void KickerSTM::run()
{
    getWheelSpeeds();
    move();

    // result[0] = 0.0f;
    // result[1] = 0.0f;
    // kik_sig = 1;

    digitalWrite(LED_PIN, LOW);
}