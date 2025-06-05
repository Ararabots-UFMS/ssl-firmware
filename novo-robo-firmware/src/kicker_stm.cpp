#include "kicker_stm.h"
#include "stm32f1xx_hal.h"

KickerSTM::KickerSTM(HardwareSerial *s, BLDCDriver3PWM *d1, uint8_t e1, BLDCDriver3PWM *d2, uint8_t e2)
{
    // turn led on
    // pinMode(LED_PIN, OUTPUT);
    // digitalWrite(LED_PIN, HIGH);
    // uart = new HardwareSerial(PA3, PA2); // RX, TX
    // uart->begin(115200);

    uint8_t rxBuffer[SERIAL_BUFFER_SIZE];

    // Initialize UART1 with DMA for RX
    // HAL_UART_Receive_DMA(uart->getHandle(), rxBuffer, SERIAL_BUFFER_SIZE);

    serial = s;
    driver1 = d1;
    en1 = e1;
    driver2 = d2;
    en2 = e2;

    result = (float *)malloc(2 * sizeof(float));
    result[0] = 0.0f;
    result[1] = 0.0f;

    serial->begin(SERIAL_BAUDRATE, SERIAL_CONFIG);

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

    pinMode(KICKER_PIN, OUTPUT_OPEN_DRAIN);
    HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET);

    _delay(1000);

    serial->write("R"); // Send a ready signal to the other STM32
}

KickerSTM::~KickerSTM()
{
}

void KickerSTM::readSerialMsg()
{

    // If the message is not complete, return
    if (serial->available() < SERIAL_BUFFER_SIZE)
    {
        if (millis() - lastTime > 1000) // If no message is received for more than 1 second
            serial->write("R");         // Send another ready signal to the other STM32
        return;
    }

    uint8_t buffer[SERIAL_BUFFER_SIZE] = {0}; // Buffer to store the serial data.
    serial->readBytes(buffer, sizeof(buffer));

    uint8_t index = 0; // Discard the first byte (0xAA)

    binaryFloat.binary[0] = buffer[index++];
    binaryFloat.binary[1] = buffer[index++];
    binaryFloat.binary[2] = buffer[index++];
    binaryFloat.binary[3] = buffer[index++];
    float temp = binaryFloat.floatingP;

    if (temp < MAX_RPM && temp > -MAX_RPM)
        result[0] = temp; // Set the first result if it is within the valid range

    binaryFloat.binary[0] = buffer[index++];
    binaryFloat.binary[1] = buffer[index++];
    binaryFloat.binary[2] = buffer[index++];
    binaryFloat.binary[3] = buffer[index++];
    temp = binaryFloat.floatingP;

    if (temp < MAX_RPM && temp > -MAX_RPM)
        result[1] = temp; // Set the second result if it is within the valid range

    kik_sig = buffer[index] == 1;

    lastTime = millis();
    serial->write("R");
}

void KickerSTM::move()
{
    delay(10); // Small delay to ensure the motors are ready
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

    // uart->println("> Kik Sig: " + String(kik_sig));
}

void KickerSTM::kick()
{
    if (kik_sig && !digitalRead(INFRARED_PIN))
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
    kick();
    move();
    deactivateKick();

    digitalWrite(LED_PIN, LOW);
}