#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <math.h>

#include <SimpleFOC.h>

#include "HardwareSerial.h"
// HardwareSerial Serial2(PA10, PA9);
HardwareSerial stmSerial(PB11, PB10);

#define led_pin PC13

/////////////////////////////////////////////////////////////////////////

#define WHEEL_RADIUS 0.034

#define ROBOT_RADIUS 0.072

const float wheel_angles[4] = { PI * (1.0 / 6.0), PI *(5.0 / 6.0), PI *(5.0 / 4.0), PI *(7.0 / 4.0) };

float jacobian[4][3];

float *result = (float *)malloc(4 * sizeof(float));
;

/////////////////////////////////////////////////////////////////////////

union {
  float floatingP;
  byte binary[4];
} binaryFloat;

float vx = 0.0;
float vy = 0.0;
float vt = 0.0;
int kik_sig = 0;

/////////////////////////////////////////////////////////////////////////

//RF24 radio(9, 10); // CE, CSN
RF24 radio(PB0, PA4);  // CE, CSN on Blue Pill
const uint64_t address = 0xFAB10FAB10LL;

/////////////////////////////////////////////////////////////////////////

BLDCMotor motor1 = BLDCMotor(11, 11, 60);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA10, PA9, PA8);
#define en1 PB15

BLDCMotor motor2 = BLDCMotor(11, 11, 60);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PB9, PB8, PB7);
#define en2 PB6

/////////////////////////////////////////////////////////////////////////

void setup() {

  for (int i = 0; i < 4; i++) {
    jacobian[i][0] = cos(wheel_angles[i]);
    jacobian[i][1] = sin(wheel_angles[i]);
    jacobian[i][2] = ROBOT_RADIUS;
  }

  pinMode(led_pin, OUTPUT);

  stmSerial.begin(115200);

  // Serial2.begin(57600);
  // Serial2.println("radio");
  radio.begin();
  // Serial2.prkintln("Address");
  radio.openReadingPipe(0, address);  //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);      //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(11);
  radio.startListening();  //This sets the module as receiver



  // SimpleFOCDebug::enable(&Serial2);

  driver1.voltage_power_supply = 20;
  driver1.init();
  motor1.linkDriver(&driver1);

  motor1.controller = MotionControlType::velocity_openloop;
  motor1.voltage_limit = 3.5;  // Volts
  motor1.init();

  pinMode(en1, OUTPUT);
  digitalWrite(en1, HIGH);
  
  driver2.voltage_power_supply = 20;
  driver2.init();
  motor2.linkDriver(&driver2);

  motor2.controller = MotionControlType::velocity_openloop;
  motor2.voltage_limit = 3.5;  // Volts
  motor2.init();

  pinMode(en2, OUTPUT);
  digitalWrite(en2, HIGH);
  _delay(1000);
}

void loop() {

  digitalWrite(led_pin, LOW);

  if (radio.available())  //Looking for the data.
  {
    digitalWrite(led_pin, HIGH);
    // Serial2.println("Radio is sniffing");

    char buffer[14];
    radio.read(&buffer, sizeof(buffer));
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

  // Serial2.println(vx);
  // Serial2.println(vy);
  // Serial2.println(vt);
  // Serial2.println(kik_sig);

  for (int i = 0; i < 4; i++) {
    result[i] = 0;
    result[i] = result[i] + jacobian[i][0] * vx;
    result[i] = result[i] + jacobian[i][1] * vy;
    result[i] = result[i] + jacobian[i][2] * vt;
    result[i] = 1 / WHEEL_RADIUS * result[i];
    if (result[i] > 50)
        result[i] = 50;
    // else if (result[i] < 1)
    //   result[i] = 0;
  }

  // Serial2.println(result[0]);
  // Serial2.println(result[1]);
  // Serial2.println(result[2]);
  // Serial2.println(result[3]);

  // stmSerial.printf("<%f,%f,%d>", result[3], result[4], kik_sig);

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

  stmSerial.write(stm,10);

  motor1.voltage_limit = (10*abs(result[0]))/(600*	0.10472)+2; //volts

  motor1.loopFOC();
  motor1.move(result[0]);

  motor2.voltage_limit = (10*abs(result[1]))/(600*	0.10472)+2; //volts

  motor2.loopFOC();
  motor2.move(result[1]);
  // Serial2.println(result[0]);
}
