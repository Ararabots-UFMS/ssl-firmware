#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <math.h>

#include <SimpleFOC.h>

// #include <string.h>
// #include <stdlib.h>
// #include <stdio.h>

#include "HardwareSerial.h"

//HardwareSerial Serial2(PA10, PA9);
HardwareSerial stmSerial(PB11, PB10);

/////////////////////////////////////////////////////////////////////////

union {
  float floatingP;
  byte binary[4];
} binaryFloat;

float result3 = 0.0;
float result4 = 0.0;
int kik_sig = 0;

/////////////////////////////////////////////////////////////////////////

BLDCMotor motor3 = BLDCMotor(11, 11, 60);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PA10, PA9, PA8);
#define en3 PB15

BLDCMotor motor4 = BLDCMotor(11, 11, 60);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(PB9, PB8, PB7);
#define en4 PB6

/////////////////////////////////////////////////////////////////////////

#define kickerPin PB12
#define infraRedPin PB0

unsigned long last_kick_time = 0;
uint8_t kicker_on = 0;

/////////////////////////////////////////////////////////////////////////

void setup() {
  //Serial2.begin(9600);
  stmSerial.begin(115200);
  //Serial2.println("setup");

  //SimpleFOCDebug::enable(&Serial2);

  driver3.voltage_power_supply = 20;
  driver3.init();
  motor3.linkDriver(&driver3);

  motor3.controller = MotionControlType::velocity_openloop;
  motor3.voltage_limit = 3.5;  // Volts
  motor3.init();

  pinMode(en3, OUTPUT);
  digitalWrite(en3, HIGH);
  
  driver4.voltage_power_supply = 20;
  driver4.init();
  motor4.linkDriver(&driver4);

  motor4.controller = MotionControlType::velocity_openloop;
  motor4.voltage_limit = 3.5;  // Volts
  motor4.init();

  pinMode(en4, OUTPUT);
  digitalWrite(en4, HIGH);

  pinMode(kickerPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(kickerPin, LOW);

  _delay(1000);
}

void loop() {
  char buffer[10];
  // String buffer;
   
  // if (kicker_on == 1) {
  //   if ((millis() - last_kick_time) > 5){
  //     digitalWrite(kickerPin, LOW);
  //     kicker_on = 0;
  //   }
  // }

  while (stmSerial.available())  //Looking for the data.
  {
    stmSerial.readBytesUntil('\0', buffer, sizeof(buffer));
    stmSerial.flush();
    if(buffer[0] == '<'){

      binaryFloat.binary[0] = buffer[1];
      binaryFloat.binary[1] = buffer[2];
      binaryFloat.binary[2] = buffer[3];
      binaryFloat.binary[3] = buffer[4];
      result3 = binaryFloat.floatingP;
    
      binaryFloat.binary[0] = buffer[5];
      binaryFloat.binary[1] = buffer[6];
      binaryFloat.binary[2] = buffer[7];
      binaryFloat.binary[3] = buffer[8];
      result4 = binaryFloat.floatingP;

      kik_sig = buffer[9];

      

      //for(int i = 0; i < sizeof(buffer); i++){
      //  Serial2.printf("%d ", buffer[i]);
      //}
      //Serial2.printf("\n");

      //Serial2.println(result3);
      //Serial2.println(result4);
      //Serial2.println(kik_sig);
    }
  }// else {
  //   Serial2.println("nada");
  // }


  // if (stmSerial.available()){
  //    //Looking for the data.
  //   buffer = stmSerial.readString();
  //   char *buffer = buffer;
  //   if (strspn("<>", buffer) == 2){
  //     char *p = strtok(buffer,"<>,");
  //     result3 = atof(p);
  //     p = strtok(NULL,"<>,");
  //     result4 = atof(p);
  //     p = strtok(NULL,"<>,");
  //     kik_sig = atoi(p);
  //   }
  //   //printf("%ld",strspn("<>", a));
    
  // }


  // if (kik_sig && !digitalRead(infraRedPin)) {
  //   if ((millis() - last_kick_time) > 500){
  //     digitalWrite(kickerPin, HIGH);
  //     last_kick_time = millis();
  //     kicker_on = 1;
  //   }
  // }

  // if (kicker_on == 1) {
  //   if ((millis() - last_kick_time) > 5){
  //     digitalWrite(kickerPin, LOW);
  //     kicker_on = 0;
  //   }
  // }

  motor3.voltage_limit = (10*abs(result3))/(600*	0.10472)+2; //volts

  motor3.loopFOC();
  motor3.move(result3);

  motor4.voltage_limit = (10*abs(result4))/(600*	0.10472)+2; //volts

  motor4.loopFOC();
  motor4.move(result4);
  
  // if (kicker_on == 1) {
  //   if ((millis() - last_kick_time) > 5){
  //     digitalWrite(kickerPin, LOW);
  //     kicker_on = 0;
  //   }
  // }
}
