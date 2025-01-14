#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <SimpleFOC.h>

#include "HardwareSerial.h"

HardwareSerial Serial2(PA10, PA9);

//RF24 radio(9, 10); // CE, CSN
RF24 radio(PB0, PA4); // CE, CSN on Blue Pill
const uint64_t address = 0xFAB10FAB10LL;

void setup() {
  Serial2.begin(9600);
  Serial2.println("radio");
  radio.begin();
  Serial2.println("Address");
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(11);
  radio.startListening();              //This sets the module as receiver
}

void loop()
  {

  if (radio.available())              //Looking for the data.
  {
    Serial2.println("Radio is sniffing");
    
    char msg[14];
    radio.read(&msg, sizeof(msg));  
    for (uint i = 0; i < sizeof(msg) ; i++){
      Serial2.printf("%d, ",msg[i]);
    }
  }
  else {
    Serial2.println("nada");
  }

}



