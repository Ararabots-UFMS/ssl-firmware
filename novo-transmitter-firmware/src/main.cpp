#include <Arduino.h>
#include <RF24.h>
#include "main.h"

HardwareSerial serial(PB11, PB10); // RX, TX pins for Serial communication
RF24 radio(PB0, PA4);              // CE, CSN pins
byte buffer[14] = {0};

const uint32_t timeout = 1000; // milliseconds

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // setbuf(stdout, NULL);              // Disable buffering for stdout
  serial.begin(230400, SERIAL_8E1); // Initialize Serial communication
  serial.setTimeout(1);

  radio.begin();
  radio.openWritingPipe(ADDRESS);
  radio.setPALevel(RF24_PA_MIN); // You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(11);
  radio.stopListening(); // This sets the module as receiver
}

void loop()
{
  serial.write("R", 1); // Indicate ready
  serial.flush();
  uint32_t start_time = millis(); // Initialize start_time

  while (serial.available() < 14) // Wait until at least 14 bytes are available
  {
    // If too much time has passed, exit the loop (timeout)
    if (millis() - start_time > timeout)
      return; // Not enough time has passed, exit loop
  }

  // Clear the buffer before reading
  memset(buffer, 0, sizeof(buffer)); // Clear the buffer to avoid garbage data

  serial.readBytes(buffer, sizeof(buffer));
  radio.write(buffer, sizeof(buffer), true); // Send the data over RF24
}