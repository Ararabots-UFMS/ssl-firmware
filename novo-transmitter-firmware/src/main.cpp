#include <Arduino.h>
#include <RF24.h>
#include "main.h"

HardwareSerial Serial2(PB11, PB10); // RX, TX pins for Serial communication
RF24 radio(PB0, PA4);               // CE, CSN pins
byte buffer[14] = {0};

const uint32_t timeout = 100; // milliseconds

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // setbuf(stdout, NULL);              // Disable buffering for stdout
  Serial2.begin(230400, SERIAL_8E1); // Initialize Serial communication
  Serial2.setTimeout(1);

  radio.begin();
  radio.openWritingPipe(ADDRESS);
  radio.setPALevel(RF24_PA_MIN); // You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(11);
  radio.stopListening(); // This sets the module as receiver
}

void loop()
{
  Serial2.write("R", 1); // Indicate ready
  Serial2.flush();
  uint32_t start_time = millis(); // Initialize start_time

  while (Serial2.available() < 14) // Wait until at least 14 bytes are available
  {
    // If too much time has passed, exit the loop (timeout)
    if (millis() - start_time > timeout)
      return; // Not enough time has passed, exit loop
  }

  // Clear the buffer before reading
  memset(buffer, 0, sizeof(buffer)); // Clear the buffer to avoid garbage data

  Serial2.readBytes(buffer, sizeof(buffer));
  radio.write(buffer, sizeof(buffer), true); // Send the data over RF24

  // Serial2.println(lido);
  // Serial2.write(buffer, 14); // Echo the data back to Serial
  // Serial2.flush();           // Ensure all data is sent
}