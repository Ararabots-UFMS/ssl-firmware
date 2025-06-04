#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

#define INFRARED_PIN PB0

#define KICKER_PIN PB12
#define KICKER__GPIO_PORT GPIOB
#define KICKER_GPIO_PIN GPIO_PIN_12

#define KICK_DURATION 5
#define KICK_COOLDOWN 500

uint8_t kik_sig = 0;
uint8_t kicker_activated = 0;

uint32_t last_kick_time = 0;

void setup()
{

  pinMode(KICKER_PIN, OUTPUT_OPEN_DRAIN);
  HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET);

  pinMode(INFRARED_PIN, INPUT_PULLDOWN);
}

void loop()
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

  if (kicker_activated == 1)
  {
    if ((millis() - last_kick_time) > KICK_DURATION)
    {
      HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET);
      kicker_activated = 0;
    }
  }
}