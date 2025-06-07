#include <Arduino.h>
#include "stm32f1xx_hal.h"

// put function declarations here:
int myFunction(int, int);

#define INFRARED_PIN PB0

#define KICKER_PIN PB12
#define KICKER__GPIO_PORT GPIOB
#define KICKER_GPIO_PIN GPIO_PIN_12

#define KICK_DURATION 5
#define KICK_COOLDOWN 500

uint8_t kik_sig = 1;  // set to 1 to trigger kick logic
uint8_t kicker_activated = 0;

uint32_t last_kick_time = 0;

// Configure PB12 as GPIO Output Push-Pull using HAL directly
void initKickerGPIO_Hal() {
  __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable clock to GPIOB

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = KICKER_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;             // No pull-up/down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KICKER__GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET); // Set HIGH
}

void setup()
{
  initKickerGPIO_Hal();  // configure PB12 manually and set HIGH

  // Optional: still define as open-drain via Arduino if you want
  // pinMode(KICKER_PIN, OUTPUT_OPEN_DRAIN);
  // HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET);

  pinMode(INFRARED_PIN, INPUT_PULLDOWN);
}

void loop()
{
  if (kik_sig && !digitalRead(INFRARED_PIN))
  {
    HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_SET);
    last_kick_time = millis();
    kicker_activated = 1;
  }

  // Optional: Uncomment to reset pin after duration
  // if (kicker_activated == 1)
  // {
  //   if ((millis() - last_kick_time) > KICK_DURATION)
  //   {
  //     HAL_GPIO_WritePin(KICKER__GPIO_PORT, KICKER_GPIO_PIN, GPIO_PIN_RESET);
  //     kicker_activated = 0;
  //   }
  // }
}
