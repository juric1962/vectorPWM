#include "main.h"
// init pin sim800
//---------------SIM800------------------------------------------  
void init_pin_sim800(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;
 // PD9 DCD INPUT
 //PD12 CTS INUT
 //PD11 RTS OUTPUT
 //PA8 PWR OUTPUT  включение питания модуля SIM800
 //PG1 PWR_KEY OUTPUT сброс модуля SIM800
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 // GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 
  
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}