#include "stm32f4xx_hal.h"
//#include <iom1280.h> 
//#include "dfpin.h"
//#include "dfproc.h"
#include <string.h> 
//#include <inavr.h>
#include "dfcnst.h"
//#include "map_mbus.h"
//#include "ozu_map.h"
//#include "def_at.h"
//#include "map_ad.h"


//void HAL_GPIO_WritePin(unsigned char port, char GPIO_Pin,char bit);
//void HAL_GPIO_ToglePin(unsigned char port, char GPIO_Pin);
         // зел

// для отладки генерю импульсы по ту4 и коммтирю их на ТС10

#define DEBUG_TU4 GPIO_PIN_3  //  TU4 GPIOD

void CLR_DEBUG_TU4(void)
{
 HAL_GPIO_WritePin(GPIOD, DEBUG_TU4,(GPIO_PinState)0);
 }


void SET_DEBUG_TU4(void)
{
 HAL_GPIO_WritePin(GPIOD, DEBUG_TU4,(GPIO_PinState)1);
 }


void CLR_RTS0(void)
{
 HAL_GPIO_WritePin(GPIOD, RTS0,(GPIO_PinState)1);
 }

void SET_RTS0(void)
{
  HAL_GPIO_WritePin(GPIOD, RTS0,(GPIO_PinState)0);
 }
/*   совместил с RTS */
void CLR_DTR0(void)
{
// HAL_GPIO_WritePin(GPIOF, DTR0,(GPIO_PinState)1);
 }

void SET_DTR0(void)
{
 // HAL_GPIO_WritePin(GPIOF, DTR0,(GPIO_PinState)0);
 }



void SET_PWR(void)
{
  HAL_GPIO_WritePin(GPIOA, PWR_SIM,(GPIO_PinState)1);
 }


/*
#define CLR_PWR\
{\
  PORTJ&=~PWR;\
 }
*/
void CLR_PWR(void)
{
HAL_GPIO_WritePin(GPIOA, PWR_SIM,(GPIO_PinState)0);
  }

//нога включения модема/////////////////////////////////


void SET_PWRK(void)
{
 HAL_GPIO_WritePin(GPIOG, PWRK_SIM,(GPIO_PinState)0);
 }

void CLR_PWRK(void)
{
  HAL_GPIO_WritePin(GPIOG, PWRK_SIM,(GPIO_PinState)1);
 }
/////////////////////////


// светодиоды не выведены
///S1
 void S1_OFF(void)
{
 
  
  HAL_GPIO_WritePin(GPIOC, PR1,(GPIO_PinState)0);
  HAL_GPIO_WritePin(GPIOC, PR2,(GPIO_PinState)0);
 
 }
void S1_YL(void)
{
  
  
   HAL_GPIO_WritePin(GPIOC, PR1,(GPIO_PinState)1);
   HAL_GPIO_WritePin(GPIOC, PR2,(GPIO_PinState)1);
 
 }
void S1_GR(void)
{
  
   
    HAL_GPIO_WritePin(GPIOC, PR1,(GPIO_PinState)1);
    HAL_GPIO_WritePin(GPIOC, PR2,(GPIO_PinState)0);
  
 }
void S1_RD(void)
{
  
   
    HAL_GPIO_WritePin(GPIOC, PR1,(GPIO_PinState)0);
    HAL_GPIO_WritePin(GPIOC, PR2,(GPIO_PinState)1);
 
 }
void S1_CH(void)
{
  
   
   HAL_GPIO_TogglePin(GPIOC, PR1);
   HAL_GPIO_TogglePin(GPIOC, PR2);
    
 
 }


