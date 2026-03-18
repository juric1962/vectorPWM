#include "bvi.h"
#include "main.h"
#include "pwm.h"
#include "dynaGram.h"
/*
#define BEEPER(a) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (GPIO_PinState)a) 
SPI_HandleTypeDef hspi2;
uint8_t bviCnt[10]={0,0,0,0,0,0,0,0,0,0}; 
uint8_t bviLim[10]={0,0,0,0,0,0,0,0,0,0}; 
*/

//volatile uint8_t bviArray[10]={0,0,0,0,0,0,0,0,0,0}; 

/*void forceBVI(uint8_t lineNum, uint8_t type) // lineNum - номер линии БВИ, 0-8 - светодиоды, 9 - динамик, - самый дальний от разьъема светодиод
{                                            // type: 0 - выкл, 1 - вкл постоянно, > 1 - период изменения X 100 мс      
  if(lineNum<10)
  {
    bviArray[lineNum]=type;
    if(type>1)bviLim[lineNum]=type-2;
  }  
   
}
*/
/*
void bviInit(void)
{
  
  HAL_SPI_MspInit(&hspi2);
  MX_SPI2_Init();  
  for(int i=0;i<8;i++)forceBVI(i, 5);
  //forceBVI(9, 5);    
}
*/
void newBVITask()
{
  uint32_t curTick=HAL_GetTick();    
  static uint32_t prevTick=0; 
  uint32_t ms;    
  static uint8_t greenLEDCnt=0;
  static uint8_t redLEDCnt=0;
  static uint8_t buzzerCnt=0;
   
  if(curTick>prevTick) 
  {
    ms=curTick-prevTick;
  }else{ms=(0xFFFFFFFFL-prevTick)+(curTick+1L);}
  
  if(ms>500) 
  {     
    if(systemState.startDisable) // Флаг запрета пуска
    {    
      DOUT5_RED_LED(1);   
      DOUT6_GREEN_LED(0);
      DOUT7_BUZZER(0);
    }else{
      if(systemState.startDisableCnt)
      {
        DOUT6_GREEN_LED(0);
        redLEDCnt++;// Мигать красным светодиодом: 3_on/1_off
        if(redLEDCnt<=3)DOUT5_RED_LED(1);
        else{ 
          if(redLEDCnt<=4)DOUT5_RED_LED(0);                     
          else redLEDCnt=0;
        }                
        if(systemState.startDisableCnt<command.beepDelayBeforeStartIter)
        {          
          buzzerCnt++;// Включить пищалку 1_on/1_off
          if(buzzerCnt<=1)DOUT7_BUZZER(1);
          else{ 
            if(buzzerCnt<=2)DOUT7_BUZZER(0);                     
            else buzzerCnt=0;
          } 
        }   
      }else{
        DOUT7_BUZZER(0);
        DOUT5_RED_LED(0); 
        if(vectorPWM.isOn) // ШИМ включен
        {
          if(dynaGram.algorithmState) // Включен алгоритм работы с динамографом 
          {                  
            greenLEDCnt++;// Зеленый мигает 1_on/1_off
            if(greenLEDCnt<=1)DOUT6_GREEN_LED(1);
            else{ 
              if(greenLEDCnt<=2)DOUT6_GREEN_LED(0);                     
              else greenLEDCnt=0;
            }           
          }else{ // Выключен алгоритм работы с динамографом            
            greenLEDCnt++;// Зеленый мигает 3_on/1_off 
            if(greenLEDCnt<=3)DOUT6_GREEN_LED(1);
            else{ 
              if(greenLEDCnt<=4)DOUT6_GREEN_LED(0);                     
              else greenLEDCnt=0;
            }            
          } 
        }else{
          if(command.delayBeforeStartIter)
          {
            DOUT6_GREEN_LED(0);
            redLEDCnt++;// Мигать красным светодиодом: 3_on/1_off
            if(redLEDCnt<=3)DOUT5_RED_LED(1);
            else{ 
              if(redLEDCnt<=4)DOUT5_RED_LED(0);                     
              else redLEDCnt=0;
            }                
            if(command.delayBeforeStartIter<command.beepDelayBeforeStartIter)
            {          
              buzzerCnt++;// Включить пищалку 1_on/1_off
              if(buzzerCnt<=1)DOUT7_BUZZER(1);
              else{ 
                if(buzzerCnt<=2)DOUT7_BUZZER(0);                     
                else buzzerCnt=0;
              } 
            }   
          }else DOUT6_GREEN_LED(1);  
        }      
      } 
    }
    prevTick=curTick;                              
  }             
}


/*
void bviTask() // Периодически (100 мс) вызываемая функция
{  
  uint32_t curTick=HAL_GetTick();    
  static uint32_t prevTick=0; 
  static uint16_t reg=0;
  uint32_t ms;  
  uint16_t i;
  
  if(curTick>prevTick) 
  {
    ms=curTick-prevTick;
  }else{ms=(0xFFFFFFFFL-prevTick)+(curTick+1L);}
  
  if(ms>100) // Прошлo 100 мс
  { 
    for(i=0;i<10;i++)
    {
      switch(bviArray[i])
      {
        case 0:
          reg&=~(1<<i);
        break;
        case 1:
          reg|=1<<i;
        break;
        default:
          if(bviCnt[i]>=bviLim[i])
          {
            bviCnt[i]=0;
            if(reg&(1<<i))
            {
              reg&=~(1<<i);              
            }else{              
              reg|=(1<<i);            
            }
          }
        break;    
      }
      bviCnt[i]++;
    }       
    if(reg&0x0200)BEEPER(1);
    else BEEPER(0);  
    i=reg;    
    i=((reg<<3)&0x0F00|(reg&0x001F));        
    i=~i;    
    HAL_SPI_Transmit(&hspi2,(uint8_t *)&i,1,1);    
    prevTick=curTick;
  }  
}*/
/*
void MX_SPI2_Init(void)
{  
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;//SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;//SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    //Error_Handler();
  }
  
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{  
  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {    
    __HAL_RCC_SPI2_CLK_ENABLE();  
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12; // BVI_SOUND
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
    
  }
  
}*/

// test branch MAC commit