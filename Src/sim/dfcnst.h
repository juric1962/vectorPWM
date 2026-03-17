// pin for sim800 UART3
// НОВАЯ РАЗВОДКА ПЛАТЫ С МОДУЛЕМ SIM800
// инициализировать эти ноги в модуле MX_GPIO_Init()
#define RTS0 GPIO_PIN_11  //RTS0 + DTR0 аппаратно   совмещенный выход GPIOD

#define CTS0 GPIO_PIN_12  //CTS0  вход GPIOD
#define DCD0 GPIO_PIN_9  //DCD0  вход GPIOD


#define PWR_SIM  GPIO_PIN_8  // выход включение питания модуля GPIOA
#define PWRK_SIM  GPIO_PIN_1  // выход сброс модуля GPIOG

//ноги светодиода связанного с gsm /////////////////////////////////


#define PR1 GPIO_PIN_8              // выходы светодиода GREEN GPIOC НА СХЕМЕ LED3
#define PR2 GPIO_PIN_9              // выходы светодиода RED GPIOC

////////////////////////////////////////////////////////////////////

// для задачи SIM800
/*
void milisecund_handler_sim800(void);
void RxSim800Handler(void);
void TxSim800Handler(void);
void PreparaeSim800Work(void);
void modem_engine(void);
extern unsigned char sim800Data;
*/
// для задачи SIM800



/*
// конфигурация ног связанных с модулем sIM800
//Configure GPIO pins : PC8-LED3GREEN, PC9-LED3RED 
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 ; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
//Configure GPIO pins : PA8-выход включение питания модуля SIM800
  GPIO_InitStruct.Pin = GPIO_PIN_8; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
//Configure GPIO pins : PG1-выход  сброс модуля SIM800
  GPIO_InitStruct.Pin = GPIO_PIN_1; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);


//Configure GPIO pins : PD12-выход  RTS0 + DTR0 аппаратно   совмещенный SIM800
  GPIO_InitStruct.Pin = GPIO_PIN_12; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

// Configure GPIO pins : PD9 PD11 - вход  CTS0 DCD0 от SIM800
 GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11; 
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  
*/



/*
// этот кусок надо вставить в файле stm32f4xx_hal_msp.c при инициализации портов 
// инициализация последовательного порта для работы с SIM800
void HAL_UART_MspInit(UART_HandleTypeDef* huart)


else if(huart->Instance==USART3)
  {
 
    
    
    __USART3_CLK_ENABLE();
  
    //USART3 GPIO Configuration    
    //PB10     ------> USART3_TX
    //PB11     ------> USART3_RX 
    
    
    
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  
  
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
 

  // USER CODE END USART3_MspInit 1 

}


*/



/*
// вставить в функции main
// UART5 init function 
//       для SIM800                      
static void MX_UART3_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}


*/

/*
//  в файле stm32f4xx_it.c  разрешить прерывания
void USART3_IRQHandler(void)
{
  // USER CODE BEGIN USART3_IRQn 0

 
 HAL_UART_IRQHandler(&huart3);
 
}

*/


// в системном таймере сделать вызов моей милисекндной процедуры
/*
void HAL_SYSTICK_Callback(void) // Для отслеживания тайм-аутов при работе с Wi-Fi модулем
{
 

milisecund_handler_sim800();

 
}
*/

/*
// перед вычным циклом сделать подготовку 
 MX_UART3_Init();  
  
   HAL_UART_Receive_IT(&huart3,&sim800Data,1);    // Каждый раз принимать один байт 
PreparaeSim800Work();

 в цикле while вызываем

modem_engine();

// 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

  if(huart->Instance==USART3)
  {
    RxSim800Handler();
      
  } 



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  
  
   if(huart->Instance==USART3)
  {
    TxSim800Handler();
      
  } 



*/



#define STM32 0
//#define RS232_handler_ON 1

//#define FLAG_FIRS_ON 0x55aa

#define DEF_TM_VHOD 30

#define TCH18_B1 0x0e
#define TCH18_B2 0x99
#define TCH18_B3 0xd6
#define TCH18_B4 0x00

#define TCH19_B1 0xd4
#define TCH19_B2 0xb9
#define TCH19_B3 0xd2
#define TCH19_B4 0x00


#define TCH_B1 0x30
#define TCH_B2 0xaf
#define TCH_B3 0x4c
#define TCH_B4 0x00

#define RG_WORK      0x00
#define RG_DEBAG     0x01 //dobavka
#define MODEM_ONLY   0x02 //dobavka
#define MODEM_ONLY_R 0x03 //dobavka
#define MODEM_CALL   0x04

#define DEF_CNT_SEC 1000  // тут точно 1 милисекунда

enum t_version {VER1 = 1 ,VER2,VER3,VER4};
enum t_type {ZAPR,OTV,SOOB,KVIT};

#define LED_MD_OFF   1
#define LED_MD_NO_CREG  2
#define LED_MD_YES_CREG 3
#define LED_MD_GPRS  4
#define LED_MD_PPP   5

#define T_MINUS10 749
#define T_MINUS5  706

# define SIM_BASE 0             //dobavka
# define SIM_RES  1             //dobavka
# define SIM_TRIG  1            //dobavka

#define OVER_SND 30
//#define TAKT   (0x1)  /*   */
#define DS_DETECT   (0x4)  /*   */
#define TABLETKA   (0x8)  /*   */
#define BLOCK_TABLETKA   (0x10)  /*   */

#define VZAT   (0x0)  /*   */
#define SNAT   (0x1)  /*   */
#define ALARM   (0x2)  /*   */
#define VHOD   (0x3)  /*   */
#define VIHOD   (0x4)  /*   */






     


