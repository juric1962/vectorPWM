#include "main.h"
#include "pwm.h"
#include "modbus.h"
#include "DS18B20.h"
#include "sram_rtc.h"
#include "alarms.h"
#include "arm_math.h"
#include "fram.h"
#include <math.h>
#include "bvi.h"
#include "archive.h"
#include "scenario_softAP.h"
#include "dynaGram.h"

// для задачи SIM800
//#define Task800On   1             //  1- условная компиляция задачи Sim800 включена 
void milisecund_handler_sim800(void);
void RxSim800Handler(void);
void TxSim800Handler(void);
void PreparaeSim800Work(void);
void modem_engine(void);
extern unsigned char sim800Data;
// для задачи SIM800

volatile SYSTEM_STATE_STRUCT systemState; 
#define ADC1_DR_ADDRESS  ((uint32_t)0x4001204C) // Адрес регистра данных АЦП №1

ADC_HandleTypeDef hadc1,hadc2,hadc3;
DMA_HandleTypeDef hdma_adc1;
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
//UART_HandleTypeDef huart6;

DMA_HandleTypeDef hdma_usart2_tx;
//DMA_HandleTypeDef hdma_uart3_tx;
//DMA_HandleTypeDef hdma_uart5_tx;

IWDG_HandleTypeDef hiwdg;

#define ADC_WORDS_NUM 9//6
volatile uint16_t adc_data[ADC_WORDS_NUM+1];
volatile uint8_t adc_data_index=0;
volatile uint8_t adc_dataDescr[10];

uint8_t rs485RxBuf[ELAM_BUF_SIZE];
uint8_t rs485TxBuf[ELAM_BUF_SIZE];
uint8_t elamBuf[ELAM_BUF_SIZE];
uint8_t rs232RxBuf[COM_RX_BUF_SIZE];
uint8_t rs232TxBuf[COM_TX_BUF_SIZE];
uint8_t wifiRxBuf[COM_RX_BUF_SIZE];
uint8_t wifiTxBuf[COM_TX_BUF_SIZE];

uint8_t rs232InputByte;
uint8_t rs485InputByte;
uint8_t wifiInputByte;

volatile uint16_t rs485RxPtr=0;
uint16_t rs485TxPtr=0;
volatile uint8_t rs485DataReady=0;
volatile uint16_t rs485TxReady=1;

volatile uint16_t rs232RxPtr=0;
uint16_t rs232TxPtr=0;
volatile uint8_t rs232DataReady=0;

volatile uint16_t wifiRxPtr=0;
uint16_t wifiTxPtr=0;
volatile uint8_t wifiDataReady=0;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
//static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
//static void MX_USART3_UART_Init(void);
//static void MX_UART5_Init(void);
static void MX_UART3_Init(void);
void MX_DAC_Init(void);
static void MX_IWDG_Init(void);

void commandInit(void);
void contextInit(void);
void engineInit(void);
void analogCalibration(void);
void adcInputsInit(void);
void dacOutputsInit(void);

volatile TC_STRUCT TCInputs;

uint8_t faultCnt=0;
uint8_t D0Cnt=0;
uint8_t D1Cnt=0;

#define samplesNum 1000//996    // 1(начало кадра)+4*249
#define decimatorRatio 20 // Дециматор семплирования, один отсчет берется из decimatorRatio семплов  
volatile int16_t array1[samplesNum+9];
volatile int16_t array2[samplesNum+9];
volatile uint8_t arrayRdy=0;       // Указывает, какой из массивов заполнен
volatile uint8_t arrayTransmit=0;  // Указывает, какой из массивов передается в порт
volatile uint16_t arrayPtr=0;      // Указатель заполнения текущего массива
volatile uint8_t decimatorCnt=0;
volatile uint8_t packetCnt=0;

volatile DIAG_STRUCT diag;

volatile PID_VOLTAGE_STRUCT pidVoltage;

uint8_t iwdgFlag=0;

//------------------------------------------------------------------------------
volatile int16_t udc_1kHz_data[100];
volatile uint8_t udc_1kHz_ptr=0;
volatile uint8_t udc_1kHz_decimator=0;
volatile uint8_t udc_1kHz_decimator_max;
volatile int32_t q10Real,q10Im;
volatile int16_t Q10;

//------------------------------------------------------------------------------
int8_t cos10Int8[100]={ 
   103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127,    103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127, 
   103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127,    103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127, 
   103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127,    103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127, 
   103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127,    103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127, 
   103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127,    103,     39,    -39,   -103,   -127,   -103,    -39,     39,    103,    127};
int8_t sin10Int8[100]={ 
    75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0,     75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0, 
    75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0,     75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0, 
    75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0,     75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0, 
    75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0,     75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0, 
    75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0,     75,    121,    121,     75,      0,    -75,   -121,   -121,    -75,      0};

//------------------------------------------------------------------------------
void noPhaseTask(void) // Пока функция жестко привязана к частоте ШИМ в 10кГц
{ 
  uint8_t i;
  int32_t mean;  
  if(udc_1kHz_ptr>99)
  {     
    mean=0;
    q10Real=0;
    q10Im=0;      
    for(i=0;i<100;i++)mean+=udc_1kHz_data[i];
    mean=mean/100;    
    for(i=0;i<100;i++)
    { 
      udc_1kHz_data[i]-=(int16_t)mean;
      q10Real+=(int32_t)udc_1kHz_data[i]*(int32_t)cos10Int8[i];
      q10Im+=(int32_t)udc_1kHz_data[i]*(int32_t)sin10Int8[i];          
    }
    float x=(float)q10Real*(float)q10Real+(float)q10Im*(float)q10Im; // Контроль над превышением разрядности
    x=sqrt(x);
    Q10=(int16_t)(x/3175.0f); // Считаю не амплитуду гармоники (127*50=6350, а размах, 127 - амплитуда массивов cos10Int8 и sin10Int8, 50 - количество отсчетов для вещественного спектра)
    udc_1kHz_ptr=0;        
    measurements.Q10=Q10;
    if(alarms.fixFreqPhaseErrorMS)
    {
      if((fixFreq[0].fixFreqMode==0)||(fixFreq[0].fixFreqControl==2))
      {      
        if(Q10>=alarms.fixFreqPhaseErrorValue)
        {
          alarms.fixFreqPhaseErrorCnt++;
          if((uint32_t)alarms.fixFreqPhaseErrorMS<(((uint32_t)alarms.fixFreqPhaseErrorCnt*100L)))  // Время удержания для перехода на фиксированную частоту                           
          {
            if(fixFreq[0].fixFreqControl==2)
            {              
               eventsCnt.fixFreqNoPhase++;
               alarms.faultBitsExtended|=0x0010;                         
               systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                        
               eventLogWrite(22);  
               forceBVI(8, 0);
               FAULT_OPTO_OUT(1);               
            }else{
              if(alarms.fixFreqEnable) // Разрешен переход на фиксированную частоту
              {                    
                fixFreq[0].fixFreqMode=2;
                fixFreq[0].eventTimeStamp=RTC_UnixTime.timeStamp;
                fixFreq[0].fixFreqAttempt++;
                fixFreq[0].CRC16=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH);                                     
                if(fixFreq[0].fixFreqAttempt>alarms.fixFreqAttemptsNumMax) // Счетчик превышен, сработала авария
                {
                  eventsCnt.fixFreqNoPhase++;
                  alarms.faultBitsExtended|=0x0010;                         
                  systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                        
                  eventLogWrite(22);  
                  forceBVI(8, 0);
                  FAULT_OPTO_OUT(1);                      
                }else{
                  alarms.faultBitsExtended|=0x0010;  
                  eventLogWrite(23); 
                }
              }else{                   
                 eventsCnt.fixFreqNoPhase++;
                 alarms.faultBitsExtended|=0x0010;                         
                 systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                        
                 eventLogWrite(22);  
                 forceBVI(8, 0);
                 FAULT_OPTO_OUT(1);                      
              }  
            }            
          }                              
        }else alarms.fixFreqPhaseErrorCnt=0;
      }else alarms.fixFreqPhaseErrorCnt=0;
    }    
  }
}
//------------------------------------------------------------------------------
uint16_t modbusAddr; // Модбас-адрес преобразователя 
const char progVersionString[40]="CHREP_V1_7";
uint8_t stateDOUT=0;

uint32_t wifiTimeOutTimer=0; // Декрементируемый счетчик ожидания ответа от Wi-Fi модуля
uint8_t waitForWiFiFlag=0;   // Флаг ожидания ответа от Wi-Fi модуля
volatile uint8_t rinatTimer=0;
uint8_t oneSec;
uint32_t  startTick,stopTick;  // Для вычисления время выполнения задачи основного цикла
uint32_t getTimeMS(uint32_t tick)
{ 
  if(tick>=startTick)tick=tick-startTick;
  else{tick=(0xFFFFFFFFL-startTick)+(tick+1L);}    
  return tick;
}
//uint8_t rs232_Ready=1;   // Флаг готовности порта RS232 при переключении режимов работы
int main(void)
{   
  vectorPWM.activePdPhi=0.0;
  vectorPWM.activePddPhi=0.0;  
  systemState.curState=0;         // Состояние - ожидание управляющих команд
  systemState.nextState=0;        // Состояние - ожидание управляющих команд 
  systemState.faults=0;           // Сброс битов текущих аварий
  systemState.curMode=0;          // Неопределенное состоняие системы при включении
  systemState.prevMode=0;         // Неопределенное состоняие системы при включении
  systemState.tempMode=0;         // Временный режим для того, чтобы можно было сбрасывать ошибку без изменения режима, задержка на 3 сек между переключением режимов
  systemState.modeSwitchTimer=0;  // Таймер задержки между переключением режимов работы
  systemState.stateTC=0;          // Неопределенное состоняие ТС'ов  
  systemState.manualStart=0;      // Убран флаг ручного пуска 
  systemState.Ready=0;            // Флаг готовности привода к выполлению команд 
  systemState.startDisable=0;     // Запрет запуска
  systemState.kpStart=1;
  systemState.desiredFreqOrRPM=0.0f;
  systemState.startDisableCnt=0;   
  systemState.resetTC=0;
  systemState.wasStopCHREP=0;
  systemState.thyristorON=0;
  systemState.tempHoldON=0;
  systemState.etrHoldON=0;
  systemState.skipDoublePress=0;
  systemState.eventLogCurMode=0;
  systemState.eventLogPrevMode=0;  
  if((RCC->CSR&RCC_CSR_WDGRSTF) == RCC_CSR_WDGRSTF) // Ресет был вызван сторожевым таймером   
  {
    iwdgFlag=1;
  }
  HAL_Init();
  SystemClock_Config(); 
  bkpSRAMInit();
  eventLogInit();    
  MX_GPIO_Init();      
  rtcInit();
  eventLogWrite(13);  
  MX_DMA_Init();  
  MX_ADC1_Init();
  MX_ADC2_Init();  
  MX_ADC3_Init();  
  MX_DAC_Init();     
  eventsCntInit();
  if(iwdgFlag)
  { 
    LED2_RED(1);
    eventsCnt.iwdg++;
  }
  engineInit();  
  MX_TIM1_Init();  
  MX_TIM2_Init();
  MX_TIM3_Init();  
  //MX_TIM4_Init();
  MX_TIM5_Init();
  energyCalcInit();     
  contextInit();
  commandInit();  
  alarmsInit();
  bviInit(); 
  diagInit();   
  adcInputsInit();
  dacOutputsInit();
  HAL_RTC_GetTime(&hrtc,&RTC_TimeStructure,RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc,&RTC_DateStructure,RTC_FORMAT_BIN); 
  RTC_UnixTime.timeStamp=toUnix(RTC_DateStructure,RTC_TimeStructure);
  eventLogWrite(12);  
  TCInputsInit();
  if(faultsInit())// Есть события, по которым запрещен пуск двигателя        
  {
    systemState.startDisable=1;
    FAULT_OPTO_OUT(1);
  }  
  LED1_RED(1);  
  Delay_us(5000000); // Задержка на стабилизацию питающих напряжений  
  LED1_RED(0);     
  DOUT4(1); // Сигнал того, что ЧРЭП включен в сеть
  stateDOUT|=8;
  activeEnergyInit();
  tiiInit();
  fixFreqInit();
  MX_USART2_UART_Init(); 
  #ifndef WI_FI_DEBUG // Не принимать данные с rs485 порта в режиме отладки Wi-Fi модуля
    HAL_UART_Receive_IT(&huart2, &rs232InputByte, 1); // Каждый раз принимать один байт    
  #endif
    
    // uart3 вообще блокируем
 // MX_USART3_UART_Init(); 
  
  //HAL_UART_Receive_IT(&huart3, &rs232InputByte,1); // Каждый раз принимать один байт  
    
// UART5 используется в этой версии для SIM800  
  MX_UART3_Init();  
  //HAL_UART_Receive_IT(&huart5, &wifiInputByte,1); // Каждый раз принимать один байт 
   HAL_UART_Receive_IT(&huart3,&sim800Data,1);    // Каждый раз принимать один байт 
  vectorPWM.curDeltaPhi=0.0;  
  vectorPWM.outputsCommand=0;  
  vectorPWM.isOn=0;
  vectorPWMSetPhi0();  
  pidVoltage.pwmCorrection=0;
  pidVoltage.UdcSum=0;
  pidVoltage.samplesCnt=0;
  pidVoltage.Udc=0;
  pidVoltage.UdcRdy=0;  
  pidVoltage.error=0;
  pidVoltage.errorIntegral=0;  
  vectorPWM.pidCurDeltaPhi=0; 
  framRead(MODBUS_ADDR_FRAM_ADDR,(uint8_t*)&modbusAddr,MODBUS_ADDR_FRAM_LENGTH);     
  dynaGramInit();
  MX_IWDG_Init(); 
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);    
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&adc_data,ADC_WORDS_NUM);
  PWM_Start(&htim1);  
  PWM_DisableOutputs(&htim1);  
  F_RESET();    
  tempSensorsInit();     
  HAL_IWDG_Start(&hiwdg); 
  
  
  
  PreparaeSim800Work();
   

  
  
  while (1)    
  {   
    if(command.updateFromShadow) // Сохранить во FRAM текущее состояние двигателя
    {       
      copymas((uint8_t*)&commandShadow,(uint8_t*)&command,COMMAND_MODBUS_REGNUM*2);         
      commandShadow.CRC16=modBusCRC16((uint8_t*)&commandShadow,COMMAND_MODBUS_REGNUM*2);
      framWrite(COMMAND_FRAM_ADDR,(uint8_t*)&commandShadow,COMMAND_FRAM_LENGTH);
      command.updateFromShadow=0;
    }   
    
    
    
  
   modem_engine();
  
    
      startTick=HAL_GetTick();
    bviTask();          
      stopTick=getTimeMS(HAL_GetTick());
      if(diag.bviTaskMS<stopTick)diag.bviTaskMS=stopTick;      
      
      startTick=HAL_GetTick();      
    activeEnergyTask();
      stopTick=getTimeMS(HAL_GetTick());
      if(diag.activeEnergyTaskMS<stopTick)diag.activeEnergyTaskMS=stopTick;          
    
      startTick=HAL_GetTick(); 
    tiiTask();
      stopTick=getTimeMS(HAL_GetTick());
      if(diag.tiiTaskMS<stopTick)diag.tiiTaskMS=stopTick;    
    
      startTick=HAL_GetTick(); 
    noPhaseTask();    
      stopTick=getTimeMS(HAL_GetTick());
      if(diag.noPhaseTaskMS<stopTick)diag.noPhaseTaskMS=stopTick;
    
      startTick=HAL_GetTick();
    getTemperatureTask();
      stopTick=getTimeMS(HAL_GetTick());    
      if(diag.getTemperatureTaskMS<stopTick)diag.getTemperatureTaskMS=stopTick;
      
      startTick=HAL_GetTick();  
    getDataTimeTask();
      stopTick=getTimeMS(HAL_GetTick());      
      if(diag.getDataTimeTaskMS<stopTick)diag.getDataTimeTaskMS=stopTick;    
      
    //startTick=HAL_GetTick();  
    //energyCalcTask();        
    //stopTick=getTimeMS(HAL_GetTick());              
    //if(diag.energyCalcTaskMS<stopTick)diag.energyCalcTaskMS=stopTick;    
      
      
      /*  задачу для порта отладки  закоментарить
      startTick=HAL_GetTick();      
    debugRS232Task();
      stopTick=getTimeMS(HAL_GetTick());             
      if(diag.debugRS232TaskMS<stopTick)diag.debugRS232TaskMS=stopTick;     
      */
      
      
      startTick=HAL_GetTick();      
    modbusTask();    
      stopTick=getTimeMS(HAL_GetTick());             
      if(diag.modbusTaskMS<stopTick)diag.modbusTaskMS=stopTick;          
       
      /*  сценарий для WiFi отключить*/
      /*
      startTick=HAL_GetTick();
    scenario();
      stopTick=getTimeMS(HAL_GetTick());  
      if(diag.scenarioMS<stopTick)diag.scenarioMS=stopTick;
      */
      
      
      startTick=HAL_GetTick();
    dynaGramTask();                 
      stopTick=getTimeMS(HAL_GetTick());  
      if(diag.dynaGramTaskMS<stopTick)diag.dynaGramTaskMS=stopTick;    
    
    HAL_IWDG_Refresh(&hiwdg); 
  }  
}

void HAL_SYSTICK_Callback(void) // Для отслеживания тайм-аутов при работе с Wi-Fi модулем
{
 rinatTimer++;
 if(waitForWiFiFlag)
 {
   if(wifiTimeOutTimer)wifiTimeOutTimer--; 
 }
 energyCalcTask();

milisecund_handler_sim800();

 
}

void TCInputsInit(void)
{ 
  uint8_t i;
  TCInputs.transitionsNum=(uint16_t*)((uint8_t*)(BKPSRAM_BASE+TCINPUTS_SRAM_ADDR));
  for(i=0;i<16;i++)TCInputs.initState[i]=0; 
  TCInputs.modBusWrite=0;
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    while(1);
  }
}
/* DAC init function */
void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig;
  /**DAC Initialization */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);
  /**DAC channel OIT1, OUT2 config */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);  
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,0x0000);  
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);  
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,0x0000);   
}

void diagInit(void)
{
  diag.framReadErr=0;
  diag.framWriteErr=0;
  diag.rs232_CRC_Err=0;
  diag.rs232_IRQ_ERR=0;
  diag.rs232_recPacketNum=0;
  diag.rs232_transPacketNum=0;
  diag.rs485_CRC_Err=0;
  diag.rs485_IRQ_ERR=0;
  diag.rs485_recPacketNum=0;
  diag.rs485_transPacketNum=0;
  diag.temp85=0;
  diag.tempCRCerr=0;
  diag.tempInitErr=0;
  diag.tempReadsNum=0;  
  diag.sequenceADCPWM=0;
  diag.seqErr_ADC_PWM=0;
  diag.sequence_EnergyCalc=0;
  diag.seqErr_EnergyCalc=0;  
  
  diag.getTemperatureTaskMS=0;
  diag.energyCalcTaskMS=0;
  diag.getDataTimeTaskMS=0;
  diag.debugRS232TaskMS=0;
  diag.modbusTaskMS=0;  
  
  diag.wifi_IRQ_ERR=0;
  diag.wifi_recPacketNum=0;
  diag.wifi_transPacketNum=0;
  
  diag.bviTaskMS=0;  
  diag.activeEnergyTaskMS=0;
  diag.tiiTaskMS=0;
  diag.noPhaseTaskMS=0;
  diag.scenarioMS=0;
  diag.dynaGramTaskMS=0;  
}

void commandInit(void)
{
  framRead(COMMAND_FRAM_ADDR,(uint8_t*)&command,COMMAND_FRAM_LENGTH);   
  if(command.CRC16!=modBusCRC16((uint8_t*)&command, COMMAND_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {
    systemState.eventLogHardwareFault=7; // Сброс настроек ЦАП
    eventLogWrite(18);
    eventsCnt.commandDefaults++;
    command.desiredFreqOrRPM=0.0f;       
    command.rotationDIR=0;   
    command.stopCHREP=0;
    command.delayBeforeRestartSec=20;        
    command.delayBeforeStartKPSec=3;        
    command.faultsReset=0;
    command.autoMode=0;    
    command.testMode=0;           
    command.currentFreqOrRPM=0.0f;  
    command.fanOnTemp=40;
    command.fanOnTempHyst=3;
    command.pidPrescaler=10;
    command.powerStab=0.0f;
    command.Kp=0.04f;  
    command.Ki=0.0003f;
    command.Kd=0.0f;
    command.dphiLimit=0.000001;  // Ограничение на приращение фазы и добавочной частоты в генераторном режиме: dPhi=Fpwm*1e-6=10e-2(Fpwm=1e4)  => dPhi*Fpwm=100 - 100 Гц/сек !!!
    command.voltageStab=250;
    command.dFMaxCorrection=50;    
    command.voltageStabPowerLow=3000;
    command.voltageStabPowerHigh=5000;   
    command.startCurrentFreezeLow=40;
    command.startCurrentFreezeHigh=50;
    command.startCurrentFreezeMS=3000;
    command.powerInsensitiveDelta=50;
    command.freeRunTimeMS=0;    
    command.CRC16=modBusCRC16((uint8_t*)&command, COMMAND_MODBUS_REGNUM*2);     
    framWrite(COMMAND_FRAM_ADDR,(uint8_t*)&command,COMMAND_FRAM_LENGTH);    
  }
  commandShadow.updateFromShadow=0; 
  command.currentFreqOrRPM=0.0f;    
  command.delayBeforeStartIter=0;  
  copymas((uint8_t*)&commandShadow,(uint8_t*)&command,COMMAND_MODBUS_REGNUM*2);
}

void contextInit(void)
{
  uint8_t i;
  context.DI_state=0;
  context.DI_cnt=0;  
  context.curContextNum=0;
  context.prevContextNum=0;
  contextShadow.updateFromShadow=0;
  
  framRead(CONTEXT_FRAM_ADDR,(uint8_t*)&context,CONTEXT_FRAM_LENGTH);  
  if(context.CRC16!=modBusCRC16((uint8_t*)&context,CONTEXT_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {   
    systemState.eventLogHardwareFault=6; // Сброс настроек контекста
    eventLogWrite(18);
    eventsCnt.contextDefaults++;
    for(i=0;i<CONTEXT_NUM;i++)
    {
      context.startFreq[i]=1;         // Стартовая частота, имеет смысл только при старте привода   
      context.dfdtUP[i]=5.0f;  
      context.dfdtDOWN[i]=5.0f;  
      context.curveStimeUP[i]=2;            // Задавать отдельно, как в ВЕСПЕРЕ, прибавка ко времени линейного участка  
      context.curveStimeDOWN[i]=2;          // Задавать отдельно, как в ВЕСПЕРЕ, прибавка ко времени линейного участка      
      context.ufTableNum[i]=i;
      context.freqSource[i]=0;                // Источник задания частоты: 0 - модбас, 1 - ТИТ1, 2 - ТИТ2 	        
    }
    context.voltageShiftTime=1;    
    context.contextSource=0;                   // Источник задания контекста: 0 - состояние дискретного входа DI0 ( DI0_0_context/DI0_1_context ), 1 - модбас (modbus_context)
    context.DI0_0_contextNum=0;
    context.DI0_1_contextNum=1;
    context.modbus_contextNum=0;        
    context.curContextNum=context.modbus_contextNum;  
    context.prevContextNum=context.curContextNum;      
    context.CRC16=modBusCRC16((uint8_t*)&context,CONTEXT_MODBUS_REGNUM*2);
    framWrite(CONTEXT_FRAM_ADDR,(uint8_t*)&context,CONTEXT_FRAM_LENGTH); 
  }  
  copymas((uint8_t*)&contextShadow,(uint8_t*)&context,CONTEXT_MODBUS_REGNUM*2);
}


void adcInputsInit(void)
{
  uint8_t i;
  framRead(ADC_FRAM_ADDR,(uint8_t*)&adcInputs,ADC_FRAM_LENGTH);   
  if(adcInputs.CRC16!=modBusCRC16((uint8_t*)&adcInputs, ADC_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {
    systemState.eventLogHardwareFault=14; // Сброс настроек АЦП
    eventLogWrite(18);
    eventsCnt.adcDefaults++;
    adcInputs.maxMV[0]=10000; //4000 3672 соотв. 10 В на входе 1-го ТИТ
    adcInputs.maxMV[1]=5000;  // Грубая калибровка ТИТ2 для потенциометра: 0-5 В    
    for(i=0;i<2;i++)
    {
      adcInputs.minMV[i]=0;//379;  //100; 379 соотв. 1 В на входе 1-го ТИТ
      adcInputs.minFreq[i]=0;//1;     
      adcInputs.maxFreq[i]=50;  
      adcInputs.oneHandrethDF[i]=10;
      adcInputs.meanMS[i]=100;
      adcInputs.Ready[i]=0;
      adcInputs.dmV[i]=100;
   }
    adcInputs.CRC16=modBusCRC16((uint8_t*)&adcInputs, ADC_MODBUS_REGNUM*2);      
    framWrite(ADC_FRAM_ADDR,(uint8_t*)&adcInputs,ADC_FRAM_LENGTH);    
  }
  adcInputs.calibr[0]=0.36855f;
  adcInputs.calibr[1]=0.36914f;
  for(i=0;i<2;i++)
  { 
    adcInputs.outPutValue[i]=0.0f;
    adcInputs.meanCntMax[i]=(uint32_t)(((float)adcInputs.meanMS[i])*(float)engine.freqPWM/1000.0f);        
    adcInputs.maxCode[i]=(uint16_t)(adcInputs.calibr[i]*(float)adcInputs.maxMV[i]);
    adcInputs.minCode[i]=(uint16_t)(adcInputs.calibr[i]*(float)adcInputs.minMV[i]);        
    adcInputs.df[i]=((float)adcInputs.oneHandrethDF[i])/100.0f;  
    adcInputs.k[i]=((float)(adcInputs.maxFreq[i]-adcInputs.minFreq[i]))/((float)(adcInputs.maxCode[i]-adcInputs.minCode[i]));  
    adcInputs.b[i]=(float)(adcInputs.minFreq[i])-((float)adcInputs.minCode[i])*adcInputs.k[i];      
  } 
  //-----Добавление еще двух ТИТ'ов, только для индикации  ---------------------
  adcInputs.meanMS[2]=100; 
  adcInputs.meanMS[3]=100;
  adcInputs.meanCntMax[2]=(uint32_t)(((float)adcInputs.meanMS[2])*(float)engine.freqPWM/1000.0f); 
  adcInputs.meanCntMax[3]=(uint32_t)(((float)adcInputs.meanMS[3])*(float)engine.freqPWM/1000.0f); 
  //----------------------------------------------------------------------------  
  adcInputsShadow.updateFromShadow=0;  
  copymas((uint8_t*)&adcInputsShadow,(uint8_t*)&adcInputs,ADC_MODBUS_REGNUM*2);
}

void dacOutputsInit(void)
{  
  uint8_t i;
  framRead(DAC_FRAM_ADDR,(uint8_t*)&dacOutputs,DAC_FRAM_LENGTH);   
  if(dacOutputs.CRC16!=modBusCRC16((uint8_t*)&dacOutputs, DAC_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {
    systemState.eventLogHardwareFault=13; // Сброс настроек ЦАП
    eventLogWrite(18);
    eventsCnt.dacDefaults++;
    for(i=0;i<2;i++)
    {     
      dacOutputs.source[i]=1;
      dacOutputs.maxMV[i]=10000;
      dacOutputs.maxValue[i]=50.0f;
      dacOutputs.minMV[i]=0;
      dacOutputs.minValue[i]=0.0f;        
    }
    dacOutputs.CRC16=modBusCRC16((uint8_t*)&dacOutputs, DAC_MODBUS_REGNUM*2);      
    framWrite(DAC_FRAM_ADDR,(uint8_t*)&dacOutputs,DAC_FRAM_LENGTH);    
  }  
  dacOutputs.calibr[0]=0.3955f;    // calibr=code/mV -> (4095/10300) 
  dacOutputs.calibr[1]=0.3938f;    // calibr=code/mV -> (4095/10300)  
  for(i=0;i<2;i++)
  {     
    dacOutputs.minCode[i]=(uint16_t)(dacOutputs.calibr[i]*(float)dacOutputs.minMV[i]);
    dacOutputs.maxCode[i]=(uint16_t)(dacOutputs.calibr[i]*(float)dacOutputs.maxMV[i]);      
    dacOutputs.k[i]=((float)(dacOutputs.maxCode[i]-dacOutputs.minCode[i]))/((float)(dacOutputs.maxValue[i]-dacOutputs.minValue[i]));  
    dacOutputs.b[i]=(float)(dacOutputs.minCode[i])-((float)dacOutputs.minValue[i])*dacOutputs.k[i];     
  } 
  dacOutputsShadow.updateFromShadow=0;  
  copymas((uint8_t*)&dacOutputsShadow,(uint8_t*)&dacOutputs,DAC_MODBUS_REGNUM*2);  
}

void engineInit(void)
{
  uint8_t i;
  float tempFloat;
  engineShadow.updateFromShadow=0;
  framRead(ENGINE_FRAM_ADDR,(uint8_t*)&engine,ENGINE_FRAM_LENGTH); 
  if(engine.CRC16!=modBusCRC16((uint8_t*)&engine, ENGINE_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {
    systemState.eventLogHardwareFault=5; // Сброс настроек двигателя 
    eventLogWrite(18);
    eventsCnt.engineDefaults++;
    engine.Inom=30.0f;
    engine.Iidle=10.0f;
    engine.nomSpeed=1460.0f;
    engine.nomFreq=50.0f;
    engine.efficiency=89.0f;
    engine.minFreq=0.5f;
    engine.maxFreq=50.0f;
    engine.freqPWM=10000;
    engine.nomPower=15000.0f;
    engine.polesPairsNum=4;   
    engine.debugMode=0;
    for(i=0;i<CONTEXT_NUM;i++)
    {
      engine.ufTable[i].freq=25.0f;
      engine.ufTable[i].voltagePercent[0]=4.5f;
      engine.ufTable[i].voltagePercent[1]=50.0f;
      engine.ufTable[i].voltagePercent[2]=100.0f; 
    }    
    engine.CRC16=modBusCRC16((uint8_t*)&engine, ENGINE_MODBUS_REGNUM*2);
    framWrite(ENGINE_FRAM_ADDR,(uint8_t*)&engine,ENGINE_FRAM_LENGTH);    
    modbusAddr=1;
    framWrite(MODBUS_ADDR_FRAM_ADDR,(uint8_t*)&modbusAddr,MODBUS_ADDR_FRAM_LENGTH);   
  }
  for(i=0;i<CONTEXT_NUM;i++)
  {  
    tempFloat=engine.ufTable[i].freq-engine.minFreq;
    if(tempFloat>0.0f)engine.k1[i]=(engine.ufTable[i].voltagePercent[1]-engine.ufTable[i].voltagePercent[0])*3.8f/tempFloat;
    else engine.k1[i]=0;
    engine.C1[i]=-engine.k1[i]*engine.minFreq+engine.ufTable[i].voltagePercent[0]*3.8f;  
        
    tempFloat=engine.maxFreq-engine.ufTable[i].freq;
    if(tempFloat>0.0f)engine.k2[i]=(engine.ufTable[i].voltagePercent[2]-engine.ufTable[i].voltagePercent[1])*3.8f/tempFloat;
    else engine.k2[i]=0;
    engine.C2[i]=-engine.k2[i]*engine.ufTable[i].freq+engine.ufTable[i].voltagePercent[1]*3.8f;         
  }  
  if((engine.freqPWM<2000)||(engine.freqPWM>12000))engine.freqPWM=10000;
  switch(engine.freqPWM)
  {
    case 2000:
      udc_1kHz_decimator_max=1;
    break;          
    case 3000:        
      udc_1kHz_decimator_max=2;
    break;          
    case 4000:                
      udc_1kHz_decimator_max=3;
    break;          
    case 5000:      
      udc_1kHz_decimator_max=4;
    break;                    
    case 6000:      
      udc_1kHz_decimator_max=5;
    break;                    
    case 7000:      
      udc_1kHz_decimator_max=6;
    break;                    
    case 8000:      
      udc_1kHz_decimator_max=7;
    break;                    
    case 9000:      
      udc_1kHz_decimator_max=8;
    break;                    
    case 10000:     
      udc_1kHz_decimator_max=9;
    break;                    
    case 11000:     
      udc_1kHz_decimator_max=10;
    break;                    
    case 12000:                        
      udc_1kHz_decimator_max=11;
    break;        
    default:
      engine.freqPWM=10000;
      udc_1kHz_decimator_max=9;
    }     
  vectorPWMInit(engine.freqPWM);    
  copymas((uint8_t*)&engineShadow,(uint8_t*)&engine,ENGINE_MODBUS_REGNUM*2);
}

void Delay_us(volatile uint32_t nCount)
{
  nCount*=24;//24
  while(nCount--)
  {
  }
}

void F_RESET(void)
{ 
  DRIVER_STOP(0);
  Delay_us(1);
  DRIVER_F_RESET(0);
  Delay_us(1);
  DRIVER_F_RESET(1);
  Delay_us(1);
  DRIVER_F_RESET(0);
}

void modbusTask(void)
{
  if(rs485DataReady)
  {      
    if(modbusSlaveTask(rs485RxBuf, rs485RxPtr, rs485TxBuf, &rs485TxPtr)) 
    {
      RS485_DIR(1);
      diag.rs485_recPacketNum++;
      HAL_UART_Transmit_DMA(&huart2,(uint8_t*)&rs485TxBuf,rs485TxPtr);
      LED1_RED(1);  
    }else{ 
      diag.rs485_CRC_Err++;    
    }
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    rs485RxPtr=0;
    rs485DataReady=0;            
  }  
  /*if((engine.debugMode==1)&&(rs232_Ready)) // Работает в режиме модбас-слейва
  {
    if(rs232DataReady)
    {      
      if(modbusSlaveTask(rs232RxBuf, rs232RxPtr, rs232TxBuf, &rs232TxPtr)) 
      {      
        diag.rs232_recPacketNum++;
        HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&rs232TxBuf,rs232TxPtr);
        rs232_Ready=0;
        LED1_RED(1);  
      }else{ 
        diag.rs232_CRC_Err++;    
      }
      __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
      rs232RxPtr=0;
      rs232DataReady=0;            
    }
  }*/  
}
/*
void debugRS232Task(void)
{    
  if(engine.debugMode==0)//&&(rs232_Ready))
  {    
    if(arrayTransmit) // В данный момент передатчик свободен
    {    
      LED2_GREEN(1);
      if(arrayTransmit==1)
      { 
        array1[samplesNum+4]= modBusCRC16((uint8_t*)&array1,(samplesNum+4)*2);        
        HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&array1,(samplesNum+5)*2);            
      }else{         
        array2[samplesNum+4]= modBusCRC16((uint8_t*)&array2,(samplesNum+4)*2);        
        HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&array2,(samplesNum+5)*2);                          
      }  
      //rs232_Ready=0;
      arrayTransmit=0;
      diag.rs232_transPacketNum++;      
    }    
  }
}
*/

void energyCalcInit(void)
{
  measurements.meanReady=0;
  measurements.meanIaCounter=0;
  measurements.meanIbCounter=0;  
  measurements.meanIbCounter=0;     
  measurements.meanCodeItems=0;    
  measurements.meanCodeItemsNum=5*engine.freqPWM; // Вычисление средних значений датчиков тока в течение 5 сек
  for(int i=0;i<2;i++)
  {
    measurements.sumIa[i]=0.0f;
    measurements.sumIb[i]=0.0f;
    measurements.sumIc[i]=0.0f;
    
    measurements.sumUa[i]=0.0f;
    measurements.sumUb[i]=0.0f;
    measurements.sumUc[i]=0.0f;
    
    measurements.sumPa[i]=0.0f;
    measurements.sumPb[i]=0.0f;
    measurements.sumPc[i]=0.0f;    

    measurements.sumUdc[i]=0.0f;    
    measurements.sampNum[i]=0.0f;    
  }
  
  measurements.endOfPeriodCnt=0;        
  measurements.endOfPeriodsNum=1;  
  measurements.energyRdy=0;
  measurements.indexToProcess=0;
  
  
  measurements.InstPhACurSum=0;  
  measurements.InstPhBCurSum=0;  
  measurements.InstPhCCurSum=0;  
  measurements.InstPhCurSampIndex=0;    
 
  measurements.maxUdc=0;
  measurements.minUdc=0;
  measurements.ppUdc=0;
  measurements.maxIrms=0;
  measurements.minIrms=0;
  measurements.ppIrms=0;             
  measurements.periodLast=0;
  
  measurements.uMAX=0;
  measurements.iMAX=0;
  measurements.uMIN=32767;
  measurements.iMIN=32767;   
}

void energyCalcTask(void)
{   
  static uint32_t prevTick; 
  if(measurements.energyRdy)
  {     
    uint32_t curTick=HAL_GetTick();      
    uint32_t dTms;   
    if(measurements.periodLast)
    {      
      if(diag.sequence_EnergyCalc!=1)
      {
        diag.seqErr_EnergyCalc++;
        eventsCnt.adc_energy_err++;
      }
      diag.sequence_EnergyCalc=2;       
      float tempFloat;
      arm_sqrt_f32((float)(measurements.sumIa[measurements.energyRdy-1]/measurements.sampNum[measurements.energyRdy-1]),(float*)&measurements.Ia);      
      arm_sqrt_f32((float)(measurements.sumIb[measurements.energyRdy-1]/measurements.sampNum[measurements.energyRdy-1]),(float*)&measurements.Ib);      
      arm_sqrt_f32((float)(measurements.sumIc[measurements.energyRdy-1]/measurements.sampNum[measurements.energyRdy-1]),(float*)&measurements.Ic);            
      measurements.Iabc=measurements.Ia+measurements.Ib+measurements.Ic;     
      measurements.sumIa[measurements.energyRdy-1]=0;
      measurements.sumIb[measurements.energyRdy-1]=0;
      measurements.sumIc[measurements.energyRdy-1]=0;      
      
      if(alarmsShadow.updateFromShadow==3)      
      {       
        alarms.breakCurMin=alarmsShadow.breakCurMin;
        alarms.breakCurMinMS=alarmsShadow.breakCurMinMS;
        alarms.breakCurAMinCounterMS=0;
        alarms.breakCurBMinCounterMS=0;
        alarms.breakCurCMinCounterMS=0;        
        
        alarms.curDisbalanceMax=alarmsShadow.curDisbalanceMax;
        alarms.curDisbalanceMaxMS=alarmsShadow.curDisbalanceMaxMS; 
        alarms.curDisbalanceMaxCounterMS=0;           
        
        alarms.noLoadWatts=alarmsShadow.noLoadWatts;
        alarms.noLoadWattsSec=alarmsShadow.noLoadWattsSec;
        alarms.noLoadWattsSecCnt=0.0f;         
        alarmsShadow.updateFromShadow=0;        
      }
      
      if(curTick>=prevTick)dTms=curTick-prevTick;
      else{dTms=(0xFFFFFFFFL-prevTick)+(curTick+1L);}       
        
      if(alarms.breakCurMinMS)
      {
        if(measurements.Ia<alarms.breakCurMin)
        {        
          alarms.breakCurAMinCounterMS+=dTms;
          if(alarms.breakCurAMinCounterMS>alarms.breakCurMinMS)
          {        
            eventLogWrite(6);
            vectorPWM.outputsCommand=2;                                   
            alarms.faultBits|=0x0400;        
            eventsCnt.breakA++;            
            alarms.breakCurAMinCounterMS=0;
            DRIVER_STOP(1);                                   
            systemState.faults=(1L<<BREAK_PHASE_CURRENT_FAULT);        
            forceBVI(8, 0);
            FAULT_OPTO_OUT(1);            
          }      
        }else{
          alarms.breakCurAMinCounterMS=0;      
        }        
          
        if(measurements.Ib<alarms.breakCurMin)
        {        
          alarms.breakCurBMinCounterMS+=dTms;
          if(alarms.breakCurBMinCounterMS>alarms.breakCurMinMS)
          {          
            alarms.faultBits|=0x0800;        
            eventsCnt.breakB++;
            eventLogWrite(6);
            alarms.breakCurBMinCounterMS=0;
            DRIVER_STOP(1);                
            vectorPWM.outputsCommand=2;                                 
            systemState.faults=(1L<<BREAK_PHASE_CURRENT_FAULT);        
            forceBVI(8, 0);
            FAULT_OPTO_OUT(1);            
          }      
        }else{
          alarms.breakCurBMinCounterMS=0;      
        }
    
        if(measurements.Ic<alarms.breakCurMin)
        {        
          alarms.breakCurCMinCounterMS+=dTms;
          if(alarms.breakCurCMinCounterMS>alarms.breakCurMinMS)
          {           
            alarms.faultBits|=0x1000;        
            eventLogWrite(6);
            eventsCnt.breakC++;
            alarms.breakCurCMinCounterMS=0;
            DRIVER_STOP(1);                
            vectorPWM.outputsCommand=2;                                 
            systemState.faults=(1L<<BREAK_PHASE_CURRENT_FAULT);        
            forceBVI(8, 0);
            FAULT_OPTO_OUT(1);            
          }      
        }else{
          alarms.breakCurCMinCounterMS=0;      
        }
      }else{
        alarms.breakCurAMinCounterMS=0; 
        alarms.breakCurBMinCounterMS=0; 
        alarms.breakCurCMinCounterMS=0; 
      } 
      
      if(alarms.curDisbalanceMaxMS)
      {
        if((alarms.curDisbalanceMax<fabs(measurements.Ia-measurements.Ib))||(alarms.curDisbalanceMax<fabs(measurements.Ib-measurements.Ic))||(alarms.curDisbalanceMax<fabs(measurements.Ic-measurements.Ia)))
        {
          alarms.curDisbalanceMaxCounterMS+=dTms;
          if(alarms.curDisbalanceMaxCounterMS>alarms.curDisbalanceMaxMS)
          {         
            alarms.faultBits|=0x2000;                    
            eventLogWrite(7);
            eventsCnt.disbalance++;
            alarms.curDisbalanceMaxCounterMS=0;
            DRIVER_STOP(1);                
            vectorPWM.outputsCommand=2;                                 
            systemState.faults=(1L<<BREAK_PHASE_CURRENT_FAULT);        
            forceBVI(8, 0);
            FAULT_OPTO_OUT(1);            
          }     
        }else{
          alarms.curDisbalanceMaxCounterMS=0;      
        }          
      }else alarms.curDisbalanceMaxCounterMS=0;
      
      arm_sqrt_f32((float)(measurements.sumUa[measurements.energyRdy-1]/(float)measurements.sampNum[measurements.energyRdy-1]),(float*)&measurements.Ua);
      arm_sqrt_f32((float)(measurements.sumUb[measurements.energyRdy-1]/(float)measurements.sampNum[measurements.energyRdy-1]),(float*)&measurements.Ub);
      arm_sqrt_f32((float)(measurements.sumUc[measurements.energyRdy-1]/(float)measurements.sampNum[measurements.energyRdy-1]),(float*)&measurements.Uc);   
      measurements.Udc=measurements.sumUdc[measurements.energyRdy-1]/(float)measurements.sampNum[measurements.energyRdy-1];
              
      measurements.Uavg=(measurements.Ua+measurements.Ub+measurements.Uc)/3.0f;      
      measurements.sumUa[measurements.energyRdy-1]=0;
      measurements.sumUb[measurements.energyRdy-1]=0;
      measurements.sumUc[measurements.energyRdy-1]=0;      
      measurements.sumUdc[measurements.energyRdy-1]=0; 
      
      measurements.Pa=measurements.sumPa[measurements.energyRdy-1]/measurements.sampNum[measurements.energyRdy-1];
      measurements.Pb=measurements.sumPb[measurements.energyRdy-1]/measurements.sampNum[measurements.energyRdy-1];
      measurements.Pc=measurements.sumPc[measurements.energyRdy-1]/measurements.sampNum[measurements.energyRdy-1];      
      measurements.Pabc=measurements.Pa+measurements.Pb+measurements.Pc;
      measurements.sumPa[measurements.energyRdy-1]=0;
      measurements.sumPb[measurements.energyRdy-1]=0;
      measurements.sumPc[measurements.energyRdy-1]=0;      
      
      measurements.Sa=measurements.Ia*measurements.Ua;
      measurements.Sb=measurements.Ib*measurements.Ub;
      measurements.Sc=measurements.Ic*measurements.Uc;      
      measurements.Sabc=measurements.Sa+measurements.Sb+measurements.Sc;
      
      tempFloat=measurements.Sa*measurements.Sa-measurements.Pa*measurements.Pa;      
      if(tempFloat>=0.0f)arm_sqrt_f32(tempFloat,(float*)&measurements.Qa);
      else tempFloat=0;
      tempFloat=measurements.Sb*measurements.Sb-measurements.Pb*measurements.Pb;      
      if(tempFloat>=0.0f)arm_sqrt_f32(tempFloat,(float*)&measurements.Qb);
      else tempFloat=0;
      tempFloat=measurements.Sc*measurements.Sc-measurements.Pc*measurements.Pc;      
      if(tempFloat>=0.0f)arm_sqrt_f32(tempFloat,(float*)&measurements.Qc);
      else tempFloat=0;      
      measurements.Qabc=measurements.Qa+measurements.Qb+measurements.Qc;
      
      measurements.nomTorq=9.55f*engine.nomPower/engine.nomSpeed;       
      float nomSyncSpeed=engine.nomFreq*120.0f/(float)engine.polesPairsNum;
      float curSyncSpeed=((float)measurements.curFreq)*1.2f/(float)engine.polesPairsNum;     
      float slip=(nomSyncSpeed-engine.nomSpeed);
      float currentPercent=((measurements.Iabc/3.0f)-engine.Iidle)/(engine.Inom-engine.Iidle);     
      measurements.curSpeed=curSyncSpeed-slip*currentPercent;      
      measurements.curTorq=measurements.Pabc*engine.efficiency*0.0955f/measurements.curSpeed;
      measurements.torqPercent=100.0f*measurements.curTorq/measurements.nomTorq;  
   
      if(alarms.noLoadWattsSec)
      {
        if(measurements.Pabc<alarms.noLoadWatts)
        {
          alarms.noLoadWattsSecCnt=alarms.noLoadWattsSecCnt+(float)dTms;
          if(alarms.noLoadWattsSec<((int16_t)(alarms.noLoadWattsSecCnt/1000.0f)))
          {
            DRIVER_STOP(1);    
            eventsCnt.noLoad++;
            eventLogWrite(10); 
            alarms.faultBitsExtended|=0x0002;            
            vectorPWM.outputsCommand=2;                                 
            systemState.faults=(1L<<NO_LOAD_FAULT);     
            forceBVI(8, 0);           
            FAULT_OPTO_OUT(1);
          }      
        }else alarms.noLoadWattsSecCnt=0.0f;
      }else alarms.noLoadWattsSecCnt=0.0f;
      
      measurements.cosPhiA=measurements.Pa/measurements.Sa;
      measurements.cosPhiB=measurements.Pb/measurements.Sb;
      measurements.cosPhiC=measurements.Pc/measurements.Sc;
      measurements.cosPhiAvg=(measurements.cosPhiA+measurements.cosPhiB+measurements.cosPhiC)/3.0f;      
      measurements.sampNum[measurements.energyRdy-1]=0;                  
      if(measurements.periodLast) // Если вклинилось более высокоприоритетное прерывание
      { 
        if(diag.sequence_EnergyCalc!=2)
        {
          diag.seqErr_EnergyCalc++;
          eventsCnt.adc_energy_err++;
        }
      }
      diag.sequence_EnergyCalc=0;          
    }else{
      alarms.breakCurAMinCounterMS=0; 
      alarms.breakCurBMinCounterMS=0; 
      alarms.breakCurCMinCounterMS=0;     
      alarms.curDisbalanceMaxCounterMS=0;   
      alarms.noLoadWattsSecCnt=0.0f;      
      measurements.periodLast=1;      
    }
    prevTick=curTick;
    measurements.energyRdy=0;    
  }
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct)) 
  {
    systemState.eventLogHardwareFault=2; // Ошибка инициализации системного кварцевого резонатора
    eventsCnt.HSE_init_err++;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;//ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;//EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);  
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ENABLE;//DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc2);

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  HAL_ADCEx_MultiModeConfigChannel(&hadc2, &multimode);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

}

/* ADC3 init function */
void MX_ADC3_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION12b;
  hadc3.Init.ScanConvMode = ENABLE;//DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc3);

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
  
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;  
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;  //  TIM_COUNTERMODE_CENTERALIGNED2 - только, прерывание по регистру сравнения только когда счетчик наращивается
  htim1.Init.Period = vectorPWM.arr;//8399;//10499;//8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime =148;//148 - 1us//192;-1.5us//126;//84;//210; //210 - примерно 2.4 мкс;      1 тик - 84 МГц
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM2; // В режиме pwm2 - счетчик наращивается с 0, чем больше регистр сравнения, тем длительность импульса меньше !!!
  sConfigOC.Pulse = 7000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.Pulse = 4000;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  sConfigOC.Pulse = 1000;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  
  sConfigOC.Pulse = 500;    
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
  vectorPWM.cc[3]=sConfigOC.Pulse;
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
}

/* TIM4 init function */
/*
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 168;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;//10000;  for Wi-Fi module
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 5000;//1000; for wi-fi module
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
}*/

void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 168;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;//10000;  for Wi-Fi module
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim5);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim5);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 5000;//1000; for wi-fi module
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
}
/* TIM5 init function */
/*void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 168;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim5);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 50000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
}*/

void DS18B20_UART4_Init(uint32_t baudRate)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = baudRate;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);  
}

/* USART3 init function */

/*
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;//_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}

*/






/* USART2 init function */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2; 
  #ifndef WI_FI_DEBUG 
    huart2.Init.BaudRate = 19200;//115200;
    RS485_DIR(0); // Прием  
  #else 
    huart2.Init.BaudRate = 115200;
    RS485_DIR(1); // Передача  
  #endif
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
  
}

/* UART3 init function */
/*       для SIM800                      */
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

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  
  /*
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);   //UART3 НАДО БУДЕТ ЗАКОМИТЬ
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  */
  
  
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);   //UART2
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /*
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0); // For UART5 (WI-FI module) для sim DMA не нужен
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);  
  */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();    
  __GPIOC_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  
  __GPIOF_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();  
  
  //Configure GPIO pin : PF15  L-активный FAULT
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  // TC1-TC14: PG2-PG15; TC15-PG1; 
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5  | 
                        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | 
                        GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  /*Configure GPIO pin : PF12 */  // F-RESET
  /*Configure GPIO pin : PF11 */  // STOP
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; //high
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  DRIVER_STOP(0);
  DRIVER_F_RESET(0);

  /*Configure GPIO pins : PD10-LED1GREEN, PD13-LED1RED, PD14-LED2GREEN, PD15-LED2RED */ 
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pins : ESP8266EX !RESET*/
  /*Configure GPIO pins : sim800  rts pwr_k*/
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_8; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; 
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
  
  /*Configure GPIO pins : sim800  pwr*/
   GPIO_InitStruct.Pin = GPIO_PIN_8; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; 
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
   /* Сигналы CTS DCD на плате подтянуты к питанию*/
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 ;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  
  /*Configure GPIO pin : PG0-RCHARGE, PE7-THYRISTOR, PE15 - FAN*/
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
    /* Сигналы ошибок с драйверов*/
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);  
  
  /* Сигнал на оптрон АВАРИЯ+/-*/
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  
  
   /* PD0, PD1, PD3, PA15 - DO2, DO3, DO4, DO1*/
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);       
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  //--------------- Temperature sensors mux --------------------
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);    
  //------------------------------------------------------------
  //---------------- FRAM MEMORY--------------------------------
  GPIO_InitStruct.Pin = GPIO_PIN_0; // FRAM_WP
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
  GPIO_InitStruct.Pin = GPIO_PIN_8; // FRAM_I2C_SCL
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;//FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
  
  GPIO_InitStruct.Pin = GPIO_PIN_9; // FRAM_I2C_SDA
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
  //------------------------------------------------------------   
  
  
  
  THYRISTOR_ON(0);
  RCHARGE_ON(0);
  FAN_ON(0);
  FAULT_OPTO_OUT(0);
  DOUT1(0);
  DOUT2(0);  
  DOUT3(0);
  DOUT4(0);   
  stateDOUT=0x00;
  LED1_GREEN(0);
  LED1_RED(0);
  LED2_GREEN(0);
  LED2_RED(0);        
  //ESP_RESET(1);
  Delay_us(10000);
  //ESP_RESET(0);
}
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
  if(huart->Instance==USART2)
  {
    rs485RxBuf[rs485RxPtr++]=rs485InputByte;
    if(rs485RxPtr>=COM_RX_BUF_SIZE)
    {
      rs485RxPtr=0;
    }
    HAL_UART_Receive_IT(&huart2, &rs485InputByte, 1); // Каждый раз принимать один байт  
    LED1_GREEN(1); 
    HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);  
    __HAL_TIM_SET_COUNTER(&htim2,0x0001);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);    
  }
  /*if(huart->Instance==USART3)
  {
    rs232RxBuf[rs232RxPtr++]=rs232InputByte;
    if(rs232RxPtr>=COM_RX_BUF_SIZE)
    {
      rs232RxPtr=0;
    }
    HAL_UART_Receive_IT(&huart3, &rs232InputByte, 1); // Каждый раз принимать один байт  
    LED1_GREEN(1); 
    HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);  
    __HAL_TIM_SET_COUNTER(&htim4,0x0001);
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);    
  }*/
  
  /*
  if(huart->Instance==UART5)
  {
    wifiRxBuf[wifiRxPtr++]=wifiInputByte;
    if(wifiRxPtr>=COM_RX_BUF_SIZE)
    {
      wifiRxPtr=0;
    }
    HAL_UART_Receive_IT(&huart5, &wifiInputByte, 1); // Каждый раз принимать один байт  
    LED1_GREEN(1); 
    HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);  
    __HAL_TIM_SET_COUNTER(&htim5,0x0001);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);    
  } 
  */

  if(huart->Instance==USART3)
  {
    RxSim800Handler();
      
  } 
  

  
  
  
  
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /*
  if(huart->Instance==USART3)
  {    
    LED2_GREEN(0);    
    //rs232_Ready=1;
    diag.rs232_transPacketNum++;
  }
  
  */
  
   if(huart->Instance==USART3)
  {
    TxSim800Handler();
      
  } 
  
  if(huart->Instance==USART2)
  {
    LED1_RED(0);
    #ifndef WI_FI_DEBUG 
      RS485_DIR(0);
    #else
      rs485TxReady=1;
      RS485_DIR(1);
    #endif
    diag.rs485_transPacketNum++;
  }
  
  /*
  if(huart->Instance==UART5)
  {    
    LED2_GREEN(0);       
    diag.wifi_transPacketNum++;
    waitForWiFiFlag=1; // Запустить тайм-аут по wi-fi
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
    wifiRxPtr=0;
    wifiDataReady=0;              
  }
  */

  
  
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{ 
  
  /*
  if(huart->Instance==USART3)
  {
    HAL_UART_DeInit(&huart3);
    MX_USART3_UART_Init();
    //HAL_UART_Receive_IT(&huart3, &rs232InputByte, 1); // Каждый раз принимать один байт                        
    diag.rs232_IRQ_ERR++;
  }
  */
  
  if(huart->Instance==USART2)
  {    
    #ifndef WI_FI_DEBUG 
      HAL_UART_DeInit(&huart2);
      MX_USART2_UART_Init();
      HAL_UART_Receive_IT(&huart2, &rs485InputByte, 1); // Каждый раз принимать один байт        
      diag.rs485_IRQ_ERR++;
    #else
      HAL_UART_DeInit(&huart2);
      MX_USART2_UART_Init();
    #endif
  } 
  
  /*
  if(huart->Instance==UART5)
  {
    HAL_UART_DeInit(&huart5);
    MX_UART5_Init();
    HAL_UART_Receive_IT(&huart5, &wifiInputByte, 1); // Каждый раз принимать один байт                    
    diag.wifi_IRQ_ERR++;
  } 
  */
  
  
  if(huart->Instance==USART3)
  {
    HAL_UART_DeInit(&huart3);
    MX_UART3_Init();                 
    HAL_UART_Receive_IT(&huart3,&sim800Data,1);    // Каждый раз принимать один байт 
  } 
  
  
  
}

float id,iq;
float vd,vq;
float scalarProduct;
float pErr;
float pErrPrev=0.0f;
uint8_t pErrPrevReady=0;

float scalarProductSum=0.0f;
float scalarProductMean=0.0f;
float slideWindow[101];
uint16_t slideWindowPtr=0;
uint8_t slideWindowFull=0;

double prevDP=0;

uint16_t prevPidPrescaler=0;
uint16_t tcTemp=0;
uint16_t tcStart=0;

uint8_t voltageStabHyst=0;
uint16_t voltageStabShift=0;

uint8_t powerStabTrigger=0; // Триггер стабилизации мощности в генераторном режиме  

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{   
  uint16_t ia,ib,ic,udc;
  uint32_t tempUINT32;        
  if(diag.sequenceADCPWM!=2)
  {
    diag.seqErr_ADC_PWM++;
    if(measurements.meanReady)eventsCnt.adc_pwm_err++;
  }
  diag.sequenceADCPWM=3;
  
  ia=adc_data[0];
  ib=adc_data[1];
  ic=adc_data[2];
  adcInputs.adcData[0]=adc_data[6];
  adcInputs.adcData[1]=adc_data[3];  
  adcInputs.adcData[2]=adc_data[4];
  adcInputs.adcData[3]=adc_data[7];  
  udc=(adc_data[5]+adc_data[8])/2; 
  
  if(measurements.meanReady)
  {        
    measurements.InstPhACurArray[measurements.InstPhCurSampIndex]=(-measurements.meanIaCode+ia);
    measurements.InstPhBCurArray[measurements.InstPhCurSampIndex]=(-measurements.meanIbCode+ib);
    measurements.InstPhCCurArray[measurements.InstPhCurSampIndex]=(-measurements.meanIcCode+ic);        
    measurements.InstUdcArray[measurements.InstPhCurSampIndex]=udc;
    
    vectorPWM.ia=measurements.InstPhACurArray[measurements.InstPhCurSampIndex];
    vectorPWM.ib=measurements.InstPhBCurArray[measurements.InstPhCurSampIndex];
    vectorPWM.ic=measurements.InstPhCCurArray[measurements.InstPhCurSampIndex];
      
    measurements.InstPhACurSum+=measurements.InstPhACurArray[measurements.InstPhCurSampIndex];
    measurements.InstPhBCurSum+=measurements.InstPhBCurArray[measurements.InstPhCurSampIndex];
    measurements.InstPhCCurSum+=measurements.InstPhCCurArray[measurements.InstPhCurSampIndex];
    measurements.InstUdcSum+=measurements.InstUdcArray[measurements.InstPhCurSampIndex];  
    
    measurements.InstPhCurSampIndex++;
    
    if(measurements.InstPhCurSampIndex>=InstPhCurSampNum)    
    {
      measurements.InstPhCurSampIndex=0;
      measurements.InstPhCurReady=1;
    } 
    
    if(measurements.InstPhCurReady)
    {       
      uint32_t tempU32=819*InstPhCurSampNum;
      measurements.InstPhACur=(int16_t)((5000.0f*measurements.InstPhACurSum)/(float)tempU32); //2500*1000/(4095*10); // В мА/10, с учетом делителя 10мВ/А      
      measurements.InstPhBCur=(int16_t)((5000.0f*measurements.InstPhBCurSum)/(float)tempU32);
      measurements.InstPhCCur=(int16_t)((5000.0f*measurements.InstPhCCurSum)/(float)tempU32);         
              
      measurements.InstUdc=(int16_t)(0.2442f*(measurements.InstUdcSum/InstPhCurSampNum));   // 1000В/4095=0.2442 - без учета калибровки
     
      measurements.InstPhACurSum-=measurements.InstPhACurArray[measurements.InstPhCurSampIndex];
      measurements.InstPhBCurSum-=measurements.InstPhBCurArray[measurements.InstPhCurSampIndex];      
      measurements.InstPhCCurSum-=measurements.InstPhCCurArray[measurements.InstPhCurSampIndex];      
      measurements.InstUdcSum-=measurements.InstUdcArray[measurements.InstPhCurSampIndex];        
      arm_rms_q15((q15_t*)&measurements.InstPhACur,3,(q15_t*)&measurements.Irms);

/*----------------------------------------------------------------------------*/      
      if(etrEngineShadow.updateFromShadow==2) // Произошли изменения в настройках двигателя по модбас   
      {   
        etrEngine.C1=etrEngineShadow.C1;
        etrEngine.C2=etrEngineShadow.C2;
        etrEngine.C3=etrEngineShadow.C3;        
        etrEngine.k1=etrEngineShadow.k1;
        etrEngine.k2=etrEngineShadow.k2;
        etrEngine.k3=etrEngineShadow.k3;        
        etrEngine.overloadCurrent=etrEngineShadow.overloadCurrent;
        etrEngine.overloadLimit=etrEngineShadow.overloadLimit;          
        copymas((uint8_t*)&etrEngine,(uint8_t*)&etrEngineShadow,ETR_ENGINE_MODBUS_REGNUM*2);
        etrEngineShadow.updateFromShadow=0;
      }                   
      
      if(etrEngine.isOn==1)
      {
        etrEngine.Isum=etrEngine.Isum+(uint32_t)measurements.Irms;
        etrEngine.IsumCnt++;
        if(etrEngine.IsumCnt>9) // Количество семплов для для одного значения счетчика ЭТР, удобно взять 10, чтобы получить мА из десяти мА/10
        {                 
          if(vectorPWM.isOn)
          {
            etrEngine.I=((float)etrEngine.Isum)/1000.0f;                 // Измеренный ток в Амперах                  
            if(etrEngine.externalCoolingOn==0)
            {             
              etrEngine.I=etrEngine.I/freqCorrection(vectorPWM.curFreq);         
            }
            if(etrEngine.I>etrEngine.overloadCurrent)etrEngine.overloadCnt=etrEngine.overloadCnt+((float)(10.f*etrEngine.I*etrEngine.I))/((float)engine.freqPWM);   // Получаем размерность А^2*dT
            else{
              if(etrEngine.overloadCnt>0.0f)etrEngine.overloadCnt=etrEngine.overloadCnt-(etrEngine.overloadCurrent*etrEngine.overloadCurrent)/((float)engine.freqPWM);
            }          
          }else{                      
            if(etrEngine.overloadCnt>0.0f)etrEngine.overloadCnt=etrEngine.overloadCnt-(etrEngine.overloadCurrent*etrEngine.overloadCurrent)/((float)engine.freqPWM); 
          }        
            
          if(vectorPWM.isOn)
          {
            if(etrEngine.overloadCnt>etrEngine.overloadLimit)
            {
              DRIVER_STOP(1);          
              alarms.faultBits|=0x0200;   
              eventLogWrite(5);
              vectorPWM.outputsCommand=2;
              eventsCnt.etr++;              
              systemState.faults=(1L<<ETR_OVERLOAD_FAULT);        
              forceBVI(8, 0);
              FAULT_OPTO_OUT(1);              
            }  
          }
          etrEngine.Isum=0;
          etrEngine.IsumCnt=0;                     
        }        
      }else if(etrEngine.isOn==2)etrEngine.overloadCnt=0.0f;
      
    
      
/*----------------------------------------------------------------------------*/      
      /*
      #define dV          20     // +/-dV - в данных пределах ошибки нет стабилизации напряжения
      #define dCode       61*dV  // dV в кодах ШИМ
      */
      
      if(measurements.InstUdc>=Ustop)
      {
          if(vectorPWM.isOn)
          {            
            DRIVER_STOP(1);          
            alarms.faultBits|=0x0008;  
            eventLogWrite(4);
            vectorPWM.outputsCommand=2;              
            eventsCnt.overVoltage++;
            systemState.faults=(1L<<UDC_OVERVOLTAGE_FAULT);        
            forceBVI(8, 0);
            FAULT_OPTO_OUT(1);            
          }
      }else{      
        //pidVoltage.UdcSum+=measurements.InstUdc;
        //pidVoltage.samplesCnt++;
        //if(pidVoltage.samplesCnt>=1)//10)     // Регулировка раз в 10 периодов ШИМ
        //{                                     // Регулировка в 1 мс вызывает колебания тока двигателя при заданном коэфф df от dU, при уменьшении време
          pidVoltage.Udc=measurements.InstUdc;  //pidVoltage.UdcSum/pidVoltage.samplesCnt;
          pidVoltage.UdcSum=0;
          pidVoltage.samplesCnt=0;       
          pidVoltage.UdcRdy=1;              
        //}        
        
        if(systemState.Ready==0) // Контроль заряда DC-шины перед включением тиристоров и выключением токоограничевающего резистора
        {
          if(pidVoltage.UdcRdy)
          {          
            systemState.Ready=1;
            if(pidVoltage.Udc>=Uunfix)
            {
              THYRISTOR_ON(1);
              RCHARGE_ON(1);
              //FAN_ON(1); <--- теперь управление по датчику температуры радиатора                  
              systemState.thyristorON=1;
              //systemState.Ready=1;                            
            }else{
              eventsCnt.lowVoltage++;
              alarms.faultBitsExtended|=0x0001;                         
              systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                      
              eventLogWrite(9);  
              forceBVI(8, 0);
              FAULT_OPTO_OUT(1);              
            }
          }                
        } 
        
        if(pidVoltage.UdcRdy)
        {
          if(pidVoltage.Udc<Umin)
          {
            if(vectorPWM.isOn)
            { 
              DRIVER_STOP(1);        
              eventsCnt.lowVoltage++;
              alarms.faultBitsExtended|=0x0001; 
              eventLogWrite(9);
              vectorPWM.outputsCommand=2;                                 
              systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                
              forceBVI(8, 0);
              FAULT_OPTO_OUT(1);              
            }
          }         
        }
          if(vectorPWM.isOn)
          {        
            if(measurements.maxIstartFlag)
            {
              if(measurements.maxIstartValueTemp<measurements.Irms)
              {
                measurements.maxIstartValueTemp=measurements.Irms;
              }
            }
            
            id=-0.00577350f*((float)measurements.InstPhACur+2.0f*(float)measurements.InstPhBCur);            
            iq=(float)measurements.InstPhACur/100.0f;
            vd=0.5773503f*(float)measurements.InstUdc*(measurements.tempUc-measurements.tempUb);
            vq=0.3333333f*(float)measurements.InstUdc*(2.0f*measurements.tempUa-measurements.tempUb-measurements.tempUc);
            scalarProduct=1.5f*(id*vd+iq*vq);
            
            if(pErrPrevReady)
            {
              if(prevPidPrescaler!=command.pidPrescaler) // Для того, чтобы можно было менять предделитель во время работы
              {
                slideWindowFull=0;
                slideWindowPtr=0;
                scalarProductSum=0;
                prevPidPrescaler=command.pidPrescaler;
              }
            }
              
            slideWindow[slideWindowPtr]=scalarProduct;            
            scalarProductSum+=scalarProduct;
            slideWindowPtr++;

            if(slideWindowPtr>=command.pidPrescaler)
            {
              slideWindowFull=1;
              slideWindowPtr=0;
            }           
            
            if(slideWindowFull)
            {
              scalarProductMean=scalarProductSum/(float)command.pidPrescaler;
              scalarProductSum-=slideWindow[slideWindowPtr]; // Вычесть из суммы выпавший из кольцевого массива элемент
            }else{
              scalarProductMean=scalarProductSum/(float)slideWindowPtr;
            } 
                         
            measurements.scalarProduct=scalarProductMean;
            
            if(measurements.maxIstartFlag) // Флаг активен в процессе пуска
            {
              if(powerStabTrigger==0)
              {
                command.powerStabDelta=command.powerStab/((float)(context.Tl_d+context.Ts_d)); // Линейное увеличение стабилизируемой мощности
                command.powerStabSum=0.0f;
                powerStabTrigger=1;
              }else{
                command.powerStabSum+=command.powerStabDelta;
              }
              if(command.freeRunState)pErr=0.0f-scalarProductMean;
              else pErr=command.powerStabSum-scalarProductMean;              
            }else{
                if(command.freeRunState)pErr=0.0f-scalarProductMean;
                else pErr=command.powerStab-scalarProductMean; 
                powerStabTrigger=0;
            }
            
            
            if(pErr>0.0f)
            {
              if(pErr<(float)command.powerInsensitiveDelta)pErr=0.0f;
            }else{
              if(pErr>(float)(-1*command.powerInsensitiveDelta))pErr=0.0f;               
            }       
            
            vectorPWM.activePddPhi+=(command.Ki*pErr);            
            vectorPWM.activePdPhi=(command.Kp*pErr+vectorPWM.activePddPhi)/1000000.0f;
            if(pErrPrevReady)
            {
              vectorPWM.activePdPhi+=command.Kd*(pErr-pErrPrev)/1000000.0f;        
              if(vectorPWM.activePdPhi>prevDP)
              {
                if((vectorPWM.activePdPhi-prevDP)>command.dphiLimit)
                {                
                  vectorPWM.activePdPhi=prevDP+(double)command.dphiLimit;
                }
              }else{
                if((prevDP-vectorPWM.activePdPhi)>command.dphiLimit)
                {                
                  vectorPWM.activePdPhi=prevDP-(double)command.dphiLimit;
                }                
              }
            }            
            pErrPrev=pErr;
            pErrPrevReady=1;                        
            prevDP=vectorPWM.activePdPhi;            
            if(vectorPWM.activePdPhi<0.0){vectorPWM.activePdPhi=0;vectorPWM.activePddPhi=0;}            
            prevPidPrescaler=command.pidPrescaler;

            if(command.voltageStab)
            {              
              if(voltageStabHyst)voltageStabShift=command.voltageStabPowerLow;              
              else voltageStabShift=command.voltageStabPowerHigh;
              if((int16_t)scalarProductMean>(int16_t)voltageStabShift)
              {
                pidVoltage.pwmCorrection=(int16_t)(vectorPWM.amplitudePWM*(1.0f-(float)pidVoltage.Udc/537.0f));// 32767/537=61.018622 -  коэффициент перевода из напряжения в код ШИМ                           
                voltageStabHyst=1;
              }else{
                voltageStabHyst=0;              
              }
            }else pidVoltage.pwmCorrection=0;
            
            //------- Аккумулятор активной мощности ----------------------------            
            if(scalarProductMean>0.0f)activePower.instantSum+=scalarProductMean; // Складывать только положительную энергию
            activePower.sampNum++;
            if(activePower.sampNum>engine.freqPWM)
            {
              activePower.secondsSum=activePower.instantSum;
              activePower.instantSum=0;
              activePower.sampNum=1;
              activePower.secondsSumRdy=1;
            }
            //------------------------------------------------------------------  
            //----------------- Определение размаха гармоники 100 Гц ------------- 
            udc_1kHz_decimator++;
            if(udc_1kHz_decimator>udc_1kHz_decimator_max)
            {
              udc_1kHz_decimator=0;
              if(udc_1kHz_ptr<100)
              {
                udc_1kHz_data[udc_1kHz_ptr]=measurements.InstUdc;
                udc_1kHz_ptr++;
              }            
            }                   
            
            if(alarms.fixFreqLowVoltageMS)
            {
              if((fixFreq[0].fixFreqMode==0)||(fixFreq[0].fixFreqControl==2))
              {
                if(measurements.InstUdc<alarms.fixFreqLowVoltageValue)
                {
                  alarms.fixFreqLowVoltageCnt++;                   
                  if((uint32_t)alarms.fixFreqLowVoltageMS<(((uint32_t)alarms.fixFreqLowVoltageCnt*1000L)/(uint32_t)engine.freqPWM))  // Время удержания для перехода на фиксированную частоту                           
                  {
                    if(fixFreq[0].fixFreqControl==2)
                    {
                      eventsCnt.fixFreqLowVoltage++;
                      alarms.faultBitsExtended|=0x0008;                         
                      systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                        
                      eventLogWrite(20);  
                      forceBVI(8, 0);
                      FAULT_OPTO_OUT(1);                       
                    }else{
                      if(alarms.fixFreqEnable) // Разрешен переход на фиксированную частоту
                      {                    
                        fixFreq[0].fixFreqMode=1;
                        fixFreq[0].eventTimeStamp=RTC_UnixTime.timeStamp;
                        fixFreq[0].fixFreqAttempt++;
                        fixFreq[0].CRC16=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH);                                     
                        if(fixFreq[0].fixFreqAttempt>alarms.fixFreqAttemptsNumMax) // Счетчик превышен, сработала авария
                        {
                          eventsCnt.fixFreqLowVoltage++;
                          alarms.faultBitsExtended|=0x0008;                         
                          systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                        
                          eventLogWrite(20);  
                          forceBVI(8, 0);
                          FAULT_OPTO_OUT(1);                      
                        }else{
                          alarms.faultBitsExtended|=0x0008;  
                          eventLogWrite(21); 
                        }
                      }else{                   
                        eventsCnt.fixFreqLowVoltage++;
                        alarms.faultBitsExtended|=0x0008;                         
                        systemState.faults=(1L<<LOW_VOLTAGE_FAULT);                        
                        eventLogWrite(20);  
                        forceBVI(8, 0);
                        FAULT_OPTO_OUT(1);                      
                      }
                    }
                  }                              
                }else alarms.fixFreqLowVoltageCnt=0;
              }else alarms.fixFreqLowVoltageCnt=0;
            }            
            //--------------------------------------------------------------------             
          }else{
            scalarProductSum=0.0f;
            slideWindowPtr=0;
            slideWindowFull=0;          
          }                       
      }
    
      if(alarmsShadow.updateFromShadow==2)      
      {               
        alarms.InstPhCurMaxSumA=0;
        alarms.InstPhCurMaxSumB=0;
        alarms.InstPhCurMaxSumC=0;
        alarms.InstPhCurMaxCnt=0;         
        
        alarms.IrmsMax=alarmsShadow.IrmsMax;
        alarms.IrmsMaxMS=alarmsShadow.IrmsMaxMS;
        alarms.IrmsMaxCounter=0;           
       
        alarms.fixFreqAttemptsNumMax=alarmsShadow.fixFreqAttemptsNumMax;
        alarms.fixFreqAttemptsNumResetMin=alarmsShadow.fixFreqAttemptsNumResetMin;
        alarms.fixFreqAttemptsPeriodMin=alarmsShadow.fixFreqAttemptsPeriodMin;
        alarms.fixFreqLowVoltageMS=alarmsShadow.fixFreqLowVoltageMS;
        alarms.fixFreqLowVoltageValue=alarmsShadow.fixFreqLowVoltageValue;
        alarms.fixFreqValue=alarmsShadow.fixFreqValue;
         
        alarmsShadow.updateFromShadow=3;        
      }      
         
      alarms.InstPhCurMaxSumA+=measurements.InstPhACur;
      alarms.InstPhCurMaxSumB+=measurements.InstPhBCur;
      alarms.InstPhCurMaxSumC+=measurements.InstPhCCur;
      alarms.InstPhCurMaxCnt++;
      
      if(alarms.InstPhCurMaxCnt>1) // Береться сумма 2 отсчетов по мгновенному максимальному току, жестко, проверено экспериментально 22.03.2018                         
      {
        int32_t tempInt32;        
        tempInt32=alarms.InstPhCurMaxSumA/alarms.InstPhCurMaxCnt;
        if(tempInt32<0)tempInt32*=-1;
        if(tempInt32>chrep.maxInstCurrent)
        {
          measurements.InstPhACur=(int16_t)tempInt32;
          eventLogWrite(1);
          eventsCnt.instA++;
          DRIVER_STOP(1);          
          alarms.faultBits|=0x0010;            
          vectorPWM.outputsCommand=2;                                 
          systemState.faults=(1L<<BRIDGE_SATURATION_FAULT);        
          forceBVI(8, 0);
          FAULT_OPTO_OUT(1);          
        }else{
          tempInt32=alarms.InstPhCurMaxSumB/alarms.InstPhCurMaxCnt;
          if(tempInt32<0)tempInt32*=-1;
          if(tempInt32>chrep.maxInstCurrent)
          {
            measurements.InstPhBCur=(int16_t)tempInt32;
            eventLogWrite(1);
            eventsCnt.instB++;
            DRIVER_STOP(1);          
            alarms.faultBits|=0x0020;            
            vectorPWM.outputsCommand=2;                                 
            systemState.faults=(1L<<BRIDGE_SATURATION_FAULT);        
            forceBVI(8, 0);
            FAULT_OPTO_OUT(1);          
          }else{
            tempInt32=alarms.InstPhCurMaxSumC/alarms.InstPhCurMaxCnt;
            if(tempInt32<0)tempInt32*=-1;
            if(tempInt32>chrep.maxInstCurrent)
            {           
              measurements.InstPhCCur=(int16_t)tempInt32;
              eventLogWrite(1); 
              eventsCnt.instC++;
              DRIVER_STOP(1);          
              alarms.faultBits|=0x0040;            
              vectorPWM.outputsCommand=2;                                 
              systemState.faults=(1L<<BRIDGE_SATURATION_FAULT);        
              forceBVI(8, 0);
              FAULT_OPTO_OUT(1);          
            }      
          }
        }
        alarms.InstPhCurMaxCnt=0;
        alarms.InstPhCurMaxSumA=0;
        alarms.InstPhCurMaxSumB=0;
        alarms.InstPhCurMaxSumC=0;
      }
/*----------------------------------------------------------------------------*/                  
      if(alarms.IrmsMaxMS)
      {
        if(vectorPWM.isOn)
        {
          if(measurements.Irms>alarms.IrmsMax)
          {
            alarms.IrmsMaxCounter++;  
            if((uint32_t)alarms.IrmsMaxMS<(((uint32_t)alarms.IrmsMaxCounter*1000L)/(uint32_t)engine.freqPWM))
            {                 
              eventLogWrite(2); 
              eventsCnt.rms++;
              DRIVER_STOP(1);          
              alarms.faultBits|=0x0080;            
              vectorPWM.outputsCommand=2;                                 
              systemState.faults=(1L<<BRIDGE_SATURATION_FAULT);        
              forceBVI(8, 0);
              FAULT_OPTO_OUT(1);              
            }
          }else{      
            alarms.IrmsMaxCounter=0;
          }
        }else alarms.IrmsMaxCounter=0; 
      }else alarms.IrmsMaxCounter=0;
      
      if(measurements.indexToProcess<2)
      {
        measurements.sumIa[measurements.indexToProcess]+=measurements.InstPhACur*measurements.InstPhACur/10000.0f; // Перевод из ма/10 (int16)->А(float), 
        measurements.sumIb[measurements.indexToProcess]+=measurements.InstPhBCur*measurements.InstPhBCur/10000.0f;
        measurements.sumIc[measurements.indexToProcess]+=measurements.InstPhCCur*measurements.InstPhCCur/10000.0f;
        
        tempUINT32=measurements.InstUdc*measurements.InstUdc;        
        measurements.sumUdc[measurements.indexToProcess]+=measurements.InstUdc;
        measurements.sumUa[measurements.indexToProcess]+=(measurements.tempUa*measurements.tempUa*tempUINT32);
        measurements.sumUb[measurements.indexToProcess]+=(measurements.tempUb*measurements.tempUb*tempUINT32);
        measurements.sumUc[measurements.indexToProcess]+=(measurements.tempUc*measurements.tempUc*tempUINT32);
        
        measurements.sumPa[measurements.indexToProcess]+=measurements.tempUa*measurements.InstUdc*measurements.InstPhACur/100.0f;
        measurements.sumPb[measurements.indexToProcess]+=measurements.tempUb*measurements.InstUdc*measurements.InstPhBCur/100.0f;
        measurements.sumPc[measurements.indexToProcess]+=measurements.tempUc*measurements.InstUdc*measurements.InstPhCCur/100.0f;
        
        measurements.sampNum[measurements.indexToProcess]++;
        
        if(measurements.uMAX<measurements.InstUdc)measurements.uMAX=measurements.InstUdc;
        if(measurements.uMIN>measurements.InstUdc)measurements.uMIN=measurements.InstUdc;
        if(measurements.iMAX<measurements.Irms)measurements.iMAX=measurements.Irms;
        if(measurements.iMIN>measurements.Irms)measurements.iMIN=measurements.Irms;
          
        if(measurements.endOfPeriodCnt>=measurements.endOfPeriodsNum)  
        {         
          if(measurements.indexToProcess){measurements.indexToProcess=0;measurements.energyRdy=2;}
          else{measurements.indexToProcess=1;measurements.energyRdy=1;}                
          measurements.endOfPeriodCnt=0;   
          
          if(measurements.periodLast)
          {
            if(diag.sequence_EnergyCalc!=0)
            {
              diag.seqErr_EnergyCalc++;
              eventsCnt.adc_energy_err++;
            }
            diag.sequence_EnergyCalc=1;  
            
            measurements.maxUdc=measurements.uMAX;
            measurements.minUdc=measurements.uMIN;
            measurements.maxIrms=measurements.iMAX/100;
            measurements.minIrms=measurements.iMIN/100;
            measurements.uMAX=0;
            measurements.iMAX=0;
            measurements.uMIN=32767;
            measurements.iMIN=32767;            
          }
        }
        if(vectorPWM.isOn==0)
        {
          measurements.periodLast=0;              
          alarmsResetCounters();  
          measurements.energyRdy=0;
        }
      }       
      
      switch(dynaGram.state)
      {	  
        case 0: // Ожидание команды на снятие динамограммы, запуск таймера		        
        case 1: // Ожидание команды на снятие динамограммы, ожидание dynagramUpdateSec секунд		        		
        break;
        case 2: // Ожидание срабатывания датчика положения, защита от дребезга
          if((GPIOG->IDR)&0x0800)// Если истина, то датчик не сработал
          {
            dynaGram.TC10retryCnt=0;
          }else{
            dynaGram.TC10retryCnt++;
            if(dynaGram.TC10retryCnt>=dynaGram.detectorRetryNum)
            {
              dynaGram.state=3;
            }            
          }
        break;	
        case 3: // Накопление "сырых" данных 
          dynaGram.dataUpdatePWMCnt++;
          dynaGram.minPeriodOfDynagramCnt++;
          if(dynaGram.tit3Cnt<dynaGram.tit3SamplesNum)
          {
            dynaGram.tit3Sum+=adcInputs.adcData[2];
            dynaGram.tit3Cnt++;			
          }
          if(dynaGram.dataUpdatePWMCnt>=dynaGram.dataUpdatePWMCycles)		
          {
            dynaGram.dataUpdatePWMCnt=0;
            if(dynaGram.dynagramSource)
            {
              dynaRawData[dynaGram.rawDataPtr].activePower=dynaGram.tit3Sum/dynaGram.tit3Cnt;				  
              dynaGram.tit3Sum=0;
              dynaGram.tit3Cnt=0;
            }else{
              dynaRawData[dynaGram.rawDataPtr].activePower=(int16_t)(scalarProductMean/2.0f);
            }
            dynaRawData[dynaGram.rawDataPtr].phase=dynaGram.outputVoltagePhase;
            dynaGram.rawDataPtr++;		
            if(dynaGram.rawDataPtr>=DYNA_RAW_POINTS)
            {
              dynaGram.error=1;
              dynaGram.state=0;
            }		
            if((GPIOG->IDR)&0x0800)// Если истина, то датчик не сработал
            {
               dynaGram.TC10retryCnt=0;
            }else{
              dynaGram.TC10retryCnt++;
              if(dynaGram.TC10retryCnt>=dynaGram.detectorRetryNum)
              {
                if(dynaGram.minPeriodOfDynagramCnt>dynaGram.minPeriodOfDynagramCycles) 
                {
                   dynaGram.TC10retryCnt=0;							
                   dynaGram.state=4;
                }else{
                   dynaGram.TC10retryCnt=0;
                }
              }                
            }
          }		
        break;
      }  
      
      if(engine.debugMode==0) 
      {
        decimatorCnt++;
        if(decimatorCnt>=decimatorRatio)
        {    
          decimatorCnt=0;

          tcTemp=(uint16_t)((GPIOG->IDR)&0x0C00);
          tcStart=(uint16_t)((GPIOG->IDR)&0x0020);         
          if(tcStart==0x0020)tcTemp|=0x1000;
          if(arrayRdy)
          {            
            if(arrayRdy==1)
            {
              if(arrayPtr==0)
              {
                //*((uint32_t*)&array2[arrayPtr++])=0x00FFFFFF+packetCnt*0x1000000;
                *((uint16_t*)&array2[arrayPtr++])=0xAAAA;
                *((uint16_t*)&array2[arrayPtr++])=0xAAAA;
                *((uint16_t*)&array2[arrayPtr++])=0xAAAA;
                *((uint16_t*)&array2[arrayPtr++])=0x00AA+packetCnt*0x100;
                packetCnt++;
                if(packetCnt>99)packetCnt=0;
              }  // Признак начала блока данных           
                array2[arrayPtr++]=(int16_t)scalarProductMean;
                array2[arrayPtr++]=(int16_t)measurements.curFreq;
                array2[arrayPtr++]=(int16_t)command.powerStabSum;//(vectorPWM.curDeltaPhi*engine.freqPWM*100);//Q10;//vectorPWM.prevPhi;
                array2[arrayPtr++]=(int16_t)vectorPWM.amplitudePWMcorrected;
                array2[arrayPtr++]=(int16_t)measurements.InstPhACur;
                array2[arrayPtr++]=(int16_t)measurements.InstPhBCur;
                array2[arrayPtr++]=(int16_t)measurements.InstPhCCur;
                array2[arrayPtr++]=(int16_t)(measurements.InstUdc|tcTemp);
              if(arrayPtr>=samplesNum+4)//4)
              {
                arrayRdy=2;
                arrayTransmit=2;
                arrayPtr=0;
              }         
            }else{
              if(arrayRdy==2)
              {
                if(arrayPtr==0)
                {
                  //*((uint32_t*)&array1[arrayPtr++])=0x00FFFFFF+packetCnt*0x1000000;
                  *((uint16_t*)&array1[arrayPtr++])=0xAAAA;
                  *((uint16_t*)&array1[arrayPtr++])=0xAAAA;
                  *((uint16_t*)&array1[arrayPtr++])=0xAAAA;                
                  *((uint16_t*)&array1[arrayPtr++])=0x00AA+packetCnt*0x100;                
                  packetCnt++;
                  if(packetCnt>99)packetCnt=0;
                }  // Признак начала блока данных           
                array1[arrayPtr++]=(int16_t)scalarProductMean;
                array1[arrayPtr++]=(int16_t)measurements.curFreq;
                array1[arrayPtr++]=(int16_t)command.powerStabSum;//(vectorPWM.curDeltaPhi*engine.freqPWM*100);//Q10;//vectorPWM.prevPhi;
                array1[arrayPtr++]=(int16_t)vectorPWM.amplitudePWMcorrected;
                array1[arrayPtr++]=(int16_t)measurements.InstPhACur;
                array1[arrayPtr++]=(int16_t)measurements.InstPhBCur;
                array1[arrayPtr++]=(int16_t)measurements.InstPhCCur;
                array1[arrayPtr++]=(int16_t)(measurements.InstUdc|tcTemp);        
                if(arrayPtr>=samplesNum+4)//4)
                {
                  arrayRdy=1;
                  arrayTransmit=1;
                  arrayPtr=0;
                }                    
              }
            }
          }else{
            if(arrayPtr==0)
            {
              //*((uint32_t*)&array1[arrayPtr++])=0x00FFFFFF+packetCnt*0x1000000;
              *((uint16_t*)&array1[arrayPtr++])=0xAAAA;
              *((uint16_t*)&array1[arrayPtr++])=0xAAAA;
              *((uint16_t*)&array1[arrayPtr++])=0xAAAA;             
              *((uint16_t*)&array1[arrayPtr++])=0x00AA+packetCnt*0x100;            
              packetCnt++;
              if(packetCnt>99)packetCnt=0;
            }  // Признак начала блока данных           
            array1[arrayPtr++]=(int16_t)scalarProductMean;
            array1[arrayPtr++]=(int16_t)measurements.curFreq;
            array1[arrayPtr++]=(int16_t)command.powerStabSum;//(vectorPWM.curDeltaPhi*engine.freqPWM*100);//Q10;//vectorPWM.prevPhi;
            array1[arrayPtr++]=(int16_t)vectorPWM.amplitudePWMcorrected;
            array1[arrayPtr++]=(int16_t)measurements.InstPhACur;
            array1[arrayPtr++]=(int16_t)measurements.InstPhBCur;
            array1[arrayPtr++]=(int16_t)measurements.InstPhCCur;
            array1[arrayPtr++]=(int16_t)(measurements.InstUdc|tcTemp);
            if(arrayPtr>=samplesNum+4)//4)
            {
              arrayRdy=1;
              arrayTransmit=1;
              arrayPtr=0;
            }
          }            
       }
     }
   }     
  }else{
    measurements.meanIaCounter+=ia; 
    measurements.meanIbCounter+=ib;  
    measurements.meanIcCounter+=ic;    
    measurements.meanCodeItems++;
    if(measurements.meanCodeItems>measurements.meanCodeItemsNum) // Для того, чтобы при включении у нас были посчитаны смещения напряжения с датчиков тока, можно было управлять двигателем
    {                                    // Счетчик в кол-ве ШИМ-периодов
        measurements.meanIaCode=measurements.meanIaCounter/measurements.meanCodeItems;
        measurements.meanIbCode=measurements.meanIbCounter/measurements.meanCodeItems;
        measurements.meanIcCode=measurements.meanIcCounter/measurements.meanCodeItems;      
        measurements.meanReady=1;
        diag.seqErr_ADC_PWM=0; 
        diag.seqErr_EnergyCalc=0;  
    }    
  }
  
  if(diag.sequenceADCPWM!=3)
  {
    diag.seqErr_ADC_PWM++;
    if(measurements.meanReady)eventsCnt.adc_pwm_err++;
  }
  diag.sequenceADCPWM=4;    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   
  HAL_TIM_Base_Stop_IT(&htim3);     
}

void handlerTC(volatile uint16_t *state,volatile uint16_t *data,volatile uint16_t *reset) // Обработчик состояний дискретных входов ТС: TC1-bit0,...,TC15-bit14,FAULT-bit15
{  
  #define retryCnt 2//5//10            // Счетчик повторных сработок для фиксации уровня   
  uint16_t signalsTC; // Входные линии на мк заводятся через оптроны, сигналы инвертированы, 0 - через фотодиод оптрона течет ток, 1 - не течет...       
  static uint8_t retryON[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   // Количество повторов в состоянии, когда по оптрону течет ток    
  static uint8_t retryOFF[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // Количество повторов в состоянии, когда по оптрону не течет ток        
  uint16_t mask;  
  uint8_t i;
  
  signalsTC=(((GPIOG->IDR)>>2)&0x3FFF);    
  signalsTC|=(((GPIOG->IDR)<<13)&0x4000);    
  signalsTC|=((GPIOF->IDR)&0x8000);  
  
  for(i=0;i<16;i++)
  {
    mask=0x0001<<i;
    if((*reset)&mask)
    {
      retryOFF[i]=0;
      retryON[i]=0;
      *reset&=~mask;
      *state&=~mask;
    }
    
    if(signalsTC&mask)
    {// ТС разомкнут
      retryOFF[i]++;      
      retryON[i]=0;   // НУЖНО СБРАСЫВАТЬ!!!
      if(retryOFF[i]>retryCnt)
      {
        *data&=~mask;
        *state|=mask;
        retryOFF[i]=0;         
        TCInputs.curState[i]=0;
        TCInputs.bits=*data;
        if(TCInputs.initState[i])
        {
          if(TCInputs.curState[i]!=TCInputs.prevState[i])
          {
            TCInputs.transitionsNum[i]++;
          }
        }
        TCInputs.initState[i]=1; // Установлено начальное состояние
        TCInputs.prevState[i]=TCInputs.curState[i];
      }
    }else{// ТС замкнут
      retryON[i]++;  
      retryOFF[i]=0;  // НУЖНО СБРАСЫВАТЬ!!! 
      if(retryON[i]>retryCnt)
      {
        *data|=mask;
        *state|=mask;
        retryON[i]=0;    
        TCInputs.curState[i]=1;
        TCInputs.bits=*data;
        if(TCInputs.initState[i])
        {
          if(TCInputs.curState[i]!=TCInputs.prevState[i])
          {
            TCInputs.transitionsNum[i]++;
          }
        }
        TCInputs.initState[i]=1; // Установлено начальное состояние
        TCInputs.prevState[i]=TCInputs.curState[i];
      }      
    }
  }
  switch(tapsUpdateCnt.cntRequest)
  {
    case 1: 
      for(i=0;i<15;i++)tapsUpdateCnt.curCnt[i]=TCInputs.transitionsNum[i];
      tapsUpdateCnt.cntRequest=2;      
    break;
    case 4:
      for(i=0;i<15;i++)tapsUpdateCnt.curCnt[i]=TCInputs.transitionsNum[i];
      tapsUpdateCnt.cntRequest=5;        
    break;  
  }  
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{  
  if(htim->Instance==TIM1)
  {     
    if((TIM1->CR1)&0x10) // down counting, at the end of PWM period
    {           
      if(vectorPWM.arrUpdate)
      {
        htim1.Instance->ARR=vectorPWM.arr; 
        vectorPWM.arrUpdate=0;
      }
      TIM3->CNT=0;            
      TIM3->ARR=(vectorPWM.arr-vectorPWM.cc[3])/2+168;// По состоянию на 28.10.2015 "+168" - оптимальное смещение для {Ia,Ib,Ic},{Udc,Ib,Ic}
      HAL_TIM_Base_Start_IT(&htim3);      
      if(diag.sequenceADCPWM!=4)
      {
        diag.seqErr_ADC_PWM++;
        if(measurements.meanReady)eventsCnt.adc_pwm_err++;
      }
      diag.sequenceADCPWM=0;
    }else{ // up counting               
      if(diag.sequenceADCPWM!=0)
      {        
        diag.seqErr_ADC_PWM++;
        if(measurements.meanReady)eventsCnt.adc_pwm_err++;
      }
      diag.sequenceADCPWM=1; 
      vectorPWMcalc();
      if(vectorPWM.outputsCommand)
      {
        if(vectorPWM.outputsCommand==1)
        {
          PWM_EnableOutputs(&htim1);
          vectorPWM.isOn=1;            // Признак того, что сигнал ШИМ поступает на драйвер силовых ключей 
          DOUT1(1); // Сигнал того, что ШИМ включен
          stateDOUT|=0x01;
          F_RESET();
          systemState.resetTC=0x8000;
          systemState.manualStart=1;          
          eventLogWrite(14); 
        }else{          
          DRIVER_STOP(1);
          PWM_DisableOutputs(&htim1);
          shortCurrentCheck=1;         // Во время следующего пуска проверять выходы ЧРЭП на к.з.
          vectorPWM.isOn=0;            // Признак того, что сигнал ШИМ не поступает на драйвер силовых ключей  
          DOUT1(0); // Сигнал того, что ШИМ выключен
          stateDOUT&=0x8E;
          vectorPWM.curDeltaPhi=0.0f;
          command.currentFreqOrRPM=0.0f;          
          systemState.manualStart=0;
          vectorPWM.activePdPhi=0.0;
          vectorPWM.activePddPhi=0.0;
        }
        vectorPWM.outputsCommand=0;
      }      
      if(diag.sequenceADCPWM!=1)
      {
        if(measurements.meanReady)eventsCnt.adc_pwm_err++;
        diag.seqErr_ADC_PWM++;
      }
      diag.sequenceADCPWM=2;            
    }       
  }
  if(htim->Instance==TIM2)
  {
    rs485DataReady=1;   
    LED1_GREEN(0);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
    HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
  }   
  /*if(htim->Instance==TIM4)
  {
    rs232DataReady=1; 
    waitForWiFiFlag=0;    
    LED1_GREEN(0);
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
    HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
  } 
  if(htim->Instance==TIM5)
  {
    wifiDataReady=1; 
    waitForWiFiFlag=0;    
    diag.wifi_recPacketNum++;
    LED1_GREEN(0);
    __HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
    HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);
  }   */
}

void copymas(uint8_t *dst, uint8_t *src, uint32_t bytesNum) // Функция копирования массивов
{
  uint32_t i=0;
  while(bytesNum--)
  {
    dst[i]=src[i];
    i++;
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
