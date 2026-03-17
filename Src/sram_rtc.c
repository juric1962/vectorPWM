#include "sram_rtc.h"
#include "stm32f4xx_hal_rtc.h"
#include "alarms.h"
#include "fram.h"
#include "main.h"
#include "pwm.h"
#include "dynaGram.h"
#include "archive.h"

//------------------------ RTC ------------------------------------------------- 
#define _TBIAS_DAYS		((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS		(_TBIAS_DAYS * (uint32_t)86400)
#define	_TBIAS_YEAR		1900
#define MONTAB(year)		((((year) & 0x03) || ((year) == 0)) ? mos : lmos)
const int16_t	lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t	mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
#define	Daysto32(year, mon)	(((year - 1) / 4) + MONTAB(year)[mon])
//------------------------------------------------------------------------------
//-------------------- EVENT LOG -----------------------------------------------
volatile EVENT_RECORD_UNION *eventRecord;
volatile uint16_t *eventRecordPtr;
//------------------------------------------------------------------------------
RTC_TimeTypeDef  RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure; 
volatile RTC_UNIX_TIME_STRUCT RTC_UnixTime;
RTC_HandleTypeDef hrtc; 

volatile ACTIVE_POWER_STRUCT activePower;

void bkpSRAMInit(void)
{
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  HAL_PWREx_EnableBkUpReg();  
}

void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();  
 
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    systemState.eventLogHardwareFault=3; // Ошибка инициализации часового кварцевого резонатора 
    eventLogWrite(18);      
    eventsCnt.RTC_init_err++;    
  }
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  { 
    //Error_Handler();
  }

  __HAL_RCC_RTC_ENABLE();
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{  
  __HAL_RCC_RTC_DISABLE();  
  HAL_PWR_DisableBkUpAccess();
  __HAL_RCC_PWR_CLK_DISABLE();  
}

static void RTC_CalendarConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  sdatestructure.Year = 0x00;
  sdatestructure.Month = RTC_MONTH_JANUARY ;
  sdatestructure.Date = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler();
  }

  stimestructure.Hours = 0x00;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler();
  }  
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
}

void rtcInit(void)
{
  hrtc.Instance = RTC; 
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 0x7F;
  hrtc.Init.SynchPrediv = 0xFF;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    /* Initialization Error */
  }    

  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
  {
    /* Configure RTC Calendar */    
    systemState.eventLogHardwareFault=4; // Сброс настроек RTC
    eventLogWrite(18);      
    eventsCnt.RTC_init_err++;
    RTC_CalendarConfig();
  }
  else
  {
    /* Check if the Power On Reset flag is set */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
      /* Turn on LED2: Power on reset occurred */     
    }
    /* Check if Pin Reset flag is set */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
    {
      /* Turn on LED1: External reset occurred */
      
    }
    /* Clear source Reset Flag */
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
}

void systemTimeUpdate(void)
{
  HAL_RTC_GetTime(&hrtc,&RTC_TimeStructure,RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc,&RTC_DateStructure,RTC_FORMAT_BIN); 
  RTC_UnixTime.timeStamp=toUnix(RTC_DateStructure,RTC_TimeStructure);
  RTC_UnixTime.CRC16=modBusCRC16((uint8_t*)&RTC_UnixTime,4);
  bkpSRAM_Write(TIMESTAMP_SRAM_ADDR,(uint8_t*)&RTC_UnixTime,TIMESTAMP_SRAM_LENGTH);
}

void getDataTimeTask(void)
{  
  uint32_t curTick=HAL_GetTick();    
  static uint32_t prevTick=0;   
  uint32_t ms;
  uint8_t i;
  
  if(curTick>prevTick) 
  {
    ms=curTick-prevTick;
  }else{ms=(0xFFFFFFFFL-prevTick)+(curTick+1L);}
  
  if(ms>1000) // Прошла 1 секунда, вес тика 1 мс
  {  
    if(chrep.writeAccess)chrep.writeAccess--; // Период разрешения записи максимального тока преобразователя
    oneSec=1;
    systemTimeUpdate();
    fixfreqTask();//fixfreqTask(); // Обновление счетчика попыток режима ограничения максимальной выходной частоты
    etrEngineShadow.overloadCnt=etrEngine.overloadCnt;
    etrEngineShadow.overloadCntCRC=modBusCRC16((uint8_t*)&etrEngineShadow.overloadCnt,4);      
    framWrite(ETR_ENGINE_FRAM_OVERLOAD_CNT_ADDR,(uint8_t*)&etrEngineShadow.overloadCnt,ETR_ENGINE_FRAM_OVERLOAD_CNT_LENGTH);            
    eventsCnt.CRC16=modBusCRC16((uint8_t*)&eventsCnt,EVENTS_COUNTER_MODBUS_REGNUM*2);
    framWrite(EVENTS_COUNTER_FRAM_ADDR,(uint8_t*)&eventsCnt,EVENTS_COUNTER_FRAM_LENGTH);   
    for(i=0;i<FAULT_ARRAY_ITEMS_NUM;i++)faultCountersUpdate(i);//for(i=0;i<FAULT_ARRAY_ITEMS_NUM;i++)faultCountersUpdate(i);
    if((diag.seqErr_ADC_PWM>100)||(diag.seqErr_EnergyCalc>100))
    {
      DRIVER_STOP(1);
      while(1){;}
    }else{
      diag.seqErr_ADC_PWM=0;
      diag.seqErr_EnergyCalc=0;
    }  
    prevTick=curTick;           
  }      
}

uint32_t toUnix(RTC_DateTypeDef RTC_DateStructure, RTC_TimeTypeDef RTC_TimeStructure)
{	/* convert time structure to scalar time */
uint32_t	days;
uint32_t	secs;
int32_t		mon, year;

	/* Calculate number of days. */
	mon = RTC_DateStructure.Month-1;
	year = RTC_DateStructure.Year+2000 - _TBIAS_YEAR;
	days  = Daysto32(year, mon) - 1;
	days += 365 * year;
	days += RTC_DateStructure.Date;
	days -= _TBIAS_DAYS;

	/* Calculate number of seconds. */
	secs  = 3600 * RTC_TimeStructure.Hours;
	secs += 60 * RTC_TimeStructure.Minutes;
	secs += RTC_TimeStructure.Seconds;

	secs += (days * (uint32_t)86400);

	return (secs-946684800L);
}

void fromUnix(RTC_DateTypeDef *RTC_DateStructure, RTC_TimeTypeDef *RTC_TimeStructure, uint32_t secsarg)
{
uint32_t	secs;
int32_t		days;
int32_t		mon;
int32_t		year;
int32_t		i;
const int16_t*	pm;

  secs = secsarg+946684800L;
  days = _TBIAS_DAYS;

  /* days, hour, min, sec */
  days += secs / 86400;		secs = secs % 86400;
  RTC_TimeStructure->Hours = secs / 3600;	secs %= 3600;
  RTC_TimeStructure->Minutes = secs / 60;	RTC_TimeStructure->Seconds = secs % 60;

  /* determine year */
  for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
  days -= i;
  RTC_DateStructure->Year = year + _TBIAS_YEAR-2000;

  /* determine month */
  pm = MONTAB(year);
  for (mon = 12; days < pm[--mon]; );
  RTC_DateStructure->Month = mon + 1;
  RTC_DateStructure->Date = days - pm[mon] + 1;
}

uint8_t bkpSRAM_Write(uint16_t addr,uint8_t *src,uint16_t len)
{
  uint16_t i,j,n;
  n=addr+len;
  j=0;
  if(n<4097)
  {  
    for(i=addr;i<n;i++)
    {
      *(__IO uint8_t *)(BKPSRAM_BASE + i) = src[j];
      j++;
    }    
    return 0;
  }else return 1;
}

uint8_t bkpSRAM_Read(uint16_t addr,uint8_t *dst,uint16_t len)
{
  uint16_t i,j,n;
  n=addr+len;
  j=0;
  if(n<4097)
  {  
    for(i=addr;i<n;i++)
    {
      dst[j]=*(__IO uint8_t *)(BKPSRAM_BASE + i);
      j++;
    }    
    return 0;
  }else return 1;
}


void eventLogInit(void)
{ 
  eventRecord=(volatile EVENT_RECORD_UNION*)((volatile uint8_t*)(BKPSRAM_BASE+EVENT_LOG_ADDR+2));
  eventRecordPtr=(volatile uint16_t*)(BKPSRAM_BASE+EVENT_LOG_ADDR);
  
  bkpSRAM_Read(TIMESTAMP_SRAM_ADDR,(uint8_t*)&RTC_UnixTime,TIMESTAMP_SRAM_LENGTH);        
  if(RTC_UnixTime.CRC16!=modBusCRC16((uint8_t*)&RTC_UnixTime,4)) // ЭНОЗУ не сохраняется при выключении питания
  {    
    *eventRecordPtr=0;  
    if(systemState.eventLogHardwareFault==2)eventLogWrite(18); // Ошибка инициализации системного кварцевого резонатора   
    systemState.eventLogHardwareFault=1; // Ошибка чтения временной метки
    eventLogWrite(18);     
    
  }   
}

void eventLogWrite(uint16_t eventCode)
{   
  (*eventRecordPtr)++; 
  if(*eventRecordPtr>=EVENT_LOG_RECORDS_NUM)*eventRecordPtr=0;
  eventRecord[*eventRecordPtr].fld.unixTimeLSB=(uint16_t)(RTC_UnixTime.timeStamp&0xFFFF);              
  eventRecord[*eventRecordPtr].fld.unixTimeMSB=(uint16_t)(RTC_UnixTime.timeStamp/65536L); 
  switch(eventCode)
  {
    case 1: // Перегруз по мгновенному току 
    case 2: // Перегруз по мгновенному действующему значению тока     
    case 3: // Авария по насыщению силовых ключей      
      eventRecord[*eventRecordPtr].bytes[4]=(int8_t)(measurements.InstPhACur/100);        
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)(measurements.InstPhBCur/100);        
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)(measurements.InstPhCCur/100);              
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.Irms/100);      
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);        
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.scalarProduct/1000.0f);            
      eventRecord[*eventRecordPtr].fld.code=eventCode;  
    break;      
    case 4:  // Перенапряжение DC-шины
    case 11: // Превышение выходной частоты в генераторном режиме
      eventRecord[*eventRecordPtr].bytes[4]=0;        
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)(measurements.setFreq/100);       
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)(measurements.curFreq/100);              
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.Irms/100);       
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);        
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.scalarProduct/1000.0f);            
      eventRecord[*eventRecordPtr].fld.code=eventCode;  
    break;      
    case 5: // Перегрузка по ЭТР
    case 8: // Авария по температуре
      eventRecord[*eventRecordPtr].bytes[4]=(int8_t)measurements.tempSensor1;
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)measurements.tempSensor2;
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)measurements.tempSensor3;
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.Irms/100);      
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);        
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.scalarProduct/1000.0f);            
      eventRecord[*eventRecordPtr].fld.code=eventCode;  
    break;    
    case 6: // Обрыв фазы
    case 7: // Перекос тока     
      eventRecord[*eventRecordPtr].bytes[4]=(int8_t)measurements.Ia;        
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)measurements.Ib;        
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)measurements.Ic;              
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.Irms/100);      
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);        
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)measurements.ppIrms;
      eventRecord[*eventRecordPtr].fld.code=eventCode;  
    break;     
    case 9: // Авария по низкому напряжению на DC-шине      
      eventRecord[*eventRecordPtr].bytes[4]=(int8_t)measurements.Ia;        
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)measurements.Ib;        
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)measurements.Ic;              
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.Irms/100);      
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);     
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.Uavg/10.0f);
      eventRecord[*eventRecordPtr].fld.code=eventCode;      
    break;
  case 10: // Холостой ход
      eventRecord[*eventRecordPtr].bytes[4]=(int8_t)measurements.Ia;        
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)measurements.Ib;        
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)measurements.Ic;              
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.Irms/100);      
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);        
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.Pabc/1000.0f);
      eventRecord[*eventRecordPtr].fld.code=eventCode;        
    break;    
    case 12: // Включение питания             
    case 13: // Выключение питания                   
      eventRecord[*eventRecordPtr].fld.reg[0]=0;
      eventRecord[*eventRecordPtr].fld.reg[1]=0;      
      eventRecord[*eventRecordPtr].fld.reg[2]=0;       
      eventRecord[*eventRecordPtr].fld.code=eventCode;               
    break;
    case 14: // Пуск двигателя
      eventRecord[*eventRecordPtr].fld.reg[0]=systemState.curMode;      
      eventRecord[*eventRecordPtr].fld.reg[1]=0;      
      eventRecord[*eventRecordPtr].fld.reg[2]=0;      
      eventRecord[*eventRecordPtr].fld.code=eventCode;            
    break;          
    case 15: // Останов двигателя     
      if(command.stopCHREP)eventRecord[*eventRecordPtr].fld.reg[0]=4;
      else eventRecord[*eventRecordPtr].fld.reg[0]=systemState.curMode;
      eventRecord[*eventRecordPtr].fld.reg[1]=0;      
      eventRecord[*eventRecordPtr].fld.reg[2]=0;      
      eventRecord[*eventRecordPtr].fld.code=eventCode;            
    break;    
    case 16: // Сброс ошибок     
      if(command.faultsReset)eventRecord[*eventRecordPtr].fld.reg[0]=2;
      else eventRecord[*eventRecordPtr].fld.reg[0]=1;      
      eventRecord[*eventRecordPtr].fld.reg[1]=alarms.faultBits;      
      eventRecord[*eventRecordPtr].fld.reg[2]=alarms.faultBitsExtended;     
      eventRecord[*eventRecordPtr].fld.code=eventCode;       
    break;
    case 17: // Изменение режима работы преобразователя      
      eventRecord[*eventRecordPtr].fld.reg[0]=systemState.eventLogPrevMode;
      eventRecord[*eventRecordPtr].fld.reg[1]=systemState.eventLogCurMode;      
      eventRecord[*eventRecordPtr].fld.reg[2]=0;        
      eventRecord[*eventRecordPtr].fld.code=eventCode;       
    break;    
    case 18: // Аппаратные ошибки      
      eventRecord[*eventRecordPtr].fld.reg[0]=systemState.eventLogHardwareFault;
      eventRecord[*eventRecordPtr].fld.reg[1]=0;      
      eventRecord[*eventRecordPtr].fld.reg[2]=0;       
      eventRecord[*eventRecordPtr].fld.code=eventCode;         
    break;
    case 19: // Перезагрузка мк по сторожевому таймеру
      if(eventsCnt.iwdgDelayMS)eventRecord[*eventRecordPtr].fld.reg[0]=1;
      else eventRecord[*eventRecordPtr].fld.reg[0]=2;      
      eventRecord[*eventRecordPtr].fld.reg[1]=diag.seqErr_ADC_PWM;      
      eventRecord[*eventRecordPtr].fld.reg[2]=diag.seqErr_EnergyCalc;       
      eventRecord[*eventRecordPtr].fld.code=eventCode;       
    break;
    case 20: // Авария, останов двигателя по удержанию пониженного напряжения на DC-шине
    case 21: // Включен режим ограничения максимальной выходной частоты по удержанию пониженного напряжения на DC-шине
    case 22: // Авария, останов двигателя по удержанию неполнофазного режима (100 Гц-гармоники)
    case 23: // Включен режим ограничения максимальной выходной частоты по удержанию неполнофазного режима (100 Гц-гармоники)     
      eventRecord[*eventRecordPtr].bytes[4]=(int8_t)((float)Q10/10.0f);       
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)(measurements.curFreq/100);         
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)measurements.ppUdc;              
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)measurements.ppIrms;      
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);     
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.Pabc/1000.0f);
      eventRecord[*eventRecordPtr].fld.code=eventCode;        
    break;
    case 24: // Авария по ограничению пускового тока
      eventRecord[*eventRecordPtr].bytes[4]=(int8_t)(measurements.InstPhACur/100);        
      eventRecord[*eventRecordPtr].bytes[5]=(int8_t)(measurements.InstPhBCur/100);        
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)(measurements.InstPhCCur/100);              
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.Irms/100);      
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.InstUdc/10);        
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.scalarProduct/1000.0f);            
      eventRecord[*eventRecordPtr].fld.code=eventCode;  
    break;      
    case 25: // Авария по сигналу с динамографа
      if(((float)dynaGram.loadErrorAdcCode*dynaGram.tit3ToForceCoeff+dynaGram.tit3ToForceConst)>=0)
      {
          eventRecord[*eventRecordPtr].fld.reg[0]=(uint16_t)(((float)dynaGram.loadErrorAdcCode*dynaGram.tit3ToForceCoeff+dynaGram.tit3ToForceConst)/2.0f);
      }else eventRecord[*eventRecordPtr].fld.reg[0]=0;           
      eventRecord[*eventRecordPtr].bytes[6]=(int8_t)(measurements.setFreq/100);       
      eventRecord[*eventRecordPtr].bytes[7]=(int8_t)(measurements.curFreq/100);              
      eventRecord[*eventRecordPtr].bytes[8]=(int8_t)(measurements.Irms/100);             
      eventRecord[*eventRecordPtr].bytes[9]=(int8_t)(measurements.scalarProduct/1000.0f);            
      eventRecord[*eventRecordPtr].fld.code=eventCode; 
    break;
    case 101: // Архивные записи по отводу №1
    case 102: // Архивные записи по отводу №2 
    case 103:  
    case 104:  
    case 105:  
    case 106:  
    case 107:
    case 108:      
    case 109:  
    case 110:  
    case 111:  
    case 112:  
    case 113:  
    case 114:  
    case 115:  
    case 116: // Архивные записи по отводу №16     
      eventRecord[*eventRecordPtr].fld.unixTimeLSB=tapRecord.fld.timeStampLSB;
      eventRecord[*eventRecordPtr].fld.unixTimeMSB=tapRecord.fld.timeStampMSB;            
      eventRecord[*eventRecordPtr].fld.reg[0]=(tapRecord.fld.intervalMin)&0xFFF;//+((tapRecord.fld.tapNum<<4)&0xF000); - пока не писать 
      eventRecord[*eventRecordPtr].fld.reg[1]=tapRecord.fld.valueLSB;
      eventRecord[*eventRecordPtr].fld.reg[2]=tapRecord.fld.valueMSB;      
      eventRecord[*eventRecordPtr].fld.code=eventCode;        
    break;
    
  }
  
  eventRecord[*eventRecordPtr].fld.crcSum=eventRecord[*eventRecordPtr].bytes[0]+eventRecord[*eventRecordPtr].bytes[1]+eventRecord[*eventRecordPtr].bytes[2]+
                                          eventRecord[*eventRecordPtr].bytes[3]+eventRecord[*eventRecordPtr].bytes[4]+eventRecord[*eventRecordPtr].bytes[5]+
                                          eventRecord[*eventRecordPtr].bytes[6]+eventRecord[*eventRecordPtr].bytes[7]+eventRecord[*eventRecordPtr].bytes[8]+
                                          eventRecord[*eventRecordPtr].bytes[9]+eventRecord[*eventRecordPtr].bytes[10]+eventRecord[*eventRecordPtr].bytes[11];                                              
}

void activeEnergyInit(void)
{ 
  activePower.totalSum=(double*)((volatile uint8_t*)(BKPSRAM_BASE+ACTIVE_POWER_ADDR));  
  activePower.totalSumCRC=(uint16_t*)((volatile uint8_t*)(BKPSRAM_BASE+ACTIVE_POWER_ADDR+8));  
  activePower.infiniteSum=0;
  activePower.instantSum=0;
  activePower.sampNum=1;
  activePower.secondsSumRdy=0;
  activePower.resetInfiniteSum=0;    
    
  if((*activePower.totalSumCRC)!=modBusCRC16((uint8_t*)activePower.totalSum,8)) // ЭНОЗУ не сохраняется при выключении питания
  {     
    *activePower.totalSum=0;    
    *activePower.totalSumCRC=modBusCRC16((uint8_t*)activePower.totalSum,8);
    systemState.eventLogHardwareFault=15; // Сброшен счетчик активной энергии
    eventLogWrite(18);     
  }   
}

void activeEnergyTask(void) // Задача счетчика активной энергии 
{  
  double tempDouble;
  if(activePower.secondsSumRdy) // Время буду фиксировать только по системному таймеру, т.к. погрешность вычисления заметно должна превосходить 
  {                 // расхождение между системной частотой и частотой RTC
    activePower.secondsSumRdy=0;	
    tempDouble=(double)activePower.secondsSum/((double)(engine.freqPWM));
    activePower.infiniteSum+=(uint32_t)tempDouble; // Добавляем энергию в кВт*час	
    *activePower.totalSum+=tempDouble/3600.0;
    *activePower.totalSumCRC=modBusCRC16((uint8_t*)activePower.totalSum,8);
  }
  if(activePower.resetInfiniteSum)
  {
    *activePower.totalSum=0;
    activePower.resetInfiniteSum=0;
    *activePower.totalSumCRC=modBusCRC16((uint8_t*)activePower.totalSum,8);
  }
}
