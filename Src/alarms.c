#include "alarms.h"
#include "pwm.h"
#include "sram_rtc.h"
#include "main.h"
#include "fram.h"
#include "bvi.h" 

volatile ALARMS_STRUCT alarms, alarmsShadow;        // Рабочая структура уставок, промежуточная, настроечная структура уставок   
volatile ETR_ENGINE_STRUCT etrEngine, etrEngineShadow; 
volatile FAULT_EVENT_STRUCT *faultsArrayNVRAM;
volatile EVENTS_COUNTER_STRUCT eventsCnt;
volatile FIX_FREQ_STATE_STRUCT *fixFreq; 
volatile CHREP_STRUCT chrep,chrepShadow;

void engineETRInit(void)
{  
  framRead(ETR_ENGINE_FRAM_ADDR,(uint8_t*)&etrEngine,ETR_ENGINE_FRAM_LENGTH);   
  etrEngineShadow.updateFromShadow=0;
  if(etrEngine.CRC16!=modBusCRC16((uint8_t*)&etrEngine,ETR_ENGINE_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {
    systemState.eventLogHardwareFault=10; // Сброс настроек ЭТР
    eventLogWrite(18);
    eventsCnt.etrDefaults++;
    float tempFloat;  
    etrEngine.Isum=0;  
    etrEngine.IsumCnt=0;
    etrEngine.overloadCnt=0;
    etrEngine.overloadPercent=120;
    etrEngine.overloadSec=60;
    etrEngine.overloadCurrent=engine.Inom*(float)etrEngine.overloadPercent/100.0f;
    etrEngine.overloadLimit=etrEngine.overloadCurrent*etrEngine.overloadCurrent*etrEngine.overloadSec;
    etrEngine.externalCoolingOn=0;
    etrEngine.isOn=0;

    etrEngine.freqCorrPoints[0]=10;
    etrEngine.freqCorrPoints[1]=30;
    etrEngine.currentCorrPoints[0]=10;
    etrEngine.currentCorrPoints[1]=30;
    etrEngine.currentCorrPoints[2]=50;
    etrEngine.currentCorrPoints[3]=70;
    
    tempFloat=etrEngine.freqCorrPoints[0]-engine.minFreq;
    if(tempFloat>0.0f)etrEngine.k1=(etrEngine.currentCorrPoints[1]-etrEngine.currentCorrPoints[0])/(100.0f*tempFloat);
    else etrEngine.k1=0;
    etrEngine.C1=-etrEngine.k1*engine.minFreq+etrEngine.currentCorrPoints[0]/100.0f;  
      
    tempFloat=etrEngine.freqCorrPoints[1]-etrEngine.freqCorrPoints[0];
    if(tempFloat>0.0f)etrEngine.k2=(etrEngine.currentCorrPoints[2]-etrEngine.currentCorrPoints[1])/(100.0f*tempFloat);
    else etrEngine.k2=0;
    etrEngine.C2=-etrEngine.k2*etrEngine.freqCorrPoints[0]+etrEngine.currentCorrPoints[1]/100.0f;  
      
    tempFloat=engine.maxFreq-etrEngine.freqCorrPoints[1];
    if(tempFloat>0.0f)etrEngine.k3=(etrEngine.currentCorrPoints[3]-etrEngine.currentCorrPoints[2])/(100.0f*tempFloat);
    else etrEngine.k3=0;
    etrEngine.C3=-etrEngine.k3*etrEngine.freqCorrPoints[1]+etrEngine.currentCorrPoints[2]/100.0f;          
  
    etrEngine.CRC16=modBusCRC16((uint8_t*)&etrEngine, ETR_ENGINE_MODBUS_REGNUM*2);    
    framWrite(ETR_ENGINE_FRAM_ADDR,(uint8_t*)&etrEngine,ETR_ENGINE_FRAM_LENGTH);    
  }else{    
    etrEngine.overloadCurrent=engine.Inom*((float)etrEngine.overloadPercent)/100.0f;
    etrEngine.overloadLimit=etrEngine.overloadCurrent*etrEngine.overloadCurrent*((float)etrEngine.overloadSec);    
    float tempFloat;
    tempFloat=etrEngine.freqCorrPoints[0]-engine.minFreq;
    if(tempFloat>0.0f)etrEngine.k1=(etrEngine.currentCorrPoints[1]-etrEngine.currentCorrPoints[0])/(100.0f*tempFloat);
    else etrEngine.k1=0;
    etrEngine.C1=-etrEngine.k1*engine.minFreq+etrEngine.currentCorrPoints[0]/100.0f;  
      
    tempFloat=etrEngine.freqCorrPoints[1]-etrEngine.freqCorrPoints[0];
    if(tempFloat>0.0f)etrEngine.k2=(etrEngine.currentCorrPoints[2]-etrEngine.currentCorrPoints[1])/(100.0f*tempFloat);
    else etrEngine.k2=0;
    etrEngine.C2=-etrEngine.k2*etrEngine.freqCorrPoints[0]+etrEngine.currentCorrPoints[1]/100.0f;  
      
    tempFloat=engine.maxFreq-etrEngine.freqCorrPoints[1];
    if(tempFloat>0.0f)etrEngine.k3=(etrEngine.currentCorrPoints[3]-etrEngine.currentCorrPoints[2])/(100.0f*tempFloat);
    else etrEngine.k3=0;
    etrEngine.C3=-etrEngine.k3*etrEngine.freqCorrPoints[1]+etrEngine.currentCorrPoints[2]/100.0f;          
  
    etrEngine.CRC16=modBusCRC16((uint8_t*)&etrEngine, ETR_ENGINE_MODBUS_REGNUM*2);    
    framWrite(ETR_ENGINE_FRAM_ADDR,(uint8_t*)&etrEngine,ETR_ENGINE_FRAM_LENGTH);          
    
    framRead(ETR_ENGINE_FRAM_OVERLOAD_CNT_ADDR,(uint8_t*)&etrEngine.overloadCnt,ETR_ENGINE_FRAM_OVERLOAD_CNT_LENGTH);    
    if(etrEngine.overloadCntCRC!=modBusCRC16((uint8_t*)&etrEngine.overloadCnt,4))     
    {    
      etrEngine.overloadCnt=0.0f;
    }else{    
      bkpSRAM_Read(TIMESTAMP_SRAM_ADDR,(uint8_t*)&RTC_UnixTime,TIMESTAMP_SRAM_LENGTH);        
      if(RTC_UnixTime.CRC16==modBusCRC16((uint8_t*)&RTC_UnixTime,4))
      {
        HAL_RTC_GetTime(&hrtc,&RTC_TimeStructure,RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc,&RTC_DateStructure,RTC_FORMAT_BIN);       
        uint32_t timeStamp=toUnix(RTC_DateStructure,RTC_TimeStructure);            
        etrEngine.overloadCnt=etrEngine.overloadCnt-((float)(timeStamp-RTC_UnixTime.timeStamp))*(etrEngine.overloadCurrent*etrEngine.overloadCurrent)/10.0f;      
        if(etrEngine.overloadCnt<0.0f)etrEngine.overloadCnt=0.0f;      
      }
    }
    etrEngine.Isum=0;  
    etrEngine.IsumCnt=0;     
  }  
  copymas((uint8_t*)&etrEngineShadow,(uint8_t*)&etrEngine,ETR_ENGINE_MODBUS_REGNUM*2);
}

void alarmsResetCounters(void)
{
  alarms.InstPhCurMaxSumA=0;
  alarms.InstPhCurMaxSumB=0;
  alarms.InstPhCurMaxSumC=0;
  alarms.InstPhCurMaxCnt=0;        
  alarms.breakCurAMinCounterMS=0;
  alarms.breakCurBMinCounterMS=0;
  alarms.breakCurCMinCounterMS=0;  
  alarms.curDisbalanceMaxCounterMS=0;  
  alarms.IrmsMaxCounter=0;  
  alarms.noLoadWattsSecCnt=0.0f;
  alarms.fixFreqLowVoltageCnt=0;
  alarms.fixFreqPhaseErrorCnt=0;
}

void alarmsInit(void)
{  
  //--------------Ограничение на максимальный ток преобразователя, связанный с силовыми ключами --------------
  framRead(CHREP_MAX_INST_CUR_FRAM_ADDR,(uint8_t*)&chrep.maxInstCurrent,CHREP_MAX_INST_CUR_FRAM_LENGTH); 
  if(chrep.CRC16!=modBusCRC16((uint8_t*)&chrep.maxInstCurrent,2)) // Загрузка данных по умолчанию
  {
    chrep.maxInstCurrent=7500;  
  }  
  switch(chrep.maxInstCurrent)
  {
    case 7500:  // 15 кВт
    case 8500:  // 18.5 кВт
    case 10000: // 22 кВт 
    break;
    default:
      chrep.maxInstCurrent=7500;  
  }  
  chrep.writeAccess=0;
  //------------------------------------------------------------------------------------------------------------  
  framRead(ALARMS_FRAM_ADDR,(uint8_t*)&alarms,ALARMS_FRAM_LENGTH);  
  if(alarms.CRC16!=modBusCRC16((uint8_t*)&alarms,ALARMS_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {  
    systemState.eventLogHardwareFault=8; // Сброс настроек аварий
    eventLogWrite(18);
    eventsCnt.alarmsDefaults++;
    alarms.IrmsMaxPercent=150;
    alarms.IrmsMaxMS=1000;
    alarms.breakCurMinPercent=10;
    alarms.breakCurMinMS=3000;    
    alarms.curDisbalanceMaxPercent=30;
    alarms.curDisbalanceMaxMS=5000;    
    
    alarms.internalTemperatureMAX=80;
    alarms.internalTemperatureNORM=60;
    alarms.radiatorTemperatureMAX=95;
    alarms.radiatorTemperatureNORM=75;       
    
    alarms.IrmsMax=(uint16_t)((float)alarms.IrmsMaxPercent*engine.Inom);    
    alarms.curDisbalanceMax=(uint16_t)((float)alarms.curDisbalanceMaxPercent*engine.Inom/100.0f);
    alarms.breakCurMin=(uint16_t)((float)alarms.breakCurMinPercent*engine.Inom/100.0f);    
    
    alarmsResetCounters();
    
    alarms.noLoadWatts=3000;
    alarms.noLoadWattsSec=100;
      
    alarms.fixFreqEnable=1;
    alarms.fixFreqValue=30;    
    alarms.fixFreqLowVoltageValue=450; // Напряжение на DC-шине меньше заданной в течение fixFreqLowVoltageMS миллисекунд 
    alarms.fixFreqPhaseErrorValue=50;  // Размах 100 Гц-гармоники на DC-шине достигает 50В в течение fixFreqPhaseErrorMS миллисекунд 
    alarms.fixFreqLowVoltageMS=1000;
    alarms.fixFreqPhaseErrorMS=1000;      
    alarms.fixFreqAttemptsNumMax=5;
    alarms.fixFreqAttemptsPeriodMin=15;
    alarms.fixFreqAttemptsNumResetMin=30;          
      
    alarms.curDisbalanceMaxRecoverCntLimit=3;    
    alarms.IrmsMaxRecoverCntLimit=3;    
    alarms.noLoadWattsRecoverCntLimit=3;
    alarms.lowVoltageRecoveryCntLimit=3;
    alarms.temperatureRecoveryCntLimit=3;
    alarms.etrRecoveryCntLimit=3;
    alarms.highVoltageOffRecoverCntLimit=3; 
    alarms.dynagramLoadAlarmRecoveryCntLimit=3;

    alarms.IrmsMaxResetCnt=30;    
    alarms.curDisbalanceMaxResetCnt=30;
    alarms.lowVoltageResetCnt=30;
    alarms.highVoltageOffResetCnt=30;
    alarms.noLoadWattsResetCnt=30;
    alarms.etrResetCnt=30;
    alarms.temperatureResetCnt=30;    
    alarms.dynagramLoadAlarmResetCnt=30;    
    
    alarms.faultBits=0; 
    alarms.faultBitsExtended=0;
    alarms.CRC16=modBusCRC16((uint8_t*)&alarms, ALARMS_MODBUS_REGNUM*2);    
    framWrite(ALARMS_FRAM_ADDR,(uint8_t*)&alarms,ALARMS_FRAM_LENGTH);     
  }  
  copymas((uint8_t*)&alarmsShadow,(uint8_t*)&alarms,ALARMS_MODBUS_REGNUM*2);  
  alarms.IrmsMax=(uint16_t)((float)alarms.IrmsMaxPercent*engine.Inom);  
  alarms.curDisbalanceMax=(uint16_t)((float)alarms.curDisbalanceMaxPercent*engine.Inom/100.0f);
  alarms.breakCurMin=(uint16_t)((float)alarms.breakCurMinPercent*engine.Inom/100.0f);   
  alarmsResetCounters();    
  engineETRInit();
}

void getFaultBits(void)
{ 
  alarms.faultBits|=0x0007;
  //alarms.faultBits|=(uint8_t)HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);      // FAULT_A_HI    
  //alarms.faultBits|=((uint8_t)HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2)<<1); // FAULT_B_HI    
  //alarms.faultBits|=((uint8_t)HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4)<<2); // FAULT_C_HI   
  //alarms.faultBitsExtended|=((uint8_t)HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1)<<2); // FAULT_A_LO  
  //alarms.faultBitsExtended|=((uint8_t)HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3)<<3); // FAULT_B_LO  
  //alarms.faultBitsExtended|=((uint8_t)HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5)<<4); // FAULT_C_LO  
}

float freqCorrection(float freq)
{
  float value;  
  if(freq>=etrEngine.freqCorrPoints[1]) 
  {  
    if(freq>=engine.maxFreq)value=engine.maxFreq;
    else value=etrEngine.k3*freq+etrEngine.C3;   
  }else if(freq>=etrEngine.freqCorrPoints[0])
        {    
          value=etrEngine.k2*freq+etrEngine.C2;
        }else if(freq>=engine.minFreq)
              {
                value=etrEngine.k1*freq+etrEngine.C1;    
              }else value=engine.minFreq;
   
  if(value>1.0f)value=1.0f;
  if(value<=0.01f)value=1.0f;
  
  return value;
}

uint32_t faultsInit(void) // Инициализация массива АПВ, проследить, чтобы значения по умолчанию по полям .maxCnt и .recoverCntSec
{                         // совпадали с соответствующими полями *RecoverCntLimit и *ResetCnt !!!!                                 
  uint8_t i;              // Выходное слово индицирует, по каким авариям переполнен счетчик автозапуска
  uint32_t state=0;  
  faultsArrayNVRAM=(volatile FAULT_EVENT_STRUCT*)((volatile uint8_t*)(BKPSRAM_BASE+FAULT_ARRAY_ADDR));//(volatile FAULT_EVENT_STRUCT*)(BKPSRAM_BASE+FAULT_ARRAY_ADDR);
  for(i=0;i<FAULT_ARRAY_ITEMS_NUM;i++)
  {
    if(faultsArrayNVRAM[i].CRC16==modBusCRC16((uint8_t*)&faultsArrayNVRAM[i],10))
    {           
      faultCountersUpdate(i);
      if(faultsArrayNVRAM[i].upCnt)
      { 
        if(faultsArrayNVRAM[i].maxCnt) // Если счетчик АПВ=0 - то не блокировать пуск после аварий
        {
          if(faultsArrayNVRAM[i].upCnt>=faultsArrayNVRAM[i].maxCnt)
          {
            state|=1L<<i;   
            //forceBVI(i, 1);
          }/*else{
            forceBVI(i, 3);
          }*/
        }/*else{
          forceBVI(i, 3);
        }*/
      }      
    }else{        
      switch(i)
      {
        case NO_LOAD_FAULT:
          faultsArrayNVRAM[i].upCnt=0;      
          faultsArrayNVRAM[i].recoverCntMin=alarms.noLoadWattsResetCnt;
          faultsArrayNVRAM[i].maxCnt=alarms.noLoadWattsRecoverCntLimit;          
        break;
        case LOW_VOLTAGE_FAULT:
          faultsArrayNVRAM[i].upCnt=0;      
          faultsArrayNVRAM[i].recoverCntMin=alarms.lowVoltageResetCnt;
          faultsArrayNVRAM[i].maxCnt=alarms.lowVoltageRecoveryCntLimit;                    
        break;
        case TEMP_SENS_FAULT:
          faultsArrayNVRAM[i].upCnt=0;      
          faultsArrayNVRAM[i].recoverCntMin=alarms.temperatureResetCnt;
          faultsArrayNVRAM[i].maxCnt=alarms.temperatureRecoveryCntLimit;          
        break;
        case BREAK_PHASE_CURRENT_FAULT:
          faultsArrayNVRAM[i].upCnt=0; 
          faultsArrayNVRAM[i].recoverCntMin=alarms.curDisbalanceMaxResetCnt;          
          faultsArrayNVRAM[i].maxCnt=alarms.curDisbalanceMaxRecoverCntLimit;                    
        break;
        case ETR_OVERLOAD_FAULT:
          faultsArrayNVRAM[i].upCnt=0;      
          faultsArrayNVRAM[i].recoverCntMin=alarms.etrResetCnt;
          faultsArrayNVRAM[i].maxCnt=alarms.etrRecoveryCntLimit;                    
        break;
        case UDC_OVERVOLTAGE_FAULT:
          faultsArrayNVRAM[i].upCnt=0;  
          faultsArrayNVRAM[i].recoverCntMin=alarms.highVoltageOffResetCnt;
          faultsArrayNVRAM[i].maxCnt=alarms.highVoltageOffRecoverCntLimit;                    
        break;
        case BRIDGE_SATURATION_FAULT:
          faultsArrayNVRAM[i].upCnt=0;      
          faultsArrayNVRAM[i].recoverCntMin=alarms.IrmsMaxResetCnt;
          faultsArrayNVRAM[i].maxCnt=alarms.IrmsMaxRecoverCntLimit;                    
        break; 
        case DYNAMOMETER_FAULT:
          faultsArrayNVRAM[i].upCnt=0;      
          faultsArrayNVRAM[i].recoverCntMin=alarms.dynagramLoadAlarmResetCnt;
          faultsArrayNVRAM[i].maxCnt=alarms.dynagramLoadAlarmRecoveryCntLimit;
        break;       
      }                
      systemState.eventLogHardwareFault=9; // Сброс настроек АПВ
      eventLogWrite(18);    
      faultsArrayNVRAM[i].CRC16=modBusCRC16((uint8_t*)&faultsArrayNVRAM[i],FAULT_ARRAY_ITEM_SIZE-2);  
      eventsCnt.faultsDefaults++;
    }    
  }
  return state;
}

uint8_t faultCountersUpdate(uint8_t num) // Обновить состояние счетчиков аварий, с учетом recoverCntMin
{
  uint32_t timeDiff;
  uint8_t status=0;
  
  if(faultsArrayNVRAM[num].upCnt) // Анализировать, только если была авария, иначе нечего декрементировать, тратить на это время...
  {
        if(faultsArrayNVRAM[num].upCnt<faultsArrayNVRAM[num].maxCnt) // Чтобы после включения из-за паузы не декрементировался счетчик
        {
          if(RTC_UnixTime.timeStamp>=faultsArrayNVRAM[num].recoverTime)
          {
                  timeDiff=RTC_UnixTime.timeStamp-faultsArrayNVRAM[num].recoverTime;
                  if(timeDiff)
                  {
                    timeDiff=timeDiff/(60*faultsArrayNVRAM[num].recoverCntMin);
                    if(timeDiff) // Прошло достаточно времени для декрементирования счетчика аварий
                    {
                          if(timeDiff>=faultsArrayNVRAM[num].upCnt)faultsArrayNVRAM[num].upCnt=0;
                          else faultsArrayNVRAM[num].upCnt=faultsArrayNVRAM[num].upCnt-timeDiff;      
                          faultsArrayNVRAM[num].recoverTime=RTC_UnixTime.timeStamp;
                          faultsArrayNVRAM[num].CRC16=modBusCRC16((uint8_t*)&faultsArrayNVRAM[num],10);        
                          //framWrite(FAULTS_FRAM_ADDR+12*num,(uint8_t*)&faultsArrayNVRAM[num],12);        
                    }
                 }
          }
        }
  }
  return status;
}

uint8_t faultEventAdd(uint8_t num) // Добавить событие, если функция возвращает не ноль, то выключить двигатель, 
{                                  // продолжать работу с ЧРЭП можно только после сброса ошибок при переключении тумблера "АВТО/РУЧН." или через модбас                            
  //faultCountersUpdate(num);
  faultsArrayNVRAM[num].upCnt++;  
  faultsArrayNVRAM[num].recoverTime=RTC_UnixTime.timeStamp;
  faultsArrayNVRAM[num].CRC16=modBusCRC16((uint8_t*)&faultsArrayNVRAM[num],FAULT_ARRAY_ITEM_SIZE-2);
  if(faultsArrayNVRAM[num].maxCnt)
  {
    if(faultsArrayNVRAM[num].upCnt>=faultsArrayNVRAM[num].maxCnt)
    {    
      return 1;
    }else{                 
      return 0;
    }  
  }else{
    return 0;
  }    
}

void faultCountersReset(void)
{
  uint8_t i;
  for(i=0;i<FAULT_ARRAY_ITEMS_NUM;i++)
  {      
    faultsArrayNVRAM[i].upCnt=0; // Сбросить счетчик ошибок 
    faultsArrayNVRAM[i].CRC16=modBusCRC16((uint8_t*)&faultsArrayNVRAM[i],FAULT_ARRAY_ITEM_SIZE-2);        
  }   
  systemState.startDisable=0;
  systemState.faults=0;
  //for(int i=0;i<10;i++)forceBVI(i, 0);
  //forceBVI(8, 7);
  alarms.faultBits=0;
  alarms.faultBitsExtended=0; 
  FAULT_OPTO_OUT(0);  

  fixFreq[0].fixFreqAttempt=0;
  fixFreq[0].fixFreqMode=0;
  fixFreq[0].eventTimeStamp=0;
  fixFreq[0].CRC16=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH);  
}

void eventsCntInit(void)
{  
  framRead(EVENTS_COUNTER_FRAM_ADDR,(uint8_t*)&eventsCnt,EVENTS_COUNTER_FRAM_LENGTH);  
  if(eventsCnt.CRC16!=modBusCRC16((uint8_t*)&eventsCnt,EVENTS_COUNTER_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {  
    eventsCnt.satA=0;
    eventsCnt.satB=0;
    eventsCnt.satC=0;
    eventsCnt.overVoltage=0;
    eventsCnt.instA=0;
    eventsCnt.instB=0;
    eventsCnt.instC=0;
    eventsCnt.rms=0;
    eventsCnt.etr=0;
    eventsCnt.breakA=0;
    eventsCnt.breakB=0;
    eventsCnt.breakC=0;    
    eventsCnt.disbalance=0;
    eventsCnt.temp1=0;
    eventsCnt.temp2=0;
    eventsCnt.lowVoltage=0;
    eventsCnt.fixFreqLowVoltage=0;
    eventsCnt.fixFreqNoPhase=0;
    eventsCnt.noLoad=0;   
    eventsCnt.adc_pwm_err=0;
    eventsCnt.adc_energy_err=0;    
    eventsCnt.eventsCntDefaults=1;
    eventsCnt.etrDefaults=0;
    eventsCnt.alarmsDefaults=0;
    eventsCnt.faultsDefaults=0;
    eventsCnt.commandDefaults=0;
    eventsCnt.contextDefaults=0;
    eventsCnt.adcDefaults=0;
    eventsCnt.dacDefaults=0;
    eventsCnt.engineDefaults=0;
    eventsCnt.HSE_init_err=0;
    eventsCnt.RTC_init_err=0;
    eventsCnt.iwdg=0;    
    eventsCnt.iwdgDelayMS=0;
    eventsCnt.dynaGramDefaults=0;
    eventsCnt.dynaGramLoadMax=0;
    eventsCnt.dynaGramLoadMin=0;
    eventsCnt.CRC16=modBusCRC16((uint8_t*)&eventsCnt,EVENTS_COUNTER_MODBUS_REGNUM*2);
    framWrite(EVENTS_COUNTER_FRAM_ADDR,(uint8_t*)&eventsCnt,EVENTS_COUNTER_FRAM_LENGTH);      
  }  
  eventsCnt.refresh=0;
}

void fixFreqInit(void)
{  
  fixFreq=(volatile FIX_FREQ_STATE_STRUCT*)((volatile uint8_t*)(BKPSRAM_BASE+FIX_FREQ_ADDR));
  if(fixFreq[0].CRC16!=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH)) // Загрузка данных по умолчанию
  {
    fixFreq[0].fixFreqAttempt=0;
    fixFreq[0].fixFreqMode=0;
    fixFreq[0].eventTimeStamp=0;
    fixFreq[0].CRC16=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH);
  }    
}

void fixfreqTask(void)
{
  static uint8_t reset=0;
  uint32_t timeDiff;  
  
  if(alarms.fixFreqEnable){reset=1;}   
  else{    
    if(reset)
    {
      fixFreq[0].fixFreqAttempt=0;
      fixFreq[0].fixFreqMode=0;
      fixFreq[0].eventTimeStamp=0;
      fixFreq[0].CRC16=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH);  
      reset=0;
    }
  }
  
  if(fixFreq[0].fixFreqMode) // Включен режим ограничения максимальной выходной частоты
  {    
    //if(fixFreq[0].fixFreqAttempt) // Ненулевой счетчик событий входа в режим ограничения максимальной выходной частоты   
    //{
      if(RTC_UnixTime.timeStamp>=fixFreq[0].eventTimeStamp)
      {
	timeDiff=RTC_UnixTime.timeStamp-fixFreq[0].eventTimeStamp;
	if(timeDiff>=(60*alarms.fixFreqAttemptsPeriodMin))
	{
          if(alarms.fixFreqAttemptsPeriodMin) // Выход из режима разрешен
          {           
            fixFreq[0].fixFreqMode=0; // Попытка выхода из режима фиксированной максимальной частоты
            fixFreq[0].eventTimeStamp=RTC_UnixTime.timeStamp;
            fixFreq[0].CRC16=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH);            
          }            
        }     
      }
    //}
  } 
  
  if(fixFreq[0].fixFreqAttempt) // Ненулевой счетчик событий входа в режим ограничения максимальной выходной частоты   
  {
    if(alarms.fixFreqAttemptsNumResetMin) // Сброс счетчика попыток разрешен
    {
      timeDiff=RTC_UnixTime.timeStamp-fixFreq[0].eventTimeStamp;
      timeDiff=timeDiff/(60*alarms.fixFreqAttemptsNumResetMin);       
      if(timeDiff) // Прошло достаточно времени для декрементирования счетчика аварий
      {
        if(timeDiff>=fixFreq[0].fixFreqAttempt)fixFreq[0].fixFreqAttempt=0;
        else fixFreq[0].fixFreqAttempt=fixFreq[0].fixFreqAttempt-timeDiff;     
        fixFreq[0].eventTimeStamp=RTC_UnixTime.timeStamp;
        fixFreq[0].CRC16=modBusCRC16((uint8_t*)fixFreq,FIX_FREQ_LENGTH);          
      }        
    }    
  }  
}