#include "archive.h"
#include "sram_rtc.h"
#include "fram.h"

volatile TAPS_UPDATE_COUNTERS tapsUpdateCnt;
volatile TAP_RECORD_UNION tapRecord;
volatile TAPS_CONFIG_STRUCT tapsConfig;

void tiiInit(void)
{
  uint16_t i;
  
  framRead(ARCHIVE_SETTINGS_FRAM_ADDR,(uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_FRAM_LENGTH);  // Настройки архивирования по отводам     
  if(tapsConfig.CRC16!=modBusCRC16((uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_MODBUS_REGNUM*2)) // Загрузка данных по умолчанию
  {
    for(i=0;i<16;i++)
    {
      tapsConfig.source[i]=i;
      tapsConfig.intervalMin[i]=120;           
    }    
    tapsConfig.isOn=0; // Архивирование при первом включении отключить
    tapsConfig.dayRecordEnable=0;
    tapsConfig.recordShiftSec=0;
    tapsConfig.dayRecordHour=0;
    tapsConfig.dayRecordMin=0;
    tapsConfig.dayRecordSec=0;
    tapsConfig.CRC16=modBusCRC16((uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_MODBUS_REGNUM*2);
    framWrite(ARCHIVE_SETTINGS_FRAM_ADDR,(uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_FRAM_LENGTH);
  }  
  //-----------Считывание промежуточных счетчиков-------------------------------
  tapsUpdateCnt.cntRequest=0;
  framRead(TAPS_UPDATE_CNT_FRAM_ADDR,(uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH);    
  if(tapsUpdateCnt.CRC16!=modBusCRC16((uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH-2)) // Контрольная сумма не совпадает
  {
    for(i=0;i<16;i++)
    {
      tapsUpdateCnt.timeStamp[i]=0;       // Временную метку выставить как 01.01.2000 г, не являющуюся достоверной
      tapsUpdateCnt.recordMinCnt[i]=0;    // Счетчик накопленных минут для отводов
      tapsUpdateCnt.dayRecordMinCnt[0]=0; // Счетчик накопленных минут для формирования суточной записи отвода        
      tapsUpdateCnt.framMinCnt[i]=0;      // Сбросить счетчики по отводам   
      tapsUpdateCnt.dayFramMinCnt[i]=0;   // Сбросить суточные счетчики            
    }
    tapsUpdateCnt.dayTimeStamp=0;            // Временная метка формирования суточной записи, выставить как 01.01.2000 г, не являющуюся достоверной   
    tapsUpdateCnt.recordNum=0;               // Пусть текущая запись контроллера будет нулевой
    tapsUpdateCnt.CRC16=modBusCRC16((uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH-2);
    framWrite(TAPS_UPDATE_CNT_FRAM_ADDR,(uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH); 
  }
  //----------------------------------------------------------------------------  
}

void tiiTask(void)
{
  RTC_TimeTypeDef  time;
  static uint8_t prevMinutes=60;
  static uint8_t prevSeconds=60;
  static uint8_t OffToOnTrigger=0; // Триггер процедуры включения архивирования
  uint32_t tempUINT32;    
  uint64_t tempUINT64;
  uint8_t i,j;
  uint8_t crcIndex;
  uint16_t dayRecordFlag=0;    
  
  if(tapsConfig.isOn) // Включен режим архивирования по отводам
  {    
    if(OffToOnTrigger)
    {
      OffToOnTrigger=0;
      tapsUpdateCnt.cntRequest=0;
      prevMinutes=60;
      prevSeconds=60;  
      for(i=0;i<16;i++)
      {        
        tapsUpdateCnt.recordMinCnt[i]=0;    // Счетчик накопленных минут для отводов
        tapsUpdateCnt.dayRecordMinCnt[0]=0; // Счетчик накопленных минут для формирования суточной записи отвода        
        tapsUpdateCnt.framMinCnt[i]=0;      // Сбросить счетчики по отводам   
        tapsUpdateCnt.dayFramMinCnt[i]=0;   // Сбросить суточные счетчики            
      } 
      timeStampsUpdate(); 
      tapsUpdateCnt.CRC16=modBusCRC16((uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH-2);
      framWrite(TAPS_UPDATE_CNT_FRAM_ADDR,(uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH); // Сохранять данные на случай выключения питания         
    }
    
    if(prevSeconds<60)
    {
      if(prevSeconds!=RTC_TimeStructure.Seconds)    
      {
        switch(tapsUpdateCnt.cntRequest)
        {
          case 0: // Запросить первый слепок счетчиков ТИИ
            tapsUpdateCnt.cntRequest=1; // При включении мк запросить счетчики
          break;
          case 2: // Есть слепок счетчиков 
            for(i=0;i<15;i++)tapsUpdateCnt.prevCnt[i]=tapsUpdateCnt.curCnt[i]; // Перебор ТИИ
            tapsUpdateCnt.cntRequest=3; // Пропускаем секунду, чтобы набрались импульсы по ТИИ   
            tapsUpdateCnt.prevCnt[15]=activePower.infiniteSum;
          break;        
          case 3:
            tapsUpdateCnt.cntRequest=4;
          break;
          case 5: // Есть два последовательных состояния счетчиков                    
            for(i=0;i<16;i++) // Перебор отводов
            {              
              j=i;
              if(tapsConfig.intervalMin[i]) // Есть активный отвод
              {                                 
                if(j==15)// Активная мощность считается иначе, нежели ТИИ
                {
                  if((activePower.infiniteSum)>=tapsUpdateCnt.prevCnt[j])
                  {
                    tempUINT64=(uint64_t)(activePower.infiniteSum)-(uint64_t)tapsUpdateCnt.prevCnt[j];
                  }else{
                    tempUINT64=(uint64_t)0x100000000L+(uint64_t)(activePower.infiniteSum)-(uint64_t)tapsUpdateCnt.prevCnt[j];
                  } 
                  tempUINT32=(uint32_t)tempUINT64;
                }else{                
                  if(tapsUpdateCnt.curCnt[j]>=tapsUpdateCnt.prevCnt[j]){                
                    tempUINT32=tapsUpdateCnt.curCnt[j]-tapsUpdateCnt.prevCnt[j];                                                 
                  }else{
                    tempUINT32=0x10000L+tapsUpdateCnt.curCnt[j]-tapsUpdateCnt.prevCnt[j];                                           
                  }                   
                }
                tapsUpdateCnt.framMinCnt[j]+=tempUINT32;
                if(tapsConfig.dayRecordEnable)tapsUpdateCnt.dayFramMinCnt[j]+=tempUINT32;
                else{
                  tapsUpdateCnt.dayFramMinCnt[j]=0;
                  tapsUpdateCnt.dayRecordMinCnt[j]=0;
                }
                tapsUpdateCnt.prevCnt[j]=tapsUpdateCnt.curCnt[j];                            
              } 
            }
            tapsUpdateCnt.prevCnt[15]=activePower.infiniteSum;
            tapsUpdateCnt.cntRequest=3; // Продолжить процесс
          break;
        }        
      }        
    }
    prevSeconds=RTC_TimeStructure.Seconds;       
    
    if(prevMinutes<60) 
    {
      if(prevMinutes!=RTC_TimeStructure.Minutes) // Прошла очередная минута...    
      {          
        for(i=0;i<16;i++)
        {
          if(tapsConfig.intervalMin[i]) // Есть отвод с ненулевым значением интервального замера в настройках => активный            
          {            
            tapsUpdateCnt.recordMinCnt[i]++;     // Добавляем фактических минут в интервал замера                 
            if(tapsConfig.dayRecordEnable)tapsUpdateCnt.dayRecordMinCnt[i]++;  // Добавляем число фактических минут для формирования суточного замера
            if(RTC_UnixTime.timeStamp>=tapsUpdateCnt.timeStamp[i]) // Настуило время сформировать запись по заданному отводу
            {
              tapsUpdateCnt.recordNum++;
              if(tapsUpdateCnt.recordNum>=TAPS_ARCHIVE_RECORDS_NUM)tapsUpdateCnt.recordNum=0;  
              tapsUpdateCnt.timeStamp[i]+=tapsConfig.recordShiftSec;
              tapRecord.fld.timeStampMSB=(uint16_t)(tapsUpdateCnt.timeStamp[i]>>16);
              tapRecord.fld.timeStampLSB=(uint16_t)(tapsUpdateCnt.timeStamp[i]&0xFFFF);
              tapRecord.fld.tapNum=i+1;
              tapRecord.fld.tapNum|=(recordType(tapsConfig.intervalMin[i]))*256;              
              if(i==15)
              {
                tapsUpdateCnt.framMinCnt[i]=(uint32_t)((float)tapsUpdateCnt.framMinCnt[i]/3600.0f); // Замер береться раз в 2 секунды               
              }
              tapRecord.fld.valueMSB=(uint16_t)(tapsUpdateCnt.framMinCnt[i]>>16);
              tapRecord.fld.valueLSB=(uint16_t)(tapsUpdateCnt.framMinCnt[i]&0xFFFF);
              tapsUpdateCnt.framMinCnt[i]=0;
              tapRecord.fld.intervalMin=tapsUpdateCnt.recordMinCnt[i];     
              tapsUpdateCnt.recordMinCnt[i]=0;
              tapsUpdateCnt.framMinCnt[i]=0; // Сброс обновляемого счетчика для последующего формирования записи                              
              tapRecord.fld.crcSum=0;              
              for(crcIndex=0;crcIndex<12;crcIndex++)tapRecord.fld.crcSum+=tapRecord.bytes[crcIndex];              
              eventLogWrite((tapRecord.fld.tapNum&0x001F)+100);
              framWrite(TAPS_ARCHIVE_FRAM_ADDR+tapsUpdateCnt.recordNum*TAPS_ARCHIVE_RECORD_SIZE,(uint8_t*)&tapRecord,TAPS_ARCHIVE_RECORD_SIZE);
              framWrite(TAPS_ARCHIVE_FRAM_ADDR-2,(uint8_t*)&tapsUpdateCnt.recordNum,2);
              /*----------------Вычислить время формирования новой записи-----------------------------------*/
              time.Seconds=RTC_TimeStructure.Seconds;
              time.Minutes=RTC_TimeStructure.Minutes;                  
              time.Hours=RTC_TimeStructure.Hours;                                
              switch(tapsConfig.intervalMin[i])
              { 
                case 1:      
                  time.Seconds=0;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+60;                                      
                break;
                case 5:      
                  time.Seconds=0;
                  time.Minutes=(time.Minutes/5)*5;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+300;                                       
                break;                                
                case 10:         
                  time.Seconds=0;
                  time.Minutes=(time.Minutes/10)*10;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+600;                                       
                break;                
                case 20:         
                  time.Seconds=0;                                
                  time.Minutes=(time.Minutes/20)*20;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+1200;                                       
                break;                
                case 30:         
                  time.Seconds=0;
                  time.Minutes=(time.Minutes/30)*30;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+1800;                                       
                break;                 
                case 60:    
                  time.Seconds=0;
                  time.Minutes=0;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+3600;                                       
                break;    
                case 120:        
                  time.Seconds=0;
                  time.Minutes=0;
                  time.Hours=(time.Hours/2)*2;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+7200;                                       
                break;        
                case 240:        
                  time.Seconds=0;
                  time.Minutes=0;
                  time.Hours=(time.Hours/4)*4;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+14400;                                       
                break;           
                case 360:     
                  time.Seconds=0;
                  time.Minutes=0;
                  time.Hours=(time.Hours/6)*6;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+21600;                                       
                break;                
                case 1440:
                  time.Seconds=tapsConfig.dayRecordSec;
                  time.Minutes=tapsConfig.dayRecordMin;
                  time.Hours=tapsConfig.dayRecordHour;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+86400;                  
                break;  
                default:  // По умолчанию формируются двухчасовки
                  time.Seconds=0;
                  time.Minutes=0;
                  time.Hours=(time.Hours/2)*2;
                  tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+7200;                   
                break;
              }              
            }
            
            if(tapsConfig.dayRecordEnable) //Есть команда по формирования суточных записей
            {
              if(tapsConfig.intervalMin[i]!=1440) // По тем отводам, по которым уже есть суточные замеры - не формировать их
              {                               
                if(RTC_UnixTime.timeStamp>=tapsUpdateCnt.dayTimeStamp) // Время формирования суточной записи
                {
                  uint32_t tempDayStamp;
                  tapsUpdateCnt.recordNum++;
                  if(tapsUpdateCnt.recordNum>=TAPS_ARCHIVE_RECORDS_NUM)tapsUpdateCnt.recordNum=0;                 
                  tempDayStamp=tapsUpdateCnt.dayTimeStamp;
                  tapsUpdateCnt.dayTimeStamp+=tapsConfig.recordShiftSec;
                  tapRecord.fld.timeStampMSB=(uint16_t)(tapsUpdateCnt.dayTimeStamp>>16);
                  tapRecord.fld.timeStampLSB=(uint16_t)(tapsUpdateCnt.dayTimeStamp&0xFFFF);
                  tapRecord.fld.tapNum=i+1;
                  tapRecord.fld.tapNum|=2560;
                  
                 // заменяю tapsConfig.source[i] на i -  то есть таблицу перекодировки убираю 
                  if(i==15)
                  {
                    tapsUpdateCnt.dayFramMinCnt[i]=(uint32_t)((float)tapsUpdateCnt.dayFramMinCnt[i]/3600.0f); // Замер береться раз в 2 секунды               
                  }                  
                  tapRecord.fld.valueMSB=(uint16_t)(tapsUpdateCnt.dayFramMinCnt[i]>>16);
                  tapRecord.fld.valueLSB=(uint16_t)(tapsUpdateCnt.dayFramMinCnt[i]&0xFFFF);                                   
                  tapRecord.fld.intervalMin=tapsUpdateCnt.dayRecordMinCnt[i];                                     
                  tapRecord.fld.crcSum=0;              
                  for(crcIndex=0;crcIndex<12;crcIndex++)tapRecord.fld.crcSum+=tapRecord.bytes[crcIndex];
                  eventLogWrite((tapRecord.fld.tapNum&0x001F)+100);
                  framWrite(TAPS_ARCHIVE_FRAM_ADDR+tapsUpdateCnt.recordNum*TAPS_ARCHIVE_RECORD_SIZE,(uint8_t*)&tapRecord,TAPS_ARCHIVE_RECORD_SIZE);                  
                  framWrite(TAPS_ARCHIVE_FRAM_ADDR-2,(uint8_t*)&tapsUpdateCnt.recordNum,2);
                  tapsUpdateCnt.dayTimeStamp=tempDayStamp;
                  tapsUpdateCnt.dayFramMinCnt[i]=0; // Сброс обновляемого счетчика для последующего формирования записи                                      
                  tapsUpdateCnt.dayRecordMinCnt[i]=0;
                  dayRecordFlag=1;
                }                
              }                
            }           
          }          
        } 
        if(dayRecordFlag)
        {
          time.Seconds=tapsConfig.dayRecordSec;
          time.Minutes=tapsConfig.dayRecordMin;
          time.Hours=tapsConfig.dayRecordHour;
          tapsUpdateCnt.dayTimeStamp=toUnix(RTC_DateStructure,time)+86400;        
        }
        tapsUpdateCnt.CRC16=modBusCRC16((uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH-2);
        framWrite(TAPS_UPDATE_CNT_FRAM_ADDR,(uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH); // Сохранять данные на случай выключения питания          
      }      
    }    
    prevMinutes=RTC_TimeStructure.Minutes;                       
  }else{ // Режим архивирования выключен  
    OffToOnTrigger=1; // Было состояние "выключено"       
  }	
}

void timeStampsUpdate(void) // Обновление временных меток, связанное с переводом времени
{
  uint8_t i;
  RTC_TimeTypeDef  time;
  uint32_t tempUnixTime;
  
  for(i=0;i<16;i++)
  {   
    if(tapsConfig.intervalMin[i])
    {
      time.Seconds=RTC_TimeStructure.Seconds;
      time.Minutes=RTC_TimeStructure.Minutes;                  
      time.Hours=RTC_TimeStructure.Hours;         
      switch(tapsConfig.intervalMin[i])
      { 
        case 1:      
          time.Seconds=0;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+60;                                      
        break;
        case 5:      
          time.Seconds=0;
          time.Minutes=(time.Minutes/5)*5;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+300;                                       
        break;                                
        case 10:         
          time.Seconds=0;
          time.Minutes=(time.Minutes/10)*10;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+600;                                       
        break;                
        case 20:         
          time.Seconds=0;                                
          time.Minutes=(time.Minutes/20)*20;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+1200;                                       
        break;                
        case 30:         
          time.Seconds=0;
          time.Minutes=(time.Minutes/30)*30;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+1800;                                       
        break;                 
        case 60:    
          time.Seconds=0;
          time.Minutes=0;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+3600;                                       
        break;    
        case 120:        
          time.Seconds=0;
          time.Minutes=0;
          time.Hours=(time.Hours/2)*2;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+7200;                                       
        break;        
        case 240:        
          time.Seconds=0;
          time.Minutes=0;
          time.Hours=(time.Hours/4)*4;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+14400;                                       
        break;           
        case 360:     
          time.Seconds=0;
          time.Minutes=0;
          time.Hours=(time.Hours/6)*6;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+21600;                                       
        break;                
        case 1440:
          time.Seconds=tapsConfig.dayRecordSec;
          time.Minutes=tapsConfig.dayRecordMin;
          time.Hours=tapsConfig.dayRecordHour;
          tempUnixTime=toUnix(RTC_DateStructure,time); // Эти сутки
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time);
          if(RTC_UnixTime.timeStamp>tempUnixTime)tapsUpdateCnt.timeStamp[i]+=86400; // Если прошли время взятия отсчета, то перенести на другие сутки                         
        break;  
        default:  // По умолчанию формируются двухчасовки
          time.Seconds=0;
          time.Minutes=0;
          time.Hours=(time.Hours/2)*2;
          tapsUpdateCnt.timeStamp[i]=toUnix(RTC_DateStructure,time)+7200;                   
        break;
      }
    }
  }
  time.Seconds=tapsConfig.dayRecordSec;
  time.Minutes=tapsConfig.dayRecordMin;
  time.Hours=tapsConfig.dayRecordHour;
  tapsUpdateCnt.dayTimeStamp=toUnix(RTC_DateStructure,time); // Эти сутки  
  if(RTC_UnixTime.timeStamp>tapsUpdateCnt.dayTimeStamp)tapsUpdateCnt.dayTimeStamp+=86400; // Если прошли время взятия отсчета, то перенести на другие сутки 
  tapsUpdateCnt.CRC16=modBusCRC16((uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH-2);
  framWrite(TAPS_UPDATE_CNT_FRAM_ADDR,(uint8_t*)&tapsUpdateCnt.timeStamp[0],TAPS_UPDATE_CNT_FRAM_LENGTH); // Сохранять данные на случай выключения питания    
}

uint8_t recordType(uint16_t interval)
{
  uint8_t type;
  switch(interval)
  {
    case 1:
      type=1;
    break;
    case 5:
      type=2;
    break;    
    case 10:
      type=3;
    break;
    case 20:
      type=4;
    break;
    case 30:
      type=5;
    break;
    case 60:
      type=6;
    break;
    case 120:
      type=7;
    break;
    case 240:
      type=8;
    break;
    case 360:
      type=9;
    break;
    case 1440:
      type=10;
    break;
    default:    
      type=7;
    break;
  }
  return type;
}
