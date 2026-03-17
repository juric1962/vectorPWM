#include "modbus.h"
#include "pwm.h"
#include "main.h"
#include "DS18B20.h"
#include "sram_rtc.h"
#include "alarms.h"
#include "fram.h"
#include "archive.h"
#include "dynagram.h"
#include "map_ef.h"
#include "sim800.h"

uint16_t FantomModbusAddr;
unsigned char CRC16HighTable[] = { // Таблица для вычисления CRC16 Модбас
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;

char CRC16LowTable[] = { // Таблица для вычисления CRC16 Модбас
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;


/*
typedef struct{
  // Start ModBus reg
  uint16_t ip0;   // Ip адрес сервера связи тетрада (н-р) 10.1.1.17 ->10
  uint16_t ip1;   // Ip адрес сервера связи тетрада (н-р) 10.1.1.17 ->1
  uint16_t ip2;   // Ip адрес сервера связи тетрада (н-р) 10.1.1.17 ->1
  uint16_t ip3;   // Ip адрес сервера связи тетрада (н-р) 10.1.1.17 ->17
  uint16_t port;  // UDP порт сервера связи 
  uint16_t num;   // Номер устройства в сети
  uint16_t nat;   // Время удержания сессии в сек
  uint16_t cch;   // Период контроль канала в сек
  uint16_t tm_ch; // Таймаут на квитанцию сек 
  char a_c_gprs[58];  // логин оператора связи, строка должна заканчиваться нулем 
  // End ModBus reg
}SIM800_SETTINGS_STRUCT;
SIM800_SETTINGS_STRUCT sim800Settings;

void sim800StructInit(void) // Инициализация структуры sim800 для работы с модбас
{
  uint8_t temp[4];  
  framRead(A_IP_PAR,temp,4);  
  sim800Settings.ip0=temp[0];
  sim800Settings.ip1=temp[1];
  sim800Settings.ip2=temp[2];
  sim800Settings.ip3=temp[3];  
  framRead(A_PORT,(uint8_t*)&sim800Settings.port,2); 
  framRead(A_NUM,(uint8_t*)&sim800Settings.num,2); 
  framRead(A_NAT,(uint8_t*)&sim800Settings.nat,2);   
  framRead(A_CCH,(uint8_t*)&sim800Settings.cch,2); 
  framRead(A_TM_CH,(uint8_t*)&sim800Settings.tm_ch,2); 
  framRead(A_C_GPRS,(uint8_t*)sim800Settings.a_c_gprs,2);   
  copymasNSwap((uint8_t*)sim800Settings.a_c_gprs,(uint8_t*)sim800Settings.a_c_gprs,58);
}

void sim800StructToFlash(void) // Инициализация структуры sim800 для работы с модбас
{
  uint8_t temp[4];  
  temp[0]=sim800Settings.ip0;
  temp[1]=sim800Settings.ip1;
  temp[2]=sim800Settings.ip2;
  temp[3]=sim800Settings.ip3;  
  framWrite(A_IP_PAR,temp,4);  
  framWrite(A_PORT,(uint8_t*)&sim800Settings.port,2); 
  framWrite(A_NUM,(uint8_t*)&sim800Settings.num,2); 
  framWrite(A_NAT,(uint8_t*)&sim800Settings.nat,2);   
  framWrite(A_CCH,(uint8_t*)&sim800Settings.cch,2); 
  framWrite(A_TM_CH,(uint8_t*)&sim800Settings.tm_ch,2); 
  framWrite(A_C_GPRS,(uint8_t*)sim800Settings.a_c_gprs,2);   
}


*/
uint16_t modBusCRC16(uint8_t *msg, uint16_t bytesNum) // Вычисление CRC16 Модбас
{
  unsigned char uchCRCHi = 0xFF ;
  unsigned char uchCRCLo = 0xFF ;
  unsigned char uIndex ;        

  while (bytesNum--)  
  {
    uIndex = uchCRCHi ^ *msg++ ;  
    uchCRCHi = uchCRCLo ^ CRC16HighTable[uIndex] ;
    uchCRCLo = CRC16LowTable[uIndex] ;
  }  
  return (uchCRCHi*256 + uchCRCLo) ;
}

uint8_t CheckData(uint8_t *Buff,uint16_t size) // Проверка принятого пакета Модбас, контрольная сумма в пакете сравнивается с   
{                                             // заново вычисленной: 0 - есть ошибки в пакете, 1 - нет ошибок в пакете      
  uint16_t i2c2_CRC;

  i2c2_CRC = (unsigned int)(Buff[size-2]*256);
  i2c2_CRC|= (unsigned int)(Buff[size-1]);

  if(i2c2_CRC==modBusCRC16(Buff,size-2))
  {
    return 1;
  }
  return 0;
}

void swapregs(uint8_t* pbuf,uint16_t len)
{
  uint8_t m;
  uint16_t i;

  for (i=0;i<len;i++) {
    m=pbuf[i];
    pbuf[i]=pbuf[i+1];
    i++;
    pbuf[i]=m;
  }
}

void copymasNSwap(uint8_t *dst, uint8_t *src, uint16_t bytesNum) // Функция копирования массивов с изменение порядка следования байт
{
  uint32_t i=0;
  uint32_t n=bytesNum;
  while(bytesNum--)
  {
    dst[i]=src[i];
    i++;
  }
  swapregs(dst,n);
}

uint16_t processedRegs;
uint8_t modbusSlaveTask(uint8_t *inputBuf, uint16_t inputBytesNum,uint8_t *outputBuf, uint16_t *bytesToTransmit) // Возвращает следующие значения:0 - все в порядке, 1 - не совпал адрес модбас модема, либо контрольная сумма, либо количество байт < 7 
{
    uint16_t processedBytes=0;     // Количество обработанных байт пакета ELAM    
    uint16_t processedBlocks=0;    // Количество обработанных блоков ELAM
    *bytesToTransmit=0;
    uint8_t elamError=0;
    uint16_t temp=0;
    processedRegs=0; // Ограничение на число запрашиваемых регистров
    
    if(inputBytesNum>7) // Минимальное количество байт - в запросе = 8, запись одного регистра функцией №6
    {
      if(inputBuf[0]>247) // Предположительно поступил ELAM - запрос
      {	
        FantomModbusAddr=(248+(inputBuf[0]&0x07)*256+inputBuf[1]);  // для sim запроса
        if(modbusAddr==(248+(inputBuf[0]&0x07)*256+inputBuf[1])  || (zapr_from_sim800==1) ) // Адрес контроллера совпадает с адресом в запросе
     //   if(modbusAddr==(248+(inputBuf[0]&0x07)*256+inputBuf[1])) // Адрес контроллера совпадает с адресом в запросе
        {
          copymas(elamBuf,inputBuf,inputBytesNum); // Функция копирования массивов
          while((processedBytes<inputBytesNum)&&(elamError==0)) // Осуществить распаковку пакета ELAM
          {            
            copymas(inputBuf,elamBuf,inputBytesNum); // Функция копирования массивов
            switch(inputBuf[2+processedBytes]) // Необходимо, в зависимости от номера функции, производить проверку контрольной суммы
            {
                    case 3:                                                  
                    case 4:
                    case 6:                                                       
                      if(CheckData(&inputBuf[processedBytes],9))
                      {                                                   
                        if(!modbusRegistersEdit(&inputBuf[2+processedBytes],&outputBuf[*bytesToTransmit],FantomModbusAddr,bytesToTransmit)){;}
                        else {
                          elamError=1;
                        }
                      }else elamError=1;
                      processedBytes+=9;      
                      processedBlocks++;                          
                    break;
                    case 16:                                                   
                      temp=(inputBuf[5+processedBytes]*256+inputBuf[6+processedBytes])*2;
                      if(CheckData(&inputBuf[processedBytes],10+temp))
                      {                                                   
                        if(!modbusRegistersEdit(&inputBuf[2+processedBytes],&outputBuf[*bytesToTransmit],FantomModbusAddr,bytesToTransmit)){;}
                        else elamError=1;
                      }else elamError=1;
                      processedBytes+=10+temp;      
                      processedBlocks++;                                                      
                    break; 	
                    default:
                     elamError=1;
                    break;
            }
            //if(processedBlocks>20)elamError=1; // Было обработано максимальное количество блоков    
          }				
        }
      }else{ // Предположительно ModBus - запрос
                if(inputBuf[0]==0)
                {				
                  if(CheckData(inputBuf, inputBytesNum)) 
                  {          
                    modbusRegistersEdit(&inputBuf[1],outputBuf,/*modbusAddr*/0,bytesToTransmit);	// 05.04.2018 - для того, чтобы формировался ответ от 0-го адреса устройства		     
                  } 					                                                // чтобы безошибочно можно было проверять текущий модбас-адрес ЧРЭП  
                }else{ // Предположительно поступил ModBus-запрос, адреса с 1 по 247					
                   // if(modbusAddr==inputBuf[0]) // Адрес контроллера совпадает с адресом в запросе
                      if ((modbusAddr==inputBuf[0]) ||  (zapr_from_sim800==1) )
                    {   
                       if(CheckData(inputBuf, inputBytesNum)) 
                       {	  
                         modbusRegistersEdit(&inputBuf[1],outputBuf,modbusAddr,bytesToTransmit);			     
                       }                           
                    }									
                } 		
            }
    }
    return *bytesToTransmit;
}

/*
uint8_t modbusSlaveTask(uint8_t *inputBuf, uint16_t inputBytesNum,uint8_t *outputBuf, uint16_t *bytesToTransmit) // Возвращает следующие значения:0 - все в порядке, 1 - не совпал адрес модбас модема, либо контрольная сумма, либо количество байт < 7 
{
  *bytesToTransmit=0;
  if(inputBytesNum>7) // Минимальное количество байт - в запросе = 8, запись одного регистра функцией №6
  {	
    if(inputBuf[0]==modbusAddr) // Адрес контроллера совпадает с адресом в запросе
    {   
      if(CheckData(inputBuf, inputBytesNum)) 
      {	
        modbusRegistersEdit(&inputBuf[1],outputBuf,modbusAddr,bytesToTransmit);			     
      }
    }else{
      if(inputBuf[0]==0)
      {
        if(CheckData(inputBuf, inputBytesNum)) 
        {          
          modbusRegistersEdit(&inputBuf[1],outputBuf,modbusAddr,bytesToTransmit);			     
        }        
      }
    }
  }
  return *bytesToTransmit;
}
*/
void floatToBytes(float fValue,uint8_t* bytes) // Преобразует число типа float в массив байт, порядок байт 1-0-3-2
{
  uint32_t tempUINT32;
  tempUINT32=*((uint32_t*)&fValue);
  bytes[1]=(tempUINT32>>8)&0xFF;
  bytes[0]=(tempUINT32)&0xFF;
  bytes[3]=(tempUINT32>>24)&0xFF;
  bytes[2]=(tempUINT32>>16)&0xFF;
}

float bytesToFloat(uint8_t *bytes)// Преобразует массив байт 1-0-3-2 в число типа float
{
  uint32_t tempUINT32;
  tempUINT32=0;
  tempUINT32=bytes[1];
  tempUINT32+=bytes[0]*0x100;
  tempUINT32+=bytes[3]*0x10000;  
  tempUINT32+=bytes[2]*0x1000000;  
  return (*((float*)&tempUINT32));
}

uint8_t modbusRegistersEdit(uint8_t *src, uint8_t *dst, uint16_t slaveAddr, uint16_t *bytesToTransmit)
{	
  uint16_t startAddr;  // Начальный адрес запрашиваемых регистров
  uint16_t regNum;     // Количество регистров
  uint16_t endAddr;    // Конечный адрес запрашиваемых регистров 
  uint16_t crc16;     
  uint8_t dataShift;   // Смещение указателя для копирования данных в пакете
  uint16_t i;
  uint8_t errorCode=0; // Код ошибки ModBus:   
  uint16_t tempUINT16;
  uint32_t tempUINT32;    
  float tempFloat;

        switch(src[0]) // Приведен список поддерживаемых функций, чтобы не обрабатывать заведомо ненужные
	{
		case 3:	// Функция №3				
  		  startAddr=src[1]*256+src[2];
	          regNum=src[3]*256+src[4];
		  endAddr=startAddr+regNum-1;		
                  if(!regNum){errorCode=2;break;} // Игнорировать запрос нулевого количества регистров       
                  processedRegs+=regNum;
                  regNum=regNum*2; // Для сокращения количества вычислений перевожу количество запрошенных регистров в байты		    
                  if(slaveAddr>247){
                    slaveAddr-=248;
                    dst[0]=(uint8_t)(0xF8|(slaveAddr>>8));
                    dst[1]=(uint8_t)(slaveAddr&0xFF);
                    slaveAddr+=248;
                    dst[2]=src[0];
                    dst[3]=(uint8_t)(regNum>>8);                      
                    dst[4]=(uint8_t)(regNum&0xFF);                      
                    dataShift=5;
                    if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров                      
                  }else{                      
                       dst[0]=slaveAddr;
                       dst[1]=src[0];
                       dst[2]=regNum;                      
                       dataShift=3;
                       if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров, для Рината увеличил с 125 до 2500, буферы позволяют
                  }     
                 
                  if((startAddr>=RTC_MODBUS_STARTREG)&&(endAddr<=RTC_MODBUS_ENDREG))
                  {                                              
                    src[0]=0;
                    src[1]=RTC_TimeStructure.Seconds;
                    src[2]=0;
                    src[3]=RTC_TimeStructure.Minutes;
                    src[4]=0;
                    src[5]=RTC_TimeStructure.Hours;
                    src[6]=0;
                    src[7]=RTC_DateStructure.Date;
                    src[8]=0;
                    src[9]=RTC_DateStructure.Month;
                    src[10]=0;
                    src[11]=RTC_DateStructure.Year;
                    tempUINT32=toUnix(RTC_DateStructure,RTC_TimeStructure);
                    src[14]=(uint8_t)((tempUINT32>>24)&0xFF);
                    src[15]=(uint8_t)((tempUINT32>>16)&0xFF);
                    src[12]=(uint8_t)((tempUINT32>>8)&0xFF);
                    src[13]=(uint8_t)(tempUINT32&0xFF);                       
                    tempUINT16=(startAddr-RTC_MODBUS_STARTREG)*2;                       
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                              
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                       
                  }else if((startAddr>=ETR_ENGINE_MODBUS_STARTREG)&&(endAddr<=ETR_ENGINE_MODBUS_ENDREG))			                    
                  {          
                    tempUINT16=(startAddr-ETR_ENGINE_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&etrEngineShadow,(ETR_ENGINE_MODBUS_REGNUM+1)*2);                              
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                       
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                      
                  }else if((startAddr>=ALARMS_MODBUS_STARTREG)&&(endAddr<=ALARMS_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-ALARMS_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&alarmsShadow,(ALARMS_MODBUS_REGNUM+1)*2);                              
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                    
                  }else if((startAddr>=COMMAND_MODBUS_STARTREG)&&(endAddr<=COMMAND_MODBUS_ENDREG))			
		  {                
                    tempUINT16=(startAddr-COMMAND_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&commandShadow,(COMMAND_MODBUS_REGNUM+1)*2);                              
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                        
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                    
                  }else if((startAddr>=ADC_MODBUS_STARTREG)&&(endAddr<=ADC_MODBUS_ENDREG))
                  {               
                    tempUINT16=(startAddr-ADC_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&adcInputsShadow,(ADC_MODBUS_REGNUM+1)*2);                              
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                      
                  }else if((startAddr>=DAC_MODBUS_STARTREG)&&(endAddr<=DAC_MODBUS_ENDREG))
                  {               
                    tempUINT16=(startAddr-DAC_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&dacOutputsShadow,(DAC_MODBUS_REGNUM+1)*2);                              
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                      
                  }else if((startAddr>=TC_MODBUS_STARTREG)&&(endAddr<=TC_MODBUS_ENDREG))
                  {                    
                    if(TCInputs.modBusWrite){;}
                    else for(i=0;i<15;i++)TCInputs.modBusBuffer[i]=TCInputs.transitionsNum[i];
                    tempUINT16=(startAddr-TC_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&TCInputs.modBusBuffer,(TC_MODBUS_REGNUM+1)*2);                              
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                    
                  }                  
                  else if((startAddr>=ENGINE_MODBUS_STARTREG)&&(endAddr<=ENGINE_MODBUS_ENDREG))
                  {                  
                    tempUINT16=(startAddr-ENGINE_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&engineShadow,(ENGINE_MODBUS_REGNUM+1)*2);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                 
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                                           
                  }else if((startAddr>=CONTEXT_MODBUS_STARTREG)&&(endAddr<=CONTEXT_MODBUS_ENDREG))
                  { 
                    tempUINT16=(startAddr-CONTEXT_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&contextShadow,(CONTEXT_MODBUS_REGNUM+1)*2);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;      
                  }else if((startAddr>=ARCHIVE_SETTINGS_MODBUS_STARTREG)&&(endAddr<=ARCHIVE_SETTINGS_MODBUS_ENDREG))                                          
                  {                    
                    tempUINT16=(startAddr-ARCHIVE_SETTINGS_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&tapsConfig,(ARCHIVE_SETTINGS_MODBUS_REGNUM+1)*2);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                       
                  }else if((startAddr>=EVENTS_COUNTER_MODBUS_STARTREG)&&(endAddr<=EVENTS_COUNTER_MODBUS_ENDREG))
                  { 
                    tempUINT16=(startAddr-EVENTS_COUNTER_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&eventsCnt,(EVENTS_COUNTER_MODBUS_REGNUM+1)*2);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                                                     
                  }else if((startAddr>=ACTIVE_ENERGY_MODBUS_STARTREG)&&(endAddr<=ACTIVE_ENERGY_MODBUS_ENDREG))
                  {                 
                    tempUINT16=(startAddr-ACTIVE_ENERGY_MODBUS_STARTREG)*2;                       
                    tempFloat=((float)(*activePower.totalSum))/1000.0;
                    copymasNSwap(src,(uint8_t*)&tempFloat,4);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                     
                  }else if((startAddr>=DYNAGRAM_SETTINGS_MODBUS_STARTREG)&&(endAddr<=DYNAGRAM_SETTINGS_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-DYNAGRAM_SETTINGS_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&dynaGram,(DYNAGRAM_SETTINGS_MODBUS_REGNUM+1)*2);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                 
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                       
                  }else if((startAddr>=CHREP_MAX_INST_CUR_MODBUS_STARTREG)&&(endAddr<=CHREP_MAX_INST_CUR_MODBUS_ENDREG))
                  {
                    chrepShadow.pinCode=0;
                    chrepShadow.maxInstCurrent=chrep.maxInstCurrent/100;
                    tempUINT16=(startAddr-CHREP_MAX_INST_CUR_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&chrepShadow,(CHREP_MAX_INST_CUR_MODBUS_REGNUM+1)*2);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                 
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                   
                  }else if((startAddr>=SIM800_SETTINGS_MODBUS_STARTREG)&&(endAddr<=SIM800_SETTINGS_MODBUS_ENDREG))
                  {
                    sim800StructInit();
                    tempUINT16=(startAddr-SIM800_SETTINGS_MODBUS_STARTREG)*2;                       
                    copymasNSwap(src,(uint8_t*)&sim800Settings,(SIM800_SETTINGS_MODBUS_REGNUM+1)*2);
                    for(i=0;i<regNum;i++)dst[dataShift+i]=src[tempUINT16+i];                                                 
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;            
                  }else errorCode=2;                      
                  
                  if((startAddr>=MODBUS_ADDR_STARTREG)&&(endAddr<=MODBUS_ADDR_STARTREG))
                  {
                    //if(*(src-1)==0)
                      if( (*(src-1)==0)  || (zapr_from_sim800==1) )
                    {
                      dst[dataShift]=(uint8_t)(modbusAddr>>8);
                      dst[dataShift+1]=(uint8_t)(modbusAddr&0xFF);
                      crc16=modBusCRC16(dst,regNum+dataShift);      
                      dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                      dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                
                      errorCode=0;
                      *bytesToTransmit+=regNum+dataShift+2;                         
                    }
                  }                    
                break;
                case 4: // Функция №4
  		  startAddr=src[1]*256+src[2];
	          regNum=src[3]*256+src[4];
                  processedRegs+=regNum;
		  endAddr=startAddr+regNum-1;	                  
                  if(!regNum){errorCode=2;break;} // Игнорировать запрос нулевого количества регистров                 
                  regNum=regNum*2; // Для сокращения количества вычислений перевожу количество запрошенных регистров в байты		    
                  if(slaveAddr>247){
                    slaveAddr-=248;
                    dst[0]=(uint8_t)(0xF8|(slaveAddr>>8));
                    dst[1]=(uint8_t)(slaveAddr&0xFF);
                    slaveAddr+=248;
                    dst[2]=src[0];
                    dst[3]=(uint8_t)(regNum>>8);                      
                    dst[4]=(uint8_t)(regNum&0xFF);                      
                    dataShift=5;
                    if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров                      
                  }else{                      
                    dst[0]=slaveAddr;
                    dst[1]=src[0];
                    dst[2]=regNum;                      
                    dataShift=3;
                    if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров, для Рината увеличил с 125 до 2500, буферы позволяют
                  }                   
                  if((startAddr>=MEASUREMENTS_MODBUS_STARTREG)&&(endAddr<=MEASUREMENTS_MODBUS_ENDREG))			
		  {     
                    measurements.tempSensor1=tempSensor[0].curTemp;
                    measurements.tempSensor2=tempSensor[1].curTemp;                                        
                    measurements.tempSensor3=tempSensor[2].curTemp;                                        
                    measurements.alarmBits1=alarms.faultBits;
                    measurements.alarmBits2=alarms.faultBitsExtended;         
                    measurements.maxIstartValue=(uint16_t)((float)measurements.maxIstartValueTemp/70.711f);
                    measurements.adcMV[0]=(int16_t)measurements.adcCode[0];//(2.71f*(float)measurements.adcCode[0]);   
                    measurements.adcMV[1]=(int16_t)measurements.adcCode[1];//(2.71f*(float)measurements.adcCode[1]);                      
                    measurements.adcMV[2]=(int16_t)measurements.adcCode[2];//(4.961553f*(float)measurements.adcCode[2]);
                    measurements.adcMV[3]=(int16_t)measurements.adcCode[3];//(4.961553f*(float)measurements.adcCode[3]);                                           
                    measurements.TC=TCInputs.bits&0x7FFF;
                    measurements.TU=stateDOUT;                    
                    measurements.dynaGramForce=(float)measurements.adcCode[2]*dynaGram.tit3ToForceCoeff+dynaGram.tit3ToForceConst; // Показания с динамографа                        		                    
                    if(vectorPWM.isOn==0)
                    {
                      measurements.Iabc=0.0f;  
                      measurements.Ia=0.0f;
                      measurements.Ib=0.0f;
                      measurements.Ic=0.0f;
                        
                      measurements.Uavg=0.0f;  
                      measurements.Ua=0.0f;
                      measurements.Ub=0.0f;
                      measurements.Uc=0.0f;

                      measurements.Pabc=0.0f;  
                      measurements.Pa=0.0f;
                      measurements.Pb=0.0f;
                      measurements.Pc=0.0f;
                        
                      measurements.Qabc=0.0f;  
                      measurements.Qa=0.0f;
                      measurements.Qb=0.0f;
                      measurements.Qc=0.0f;
                        
                      measurements.Sabc=0.0f;    
                      measurements.Sa=0.0f;
                      measurements.Sb=0.0f;
                      measurements.Sc=0.0f; 

                      measurements.cosPhiAvg=0.0f;
                      measurements.cosPhiA=0.0f;
                      measurements.cosPhiB=0.0f;
                      measurements.cosPhiC=0.0f;  
                      measurements.Udc=0.0f;   
                      
                      measurements.maxUdc=0;
                      measurements.minUdc=0;
                      measurements.ppUdc=0;
                      measurements.maxIrms=0;
                      measurements.minIrms=0;
                      measurements.ppIrms=0;                            
                    }else{                     
                      measurements.ppIrms=measurements.maxIrms-measurements.minIrms;
                      measurements.ppUdc=measurements.maxUdc-measurements.minUdc;
                    }
                    measurements.dynaGramState=dynaGram.state;
                    measurements.dynaGramError=dynaGram.error;                    
                    tempUINT16=(startAddr-MEASUREMENTS_MODBUS_STARTREG)*2;                    
                    copymasNSwap(src,(uint8_t*)&measurements.Iabc,(MEASUREMENTS_MODBUS_REGNUM+1)*2);                    
                    for(i=0;i<regNum;i++)
                    {
                      dst[dataShift+i]=src[tempUINT16+i];
                    }                                        
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                      
                  }else if((startAddr>=DIAG_MODBUS_STARTREG)&&(endAddr<=DIAG_MODBUS_ENDREG))
                  {                    
                    diag.I=(uint16_t)etrEngine.I;                   
                    diag.overloadCurrent=(uint16_t)etrEngine.overloadCurrent;
                    diag.overloadLimit=(uint16_t)(etrEngine.overloadLimit/100.0f);
                    diag.overloadCnt=(uint16_t)(100.0f*etrEngine.overloadCnt/etrEngine.overloadLimit);                        

                    diag.curState=systemState.curState;
                    diag.nextState=systemState.nextState;
                    diag.curMode=systemState.curMode;
                    diag.tempMode=systemState.tempMode;
                    diag.prevMode=systemState.prevMode;
                    diag.manualStart=systemState.manualStart;
                    diag.stopCHREP=command.stopCHREP;
                    diag.kpStart=systemState.kpStart;
                    diag.faults=systemState.faults;
                    diag.startDisable=systemState.startDisable;                                         
  
                    diag.currentCnt=faultsArrayNVRAM[BRIDGE_SATURATION_FAULT].upCnt;  
                    diag.disbalanceCnt=faultsArrayNVRAM[BREAK_PHASE_CURRENT_FAULT].upCnt;  
                    diag.highVoltageCnt=faultsArrayNVRAM[UDC_OVERVOLTAGE_FAULT].upCnt; 
                    diag.noLoadCnt=faultsArrayNVRAM[NO_LOAD_FAULT].upCnt;      
                    diag.etrCnt=faultsArrayNVRAM[ETR_OVERLOAD_FAULT].upCnt;      
                    diag.lowVoltageCnt=faultsArrayNVRAM[LOW_VOLTAGE_FAULT].upCnt;       
                    diag.temperatureCnt=faultsArrayNVRAM[TEMP_SENS_FAULT].upCnt;      
                    diag.fixFreqAttempt=(uint16_t)fixFreq[0].fixFreqAttempt;
                    diag.dynagramLoadErrorCnt=faultsArrayNVRAM[DYNAMOMETER_FAULT].upCnt;
                    
                    tempUINT16=(startAddr-DIAG_MODBUS_STARTREG)*2;                    
                    copymasNSwap(src,(uint8_t*)&diag.rs485_CRC_Err,(DIAG_MODBUS_REGNUM+1)*2);                    
                    for(i=0;i<regNum;i++)
                    {
                      dst[dataShift+i]=src[tempUINT16+i];
                    }                                        
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                                          
                  }else if((startAddr>=EVENT_LOG_MODBUS_STARTREG)&&(endAddr<=EVENT_LOG_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-EVENT_LOG_MODBUS_STARTREG)*2;                       
                    bkpSRAM_Read(EVENT_LOG_ADDR+tempUINT16,(uint8_t*)src,regNum*2);                    
                    copymasNSwap(&dst[dataShift],src,regNum*2);                                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                     
                  }else if((startAddr>=ARCHIVE_MODBUS_STARTREG)&&(endAddr<=ARCHIVE_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-ARCHIVE_MODBUS_STARTREG)*2;                         
                    framRead(TAPS_ARCHIVE_FRAM_ADDR-2+tempUINT16,(uint8_t*)src, regNum*2);                     
                    copymasNSwap(&dst[dataShift],src,regNum*2);                                   
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                       
                  }else if((startAddr>=PROG_VER_MODBUS_STARTREG)&&(endAddr<=PROG_VER_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-PROG_VER_MODBUS_STARTREG)*2;                    
                    copymas(src,(uint8_t*)progVersionString,(PROG_VER_MODBUS_REGNUM+1)*2);                    
                    for(i=0;i<regNum;i++)
                    {
                      dst[dataShift+i]=src[tempUINT16+i];
                    }                                        
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                     
                  }else if((startAddr>=DYNAGRAM_MODBUS_STARTREG)&&(endAddr<=DYNAGRAM_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-DYNAGRAM_MODBUS_STARTREG)*2;                    
                    copymasNSwap(src,(uint8_t*)&dynaOutputData+tempUINT16,regNum);
                    for(i=0;i<regNum;i++)
                    {
                      dst[dataShift+i]=src[i];
                    }                                        
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                     
                  }else if((startAddr>=LUFKIN_CARD_MODBUS_STARTREG)&&(endAddr<=LUFKIN_CARD_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-LUFKIN_CARD_MODBUS_STARTREG)*2;                    
                    copymasNSwap(src,(uint8_t*)&lufkinCardForce+tempUINT16,regNum);
                    for(i=0;i<regNum;i++)
                    {
                      dst[dataShift+i]=src[i];
                    }                                        
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                                        
                  }else if((startAddr>=DYNAMORGAM_PLOT_MODBUS_STARTREG)&&(endAddr<=DYNAMORGAM_PLOT_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-DYNAMORGAM_PLOT_MODBUS_STARTREG)*2;                    
                    dynaPlot.timeStamp=lufkinCardForce.timeStamp;
                    for(i=0;i<5;i++) // Сброс элементов рисования
                    {
                      dynaPlot.drawElement[i].typeDraw=0; 
                      dynaPlot.drawElement[i].dataType=0; 
                      dynaPlot.drawElement[i].dataArray[0]=0;                      
                      dynaPlot.drawElement[i].dataArray[1]=0;
                      dynaPlot.drawElement[i].dataArray[2]=0;
                      dynaPlot.drawElement[i].dataArray[3]=0;                      
                    }                    
                    
                    i=0;
                    if(dynaGram.loadMaxMinHoldMS) // Активная аварийная уставка по усилию на динамографе
                    {
                      dynaPlot.drawElement[i].typeDraw=2; // 2 Линии
                      dynaPlot.drawElement[i].dataType=5; // Тип данных REAL
                      floatToBytes(dynaGram.loadMax,(uint8_t*)&dynaPlot.drawElement[i].dataArray[0]);
                      floatToBytes(dynaGram.loadMin,(uint8_t*)&dynaPlot.drawElement[i].dataArray[2]);
                      i++; 
                      /*dynaPlot.drawElement[i].dataArray[0]=(uint16_t)(-1234); - для INT16   
                      //dynaPlot.drawElement[i].dataArray[0]=tempUINT32&0xFFFF; - UDINT
                      //dynaPlot.drawElement[i].dataArray[1]=tempUINT32>>16;
                      //dynaPlot.drawElement[i].dataArray[0]=tempINT32&0xFFFF; - DINT
                      /dynaPlot.drawElement[i].dataArray[1]=(uint16_t)(tempINT32>>16);*/ 
                    }
                    
                    if(dynaGram.algorithmOn) // Активен алгоритм регулирования выходной частоты ЧРЭП по минимальному усилию на динамограмме
                    {
                      if(dynaGram.algorithmKp>0.000001f)
                      {
                        dynaPlot.drawElement[i].typeDraw=1; // 1 Линии
                        dynaPlot.drawElement[i].dataType=5; // Тип данных REAL
                        floatToBytes(dynaGram.algorithmWorkLoadON,(uint8_t*)&dynaPlot.drawElement[i].dataArray[0]);                          
                        i++;                          
                      }else{
                        dynaPlot.drawElement[i].typeDraw=2; // 2 Линии
                        dynaPlot.drawElement[i].dataType=5; // Тип данных REAL
                        floatToBytes(dynaGram.algorithmWorkLoadON,(uint8_t*)&dynaPlot.drawElement[i].dataArray[0]);
                        floatToBytes(dynaGram.algorithmWorkLoadOFF,(uint8_t*)&dynaPlot.drawElement[i].dataArray[2]);                                             
                        i++;  
                        if(dynaGram.algorithmPumpRetryNum)
                        {
                          dynaPlot.drawElement[i].typeDraw=4; // Точка-звездочка
                          dynaPlot.drawElement[i].dataType=5; // Тип данных REAL
                          floatToBytes(dynaGram.algorithmPumpShift,(uint8_t*)&dynaPlot.drawElement[i].dataArray[0]);
                          floatToBytes(dynaGram.algorithmPumpForce,(uint8_t*)&dynaPlot.drawElement[i].dataArray[2]);                                            
                          i++;                        
                        }
                      }
                    }         
                    
                    /*-------------DEBUG ONLY, DELETE AFTER!!!!!----------------
                    //if(dynaGram.algorithmOn==2)
                    {                      
                      int16_t dLoad=(dynaGram.testDynagramLoadMax-dynaGram.testDynagramLoadMin)/50;
                      int16_t dShift=(dynaGram.testDynagramShiftMax-dynaGram.testDynagramShiftMin)/50;        
                      uint8_t j=0; 
                      for(i=0;i<50;i++)  
                      {
                        lufkinCardForce.pair[j].Position=dynaGram.testDynagramShiftMax;
                        lufkinCardForce.pair[j].Load=dynaGram.testDynagramLoadMax-i*dLoad;  
                        j++;
                      }
                      
                      for(i=0;i<50;i++)
                      {
                        lufkinCardForce.pair[j].Position=dynaGram.testDynagramShiftMax-i*dShift;
                        lufkinCardForce.pair[j].Load=dynaGram.testDynagramLoadMin;    
                        j++;
                      }
                      for(i=0;i<50;i++)  
                      {
                        lufkinCardForce.pair[j].Position=dynaGram.testDynagramShiftMin;
                        lufkinCardForce.pair[j].Load=dynaGram.testDynagramLoadMin+i*dLoad;  
                        j++;
                      }
                      
                      for(i=0;i<50;i++)
                      {
                        lufkinCardForce.pair[j].Position=dynaGram.testDynagramShiftMin+i*dShift;
                        lufkinCardForce.pair[j].Load=dynaGram.testDynagramLoadMax;    
                        j++;
                      }
                    }                    
                    ----------------------------------------------------------*/                    
                    for(i=0;i<DYNA_POINTS;i++)		
                    {      
                      dynaPlot.pair[i].Position=lufkinCardForce.pair[i].Position;
                      dynaPlot.pair[i].Load=lufkinCardForce.pair[i].Load;
                    }
                    copymasNSwap(src,(uint8_t*)&dynaPlot+tempUINT16,regNum);                    
                    for(i=0;i<regNum;i++)
                    {
                      dst[dataShift+i]=src[i];
                    }                                        
                    crc16=modBusCRC16(dst,regNum+dataShift);      
                    dst[regNum+dataShift]=(uint8_t)(crc16>>8);
                    dst[regNum+dataShift+1]=(uint8_t)(crc16&0xFF);                     
                    *bytesToTransmit+=regNum+dataShift+2;                                        
                  }else errorCode=2;
                break;                
                case 6:
  		  startAddr=src[1]*256+src[2];
	          regNum=1;                  
                  processedRegs+=1;                    
                  if(slaveAddr>247){
                    slaveAddr-=248;
                    dst[0]=(uint8_t)(0xF8|(slaveAddr>>8));
                    dst[1]=(uint8_t)(slaveAddr&0xFF);
                    slaveAddr+=248;
                    dst[2]=src[0];
                    dst[3]=src[1];
                    dst[4]=src[2];
                    dst[5]=src[3];
                    dst[6]=src[4];
                    dst[7]=src[5];
                    dst[8]=src[6];         
                    if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров 
                    dataShift=9;    // ответ-эхо                  
                  }else{                      
                    dst[0]=slaveAddr;
                    dst[1]=src[0];
                    dst[2]=src[1];
                    dst[3]=src[2];
                    dst[4]=src[3];
                    dst[5]=src[4];
                    dst[6]=src[5];
                    dst[7]=src[6];
                    if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров, для Рината увеличил с 125 до 2500, буферы позволяют
                    dataShift=8;   // ответ-эхо
                  }    
                  *bytesToTransmit+=dataShift; 
                  
                  if((startAddr>=RTC_MODBUS_STARTREG)&&(startAddr<=RTC_MODBUS_ENDREG))
                  { 
                    src[0]=0;
                    src[1]=RTC_TimeStructure.Seconds;
                    src[2]=0;
                    src[3]=RTC_TimeStructure.Minutes;
                    src[4]=0;
                    src[5]=RTC_TimeStructure.Hours;
                    src[6]=0;
                    src[7]=RTC_DateStructure.Date;
                    src[8]=0;
                    src[9]=RTC_DateStructure.Month;
                    src[10]=0;
                    src[11]=RTC_DateStructure.Year;
                    tempUINT32=toUnix(RTC_DateStructure,RTC_TimeStructure);
                    src[14]=(uint8_t)((tempUINT32>>24)&0xFF);
                    src[15]=(uint8_t)((tempUINT32>>16)&0xFF);
                    src[12]=(uint8_t)((tempUINT32>>8)&0xFF);
                    src[13]=(uint8_t)(tempUINT32&0xFF);                          
                    if(slaveAddr>247)
                    {
                      src[(startAddr-RTC_MODBUS_STARTREG)*2]=dst[5];
                      src[(startAddr-RTC_MODBUS_STARTREG)*2+1]=dst[6];   
                    }else{
                      src[(startAddr-RTC_MODBUS_STARTREG)*2]=dst[4];
                      src[(startAddr-RTC_MODBUS_STARTREG)*2+1]=dst[5];                        
                    }
                    if(src[1]<60)RTC_TimeStructure.Seconds=src[1];
                    if(src[3]<60)RTC_TimeStructure.Minutes=src[3];
                    if(src[5]<24)RTC_TimeStructure.Hours=src[5];
                    if((src[7]>=1)&&(src[7]<32))RTC_DateStructure.Date=src[7];
                    if((src[9]>=1)&&(src[9]<13))RTC_DateStructure.Month=src[9];
                    if(src[11]<100)RTC_DateStructure.Year=src[11];                                                 
                    if((startAddr-RTC_MODBUS_STARTREG)>5)
                    {
                      tempUINT32=((uint32_t)src[14])<<24;
                      tempUINT32|=((uint32_t)src[15])<<16;                        
                      tempUINT32|=((uint32_t)src[12])<<8;
                      tempUINT32|=((uint32_t)src[13]);
                      fromUnix(&RTC_DateStructure,&RTC_TimeStructure,tempUINT32);
                    }           
                    HAL_RTC_SetTime(&hrtc,&RTC_TimeStructure,RTC_FORMAT_BIN);
                    HAL_RTC_SetDate(&hrtc,&RTC_DateStructure,RTC_FORMAT_BIN);
                    RTC_UnixTime.timeStamp=toUnix(RTC_DateStructure,RTC_TimeStructure);
                    timeStampsUpdate(); // Обновление временных меток архивов
                  }else if((startAddr>=ETR_ENGINE_MODBUS_STARTREG)&&(startAddr<=ETR_ENGINE_MODBUS_ENDREG))
                  { 
                    tempUINT16=(startAddr-ETR_ENGINE_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&etrEngineShadow+tempUINT16,src+3,2);       
                    etrEngineShadow.updateFromShadow=1;                    
                  }else if((startAddr>=ALARMS_MODBUS_STARTREG)&&(startAddr<=ALARMS_MODBUS_ENDREG))                  
                  {
                    tempUINT16=(startAddr-ALARMS_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&alarmsShadow+tempUINT16,src+3,2);       
                    alarmsShadow.updateFromShadow=1;                     
                  }else if((startAddr>=COMMAND_MODBUS_STARTREG)&&(startAddr<=COMMAND_MODBUS_ENDREG))			
		  { 
                    tempUINT16=(startAddr-COMMAND_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&commandShadow+tempUINT16,src+3,2);                 
                    commandShadow.updateFromShadow=1;                                                                                       
                  }else if((startAddr>=ADC_MODBUS_STARTREG)&&(startAddr<=ADC_MODBUS_ENDREG))			
		  {  
                    tempUINT16=(startAddr-ADC_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&adcInputsShadow+tempUINT16,src+3,2);                 
                    adcInputsShadow.updateFromShadow=1;                                                                                       
                  }else if((startAddr>=DAC_MODBUS_STARTREG)&&(startAddr<=DAC_MODBUS_ENDREG))			
		  {         
                    tempUINT16=(startAddr-DAC_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&dacOutputsShadow+tempUINT16,src+3,2);                 
                    dacOutputsShadow.updateFromShadow=1;                                                                   
                    *bytesToTransmit+=dataShift;
                  }else if((startAddr>=TC_MODBUS_STARTREG)&&(startAddr<=TC_MODBUS_ENDREG))			
		  {         
                    tempUINT16=(startAddr-TC_MODBUS_STARTREG)*2;  
                    for(i=0;i<15;i++)TCInputs.modBusBuffer[i]=TCInputs.transitionsNum[i];
                    copymasNSwap((uint8_t*)&TCInputs.modBusBuffer+tempUINT16,src+3,2);                 
                    TCInputs.modBusWrite=1;                    
                  }else if((startAddr>=ENGINE_MODBUS_STARTREG)&&(startAddr<=ENGINE_MODBUS_ENDREG))
                  {                      
                    tempUINT16=(startAddr-ENGINE_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&engineShadow+tempUINT16,src+3,2);       
                    engineShadow.updateFromShadow=1;                    
                  }else if((startAddr>=CONTEXT_MODBUS_STARTREG)&&(startAddr<=CONTEXT_MODBUS_ENDREG))
                  {                      
                    tempUINT16=(startAddr-CONTEXT_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&contextShadow+tempUINT16,src+3,2);       
                    contextShadow.updateFromShadow=1;                     
                  }else if((startAddr>=ARCHIVE_SETTINGS_MODBUS_STARTREG)&&(startAddr<=ARCHIVE_SETTINGS_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-ARCHIVE_SETTINGS_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&tapsConfig+tempUINT16,src+3,2);   
                    for(i=0;i<16;i++)
                    {
                      if(tapsConfig.source[i]>15)tapsConfig.source[i]=15;                      
                    }                    
                    tapsConfig.CRC16=modBusCRC16((uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_MODBUS_REGNUM*2);
                    framWrite(ARCHIVE_SETTINGS_FRAM_ADDR,(uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_FRAM_LENGTH);                    
                    timeStampsUpdate();
                  }else if((startAddr>=EVENTS_COUNTER_MODBUS_STARTREG)&&(startAddr<=EVENTS_COUNTER_MODBUS_ENDREG))
                  {                      
                    tempUINT16=(startAddr-EVENTS_COUNTER_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&eventsCnt+tempUINT16,src+3,2);       
                    eventsCnt.refresh=1;                    
                  }else if((startAddr>=ACTIVE_ENERGY_MODBUS_STARTREG)&&(startAddr<=ACTIVE_ENERGY_MODBUS_STARTREG))
                  {                    
                     tempUINT16=src[3]+src[4];
                     if(tempUINT16==(RTC_TimeStructure.Hours+RTC_TimeStructure.Minutes)) // Читерное условие сброса счетчика активной электроэнергии
                     {
                       activePower.resetInfiniteSum=1;
                     }                       
                  }else if((startAddr>=DYNAGRAM_SETTINGS_MODBUS_STARTREG)&&(startAddr<=DYNAGRAM_SETTINGS_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-DYNAGRAM_SETTINGS_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&dynaGram+tempUINT16,src+3,2);
                    if(dynaGram.algorithmFreqChange<0.0f)dynaGram.algorithmFreqChange=-1.0f*dynaGram.algorithmFreqChange;
                    if(dynaGram.algorithmKp<0.0f)dynaGram.algorithmKp=-1.0f*dynaGram.algorithmKp;
                    if(dynaGram.algorithmFstart<dynaGram.algorithmFmin)dynaGram.algorithmFstart=dynaGram.algorithmFmin;
                    if(dynaGram.algorithmFstart>dynaGram.algorithmFmax)dynaGram.algorithmFstart=dynaGram.algorithmFmax;
                    dynaGram.CRC16=modBusCRC16((uint8_t*)&dynaGram,DYNAGRAM_SETTINGS_MODBUS_REGNUM*2);     
                    framWrite(DYNAGRAM_SETTINGS_FRAM_ADDR,(uint8_t*)&dynaGram,DYNAGRAM_SETTINGS_FRAM_LENGTH);                          
                    //--------- Аварии по датчику нагрузки ---------------------------------------
                    if(dynaGram.tit3ToForceCoeff>0.0f)
                    {
                      dynaGram.loadMinAdcCode=(int16_t)((dynaGram.loadMin-dynaGram.tit3ToForceConst)/dynaGram.tit3ToForceCoeff); 
                      if(dynaGram.loadMinAdcCode<0)dynaGram.loadMinAdcCode=0;
                      dynaGram.loadMaxAdcCode=(int16_t)((dynaGram.loadMax-dynaGram.tit3ToForceConst)/dynaGram.tit3ToForceCoeff); 
                    }
                    dynaGram.loadMinMaxHoldCnt=0;    
                    dynaGram.loadMinMaxHoldMStoCycles=(engine.freqPWM*dynaGram.loadMaxMinHoldMS)/1000;
                    //----------------------------------------------------------------------------                    
                  }else if((startAddr>=CHREP_MAX_INST_CUR_MODBUS_STARTREG)&&(startAddr<=CHREP_MAX_INST_CUR_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-CHREP_MAX_INST_CUR_MODBUS_STARTREG)*2; 
                    if(startAddr==CHREP_MAX_INST_CUR_MODBUS_STARTREG) // Вводим пин-код
                    {
                      copymasNSwap((uint8_t*)&chrepShadow+tempUINT16,src+3,2);
                      if(chrepShadow.pinCode==1162)chrep.writeAccess=60;  
                    }else{
                      if(chrep.writeAccess) // Изменение максимального тока разрешено
                      {
                        copymasNSwap((uint8_t*)&chrepShadow+tempUINT16,src+3,2);
                        switch(chrepShadow.maxInstCurrent)
                        {
                          case 75: // 15 кВт
                          case 85: // 18.5 кВт
                          case 100:// 22 кВт 
                            chrep.maxInstCurrent=chrepShadow.maxInstCurrent*100;
                          break;
                          default:
                            chrep.maxInstCurrent=7500;  
                        }
                        chrep.CRC16=modBusCRC16((uint8_t*)&chrep.maxInstCurrent,2);
                        framWrite(CHREP_MAX_INST_CUR_FRAM_ADDR,(uint8_t*)&chrep.maxInstCurrent,CHREP_MAX_INST_CUR_FRAM_LENGTH);                                                 
                      }                    
                    }                   
                  }else if((startAddr>=SIM800_SETTINGS_MODBUS_STARTREG)&&(startAddr<=SIM800_SETTINGS_MODBUS_ENDREG))
                  {           
                    sim800StructInit();
                    tempUINT16=(startAddr-SIM800_SETTINGS_MODBUS_STARTREG)*2;                                          
                    copymasNSwap((uint8_t*)&sim800Settings+tempUINT16,src+3,2);    
                    sim800Settings.a_c_gprs[57]=0;
                    sim800StructToFlash();
                  }else errorCode=2;    
                  
                  if((startAddr>=MODBUS_ADDR_STARTREG)&&(startAddr<=MODBUS_ADDR_STARTREG))
                  {
                   // if(*(src-1)==0)
                      if( (*(src-1)==0)  || (zapr_from_sim800==1) )
                    {
                      modbusAddr=src[3]*256+src[4];     
                      framWrite(MODBUS_ADDR_FRAM_ADDR,(uint8_t*)&modbusAddr,MODBUS_ADDR_FRAM_LENGTH);
                      errorCode=0; 
                    }
                  }
                break;
                case 16:
  		  startAddr=src[1]*256+src[2];
	          regNum=src[3]*256+src[4];
		  endAddr=startAddr+regNum-1;		
                  if(!regNum){errorCode=2;break;} // Игнорировать запрос нулевого количества регистров
                  processedRegs+=regNum;                        
                  regNum=regNum*2;                  
                  if(slaveAddr>247){
                    slaveAddr-=248;
                    dst[0]=(uint8_t)(0xF8|(slaveAddr>>8));
                    dst[1]=(uint8_t)(slaveAddr&0xFF);
                    slaveAddr+=248;
                    dst[2]=src[0];
                    dst[3]=src[1];
                    dst[4]=src[2];                      
                    dst[5]=src[3];
                    dst[6]=src[4];                                           
                    dataShift=7;            
                    if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров                      
                  }else{                      
                    dst[0]=slaveAddr;
                    dst[1]=src[0];
                    dst[2]=src[1];
                    dst[3]=src[2];                      
                    dst[4]=src[3];
                    dst[5]=src[4];                                            
                    dataShift=6;
                    if(processedRegs>2500){errorCode=2;break;} // Ограничение на запрос 2500 регистров, для Рината увеличил с 125 до 2500, буферы позволяют
                  }
                  crc16=modBusCRC16(dst,dataShift);      
                  dst[dataShift]=(uint8_t)(crc16>>8);
                  dst[dataShift+1]=(uint8_t)(crc16&0xFF);                     
                  *bytesToTransmit+=dataShift+2;                                      
                  
                  if((startAddr>=RTC_MODBUS_STARTREG)&&(endAddr<=RTC_MODBUS_ENDREG))
                  { 
                    dst[8]=0;
                    dst[9]=RTC_TimeStructure.Seconds;
                    dst[10]=0;
                    dst[11]=RTC_TimeStructure.Minutes;
                    dst[12]=0;
                    dst[13]=RTC_TimeStructure.Hours;
                    dst[14]=0;
                    dst[15]=RTC_DateStructure.Date;
                    dst[16]=0;
                    dst[17]=RTC_DateStructure.Month;
                    dst[18]=0;
                    dst[19]=RTC_DateStructure.Year;
                    tempUINT32=toUnix(RTC_DateStructure,RTC_TimeStructure);
                    dst[22]=(uint8_t)((tempUINT32>>24)&0xFF);
                    dst[23]=(uint8_t)((tempUINT32>>16)&0xFF);
                    dst[20]=(uint8_t)((tempUINT32>>8)&0xFF);
                    dst[21]=(uint8_t)(tempUINT32&0xFF);                                     
                    tempUINT16=(startAddr-RTC_MODBUS_STARTREG)*2;
                    for(i=0;i<regNum;i++)
                    {
                      dst[8+tempUINT16+i]=src[i+6];
                    }                        
                    if(dst[9]<60)RTC_TimeStructure.Seconds=dst[9];
                    if(dst[11]<60)RTC_TimeStructure.Minutes=dst[11];
                    if(dst[13]<24)RTC_TimeStructure.Hours=dst[13];
                    if((dst[15]>=1)&&(dst[15]<32))RTC_DateStructure.Date=dst[15];
                    if((dst[17]>=1)&&(dst[17]<13))RTC_DateStructure.Month=dst[17];
                    if(dst[19]<100)RTC_DateStructure.Year=dst[19];  
                    if((startAddr-RTC_MODBUS_STARTREG)>5)
                    {                      
                      tempUINT32=((uint32_t)dst[22])<<24;
                      tempUINT32|=((uint32_t)dst[23])<<16;                        
                      tempUINT32|=((uint32_t)dst[20])<<8;
                      tempUINT32|=((uint32_t)dst[21]);                                                   
                      fromUnix(&RTC_DateStructure,&RTC_TimeStructure,tempUINT32);
                    }
                    HAL_RTC_SetTime(&hrtc,&RTC_TimeStructure,RTC_FORMAT_BIN);
                    HAL_RTC_SetDate(&hrtc,&RTC_DateStructure,RTC_FORMAT_BIN); 
                    RTC_UnixTime.timeStamp=toUnix(RTC_DateStructure,RTC_TimeStructure);
                    timeStampsUpdate(); // Обновление временных меток архивов
                  }else if((startAddr>=ETR_ENGINE_MODBUS_STARTREG)&&(endAddr<=ETR_ENGINE_MODBUS_ENDREG))			
		  {                    
                    tempUINT16=(startAddr-ETR_ENGINE_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&etrEngineShadow+tempUINT16,src+6,regNum);                      
                    etrEngineShadow.updateFromShadow=1;                     
                  }else if((startAddr>=ALARMS_MODBUS_STARTREG)&&(endAddr<=ALARMS_MODBUS_ENDREG))			
		  {                    
                    tempUINT16=(startAddr-ALARMS_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&alarmsShadow+tempUINT16,src+6,regNum);                      
                    alarmsShadow.updateFromShadow=1;                     
                  }else if((startAddr>=COMMAND_MODBUS_STARTREG)&&(endAddr<=COMMAND_MODBUS_ENDREG))			
		  {
                    tempUINT16=(startAddr-COMMAND_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&commandShadow+tempUINT16,src+6,regNum);                      
                    commandShadow.updateFromShadow=1;                                 
                  }else if((startAddr>=ADC_MODBUS_STARTREG)&&(endAddr<=ADC_MODBUS_ENDREG))			
		  { 
                    tempUINT16=(startAddr-ADC_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&adcInputsShadow+tempUINT16,src+6,regNum);                      
                    adcInputsShadow.updateFromShadow=1;                                 
                  }else if((startAddr>=DAC_MODBUS_STARTREG)&&(endAddr<=DAC_MODBUS_ENDREG))			
		  {      
                    tempUINT16=(startAddr-DAC_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&dacOutputsShadow+tempUINT16,src+6,regNum);                      
                    dacOutputsShadow.updateFromShadow=1;                                 
                  }else if((startAddr>=TC_MODBUS_STARTREG)&&(endAddr<=TC_MODBUS_ENDREG))
                  {                  
                    tempUINT16=(startAddr-TC_MODBUS_STARTREG)*2;  
                    if(TCInputs.modBusWrite){;} //Для Рината, чтобы можно было подряд в цикле редактировать данные регистры                    
                    else{
                      for(i=0;i<15;i++)TCInputs.modBusBuffer[i]=TCInputs.transitionsNum[i];
                      TCInputs.modBusWrite=1;
                    }
                    copymasNSwap((uint8_t*)&TCInputs.modBusBuffer+tempUINT16,src+6,regNum);                                     
                    *bytesToTransmit+=dataShift;  
                  }else if((startAddr>=ENGINE_MODBUS_STARTREG)&&(endAddr<=ENGINE_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-ENGINE_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&engineShadow+tempUINT16,src+6,regNum);                      
                    engineShadow.updateFromShadow=1; 
                  }else if((startAddr>=CONTEXT_MODBUS_STARTREG)&&(endAddr<=CONTEXT_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-CONTEXT_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&contextShadow+tempUINT16,src+6,regNum);        
                    contextShadow.updateFromShadow=1; // Обновить структуру context  
                  }else if((startAddr>=ARCHIVE_SETTINGS_MODBUS_STARTREG)&&(endAddr<=ARCHIVE_SETTINGS_MODBUS_ENDREG))  
                  {
                    tempUINT16=(startAddr-ARCHIVE_SETTINGS_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&tapsConfig+tempUINT16,src+6,regNum);                            
                    for(i=0;i<16;i++)
                    {
                      if(tapsConfig.source[i]>15)tapsConfig.source[i]=15;
                    }                   
                    tapsConfig.CRC16=modBusCRC16((uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_MODBUS_REGNUM*2); 
                    framWrite(ARCHIVE_SETTINGS_FRAM_ADDR,(uint8_t*)&tapsConfig,ARCHIVE_SETTINGS_FRAM_LENGTH);
                    timeStampsUpdate();
                  }else if((startAddr>=EVENTS_COUNTER_MODBUS_STARTREG)&&(endAddr<=EVENTS_COUNTER_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-EVENTS_COUNTER_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&eventsCnt+tempUINT16,src+6,regNum);        
                    eventsCnt.refresh=1;
                  }else if((startAddr>=DYNAGRAM_SETTINGS_MODBUS_STARTREG)&&(endAddr<=DYNAGRAM_SETTINGS_MODBUS_ENDREG))
                  {
                    tempUINT16=(startAddr-DYNAGRAM_SETTINGS_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&dynaGram+tempUINT16,src+6,regNum);  
                    if(dynaGram.algorithmFreqChange<0.0f)dynaGram.algorithmFreqChange=-1.0f*dynaGram.algorithmFreqChange;
                    if(dynaGram.algorithmKp<0.0f)dynaGram.algorithmKp=-1.0f*dynaGram.algorithmKp;
                    if(dynaGram.algorithmFstart<dynaGram.algorithmFmin)dynaGram.algorithmFstart=dynaGram.algorithmFmin;
                    if(dynaGram.algorithmFstart>dynaGram.algorithmFmax)dynaGram.algorithmFstart=dynaGram.algorithmFmax;                    
                    dynaGram.CRC16=modBusCRC16((uint8_t*)&dynaGram,DYNAGRAM_SETTINGS_MODBUS_REGNUM*2);     
                    framWrite(DYNAGRAM_SETTINGS_FRAM_ADDR,(uint8_t*)&dynaGram,DYNAGRAM_SETTINGS_FRAM_LENGTH);  
                    //--------- Аварии по датчику нагрузки ---------------------------------------
                    if(dynaGram.tit3ToForceCoeff>0.0f)
                    {
                      dynaGram.loadMinAdcCode=(int16_t)((dynaGram.loadMin-dynaGram.tit3ToForceConst)/dynaGram.tit3ToForceCoeff);  
                      if(dynaGram.loadMinAdcCode<0)dynaGram.loadMinAdcCode=0;
                      dynaGram.loadMaxAdcCode=(int16_t)((dynaGram.loadMax-dynaGram.tit3ToForceConst)/dynaGram.tit3ToForceCoeff);  
                    }
                    dynaGram.loadMinMaxHoldCnt=0;    
                    dynaGram.loadMinMaxHoldMStoCycles=(engine.freqPWM*dynaGram.loadMaxMinHoldMS)/1000;
                    dynaGram.minPeriodOfDynagramCycles=(engine.freqPWM*dynaGram.minPeriodOfDynagramMS)/1000;
                    //----------------------------------------------------------------------------                    
                  }else if((startAddr>=CHREP_MAX_INST_CUR_MODBUS_STARTREG)&&(endAddr<=CHREP_MAX_INST_CUR_MODBUS_ENDREG)) 
                  {
                    chrepShadow.pinCode=0;
                    chrepShadow.maxInstCurrent=0;
                    tempUINT16=(startAddr-CHREP_MAX_INST_CUR_MODBUS_STARTREG)*2;  
                    copymasNSwap((uint8_t*)&chrepShadow+tempUINT16,src+6,regNum); 
                    if(chrepShadow.pinCode==1162)chrep.writeAccess=60; 
                    if((chrep.writeAccess)&&(chrepShadow.maxInstCurrent))
                    {
                        switch(chrepShadow.maxInstCurrent)
                        {
                          case 75:  // 15 кВт
                          case 85:  // 18.5 кВт
                          case 100: // 22 кВт 
                            chrep.maxInstCurrent=chrepShadow.maxInstCurrent*100;
                          break;
                          default:
                            chrep.maxInstCurrent=7500;  
                        }
                        chrep.CRC16=modBusCRC16((uint8_t*)&chrep.maxInstCurrent,2);
                        framWrite(CHREP_MAX_INST_CUR_FRAM_ADDR,(uint8_t*)&chrep.maxInstCurrent,CHREP_MAX_INST_CUR_FRAM_LENGTH);                     
                    }                    
                  }else if((startAddr>=SIM800_SETTINGS_MODBUS_STARTREG)&&(endAddr<=SIM800_SETTINGS_MODBUS_ENDREG))
                  {           
                    sim800StructInit();
                    tempUINT16=(startAddr-SIM800_SETTINGS_MODBUS_STARTREG)*2;                                          
                    copymasNSwap((uint8_t*)&sim800Settings+tempUINT16,src+6,regNum);    
                    sim800Settings.a_c_gprs[57]=0;
                    sim800StructToFlash();
                  }else errorCode=2;    
                  
                  if((startAddr>=MODBUS_ADDR_STARTREG)&&(endAddr<=MODBUS_ADDR_STARTREG))
                  {
                    //if(*(src-1)==0)
                      if( (*(src-1)==0)  || (zapr_from_sim800==1) )
                    {
                      modbusAddr=src[6]*256+src[7];     
                      framWrite(MODBUS_ADDR_FRAM_ADDR,(uint8_t*)&modbusAddr,MODBUS_ADDR_FRAM_LENGTH);
                      errorCode=0; 
                    }
                  }                 
                break;                              
               default: errorCode=1; // Неподдерживаемая функция
	}
        
        if(errorCode)// Обнаружена ошибка в запросе          
        {
          if(slaveAddr>247){  
            slaveAddr-=248;
            dst[0]=(uint8_t)(0xF8|(slaveAddr>>8));
            dst[1]=(uint8_t)(slaveAddr&0xFF);
            slaveAddr+=248;            
            dst[2]=src[0]|0x80;
            dst[3]=errorCode;
            crc16=modBusCRC16(dst,4);      
            dst[4]=(uint8_t)(crc16>>8);
            dst[5]=(uint8_t)(crc16&0xFF);                     
            *bytesToTransmit+=6;             
          }else{
            dst[0]=slaveAddr;
            dst[1]=src[0]|0x80;
            dst[2]=errorCode;
            crc16=modBusCRC16(dst,3);      
            dst[3]=(uint8_t)(crc16>>8);
            dst[4]=(uint8_t)(crc16&0xFF);                     
            *bytesToTransmit+=5;        
          }         
        }
     
     
    if(eventsCnt.refresh)
    {
      eventsCnt.CRC16=modBusCRC16((uint8_t*)&eventsCnt,EVENTS_COUNTER_MODBUS_REGNUM*2);
      Delay_us(eventsCnt.iwdgDelayMS*1000);
      framWrite(EVENTS_COUNTER_FRAM_ADDR,(uint8_t*)&eventsCnt,EVENTS_COUNTER_FRAM_LENGTH);      
      eventsCnt.refresh=0;
    }
    if(engineShadow.updateFromShadow==1) // Пересчитать все зависимые настройки, чтобы можно было менять все параметры на лету
    {            
      for(i=0;i<CONTEXT_NUM;i++)
      {         
        tempFloat=engineShadow.ufTable[i].freq-engineShadow.minFreq;
        if(tempFloat>0.0f)engineShadow.k1[i]=(engineShadow.ufTable[i].voltagePercent[1]-engineShadow.ufTable[i].voltagePercent[0])*3.8f/tempFloat;
        else{engineShadow.k1[i]=0;engineShadow.ufTable[i].freq=engineShadow.minFreq;}
        engineShadow.C1[i]=-engineShadow.k1[i]*engineShadow.minFreq+engineShadow.ufTable[i].voltagePercent[0]*3.8f;  
        
        tempFloat=engineShadow.maxFreq-engineShadow.ufTable[i].freq;
        if(tempFloat>0.0f)engineShadow.k2[i]=(engineShadow.ufTable[i].voltagePercent[2]-engineShadow.ufTable[i].voltagePercent[1])*3.8f/tempFloat;
        else{engineShadow.k2[i]=0;engineShadow.ufTable[i].freq=engineShadow.maxFreq;}
        engineShadow.C2[i]=-engineShadow.k2[i]*engineShadow.ufTable[i].freq+engineShadow.ufTable[i].voltagePercent[1]*3.8f;                      
      } 
      
      if(vectorPWM.isOn) // Не менять частоту ШИМ на работающем двигателе
      {
        engineShadow.freqPWM=engine.freqPWM;
      }
      if((engineShadow.freqPWM<2000)||(engineShadow.freqPWM>12000))engineShadow.freqPWM=10000;
      switch(engineShadow.freqPWM)
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
          engineShadow.freqPWM=10000;
          udc_1kHz_decimator_max=9;
      }          
      engineShadow.CRC16=modBusCRC16((uint8_t*)&engineShadow,ENGINE_MODBUS_REGNUM*2);
      framWrite(ENGINE_FRAM_ADDR,(uint8_t*)&engineShadow,ENGINE_FRAM_LENGTH);      
      dynaGram.loadMinMaxHoldCnt=0;    
      dynaGram.loadMinMaxHoldMStoCycles=(engineShadow.freqPWM*dynaGram.loadMaxMinHoldMS)/1000;      
      engineShadow.updateFromShadow=2;
      etrEngineShadow.updateFromShadow=1;  // Структура engine влияет на etrEngine   
      alarmsShadow.updateFromShadow=1;     // Структура engine влияет на alarm
      adcInputs.updateFromShadow=1;        // Структура engine влияет на adcInputs частотой ШИМ            
    }
    
    if(contextShadow.updateFromShadow==1)
    {      
      for(i=0;i<CONTEXT_NUM;i++)        
      {        
        if(contextShadow.dfdtUP[i]<0.1f)contextShadow.dfdtUP[i]=0.1f;
        if(contextShadow.dfdtDOWN[i]<0.1f)contextShadow.dfdtDOWN[i]=0.1f;
        if(contextShadow.curveStimeUP[i]<0.0f)contextShadow.curveStimeUP[i]=0.0f;
        if(contextShadow.curveStimeDOWN[i]<0.0f)contextShadow.curveStimeDOWN[i]=0.0f;
        if(contextShadow.ufTableNum[i]>=CONTEXT_NUM)contextShadow.ufTableNum[i]=CONTEXT_NUM-1;      
        if(contextShadow.freqSource[i]>2)contextShadow.freqSource[i]=2;
      }           
      if(contextShadow.contextSource>1)contextShadow.contextSource=1;        
      if(contextShadow.DI0_0_contextNum>=CONTEXT_NUM)contextShadow.DI0_0_contextNum=CONTEXT_NUM-1;
      if(contextShadow.DI0_1_contextNum>=CONTEXT_NUM)contextShadow.DI0_1_contextNum=CONTEXT_NUM-1;
      if(contextShadow.modbus_contextNum>=CONTEXT_NUM)contextShadow.modbus_contextNum=CONTEXT_NUM-1;
      
      contextShadow.CRC16=modBusCRC16((uint8_t*)&contextShadow,CONTEXT_MODBUS_REGNUM*2);
      framWrite(CONTEXT_FRAM_ADDR,(uint8_t*)&contextShadow,CONTEXT_FRAM_LENGTH);
      contextShadow.updateFromShadow=2;      
    }    
    
    if(commandShadow.updateFromShadow==1)
    {      
      if(vectorPWM.isOn) // ШИМ работает, двигатель вращается      
      {                      
        commandShadow.rotationDIR=command.rotationDIR;            // Нельзя менять направление на работающем двигателе        
      }      
      if(commandShadow.delayBeforeRestartSec<10)commandShadow.delayBeforeRestartSec=10;
      if(commandShadow.delayBeforeRestartSec>60)commandShadow.delayBeforeRestartSec=60;
      
      if(commandShadow.delayBeforeStartKPSec<1)commandShadow.delayBeforeStartKPSec=1;
      if(commandShadow.delayBeforeStartKPSec>20)commandShadow.delayBeforeStartKPSec=20;
      if(commandShadow.workContextNum>10)commandShadow.workContextNum=10;
      commandShadow.CRC16=modBusCRC16((uint8_t*)&commandShadow,COMMAND_MODBUS_REGNUM*2);      
      framWrite(COMMAND_FRAM_ADDR,(uint8_t*)&commandShadow,COMMAND_FRAM_LENGTH);       
      commandShadow.updateFromShadow=2;                     
    }

    if(etrEngineShadow.updateFromShadow==1)
    {      
      if(etrEngineShadow.overloadPercent>=200)etrEngineShadow.overloadPercent=200.0;              
      etrEngineShadow.overloadCurrent=engineShadow.Inom*((float)etrEngineShadow.overloadPercent)/100.0f;
      etrEngineShadow.overloadLimit=etrEngineShadow.overloadCurrent*etrEngineShadow.overloadCurrent*((float)etrEngineShadow.overloadSec);
      
      tempFloat=etrEngineShadow.freqCorrPoints[0]-engineShadow.minFreq;
      if(tempFloat>0.0f)etrEngineShadow.k1=(etrEngineShadow.currentCorrPoints[1]-etrEngineShadow.currentCorrPoints[0])/(100.0f*tempFloat);
      else etrEngineShadow.k1=0;
      etrEngineShadow.C1=-etrEngineShadow.k1*engineShadow.minFreq+etrEngineShadow.currentCorrPoints[0]/100.0f;  
      
      tempFloat=etrEngineShadow.freqCorrPoints[1]-etrEngineShadow.freqCorrPoints[0];
      if(tempFloat>0.0f)etrEngineShadow.k2=(etrEngineShadow.currentCorrPoints[2]-etrEngineShadow.currentCorrPoints[1])/(100.0f*tempFloat);
      else etrEngineShadow.k2=0;
      etrEngineShadow.C2=-etrEngineShadow.k2*etrEngineShadow.freqCorrPoints[0]+etrEngineShadow.currentCorrPoints[1]/100.0f;  
      
      tempFloat=engineShadow.maxFreq-etrEngineShadow.freqCorrPoints[1];
      if(tempFloat>0.0f)etrEngineShadow.k3=(etrEngineShadow.currentCorrPoints[3]-etrEngineShadow.currentCorrPoints[2])/(100.0f*tempFloat);
      else etrEngineShadow.k3=0;
      etrEngineShadow.C3=-etrEngineShadow.k3*etrEngineShadow.freqCorrPoints[1]+etrEngineShadow.currentCorrPoints[2]/100.0f;              
      
      etrEngineShadow.CRC16=modBusCRC16((uint8_t*)&etrEngineShadow,ETR_ENGINE_MODBUS_REGNUM*2);
      framWrite(ETR_ENGINE_FRAM_ADDR,(uint8_t*)&etrEngineShadow,ETR_ENGINE_FRAM_LENGTH);
      etrEngineShadow.updateFromShadow=2;    
    }
    
    if(alarmsShadow.updateFromShadow==1)
    {      
      alarmsShadow.IrmsMax=(uint16_t)((float)alarmsShadow.IrmsMaxPercent*engineShadow.Inom);      
      alarmsShadow.curDisbalanceMax=(uint16_t)((float)alarmsShadow.curDisbalanceMaxPercent*engineShadow.Inom/100.0f);
      alarmsShadow.breakCurMin=(uint16_t)((float)alarmsShadow.breakCurMinPercent*engineShadow.Inom/100.0f);         
      alarmsShadow.CRC16=modBusCRC16((uint8_t*)&alarmsShadow,ALARMS_MODBUS_REGNUM*2);                
      framWrite(ALARMS_FRAM_ADDR,(uint8_t*)&alarmsShadow,ALARMS_FRAM_LENGTH);
      alarms.internalTemperatureMAX=alarmsShadow.internalTemperatureMAX;
      alarms.internalTemperatureNORM=alarmsShadow.internalTemperatureNORM;
      alarms.radiatorTemperatureMAX=alarmsShadow.radiatorTemperatureMAX;
      alarms.radiatorTemperatureNORM=alarmsShadow.radiatorTemperatureMAX;      
      alarms.fixFreqPhaseErrorMS=alarmsShadow.fixFreqPhaseErrorMS;
      alarms.fixFreqPhaseErrorValue=alarmsShadow.fixFreqPhaseErrorValue;  
      alarms.fixFreqEnable=alarmsShadow.fixFreqEnable;
      
      for(i=0;i<FAULT_ARRAY_ITEMS_NUM;i++)
      {      
        switch(i)
        {
          case NO_LOAD_FAULT:
            //faultsArrayNVRAM[i].upCnt=0;      
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.noLoadWattsResetCnt;
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.noLoadWattsRecoverCntLimit;          
          break;
          case LOW_VOLTAGE_FAULT:
            //faultsArrayNVRAM[i].upCnt=0;      
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.lowVoltageResetCnt;
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.lowVoltageRecoveryCntLimit;                    
          break;
          case TEMP_SENS_FAULT:
            //faultsArrayNVRAM[i].upCnt=0;      
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.temperatureResetCnt;
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.temperatureRecoveryCntLimit;          
          break;
          case BREAK_PHASE_CURRENT_FAULT:
            //faultsArrayNVRAM[i].upCnt=0; 
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.curDisbalanceMaxResetCnt; 
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.curDisbalanceMaxRecoverCntLimit;                    
          break;
          case ETR_OVERLOAD_FAULT:
            //faultsArrayNVRAM[i].upCnt=0;     
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.etrResetCnt;                    
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.etrRecoveryCntLimit;                    
          break;
          case UDC_OVERVOLTAGE_FAULT:
            //faultsArrayNVRAM[i].upCnt=0;  
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.highVoltageOffResetCnt;
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.highVoltageOffRecoverCntLimit;                    
          break;
          case BRIDGE_SATURATION_FAULT:
            //faultsArrayNVRAM[i].upCnt=0;      
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.IrmsMaxResetCnt;          
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.IrmsMaxRecoverCntLimit;                    
          break;  
          case DYNAMOMETER_FAULT:
            //faultsArrayNVRAM[i].upCnt=0;      
            faultsArrayNVRAM[i].recoverCntMin=alarmsShadow.dynagramLoadAlarmResetCnt;
            faultsArrayNVRAM[i].maxCnt=alarmsShadow.dynagramLoadAlarmRecoveryCntLimit;
          break;           
        }         
        faultsArrayNVRAM[i].CRC16=modBusCRC16((uint8_t*)&faultsArrayNVRAM[i],FAULT_ARRAY_ITEM_SIZE-2);                 
      }      
      alarmsShadow.updateFromShadow=2;        
    }

    if(adcInputsShadow.updateFromShadow==1)
    {       
      for(i=0;i<2;i++)
      {        
        adcInputsShadow.meanCntMax[i]=(uint32_t)(((float)adcInputsShadow.meanMS[i])*(float)engineShadow.freqPWM/1000.0f); 
        adcInputsShadow.maxCode[i]=(uint16_t)(adcInputs.calibr[i]*(float)adcInputsShadow.maxMV[i]);
        adcInputsShadow.minCode[i]=(uint16_t)(adcInputs.calibr[i]*(float)adcInputsShadow.minMV[i]);         
        adcInputsShadow.df[i]=((float)adcInputsShadow.oneHandrethDF[i])/100.0f;  
        adcInputsShadow.k[i]=((float)(adcInputsShadow.maxFreq[i]-adcInputsShadow.minFreq[i]))/((float)(adcInputsShadow.maxCode[i]-adcInputsShadow.minCode[i]));  
        adcInputsShadow.b[i]=(float)(adcInputsShadow.minFreq[i])-((float)adcInputsShadow.minCode[i])*adcInputsShadow.k[i];  
      }        
      adcInputsShadow.meanCntMax[2]=(uint32_t)(((float)adcInputsShadow.meanMS[2])*(float)engineShadow.freqPWM/1000.0f); 
      adcInputsShadow.meanCntMax[3]=(uint32_t)(((float)adcInputsShadow.meanMS[3])*(float)engineShadow.freqPWM/1000.0f);       
      adcInputsShadow.CRC16=modBusCRC16((uint8_t*)&adcInputsShadow,ADC_MODBUS_REGNUM*2);
      framWrite(ADC_FRAM_ADDR,(uint8_t*)&adcInputsShadow,ADC_FRAM_LENGTH);
      adcInputsShadow.updateFromShadow=2;        
    }
    
    if(dacOutputsShadow.updateFromShadow==1)
    {          
      for(i=0;i<2;i++)
      {   
        dacOutputsShadow.minCode[i]=(uint16_t)(dacOutputs.calibr[i]*(float)dacOutputsShadow.minMV[i]);
        dacOutputsShadow.maxCode[i]=(uint16_t)(dacOutputs.calibr[i]*(float)dacOutputsShadow.maxMV[i]);          
        dacOutputsShadow.k[i]=((float)(dacOutputsShadow.maxCode[i]-dacOutputsShadow.minCode[i]))/((float)(dacOutputsShadow.maxValue[i]-dacOutputsShadow.minValue[i]));  
        dacOutputsShadow.b[i]=(float)(dacOutputsShadow.minCode[i])-((float)dacOutputsShadow.minValue[i])*dacOutputsShadow.k[i];      
      }       
      dacOutputsShadow.CRC16=modBusCRC16((uint8_t*)&dacOutputsShadow,DAC_MODBUS_REGNUM*2);
      framWrite(DAC_FRAM_ADDR,(uint8_t*)&dacOutputsShadow,DAC_FRAM_LENGTH);
      dacOutputsShadow.updateFromShadow=2;        
    }       

    return errorCode;
}