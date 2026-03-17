#include "stm32f4xx_hal.h"

typedef struct{
//---- ModBus start--------  
  uint16_t source[16];       // Источник сигнала отвода:1-TC1, 2-TC2, ..., 15-TC15
  uint16_t intervalMin[16];  // Интервал замера
  uint16_t isOn;             // Архивирование включено 
  uint16_t dayRecordEnable;  // Включено формирование суммарных суточных архивов
  uint16_t dayRecordHour;    // Час формирования суммарной по отводам суточной записи
  uint16_t dayRecordMin;     // Минута -//-
  uint16_t dayRecordSec;     // Секунда -//-    
  int16_t recordShiftSec;   // Смещение времени формирования записи для целей верхнего уровня (каким часом закрывать запись...)
//---- ModBus end----------  // 
  uint16_t CRC16;            // Контрольная сумма для сохранения настроек во FRAM    
}TAPS_CONFIG_STRUCT;
extern volatile TAPS_CONFIG_STRUCT tapsConfig;
typedef struct{
//---- Сохранять во FRAM-память каждую минуту, отражает состояние текущей незакрытой записи архива----------------------    
  uint32_t timeStamp[16];       // Временная метка по которой необходимо формировать очередную запись  
  uint16_t recordMinCnt[16];    // Счетчик накопленных минут для формирования интервальных замеров    
  uint16_t dayRecordMinCnt[16]; // Счетчик накопленных минут для формирования суточной записи  
  uint32_t framMinCnt[16];      // Накапливаемый энергонезависимый, обновляемый раз в минуту счетчик импульсов по отводам 
  uint32_t dayFramMinCnt[16];   // Накапливаемый суточный энергонезависимый, обновляемый раз в минуту счетчик импульсов по отводам   
  uint32_t dayTimeStamp;        // Временная метка формирования суточной записи  
  uint16_t recordNum;           // Номер записи в контроллере, индекс для всего архива, можно только читать
  uint16_t CRC16;               // Контрольная сумма данных по отводам 
//----------------------------------------------------------------------------------------------------------------------
  volatile uint32_t curCnt[16];   // Текущее значение счетчика ТИИ    
  uint32_t prevCnt[16];  // Предыдущее значение счетчика ТИИ       
  uint8_t cntRequest;    // Флаг для атомарного запроса состояния счетчиков ТИИ  
}TAPS_UPDATE_COUNTERS;
extern volatile TAPS_UPDATE_COUNTERS tapsUpdateCnt;

typedef struct{  // MSB и LSB выделены для выравнивания полей структруры в памяти
  uint16_t timeStampLSB;   // Временная метка записи
  uint16_t timeStampMSB;   // Временная метка записи
  uint16_t intervalMin;    // Интервал замера, реальное количество минут, вошедшее в сумму
  uint16_t valueLSB;	   // Значение замера
  uint16_t valueMSB;	   // Значение замера
  uint16_t tapNum;         // Номер отвода, старший байт (байт №11, его младшие 4 бита отводятся под тип архива: 1-1мин,2-5мин,3-10мин,4-20мин,5-30мин,6-60мин,7-120мин,8-240мин,9-360мин,10-1440 мин(суточный) 
  uint16_t crcSum;         // Контрольная сумма 
}TAP_RECORD_STRUCT;

typedef union {
    uint8_t bytes[14];    
    TAP_RECORD_STRUCT fld;
}TAP_RECORD_UNION;
extern volatile TAP_RECORD_UNION tapRecord;

void tiiInit(void);
void tiiTask(void);
void timeStampsUpdate(void); 
void intervalTimeStampUpdate(void);
uint8_t recordType(uint16_t interval);