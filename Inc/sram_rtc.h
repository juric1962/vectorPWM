#include "modbus.h"
#include "stm32f4xx_hal.h"

//-------------------------------------- SRAM ----------------------------------
#define TIMESTAMP_SRAM_ADDR   0     // Стартовый адрес сохранения временной метки UNIX(since 2000), сохраняется каждый раз при 
#define TIMESTAMP_SRAM_LENGTH (4+2) // обновлении времни RTC_UNIX_TIME (4 байта) + (CRC16 MODBUS - 2 байта) 
//------------------------------------------------------------------------------
#define TCINPUTS_SRAM_ADDR     10   // Стартовый адрес счетчиков фронтов и спадов
#define TCINPUTS_SRAM_LENGTH   32   // Размер массива счетчиков фронтов и спадов
//-------------- Массив событий автоматического запуска ------------------------
#define FAULT_ARRAY_ADDR      100   // Стартовый адрес массива аварийных событий 
#define FAULT_ARRAY_ITEM_SIZE 12    // Размер записи аварийного события в байтах, sizeof(FAULT_EVENT_STRUCT)                                         
#define FAULT_ARRAY_LENGTH    (FAULT_ARRAY_ITEM_SIZE*FAULT_ARRAY_ITEMS_NUM)
//------------------------------------------------------------------------------
#define FIX_FREQ_ADDR    350  // Адрес структуры-состояния режима ограничения максимальной выходной частоты
#define FIX_FREQ_LENGTH  8    // Размер структуры-состояния режима ограничения максимальной выходной частоты в байтах
//-------------------- EVENT LOG -----------------------------------------------
#define EVENT_LOG_ADDR  498         // Стартовый адрес протокола событий, первые 4 байта - статусные, пишуться в первые 4 байта протокола событий
#define EVENT_LOG_RECORD_SIZE  14   // Размер записи в байтах
#define EVENT_LOG_RECORDS_NUM  200  // Количество записей
//-------------------- ACTIVE ENERGY ACCUMULATOR -------------------------------
#define ACTIVE_POWER_ADDR  4000     // Стартовый адрес счетчика активной мощности (байты 0-7: счетчик активной мощности, 8,9 - контрольная сумма)
//------------------------------------------------------------------------------
typedef struct{ 
  uint16_t unixTimeLSB;  
  uint16_t unixTimeMSB;    
  uint16_t reg[3];
  uint16_t code;
  uint16_t crcSum;
}EVENT_RECORD_STRUCT; // Структура записи в протокол событий
typedef union {
    uint8_t bytes[14];    
    EVENT_RECORD_STRUCT fld;
}EVENT_RECORD_UNION;
void eventLogInit(void);
void eventLogWrite(uint16_t eventCode);

typedef struct{
  uint32_t timeStamp;
  uint16_t CRC16;
}RTC_UNIX_TIME_STRUCT;

typedef struct{ 
//----- Modbus start  
  double *totalSum;          // Указатель на накапливаемый счетчик активной энергии, обновляется раз в секунду, доступ через модбас  
//----- Modbus end    
  uint16_t *totalSumCRC;  // Контрольная сумма счетчика *totalSum   
  uint32_t infiniteSum;     // Накапливаемый счетчик мгновенной мощности, обновляется раз в секунду, указатель на энергонезависимые (необходим для архивов)       
  uint32_t secondsSum;       // Секундный сумматор активной мощности, обновляется раз в секунду    
  uint32_t instantSum;       // Мгновенный сумматор активной мощности, вычисляемой на каждом такте  
  uint16_t sampNum;          // Количество элементов в сумме tempSum 
  uint8_t secondsSumRdy;     // Готовность вычисления секундной энергии
  uint8_t resetInfiniteSum;  // Сбросить бесконечную сумму
}ACTIVE_POWER_STRUCT;
extern volatile ACTIVE_POWER_STRUCT activePower;
void activeEnergyInit(void);
void activeEnergyTask(void);

extern RTC_TimeTypeDef  RTC_TimeStructure;
extern RTC_DateTypeDef RTC_DateStructure; 
extern volatile RTC_UNIX_TIME_STRUCT RTC_UnixTime;
extern RTC_HandleTypeDef hrtc; 

void systemTimeUpdate(void);
void rtcInit(void);
void getDataTimeTask(void);
uint32_t toUnix(RTC_DateTypeDef RTC_DateStructure, RTC_TimeTypeDef RTC_TimeStructure);
void fromUnix(RTC_DateTypeDef *RTC_DateStructure, RTC_TimeTypeDef *RTC_TimeStructure, uint32_t secsarg);

void bkpSRAMInit(void);
uint8_t bkpSRAM_Write(uint16_t addr,uint8_t *src,uint16_t len);
uint8_t bkpSRAM_Read(uint16_t addr,uint8_t *dst,uint16_t len);
