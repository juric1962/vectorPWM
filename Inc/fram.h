#include "stm32f4xx.h"
//-------------------------------------- FRAM ----------------------------------
#define MODBUS_ADDR_FRAM_ADDR    0  // Модбас адрес
#define MODBUS_ADDR_FRAM_LENGTH  2  // Длина модбас-адреса в байтах

#define ETR_ENGINE_FRAM_ADDR   10
#define ETR_ENGINE_FRAM_LENGTH (ETR_ENGINE_MODBUS_REGNUM*2+2)

#define ETR_ENGINE_FRAM_OVERLOAD_CNT_ADDR   50 
#define ETR_ENGINE_FRAM_OVERLOAD_CNT_LENGTH 4+2                         // Обновление счетчика etrEngine.overloadCnt 

#define ALARMS_FRAM_ADDR 60                                             // Стартовый адрес сохранения содержимого настроек аварий в ЭНОЗУ
#define ALARMS_FRAM_LENGTH (ALARMS_MODBUS_REGNUM*2+2)                   // Размер содержимого настроек аварий в ЭНОЗУ в байтах

#define COMMAND_FRAM_ADDR    140                                        // Стартовый адрес сохранения содержимого команд двигателя в ЭНОЗУ
#define COMMAND_FRAM_LENGTH  (COMMAND_MODBUS_REGNUM*2+2)                // Размер содержимого команд двигателя  в ЭНОЗУ в байтах

#define ADC_FRAM_ADDR    210//200                                            // Стартовый адрес сохранения содержимого настроек АЦП в ЭНОЗУ
#define ADC_FRAM_LENGTH  (ADC_MODBUS_REGNUM*2+2)                        // Размер содержимого настроек АЦП в ЭНОЗУ в байтах

#define DAC_FRAM_ADDR    250                                            // Стартовый адрес сохранения содержимого настроек ЦАП в ЭНОЗУ
#define DAC_FRAM_LENGTH  (DAC_MODBUS_REGNUM*2+2)                        // Размер содержимого настроек ЦАП в ЭНОЗУ в байтах

#define ENGINE_FRAM_ADDR    300                                         // Стартовый адрес сохранения содержимого настроек двигателя в ЭНОЗУ
#define ENGINE_FRAM_LENGTH  (ENGINE_MODBUS_REGNUM*2+2)                  // Размер содержимого настроек двигателя в ЭНОЗУ в байтах

#define CONTEXT_FRAM_ADDR    600                                        // Стартовый адрес сохранения содержимого контекста в ЭНОЗУ
#define CONTEXT_FRAM_LENGTH  (CONTEXT_MODBUS_REGNUM*2+2)                // Размер содержимого контекста в ЭНОЗУ в байтах

#define EVENTS_COUNTER_FRAM_ADDR   1000                                 // Стартовый адрес сохранения содержимого счетчиков событий в ЭНОЗУ
#define EVENTS_COUNTER_FRAM_LENGTH (EVENTS_COUNTER_MODBUS_REGNUM*2+2)   // Размер содержимого контекста в ЭНОЗУ в байтах

#define TAPS_UPDATE_CNT_FRAM_ADDR      1100     // По каждому отводу перед тем, как сформировать запись необходимо хранить, ежеминутно обновлять информацию 
#define TAPS_UPDATE_CNT_FRAM_LENGTH    (262+2)  // Размер счетчиков + индекс записи в архив + контрольная сумма

#define ARCHIVE_SETTINGS_FRAM_ADDR     1500
#define ARCHIVE_SETTINGS_FRAM_LENGTH   (ARCHIVE_SETTINGS_MODBUS_REGNUM*2+2) 

#define CHREP_MAX_INST_CUR_FRAM_ADDR   1650     // Адрес во фрам памяти по максимальному току ЧРЭП
#define CHREP_MAX_INST_CUR_FRAM_LENGTH 4      // Длина максимального тока ЧРЭП в байтах + CRC16

#define DYNAGRAM_SETTINGS_FRAM_ADDR   1700
#define DYNAGRAM_SETTINGS_FRAM_LENGTH  (DYNAGRAM_SETTINGS_MODBUS_REGNUM*2+2) 

#define TAPS_ARCHIVE_FRAM_ADDR        2000
#define TAPS_ARCHIVE_RECORD_SIZE        14
#define TAPS_ARCHIVE_RECORDS_NUM       200
#define TAPS_ARCHIVE_FRAM_LENGTH      (TAPS_ARCHIVE_RECORD_SIZE*TAPS_ARCHIVE_RECORDS_NUM)
//------------------------------------------------------------------------------
#define forceSDA(a) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (GPIO_PinState)a) 
#define readSDA HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define forceSCL(a) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (GPIO_PinState)a) 
#define framWriteProtect(a) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, (GPIO_PinState)a) 

#define framSize 32768       // Размер памяти FRAM FM24W256 в байтах

uint8_t framWrite(uint16_t addr, uint8_t* src, uint16_t bytesNum);
uint8_t framRead(uint16_t addr, uint8_t* src, uint16_t bytesNum);
void icReset(void);