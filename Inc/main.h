#include "stm32f4xx_hal.h"

//---- Уставки по напряжению ---------------------------------------------------
#define Ustop       800    // Выключить ШИМ
#define Ufollow     750    // Напряжение, начиная с которого двигатель начинает увеличивать приращение фазы для того, чтобы отслеживать большие изменения частоты в генераторном режиме
#define Uhi         700    // Граница работы алгоритма
#define Ulo         650    // Ниже этой границы - вернуть выходную частоту к исходному значению
#define dF          0.01f  // Скорость возврата частоты за один период П-регулятора       
#define Uunfix      430    // Udc>430 - запуск двигателя становиться возможным, по мгновенным значениям
#define Umin        373    // Отключение ШИМ, по мгновенным значениям
#define UminTimer   1000   // Время удержания ниже Umin в мс 

#define DRIVER_STOP(a) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, (GPIO_PinState)!a)
#define DRIVER_F_RESET(a) HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, (GPIO_PinState)!a)

#define LED1_GREEN(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (GPIO_PinState)a) 
#define LED1_RED(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, (GPIO_PinState)a) 

#define LED2_GREEN(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, (GPIO_PinState)a) 
#define LED2_RED(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, (GPIO_PinState)a) 

#define LED3_GREEN(a) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (GPIO_PinState)a) // Добавлен
#define LED3_RED(a) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (GPIO_PinState)a)   // Добавлен        

#define THYRISTOR_ON(a) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, (GPIO_PinState)a) 
#define RCHARGE_ON(a) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, (GPIO_PinState)a) 
#define FAN_ON(a) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, (GPIO_PinState)a) 

#define FAULT_OPTO_OUT(a) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (GPIO_PinState)a) 

#define DOUT1(a) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, (GPIO_PinState)a) 
#define DOUT2(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, (GPIO_PinState)a) 
#define DOUT3(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (GPIO_PinState)a) 
#define DOUT4(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, (GPIO_PinState)a)
#define DOUT5_RED_LED(a) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (GPIO_PinState)a)
#define DOUT6_GREEN_LED(a) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)a)
#define DOUT7_BUZZER(a) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)a)

extern uint8_t stateDOUT;

extern uint16_t modbusAddr;
extern const char progVersionString[40];

extern DAC_HandleTypeDef hdac;

typedef struct{     
  uint8_t curState;       // Состояние двигателя: 0) Простой (нет вращения, ожидание команды)
  uint8_t nextState;      //                      1) Разгон
                          //                      2) Торможение
                          //                      3) Работа на заданной частоте
                          //                      4) Смена контекста
                          //                      5) Авария
                          //                      7) Остановка выбегом    
  uint8_t curMode;        // Текущее состояние системы: 0 - неопределенное состоняние при включении питания (Сохраняется в ЭНОЗУ)   
                          //                    1 - режим "АВТОНОМНЫЙ ЧРЭП" - обеспечивается полная функциональность ЧРЭП. Режим "АВТОНОМНЫЙ ЧРЭП" и "АВТОНОМНЫЙ КП" задается регистром модбас. 
                          //                    2 - режим "АВТОНОМНЫЙ КП" - автозапуск ЧРЭП'ом запрещен, сбрасывать счетчики АПВ нельзя! Сброс только с верхнего уровня и тумблером!
                          //                    3 - режим "РУЧНОЕ УПРАВЛЕНИЕ" - любые команды управления запрещены, контекст задается только тумблером, а не RS232                              
  uint8_t tempMode;       // Временный режим, необходим для того, чтобы можно было сбрасывать ошибки без изменения режима, задержка на 3 сек
  uint32_t modeSwitchTimer; // Таймер задержки между переключением режимов работы
  uint8_t prevMode;       // Предыдущее состояние системы    
  uint8_t manualStart;    // Флаг ручного пуска
  uint8_t Ready;          // Флаг готовности выполнения команд
  uint16_t stateTC;       // Состояние побитовой готовности данных по порту ТС: 0 - данные не готовы, 1 - данные готовы 
  uint16_t dataTC;        // Состояние дискретных портов ТС: 0 - дискретный вход ТС разомкнут, 1 - дискретный вход ТС замкнут  
  uint16_t resetTC;       // Сбосить счетчики состояний  
  uint8_t autoStartDelay; // Случайная задержка на повторное включение: 5-30 сек
  uint32_t faults;        // Произошла авария, запустить обработчик аварий и повторных автозапусков, битовое поле, номер бита соответствует макросам ***_FAULT в модуле alarms.h                  
  uint8_t startDisable;   // 1 - запретить любые включения, в т.ч. автозапуски   
  uint32_t startDisableCnt;    // Задержка на автозапуск, счетчик
  uint32_t startDisableCntMax; // Задержка на автозапуск, предел счетчика
  uint8_t startAfterStopDelay;  // Задержка в секундах на включение после выключения по кнонке "СТОП" или команде модбас
  uint8_t kpStart;        // Флаг для того, чтобы можно было реагировать на кнопку "СТОП" в режиме "АВТО КП"
  uint16_t wasStopCHREP;  // Флаг того, что был останов ЧРЭП с верхнего уровня
  uint16_t thyristorON;   //
  uint16_t tempHoldON;
  uint16_t etrHoldON;
  uint32_t skipDoublePress; // Пропуск двойного нажатия кнопки старт    
  float desiredFreqOrRPM;   // Установка выходной частоты статора/об. в мин, т.к. источников частоты много, нельзя работать с command.desiredFreqOrRPM напрямую
  uint8_t eventLogCurMode;  // Текущее состояние ЧРЭП, переменная только для записи в протоколе событий
  uint8_t eventLogPrevMode; // Предыдущее состояние ЧРЭП, переменная только для записи в протоколе событий  
  uint8_t eventLogHardwareFault; // Код аппаратной ошибки для записи в протоколо событий
}SYSTEM_STATE_STRUCT;
extern volatile SYSTEM_STATE_STRUCT systemState; 

void handlerTC(volatile uint16_t *state,volatile uint16_t *data,volatile uint16_t *reset);

#define RS485_DIR(a) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, (GPIO_PinState)a) 

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart4;
void adcConv(void);

void F_RESET(void);
void energyCalcTask(void);
void energyCalcInit(void);
void copymas(uint8_t *dst, uint8_t *src, uint32_t bytesNum);
void DS18B20_UART4_Init(uint32_t baudRate);
void Delay_us(volatile uint32_t nCount);
void debugRS232Task(void);
void modbusTask(void);

typedef struct{
  uint32_t UdcSum;
  uint16_t samplesCnt;
  uint16_t Udc;
  int16_t pwmCorrection;
  uint8_t UdcRdy;   
  
  int16_t error;
  int32_t errorIntegral;    
    
}PID_VOLTAGE_STRUCT;
extern volatile PID_VOLTAGE_STRUCT pidVoltage;

typedef struct{
  //------------------------------- Start ModBus -(функц.№4)--------------------
  uint16_t rs485_CRC_Err;           // Количество ошибок контрольной суммы модбас      
  uint16_t rs485_IRQ_ERR;           // Количество прерываний по ошибке UART  
  uint16_t rs485_recPacketNum;      // Счетчик принятых по интервалу тишины пакетов 
  uint16_t rs485_transPacketNum;    // Счетчик переданных пакетов
 
  uint16_t rs232_CRC_Err;           // Количество ошибок контрольной суммы модбас      
  uint16_t rs232_IRQ_ERR;           // Количество прерываний по ошибке UART  
  uint16_t rs232_recPacketNum;      // Счетчик принятых по интервалу тишины пакетов
  uint16_t rs232_transPacketNum;    // Счетчик ппереданных пакетов
  
  uint16_t temp85;               // Количество ресетов датчика температуры во время работы ЧРЭП (это может быть и реально 85 град.!!!- есть проверка)
  uint16_t tempInitErr;          // Количество ошибок датчика температуры при инициализации  
  uint16_t tempCRCerr;           // Количество ошибок контрольной суммы датчика температуры
  uint16_t tempReadsNum;         // Счетчик успешных попыток чтения температуры 
  
  uint16_t framReadErr;          // Количество ошибочных вызовов функции framRead(...) 
  uint16_t framWriteErr;         // Количество ошибочных вызовов функции framWrite(...)   
  
  uint16_t getTemperatureTaskMS; // Максимальное время в мс, затраченное на выполнение функции getTemperatureTask() с момента включения питания, либо сброса счетчиков
  uint16_t getDataTimeTaskMS;    // Максимальное время в мс, затраченное на выполнение функции getDataTimeTask() с момента включения питания, либо сброса счетчиков
  uint16_t energyCalcTaskMS;     // Максимальное время в мс, затраченное на выполнение функции energyCalcTaskMS() с момента включения питания, либо сброса счетчиков
  uint16_t debugRS232TaskMS;     // Максимальное время в мс, затраченное на выполнение функции debugRS232TaskMS() с момента включения питания, либо сброса счетчиков       
  uint16_t modbusTaskMS;         // Максимальное время в мс, затраченное на выполнение функции modbusTaskMS() с момента включения питания, либо сброса счетчиков       
  uint16_t seqErr_ADC_PWM;       // Количество ошибок последовательности совместной работы ШИМ и АЦП 
  uint16_t seqErr_EnergyCalc;    // Ошибка вычисления параметров за период выходной частоты (совместная работа прерывания АЦП и energyCalcTask)
    
  uint16_t I;                    // Вычисленный ток ЭТР 
  uint16_t overloadCurrent;      // уставка тока 
  uint16_t overloadLimit;        // Порог срабатывания защиты ETR: ((overloadPercent*Inom)^2)*overloadSec  
  uint16_t overloadCnt;          // Счетчик для сравнения с overloadLimit   
  
  uint16_t curState;
  uint16_t nextState;
  uint16_t curMode;
  uint16_t tempMode;
  uint16_t prevMode;
  uint16_t manualStart;
  uint16_t stopCHREP;
  uint16_t kpStart;
  uint16_t faults;
  uint16_t startDisable;  
  
  uint16_t currentCnt;         //  Текущий счетчик АПВ перегрузок по току
  uint16_t disbalanceCnt;      //  Текущий счетчик АПВ перегрузок по дисбалансу и обрыву фазы
  uint16_t highVoltageCnt;     //  Текущий счетчик АПВ перегрузок по перенапряжению
  uint16_t noLoadCnt;          //  Текущий счетчик АПВ перегрузок по холостому ходу
  uint16_t etrCnt;             //  Текущий счетчик АПВ перегрузок по ЭТР
  uint16_t lowVoltageCnt;      //  Текущий счетчик АПВ перегрузок по низкому напряжению
  uint16_t temperatureCnt;     //  Текущий счетчик АПВ перегрузок по температуре  
  uint16_t fixFreqAttempt;     //  Счетчик моментов входа в режим ограничения максимальной частоты  
  
  uint16_t wifi_IRQ_ERR;           // Количество прерываний по ошибке UART  
  uint16_t wifi_recPacketNum;      // Счетчик принятых по интервалу тишины пакетов
  uint16_t wifi_transPacketNum;    // Счетчик ппереданных пакетов  
  
  uint16_t bviTaskMS;              // Максимальное время в мс, затраченное на выполнение функции bviTask() с момента включения питания, либо сброса счетчиков
  uint16_t activeEnergyTaskMS;     // ... 
  uint16_t tiiTaskMS;
  uint16_t noPhaseTaskMS;
  uint16_t scenarioMS;
  uint16_t dynaGramTaskMS;
  uint16_t dynagramLoadErrorCnt;   //Текущий счетчик АПВ перегрузок по низкому напряжению
  //------------------------------- End ModBus -(функц.№4)----------------------  
  uint8_t sequenceADCPWM;        // Последовательность работы прерываний: 0 - прерывание в начале центрированного ШИМ до vectorPWMcalc(), 1 - после vectorPWMcalc(), 
                                 // 2 - в начале прерывания АЦП, 3 - в конце прерывания АЦП, 4 - прерывание в конце центрированного ШИМ      
  uint8_t sequence_EnergyCalc;   // Последовательность корректного вычисления выходных значений за период  
}DIAG_STRUCT;
extern volatile DIAG_STRUCT diag;
void diagInit(void);
extern uint8_t errorToDebug;

typedef struct{  
  uint16_t bits;            // Битовое поле ТС  
  uint16_t *transitionsNum; // Количество переходов между логическими уровнями 
  uint8_t modBusWrite;      // Флаг изменения значения счетчиков по модбас 
  uint16_t modBusBuffer[16];// Доступны регистр текущих состоняий битов + 15 регистров-счетчиков 
  uint8_t curState[16];     // Текущее состояние входов ТС:0 - разомкнут, 1 - замкнут  
  uint8_t prevState[16];    // Текущее состояние входов ТС:0 - разомкнут, 1 - замкнут  
  uint8_t initState[16];    // Флаг фиксации состояния ТС при включении, чтобы не фиксировать ложного срабатывания
}TC_STRUCT;
extern volatile TC_STRUCT TCInputs;
void TCInputsInit(void);

extern volatile int16_t Q10;
extern volatile uint8_t udc_1kHz_decimator_max;

//#define WI_FI_DEBUG 1 // Режим отдадки WI_FI модуля через RS485, все принятые и переданные данные с модуля ESP8266EX 
#define ESP_RESET(a) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, (GPIO_PinState)!a)

extern uint32_t wifiTimeOutTimer;
extern uint8_t waitForWiFiFlag;
extern uint8_t wifiResponsed;
extern volatile uint8_t rinatTimer;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

#define ELAM_BUF_SIZE 5100
#define COM_RX_BUF_SIZE 2048
#define COM_TX_BUF_SIZE 2048
extern uint8_t rs485RxBuf[ELAM_BUF_SIZE];
extern uint8_t rs485TxBuf[ELAM_BUF_SIZE];
extern uint8_t elamBuf[ELAM_BUF_SIZE];
extern uint8_t rs232RxBuf[COM_RX_BUF_SIZE];
extern uint8_t rs232TxBuf[COM_TX_BUF_SIZE];
extern uint8_t wifiRxBuf[COM_RX_BUF_SIZE];
extern uint8_t wifiTxBuf[COM_TX_BUF_SIZE];  

extern volatile uint16_t rs232RxPtr;
extern uint16_t rs232TxPtr;
extern volatile uint8_t rs232DataReady;
extern volatile uint16_t rs485TxReady;

extern volatile uint16_t wifiRxPtr;
extern uint16_t wifiTxPtr;
extern volatile uint8_t wifiDataReady;

extern uint8_t oneSec;
extern volatile uint8_t vcpDataRdy;
extern volatile uint16_t vcpDataLen;
extern volatile uint8_t *vcpDataPtr;