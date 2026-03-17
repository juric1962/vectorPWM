#include <stdint.h>

#define DYNA_POINTS 200
#define DYNA_RAW_RECORD_SIZE 12                                     // 2 байта под автивную мощность (int16_t, деленную на 2 для увеличения динамического диапазона), 4 байта под фазу, 4 байта под семпл      
#define DYNA_RAW_ARRAY_SIZE (DYNA_POINTS*DYNA_RAW_RECORD_SIZE)
#define HALF_DYNA_POINTS (DYNA_POINTS/2)                            // Половина точек динамограммы для алгоритма заполненности насоса 

extern uint8_t dynaRawArray[DYNA_RAW_ARRAY_SIZE];  // Область памяти в ОЗУ для записи сырых значений активной мощности и фазы выходного напряжения
typedef struct{
 int16_t activePower;  // Активная мощность деленная на 2 (для увеличения динамического диапазона), взятая один на dataUpdatePWMCycles раз  	
 int16_t currentCode;  // Усредненный код ТИТ3 (tit3SamplesNum), взятая один на dataUpdatePWMCycles раз  
 float phase;          // Текущая фаза выходной частоты
 uint32_t pwmSample;   // Текущий семпл ШИМ 
}DYNA_SAMPLE_STRUCT;

typedef struct{
  int16_t Watts[DYNA_POINTS];    // Ватты(деленные на 2) или код АЦП с ТИТ3   
  int16_t Shift[DYNA_POINTS];    // Перемещение полированного штока, мм  
  float Phase[DYNA_POINTS];      // Фаза кривошипа [0;2*pi)->[0;1)
  float Time[DYNA_POINTS];       // Время взятия точки динамограммы, сек
  float Force[DYNA_POINTS];      // Усилие на полированном штоке
}DYNA_OUTPUT_DATA;
extern DYNA_OUTPUT_DATA dynaOutputData;

typedef __packed struct{  
// ModBus start reg
  uint16_t l1;  //расстояние между осью вала редуктора и балансира по горизонтали, мм
  uint16_t l2;  //расстояние между осью вала редуктора и балансира по горизонтали, мм
  uint16_t l;   //шатун, мм
  uint16_t KK;  //заднее плечо балансира, мм
  uint16_t KK1; //переднее плечо балансира, мм
  uint16_t r;   //радиус кривошипа, 	   
  uint16_t dynagramUpdateSec;          // Период обновления динамограммы, сек
  uint16_t dynagramTimeOutSec;         // Тайм-аут в секундах ожидания сигнала с датчика оборотов начала формирования динамограммы
  uint16_t detectorRetryNum;           // Защита от дребезга датчика положения, в периодах ШИМ 
  uint16_t minPeriodOfDynagramMS;  // Минимальный период снятия динамограммы в циклах ШИМ, игнорировать ложное срабатывание   
  uint16_t tit3SamplesNum;             // Количество отсчетов для усреднения показаний ТИТ3
  float tit3ToForceCoeff;              // Коэффициент пересчета (k) показаний ТИТ3 (код 0-4095) в силу (Н) - y=k*tit3+b
  float tit3ToForceConst;              // Постоянная составляющая пересчета (b) показаний ТИТ3 (код 0-4095) в силу (Н) - y=k*tit3+b
//------ Настройки модбас аварии по датчику нагрузки ---------------------------  
  float loadMin;                       // Авария по минимальной нагрузке с динамографа (в единицах y=k*tit3+b) 
  float loadMax;                       // Авария по максимальной нагрузке с динамографа (в единицах y=k*tit3+b)   
  uint16_t loadMaxMinHoldMS;           // Время удержания нагрузки по минимальной и максимальной нагрузке с динамографа, мс (0 - анализ аварии отключен)
//------ Алгоритм управления частотой-------------------------------------------  
  float algorithmFmin;                 // Минимальная частота регулирования
  float algorithmFstart;               // Стартовая частота регулирования   
  float algorithmFmax;                 // Максимальная частота регулирования   
  float algorithmWorkLoadON;           // Нагрузка, при которой начинается уменьшение частоты, чтобы не было зависания штанги(algorithmWorkLoadOFF>algorithmWorkLoadON)
  float algorithmWorkLoadOFF;          // Нагрузка, при которой прекращается уменьшение частоты, возобновляет работу алгоритм увеличения заполненности насоса
  float algorithmFreqChange;           // Изменение частоты  при алгоритмах борьбы с зависанием штанги и увеличением заполненности насоса(на каждом шаге регулирования), Гц  
  float algorithmPumpShift;            // Перемещение регулировки наполненности насоса
  float algorithmPumpForce;            // Усилие регулировки наполненности насоса
  uint16_t algorithmPumpRetryNum;      // Кол-во последовательных динамограмм для контроля наполненности насоса (по принципу "удержания"), если 0 - то алгоритм наполнения насоса не работает  
  uint16_t algorithmOn;                // Включение работы алгоритма: 0 - выкл, 1 - вкл, 2 - режим тестовой динамограммы (>0 - работает в режиме ЧРЭП с источником частоты Модбас)  
  float algorithmKp;                   // Коэффициент пропорциональности ПИД регулятора зависания штанги, (>10e-6-вкл, вместо algorithmFreqChange, относительно algorithmWorkLoadON), не работает, если включен алгоритм наполненности насоса  
  uint16_t afterAlarmDelayMin;         // Задержка после аварии по датчику усилия (мин) 
  uint16_t testDynagramShiftMin;       // Тестовая динамограмма(квадрат) - минимальное перемещение 
  uint16_t testDynagramShiftMax;       // Тестовая динамограмма(квадрат) - максимальное перемещение
  uint16_t testDynagramLoadMin;        // Тестовая динамограмма(квадрат) - минимальная нагрузка
  uint16_t testDynagramLoadMax;        // Тестовая динамограмма(квадрат) - максимальная нагрузка
  uint16_t testDynagramRawPointsNum;   // Количество отсчетов ШИМ тестовой динамограммы           
  uint16_t slipPercent;                // Допустимый для алгоритма процент проскальзывания    
  float slipCoeff;                     // Коэфф. для компенсации скольжения 
  float slipConst;                     // Константа  для компенсации скольжения 
//------------------------------------------------------------------------------  
// ModBus end reg    
  uint16_t CRC16;
  uint16_t testDynagramRawPointsCnt; 
  uint16_t rawDataPtr;
  uint16_t error;                       // error, битовое поле: бит 0 — ошибка настройки алгоритма, бит 1 — тайм-аут сигнала положения, бит 2 — авария по минимальной уставке,
  uint16_t dataUpdatePWMCnt;            //                      бит 3 — авария  по максимальной уставке, бит 4 — авария по скольжению, бит 5 — неверное положение точки наполненности насоса                   
  uint8_t state;                        //                      
  
  float outputVoltagePhase;             // Наращиваемая между срабатыванием датчика оборота фаза выходной частоты преобразователя
  float outputVoltagePhaseSlip;         // Наращиваемая между срабатыванием датчика оборота фаза выходной частоты преобразователя, скомпенсирована по скольжению ротора
  float compareVoltagePhase;            // Фаза формирования очередной точки динамограммы
  float voltagePhaseDPhi;               // Инкремент формирования очередной точки динамограммы  
  uint8_t voltagePhaseRdy;              // Произошел замер инкремента формирования очередной точки динамограммы  
  
  uint32_t minPeriodOfDynagramCnt;      // Счетчик для minPeriodOfDynagramCycles  
  uint32_t dynagramTimeOutCycles;       // Определение тайм-аута сигнала датчика оборота во время съемки динамограммы
  uint8_t TC10retryCnt;                 // Счетчик защиты от дребезга контакта
  uint32_t tit3Sum;                     // Сумматор усреднения показаний ТИТ3 
  uint16_t tit3Cnt;                     // Счетчик усреднения показаний ТИТ3   
  uint32_t minPeriodOfDynagramCycles;
//------ Вспомогательные переменные для аварии по датчику нагрузки -------------
  int16_t loadMinAdcCode;               // Код ацп ТИТ3, соответсвующий loadMin tit3=(loadMin-b)/k; если<0, то=0
  int16_t loadMaxAdcCode;               // Код ацп ТИТ3, соответсвующий loadMax tit3=(loadMax-b)/k; если>4095, то=4095 
  uint16_t loadMinMaxHoldCnt;           // Счетчик удержания для  loadMaxMinHoldMS;
  uint16_t loadMinMaxHoldMStoCycles;    // Преобразованный из мс для инкрементирования счетчик в циклы ШИМ, с ним сравнивается счетчик loadMinMaxHoldCnt
  uint8_t loadAlarmState;               // Часть обработчика аварии вывести в основной цикл, чтобы не занимать время у обработчика АЦП: 1 - авария по loadMin, 2 - авария  по loadMax 
  uint16_t loadErrorAdcCode;            // Мгновенное значение кода АЦП сигнала с датчика нагрузки во время срабатывания аварии 
//------------------------------------------------------------------------------  
//------ Вспомогательные переменные для алгоритма управления частотой ----------     
  float algorithmFreq;                  // Частота алгоритма регулирования для установки в качестве выходной
  float algorithmUpdateFreq;            // Временная переменная для атомарности операции изменения частоты, новая частота на выходе алгоритма 
  uint8_t updateFreq;                   // Временная переменная для атомарности операции изменения частоты  
  uint8_t algorithmState;               // Состояние алгоритма управления частотой: 0-алгоритм не работает, 1-перейти на стартовую частоту, 2 - управление частотой по алгоритму 
  uint8_t algorithmWorkLoadState;       // Состояние алгоритма по зависанию штанги    
  uint16_t algorithmPumpRetryCntUP;     // Счетчик для algorithmPumpRetryNum повышения частоты
  uint16_t algorithmPumpRetryCntDOWN;   // Счетчик для algorithmPumpRetryNum понижения частоты     
//------------------------------------------------------------------------------    
}DYNA_GRAM_STRUCT;

extern volatile DYNA_SAMPLE_STRUCT* dynaRawData;
extern volatile DYNA_GRAM_STRUCT dynaGram;

typedef struct{
  uint16_t Position;       // Положение полированного штока
  uint16_t Load;           // Сила на полированном штоке  
}LUFKIN_FORCE_POINT_PAIR;  // Пары точек для динамограммы
typedef struct{
  uint16_t Time;                  // Время отсчета, сек*100
  int16_t activePower;            // Активная мощность/2
}LUFKIN_ACTIVE_POWER_POINT_PAIR;  // Пары точек для ватт-время-граммы
typedef struct{
uint32_t timeStamp;        // Временная метка (сек с 2000 г.) 
uint16_t numberOfPoints;   // Количество точек динамограммы
uint16_t maxLoad;          // Максимальное усилие 
uint16_t minLoad;          // Минимальное усилие
uint16_t strokeLength;     // Длина качка, размерность задается lengthLufkinCoeff 
uint16_t strokePeriod;     // Период качка, сек*100
LUFKIN_FORCE_POINT_PAIR pair[DYNA_POINTS];
}LUFKIN_CARD_FORCE;        // Структура динамограммы LUFKIN'а
/*typedef struct{
uint32_t timeStamp;        // Временная метка (сек с 2000 г.) 
uint16_t numberOfPoints;   // Количество точек динамограммы
int16_t maxActivePower;    // Максимальная активная мощность
int16_t minActivePower;    // Минимальная активная мощность
uint16_t strokeLength;     // Длина качка, мм
uint16_t strokePeriod;     // Период качка, сек*100
LUFKIN_ACTIVE_POWER_POINT_PAIR pair[DYNA_POINTS];
}LUFKIN_CARD_ACTIVE_POWER; // Структура динамограммы LUFKIN'а*/
extern volatile LUFKIN_CARD_FORCE lufkinCardForce;
//extern volatile LUFKIN_CARD_ACTIVE_POWER lufkinCardActivePower;

typedef struct{
  uint8_t typeDraw;         // Тип рисования: 1-линия,2- 2 линии,3 - точка,4-точка в виде звездочки, если другое значение, то структуру не рисовать 
  uint8_t dataType;         // Тип данных: 1-INT,2-UINT,3-DINT,4-UDINT,5-REAL, если другое значение, то структуру не рисовать 
  uint16_t dataArray[4];    // Контейнер разных типов данных
}DYNAMOGRAM_DRAW_ELEMENT;   // Структура, определяющая вид графического элемента на динамограмме
typedef struct{
  uint32_t timeStamp;                        // Временная метка (сек с 2000 г.) 
  DYNAMOGRAM_DRAW_ELEMENT drawElement[5];    // Графический элемент на динамограмме
LUFKIN_FORCE_POINT_PAIR pair[DYNA_POINTS];   // Динамограмма
}DYNAMOGRAM_PLOT;        // Структура вывода данных на динамограмму
extern volatile DYNAMOGRAM_PLOT dynaPlot;

void dynaGramInit(void);
void dynaGramTask();