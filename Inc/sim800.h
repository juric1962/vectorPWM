
typedef struct{
  // Start ModBus reg
  uint16_t ip0;   // Ip адрес сервера св€зи тетрада (н-р) 10.1.1.17 ->10
  uint16_t ip1;   // Ip адрес сервера св€зи тетрада (н-р) 10.1.1.17 ->1
  uint16_t ip2;   // Ip адрес сервера св€зи тетрада (н-р) 10.1.1.17 ->1
  uint16_t ip3;   // Ip адрес сервера св€зи тетрада (н-р) 10.1.1.17 ->17
  uint16_t port;  // UDP порт сервера св€зи 
  uint16_t num;   // Ќомер устройства в сети
  uint16_t num_contr;   // контрольный регистр дл€ разрешени€ записи номера устройства
  uint16_t nat;   // ¬рем€ удержани€ сессии в сек
  uint16_t cch;   // ѕериод контроль канала в сек
  uint16_t tm_ch; // “аймаут на квитанцию сек 
  
  uint16_t tm_client_asinc; // “аймаут на квитанцию ƒл€ асинхронных массивов сек
  uint16_t num_client_asinc;   // Ќомер клиента асинхронных сообщений в сети
  uint16_t des_asinc; // 1- заприетить асинхронную передачу 
  uint16_t ind_kvit_asinhr; // индекс последнего переданного архива
  
  
  char a_c_gprs[58];  // логин оператора св€зи, строка должна заканчиватьс€ нулем 
  // End ModBus reg
  // дл€ секретности установки адреса устройства в сети предлагаю сделать служебный регистр contr_num
  // значение num пишетс€ во флеш только если этот регистр contr_num= равен определеноому числу
}SIM800_SETTINGS_STRUCT;
extern SIM800_SETTINGS_STRUCT sim800Settings;
// дл€ задачи SIM800
// после инициализации порта USART3 (9600 N 8 1) разрешить прием одного байта
//HAL_UART_Receive_IT(&huart3, &sim800Data, 1); //  аждый раз принимать один байт  
void init_pin_sim800(void);            // вызываетс€ в void MX_GPIO_Init(void)
void milisecund_handler_sim800(void); // милисекундный обработчик void HAL_SYSTICK_Callback(void)
// обработчики прерываний от модул€  SIM800

void TxSim800Handler(void); // void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  if(huart->Instance==USART3)
  {
    TxSim800Handler();
      
  } 
*/
void RxSim800Handler(void);  // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  if(huart->Instance==USART3)
  {
    RxSim800Handler();
      
  } 
*/


void sim800StructInit(void);
void sim800StructToFlash(void);
void PreparaeSim800Work(void);  // вызвать перед вечным циклом
void modem_engine(void);         // вызвать в основном цикле программы
void start_func_stm_off(void);
void lock_it(void);
extern unsigned char sim800Data;
extern unsigned char ResetTaskSim800;
extern unsigned char zapr_from_sim800;
// дл€ задачи SIM800