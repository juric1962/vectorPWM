//смешениz PPP    
# define PPP_BEG   0
# define PPP_PROT  1  

 //смешени IP 
# define IP_VLEN   2
# define IP_TYPE   3                  
# define IP_LN     4
# define IP_ID     6   
# define IP_FUK    8 
# define IP_TIME   10 
# define IP_PROT   11 
# define IP_CRC    12   
# define IP_SRC    14 
# define IP_DST    18  

//смешения UDP        
# define UDP_PSRC  22  
# define UDP_PDST  24 
# define UDP_LN    26 
# define UDP_CRC   28

//смешения TR
# define TR_CRC    38
# define TR_LEN    32
# define TR_KOD    34    
# define TR_V      36  
# define TR_ID     37    
# define TR_SRC    30
# define TR_DST    40 
# define TR_OP_DATA  42
 

# define MIN_TR_HEAD 12 
                             
 
# define ID_CC 0

//Опция
# define KOD_OP_ERROR   4 //опция ошибка

# define ERR_NO_INTABL  1// код ошибки - нет в таблице
# define L_ERR_NO_INTABL 5

# define NO_SINC_PG   2 //код ошибки - рассинхронизация при постраничной отправке
# define L_NO_SINC_PG 3

# define OVER_PG      3 //код ошибки - переполнение программного буфера
# define OUT_LIFE     5 //код ошибки - время вышло
# define DES_OPERATE  6 //код ошибки - операция недопустима



//Опция
# define KOD_OP_SIZE_PG 1 
# define L_OP_SIZE_PG 3  
# define L_OP_SIZE_PG_OTV 4 
 
                                                      
//Опция
# define KOD_OP_LUFKIN   6
# define L_OP_LUFKIN     4

# define KOD_OP_LIFE 7 
# define L_OP_LIFE   3

                                                      
//Опция
# define KOD_WR_PG   2
# define L_OP_WR_PG      4

//Опция
# define KOD_WR_PG_CRC   3
# define L_OP_WR_PG_CRC  4

//Флаги
# define FIRST_PG   0x01
# define NO_END_PG  0x02
# define NADO_KV    0x04

//Эмулированный порт 232
//# define MAX_BUF_RS232_2 100 




//служебный протокол сервера связи
# define PROT_SL_LS 3  //код протокола
// Команда -контроль клиента
# define COM_CNTR_CL 2
# define L_COM_CNTR_CL 2
# define L_OTV_CNTR_CL 3
//Смещения служебный протокол сервера связи относительно конца транспортного заголовка
# define SL_LS_PROT  0  //прикладной протокол




//Прикладной протокол C1
# define PROT_C1 1  //код протокола

# define PORT_NONE   0
# define PORT485_1   1 //порт приложения "RS485_1" 
# define PORT485_2   2 //порт приложения "RS485_2" 
# define PORT232_2   3 //порт приложения "RS232_2" 
# define PORT_SEQ    4 //порт приложения "охрана"
# define PORT_SYS    5 //порт приложения "служебные сообщения"
# define PORT_CONF   6 //порт приложения "конфигурирование параметров контроллера"
# define PORT_PROG   7 //порт приложения "программирование контроллера"
# define PORT_MBUS   8


//Смещения С1 относительно конца транспортного заголовка
# define C1_PROT  0  //прикладной протокол
# define C1_PORT  1  // номер порта
# define C1_CONT  2  //Контексты для портов
//# define C1_DATA  3  //данные
# define C1_DATA  2  //данные 
# define C1_DATA_RS  10  //данные входные для физических RS

/*
//Порт PORT485_1
# define LN_BUF_485_1 100 //длина доп. буфера под данные 
# define MAX_BUF_RS485_1 300 //буфер под прием передачу должен быть не менее LN_BUF_485_1!

//# define LN_BUF_485_2 50 
//# define MAX_BUF_RS485_2 70 //буфер под прием передачу должен быть не менее LN_BUF_485_2!

# define LN_BUF_485_2 200 
# define MAX_BUF_RS485_2 210 //буфер под прием передачу должен быть не менее LN_BUF_485_2!
*/

//Команды по порту "Служебные сообщения"(1 байт)


//команда - ошибка физических портов RS-485
# define RS485_ERR     1   
// информационное поле (номер физического порта + тип ошибки) всего 2 байта
//длина информационного поля 2 байта
# define LN_RS485_ERR   2   // значение
//возможные номера физического порта
# define NUM_RS485_1   1 // первый порт  
# define NUM_RS485_2   2 // второй порт
# define NUM_RS232_2   3 // второй порт

//возможные типы ошибкок     
# define RS_NO_LINK     1  // нет ответа по порту от устройства
# define RS_NO_CONT     2  // отсутствует требуемый контекст 
# define RS_BUSY        3  // порт занят
# define RS_OVER_BUF_RX 4  // переполнение буфера при приеме
# define RS_OVER_BUF_TX 5  // переполнение буфера при передаче
# define RS_DATA_ERR    6
# define RS_OVER_BUF_PPP_TX 7 //переполнение буфера передачи GPRS при попытке отправки полученных данных с порта
# define RS_TEST        8  // включен тестовый режим


//команды по порту "охрана"





# define KVITOK_SEQ 0x01     // код подтверждения охранки
# define KVITOK_SEQ_10 10    // код подтверждения архива
# define L_KVITOK 0x0000

# define COOL_RESET 0x02 //включение контоллера
//длина 0x0004
//4 байта - метка времени
# define L_COOL_RESET 0x0004


# define HOT_RESET 0x03 //включение контоллера
//длина 0x0006
# define L_HOT_RESET 0x0006
//4 байта- метка времени
//1 байт - тип ресета
# define RST_EXT   0x00
# define RST_BROWN 0x01
# define RST_WDR   0x02
//1 байт - причина



# define ALRM 0x04 //тревога
//длина 0x0006
//4 байта- метка времени
// 1 байт состояние шлейфа 0-TC нарушен 1-ТС в норме
//1 байт - резерв
# define L_ALARM 0x0006

# define VZT 0x05 //взят
//длина 0x0006
//4 байта- метка времени
// 1 байт состояние шлейфа 0-TC нарушен 1-ТС в норме
//1 байт - резерв
# define L_VZAT 0x0006

# define SEQ_STATE 0x06 //взят
//длина 0x0007
//4 байта- метка времени
//1 байт - состояние охраны    ALARM,VZAT,SEQ_NO_DEF
// 1 байт состояние шлейфа 0-TC нарушен 1-ТС в норме
//1 байта  (0-бит сост сеть(0)-акб(1) )
# define DES_SEQ  0xff// охрана дезактивирована
# define L_STATE 0x0007


# define ZAPR_TC   SEQ_STATE
# define L_ZAPR_TC 0


# define NO_NET  0x01
# define YES_NET 0x00
# define AKB_SET 0x07 //тревога
//длина 0x0006
//4 байта- метка времени
// 1 байт состояние шлейфа 0-сеть нарушен 1-акб
//1 байт - резерв
# define L_AKB_SET 0x0006

//типы тс
# define TC_OFF   0
# define TC_SUHOI 1
# define TC_SHL   2
# define VOL_BLOCK_TC 3 // Блокировка ТС в секундах













//описание контекста портов
#define B2400  4
#define B4800  5
#define B9600  6
#define B19200 8
#define B38400 9

#define STOP1  0
#define STOP2  2

#define NON   0
#define ODD   1
#define EVEN  2

#define INF7   2
#define INF8   3

//Приложение конфигурирование
//команда
#define CONF_VERS 1 //конфигурирование времени
#define L_RD_CONF_VERS 0   

#define CONF_MAP 2 //конфигурирование времени
#define L_RD_CONF_MAP 0   

#define CONF_TM_NAT    7 //
#define L_CONF_TM_NAT  2 //команды на чтение  

#define CONF_TM_CC    8 //
#define L_CONF_TM_CC  2 //команды на чтение  

#define CONF_TM_WT    9 //
#define L_CONF_TM_WT  2 //команды на чтение

#define CONF_DES_OHR  10 //
#define L_CONF_DES_OHR  1 //команды на чтение

#define CONF_NUM_CL    11 //
#define L_CONF_NUM_CL  2 //команды на чтение

#define CONF_TM_VZ    12 //
#define L_CONF_TM_VZ  2 //команды на чтение

#define CONF_TM_CL    13 //
#define L_CONF_TM_CL  2 //команды на чтение


#define CONF_TIME 15 //конфигурирование времени
#define L_WR_CONF_TIME 6 //команды на запись       
#define L_RD_CONF_TIME 0 //команды на чтение                          {


#define CONF_KEYS 0x7f //конфигурирование времени
#define L_WR_CONF_KEYS 8 //команды на запись       
//#define L_RD_CONF_KEYS 8 //команды на чтение   


#define CONF_IP    4 //
#define L_CONF_IP  4 //команды на чтение

#define CONF_UDP    5 //
#define L_CONF_UDP  2 //команды на чтение

#define CONF_NUM_SELF    6 //
#define L_CONF_NUM_SELF  2 //команды на чтение

#define CONF_RESET    14 //

#define CONF_PDP    3 //
#define L_CONF_PDP  58 //команды на чтение

#define CONF_PDP_DOP    0x11 //

#define CONF_LOG 0x10 //конфигурирование времени
#define L_CONF_LOG 0 //команды на запрос      

#define CONF_OZU 0x12 //версия ОЗУ
#define L_RD_CONF_OZU 0   
     

#define CONF_PDP_R    0x13 //
#define L_CONF_PDP_R  58 //команды на чтение

#define CONF_PDP_DOP_R    0x14 //


#define CONF_TM_NO_LINK 0x15  //конф. время отсутствия связи и перехода на др. симку в минутах          //dobavka
#define L_CONF_TM_NO_LINK  2 //команды на чтение  //dobavka

#define CONF_TM_LINK_RES 0x16   //конф. времени работы на резервной симке   в минутах          //dobavka
#define L_CONF_TM_LINK_RES  2 //команды на чтение  //dobavka


#define CONF_TP_TS 0x17   //конф. ТС
#define L_CONF_TP_TS  2 //команды на чтение  


#define CONF_TM_NAT_R    0x18 //
#define L_CONF_TM_NAT_R  2 //команды на чтение 

#define CONF_TRAF       0x19
#define L_CONF_TRAF     0

#define CONF_TEMPER    0x1a //команды на чтение температуры
#define L_CONF_TEMPER    0

// Команды по порту программирования
#define COM_CODE_MEM 0x01 //запись в программную память
#define L_COM_CODE_MEM_Z 256 // длина на запрос
#define L_COM_CODE_MEM_A 3 // длина на ответ


