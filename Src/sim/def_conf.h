//#define DEF_UDP_PORT        5002

//#define DEF_IP_SRV_0    81
//#define DEF_IP_SRV_1    22  
//#define DEF_IP_SRV_2    207
//#define DEF_IP_SRV_3    187 

#define DEF_UDP_PORT        5050

#define DEF_IP_SRV_0    185
#define DEF_IP_SRV_1    148  
#define DEF_IP_SRV_2    222
#define DEF_IP_SRV_3    10 

//#define DEF_TM_NO_LINK  10 //минут       //dobavka
#define DEF_TM_NO_LINK  0 //минут       //dobavka
#define DEF_TM_LINK_RES 60 //минут       //dobavka


#define MIN_TM_LINK_RES 30 //минут       //dobavka



#define DEF_CNTR_LINK        45                 // было 120  07 августа 2014 
#define DEF_TM_CNTR_LINK     62
#define DEF_CNTR_LINK_WAITS  12
#define DEF_CNTR_VOL_TRY     10 //5
#define BEG_CNTR_LINK        10
#define DEF_CNTR_NAT         50
#define DEF_NUM_SELF         3569
#define NUM_SRV_LINK         0
#define DEF_TM_CNTR_CL       10
#define DEF_CNTR_NAT_R       45

#define DEF_TP_TS1 1
#define DEF_TP_TS2 1


#define DEF_NUM_DST_SEQ      0xffff
#define DEF_VOL_TM_VZAT      30
//#define DEF_DES_SEQ          1
#define DEF_DES_SEQ          0

#define DEF_DES_PORT_GPRS    1 
#define DEF_PORT_GPRS_S      9600


#define PREDEL_OUT_TC        200 // 68 мсек


#define DEF_EM232_POST_TX    5 // 5 милисекунд - фиксированный защитный таймаут на обработку входных пакетов по эмул RS232
                               // если в течении этого промежутка что-то дрыгается, то потом игнорируется посылка 
                               
#define DEF_TM_STM_ON       4  // время пребывания в состоянии подачи питания
#define DEF_TM_STM_KON      3  // время пребывания в состоянии "Кнопка включения включена"
#define DEF_TM_STM_PWK      4 // время пребывания в состоянии "Модем включен" пока не активизируется порт SIM800


#define DEF_TM_STM_SCP   30 // время пребывания в состоянии STM_SCP
#define DEF_TM_STM_KOFF   9 // время пребывания в состоянии STM_KOFF
#define DEF_TM_STM_OFF   3 // время пребывания в состоянии STM_OFF

//Для таймера 2 
#define DEF_TM2_STM_ON     (DEF_TM_STM_ON+DEF_TM_STM_KON+DEF_TM_STM_PWK+5)
#define DEF_TM2_STM_SI     150
#define DEF_TM2_STM_SC     60
#define DEF_TM2_STM_SCP    (DEF_TM_STM_SCP+10)
#define DEF_TM2_STM_DCP    30
#define DEF_TM2_STM_DC     60
#define DEF_TM2_STM_OFF    16

#define DEF_TM_KEY_OFF 1500 //держать ногу отключения 1500 мсек

#define TM_TU_IMP 8



                               
                               
                               
                               
