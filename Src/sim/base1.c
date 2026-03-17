#include "DS18B20.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_def.h"
#include "sram_rtc.h"
//#include <iom1280.h> 
//#include <inavr.h> 
//#include "dfpin.h"
#include "dfproc.h"
#include "def_prot.h"
#include "def_conf.h"
#include "map_ef.h"
#include "map_ad.h"
#include "ozu_map.h"
#include "def_at.h"
#include "def_log.h"
#include "def_link.h"
#include "dfcnst.h"
//#include <string.h>
#include "sec.h"
//#include <stdio.h>
#include "map_mbus.h"
#include "sim800.h"
#include "main.h"
#include "fram.h"
#include "archive.h"
//#include "modbus.h"


#define CHECK_TIME 15

extern uint16_t *eventRecordPtr;

extern uint8_t rs485InputByte;
void MX_USART2_UART_Init_Custom(uint32_t baud, uint32_t wordlength,uint32_t stopbit, uint32_t parity);
extern void MX_USART2_UART_Init(void);
extern volatile uint16_t rs485RxPtr;
extern volatile uint8_t rs485DataReady;

void PreparaeSim800Work(void);
void reset_uart_prep_sim800(void);
extern volatile TEMP_SENSOR tempSensor[3];
extern RTC_TimeTypeDef  RTC_TimeStructure;
extern RTC_DateTypeDef RTC_DateStructure; 

extern UART_HandleTypeDef huart3;
void MX_USART3_UART_Init(void);

extern uint16_t cnt_supervosor;
extern unsigned char cnt_clr_wdt, cnt_flag_out;

void S1_OFF(void);
void S1_RD(void);
void S1_GR(void);
void S1_CH(void);
void S1_YL(void);
void SET_PWR(void);
void CLR_PWR(void);
void SET_PWRK(void);
void CLR_PWRK(void);
void CLR_RTS0(void);
void SET_RTS0(void);
void CLR_DTR0(void);
void SET_DTR0(void);


uint16_t cnt_sec;

const unsigned char tab_crc[] = { 0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
                 157,195,33,127,252,162,64,30,95,1,227,189,62,96,130,220,
                 35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
                 190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
                 70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
                 219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
                 101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
                 248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
                 140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
                 17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
                 175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
                 50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
                 202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
                 87,9,235,181,54,104,138,212,149,203,41,119,244,170,72,22,
                 233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
                 116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53 };     




extern const unsigned char keys_def[];


const char stm_pwr_on[]=        {'S','T','M',':','p','w','r',' ','o','n'};
const char stm_pwr_kon[]=       {'S','T','M',':','p','w','r',' ','k','o','n'};
const char stm_pwr_off[]=       {'S','T','M',':','p','w','r',' ','o','f','f',};
const char stm_pwr_koff[]=      {'S','T','M',':','p','w','r',' ','k','o','f','f',};
const char stm_si[]=            {'S','T','M',':','s','i'};
const char stm_sc[]=            {'S','T','M',':','s','c'};
const char stm_scp[]=           {'S','T','M',':','s','c','p'};
const char stm_ppp[]=           {'S','T','M',':','p','p','p'};
const char stm_rdc[]=           {'S','T','M',':','r','d','c'};

const char stm_base_sim[]=      {'S','T','M',':','b','a','s','e',' ','s','i','m'}; //dobavka
const char stm_res_sim[]=       {'S','T','M',':','r','e','s',' ','s','i','m'}; //dobavka

const char evm_tm[]=        {'E','V','M',':','t','m'};
const char evm_at_ok[]=     {'E','V','M',':','a','t',' ','o','k'};
const char evm_at_err[]=    {'E','V','M',':','a','t',' ','e','r','r'};
const char evm_ppp_ok[]=    {'E','V','M',':','p','p','p',' ','o','k'};
const char evm_ppp_err[]=   {'E','V','M',':','p','p','p',' ','e','r','r'};
const char evm_dcd_err[]=   {'E','V','M',':','d','c','d',' ','e','r','r'};
const char evm_no_rec[]=     {'E','V','M',':','n','o',' ','r','e','c'};
const char evm_cts_err[]=   {'E','V','M',':','c','t','s',' ','e','r','r'}; //dobavka
const char evm_ch_sim[]=    {'E','V','M',':','c','h',' ','s','i','m'};     //dobavka

//const char evc_heat_on[]=        {'E','V','C',':','h','e','a','t',' ','o','n'};
//const char evc_heat_off[]=       {'E','V','C',':','h','e','a','t',' ','o','f','f'};
const char evc_taken[]=          {'E','V','C',':','t','a','k','e','n'};
const char evc_alarm[]=          {'E','V','C',':','a','l','a','r','m'};
//const char evc_feed_akb[]=       {'E','V','C',':','f','e','e','d',' ','a','k','b'};
//const char evc_feed_220[]=       {'E','V','C',':','f','e','e','d',' ','2','2','0'};
const char evc_ts1[]=       {'E','V','C',':','t','s','1'};
const char evc_ts2[]=       {'E','V','C',':','t','s','2'};

const char evc_state[]=       {'E','V','C',':','s','t','a','t','e'};

const char evc_wdr[]=            {'E','V','C',':','w','d','r'};
const char evc_bod[]=            {'E','V','C',':','b','o','d'};
const char evc_rst[]=            {'E','V','C',':','r','s','t'};
const char evc_pwr[]=            {'E','V','C',':','p','w','r'};

const char msg_evc[]=          {'M','S','G',':','e','v','c',};
const char msg_ctrl_ch[]=      {'M','S','G',':','c','t','r','l',' ','c','h'};
const char msg_ctrl_nat[]=     {'M','S','G',':','c','t','r','l',' ','n','a','t'};


const char req_out_ctrl_cl[]={'R','E','Q','-','>','c','t','r','l',' ','c','l'};


/*
const char rec []=             {'R','E','C'};

const char req_out_ctrl_cl[]={'R','E','Q','-','>','c','t','r','l',' ','c','l'};
const char ans_in_ctrl_cl[]={'A','N','S','-','<','c','t','r','l',' ','c','l'};

const char req_in_485_1[]={'R','E','Q','-','<','4','8','5','_','1'};
const char req_in_485_2[]={'R','E','Q','-','<','4','8','5','_','2'};
const char req_in_232[]={'R','E','Q','-','<','2','3','2'};
const char req_in_st_contr[]={'R','E','Q','-','<','s','t',' ','c','o','n','t','r'};
const char req_in_config[]={'R','E','Q','-','<','c','o','n','f','i','g'};
const char req_in_prog[]={'R','E','Q','-','<','p','r','o','g'};


const char ans_out_485_1[]={'A','N','S','-','>','4','8','5','_','1'};
const char ans_out_485_2[]={'A','N','S','-','>','4','8','5','_','2'};
const char ans_out_232[]={'A','N','S','-','>','2','3','2'};
const char ans_out_st_contr[]={'A','N','S','-','>','s','t',' ','c','o','n','t','r'};
const char ans_out_config[]={'A','N','S','-','>','c','o','n','f','i','g'};
const char ans_out_prog[]={'A','N','S','-','>','p','r','o','g'};
*/
//void framReadInt(uint16_t adres_flesh, uint16_t *adres_ozu,uint16_t num);
//void WrArrayToFleshInt(uint16_t adres_flesh,  uint16_t *adres_ozu, uint16_t num,unsigned char flag,uint16_t znach); //запись конфигурации кп во флеш

int16_t crc_sum(unsigned char *ff,unsigned char kol);

void ds_handler(void);
void monitori(void);
void init_modem_call_inside(void);
void control_ts(void);

void energy_plus(void);

void init_pins_out_to_hiz(void);

void init_pins_hiz_to_out(void);

void monitor_wr_truffic(void);

void send_terminate_lcp(void);
char temp_to_grad(uint16_t t);

void supervisorwdt(void);

void kodirovka(uint16_t kol_send_byte);

void send_info(char size,char const *p,unsigned char fl_id,unsigned char id);

void    mov_massiv (char size, char *p);

void control_temperatura(void);

void init_modem_only(void);
extern unsigned char Regim;

void one_secund_proc(void);

extern unsigned char sel_modul;


void send_err485(unsigned char port,unsigned char err,unsigned char id,uint16_t dst);
uint16_t proc_config(unsigned char *buf_rx_ppp,unsigned char offset,uint16_t count_rx_ppp);
unsigned char check_ln_conf(unsigned char *buf_rx_ppp,unsigned char offset,uint16_t count_rx_ppp);
unsigned char check_cont_485_1 (unsigned char *pointer);
unsigned char check_cont_485_2 (unsigned char *pointer);

unsigned char ret_version(unsigned char unit,unsigned char *ptr);

uint16_t calc_crc_ip(unsigned char *p,uint16_t count);
uint16_t calc_crc_udp_2(unsigned char *buf_rx_ppp,uint16_t count);
uint16_t pppfcs16(uint16_t fcs,unsigned char *cp,uint16_t len);

void monitor_event_PAP(void);
void monitor_act_LCP(void);
void monitor_event_LCP(void);
void monitor_act_IPCP(void);
void monitor_event_IPCP(void);
void monitor_act_PAP(void);
void clr_var_ppp_rx(void);
void clr_var_ppp(void);

void analiz_ppp_rx(void);

void monitor1_tx_rs485_1(void);   //GPRS-штатный RS485_1
void monitor1_rec_rs485_1(void);
void monitor1_tm_rs485_1(void);

void monitor1_tx_rs485_2(void);   //GPRS-штатный RS485_2
void monitor1_rec_rs485_2(void);
void monitor1_tm_rs485_2(void);

void monitor1_tx_rs232_2(void);   //GPRS-штатный RS232_2
void monitor1_rec_rs232_2(void);
void monitor1_tm_rs232_2(void);



  extern void init_scen_stm_si(void);
  extern void init_scen_stm_sc(void);
  extern void init_scen_stm_dc(void);
  extern void at_com_rx(unsigned char cnt);
  extern void at_com_tx(unsigned char cnt);
  extern unsigned char at_com_scen_init(unsigned char *cnt,unsigned char *rp);
  extern unsigned char at_com_scen_stm_sc(unsigned char *cnt,unsigned char *rp);
  //extern unsigned char at_com_scen_stm_dc(unsigned char *cnt,unsigned char *rp);
  extern void init_scen_stm_dc_1(void);
  extern unsigned char at_com_scen_stm_dc_1(unsigned char *cnt,unsigned char *rp);

  
extern unsigned char compress_off;
  
   extern unsigned char cnt_com;
   extern unsigned char rep;
  
extern char emei[40];
uint32_t unix; 
 
extern union {
       unsigned char mb[SEG3*2];//байтовый массив 
       } c2_byte; //1я страница конфигурации 

extern union {
       unsigned char mb[SEG99*2];//байтовый массив 
       } c1_byte; //страница конфигурации ГЗУ
   
extern uint16_t modbus_mem1[SEG1];

extern struct struct_ts str_tc1,str_tc2,str_tc3,str_tc4,str_tc5,str_tc6,str_tc7,str_tc8;   
   
   

extern void framWriteSim800(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num);

extern uint8_t framRead(uint16_t addr, uint8_t* dst, uint16_t bytesNum);
//extern void framRead(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);
extern void write_log_info(unsigned char sost,unsigned char mesto);
extern uint32_t burst_ds_r(void);
extern void set_rlt(unsigned char address, unsigned char * data);
unsigned char write_to_buf(unsigned char *p_buf,uint16_t *ptr_in,uint16_t *ptr_out,uint16_t *ptr_out_kv,uint16_t *crc,unsigned char *p_data,uint16_t l_data, uint16_t max_buf);
uint16_t read_from_buf(unsigned char *p_buf,uint16_t *ptr_in,uint16_t *ptr_out,uint16_t *ptr_out_kv,uint16_t *crc,unsigned char *p_data, uint16_t max_buf);

//enum t_version {VER1 = 1 ,VER2,VER3,VER4};
//enum t_type {ZAPR,OTV,SOOB,KVIT};                
enum t_state_modem {STM_NONE=0,STM_ON,STM_KON,STM_PWK,STM_SI,STM_SC,STM_SCP,STM_PPP,STM_DCP,STM_DC,STM_KOFF,STM_OFF};
enum t_state_modem state_modem;
enum t_event_modem event_modem;
enum bool command_AT;

unsigned char cnt_stm_tm1,cnt_stm_tm2;
//uint16_t cnt_key_off;

unsigned char ResetTaskSim800;

extern unsigned char keys[10];

extern struct
        { // в двоичном коде
          char r_sec;
          char r_min;
          char r_hor;
          char r_day;
          char r_date;
          char r_month;
          char r_year;
          char r_control;
        }  real_time; 


extern struct
  {
  unsigned char  ip         :1;
  unsigned char  udp        :1;
  unsigned char  num_self   :1;
  unsigned char cnt_reset;
  }fl_rewrite;

/*
 #pragma location=A_RW_PDP 
__eeprom __no_init unsigned char e_rw_pdp;

 #pragma location=A_RW_PDP_R 
__eeprom __no_init unsigned char e_rw_pdp_r;

*/

const  unsigned char auchCRCHi[] ={
  0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,
  0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,
  0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
  0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
  0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,
  0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
  0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,
  0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
  0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,
  0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
  0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,
  0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
  0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
  0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
  0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
  0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
  0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
  0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,
  0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,
  0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40};
  
    
const   unsigned  char auchCRCLo[] = {
0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,
0xC5,0xC4,0x04,0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,
0xCB,0x0B,0xC9,0x09,0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,
0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,0x14,0xD4,0xD5,0x15,
0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,0x11,0xD1,0xD0,0x10,0xF0,
0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,
0x34,0xF4,0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,0x3B,
0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,
0xEE,0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,
0xE7,0xE6,0x26,0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,
0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,
0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,
0x69,0xA9,0xA8,0x68,0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,
0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,0x77,0xB7,
0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,0x50,0x90,0x91,
0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,
0x9C,0x5C,0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,
0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,
0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,0x44,0x84,0x85,0x45,0x87,0x47,0x46,
0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
     };



__no_init unsigned char simka; //dobavka



 struct
 {
 unsigned char  clr_wdt     :1;
 unsigned char  fl_out      :1;
 unsigned char  from_timer0 :1;
 unsigned char  fl_main     :1;
 unsigned char  from_timer3 :1;
 unsigned char  from_timer2 :1; 
 
 }fl_wdt;



extern struct                   //структура, описывающая объект передачи по PPP 
  {  
  enum bool prozr;      //версия       
  enum t_version version;      //версия 
  enum t_type   type_pac;     //тип пакета
  uint16_t num_src; // номер отправителя
  uint16_t num_dst; // номер получателя
  unsigned char id_pac; // идентификатор пакета  
  unsigned char * p_opt; // //указатель буфера опций  
  unsigned char l_opt;// длина буфера опций   
  unsigned char kol_opt;// количество опций   
  unsigned char * p_data;  //указатель буфера данных
  uint16_t l_data;   // длина данных
  }Obj_ppp_tx;

unsigned char buf_opt_tr[20]; 
uint16_t prov_ozu;


unsigned char buf_tx_232[VOL_TX_PPP];
uint16_t count_tx_ppp,vol_tx_ppp;

 //структуры буферов на прием
  struct
 {
  unsigned char data[VOL_RX_PPP];//сам буфер
  uint16_t ln_data;  // длина данных
  enum bool rec; // пакет принят
  enum bool busy;// буфер занят
  enum bool check_busy;// проверка занят ли буфер
 }Buf1_rx_ppp,Buf2_rx_ppp; // 
 
enum bool fl_cts_232_ignor;
uint16_t cnt_cts_off;
struct
 {
 unsigned char on: 1;
 } fl_cts_232;


extern char ip_change;

//unsigned char count_block_net,byte_state_net;

//!!!!!!!!!!!!!!!!1параметры контроллера
unsigned char ip_ls[4]; 
uint16_t num_self,num_seq_cl,port_udp;



struct                   //структура, описывающая объект передачи по PPP 
  {  
  enum bool link_no ; //связи нет
  enum bool link_waits; //ожидание квитка на контроль связи
  enum bool link;      //контроль связи    
  enum bool nat;      //контроль nat 
  uint16_t cnt_link; // счетчик
  uint16_t cnt_nat; // счетчик
  uint16_t vol_link; // значение  
  uint16_t vol_nat; // //значение 
  uint16_t vol_waits; // значение времени квитка ожидания контроля связи
  unsigned char cnt_try; // счетчик попыток получить квиток
  unsigned char vol_try; // rjkbxtcndj попыток получить квиток
  uint16_t vol_nat_r; // счетчик
  }Control;


 uint32_t vol_cnt_no_link, vol_cnt_link_res, cnt_no_link, cnt_link_res;; //dobavka


  unsigned char Appl_RS485_1_data_buf[LN_BUF_485_1];// сам буфер
  unsigned char Appl_RS485_2_data_buf[LN_BUF_485_2];// сам буфер
  unsigned char Appl_RS232_2_data_buf[LN_BUF_232_2];// сам буфер
 
    struct                   //структура описывающая работу приложения "RS485_1" 
  {  
  enum bool fl_data_buf; //имеется ли забуферизированный пакет для передачи в порт RS485_1
  uint16_t ln_data_buf; // забуферизированная длина буфера
  uint16_t dst_buf; // забуферизированный получатель
  unsigned char cont_buf[8];// забуферизированный контекст
  unsigned char id_buf; // забуферизированный id
  uint16_t dst_tek;// текущий получатель
  unsigned char id_tek; // текущий id
  uint16_t pre_tx;//задержка перед передачей
  }Appl_RS485_1,Appl_RS485_2,Appl_RS232_2;

  
     struct
 {
 unsigned char mon1 : 1;
 unsigned char mon2 : 1;
 unsigned char mon232_2 : 1;
 } fl_tx485;
 
extern struct
 {
 unsigned char busy : 1;
 unsigned char rec : 1;
 unsigned char tm_out : 1;
 unsigned char tx : 1;
 unsigned char over : 1;
 unsigned char buffed : 1;
 }fl_485_1,fl_485_2,fl_232_2;

 
///////////////////EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!структуры приложения
 struct
{
unsigned char id;
uint16_t p_in;// указатель свободной ячейки
uint16_t p_out;
uint16_t p_out_kv;
uint16_t crc;
uint16_t cnt_waits; // счетчик ожидания квитка
unsigned char cnt_try; // счетчик попыток получить квиток 
uint16_t l_data; // длина посылаемых данных
unsigned char state;
unsigned char event;
uint16_t tm_vzat;
uint16_t vol_tm_vzat;
enum bool cntr_cl;
enum bool en_cntr_cl;
uint16_t cnt_cntr_cl;
uint16_t vol_cntr_cl;
unsigned char type_arhiv;          // признак какой архив мы асинхронно передаем
}Appl_seq;

unsigned char Appl_seq_des;

unsigned char state_seq;

unsigned char Appl_seq_buf[MAX_BUF_SEQ]; 


struct
{
unsigned char en_tx :1;
unsigned char kv_waits :1;
unsigned char en_povtor:1;
}fl_appl_seq;

struct
{
unsigned char over_buf :1;
unsigned char send_state :1;
unsigned char enable:1; 
}fl_appl_seq1; 



struct
{
  unsigned char tc:1;
  unsigned char tc_old :1;
}sv1,sv2;
  //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!для РРР
extern  struct
 {
 unsigned char fl_7e : 1;//
 unsigned char fl_7d : 1;//
 unsigned char switcher : 1;//переключатель буферов
 } fl_rx_ppp;


unsigned char layer_PPP;

extern struct
 {
  unsigned char act_lcp_end :1;  
  unsigned char lcp_tm_out_en :1;
  unsigned char t0_pl:1;
  unsigned char t0_mi:1;
 }fl_lcp; 
 


extern struct
 {
  unsigned char act_ipcp_end :1;  
  unsigned char ipcp_tm_out_en :1;
  unsigned char t0_pl:1;
  unsigned char t0_mi:1;
 }fl_ipcp; 


extern struct
 {
  unsigned char act_ip_end :1; 
  unsigned char ip_tm_cntr_en :1; 
  unsigned char ip_tm_nat_en :1;
 }fl_ip;



  
extern struct
 {
  unsigned char act_pap_end :1;  
  unsigned char pap_tm_out_en:1;
  unsigned char t0_pl:1;
  unsigned char t0_mi:1;  
  unsigned char up: 1; 
 }fl_pap; 


struct
 {
  unsigned char add_byte :1; 
 }fl_reg3;


 extern struct
 {
  unsigned char up :1;  
  unsigned char down :1;
 }fl_lcp1; 


extern struct
 {
  unsigned char up :1;  
  unsigned char down :1;
 }fl_ipcp1;
 
 extern struct
 {
  unsigned char nac_rej :1; // reject - 1 
 }fl_lcp2;
   
 extern struct
 {
  unsigned char nac_rej :1; // reject - 1 
 }fl_ipcp2;
 

 
 
extern struct
{
char buf[LN_BUF_AT];
unsigned char cnt_tx;
unsigned char cnt_rx;
unsigned char ln_buf;
unsigned char list_com[VOL_LIST];// перечень исполняемых команд
unsigned char ln_list;//длина перечня
unsigned char cnt_com;//счетчик команд
uint16_t cnt_tm_out; //счетчик времени ожидания ответа
uint16_t vol_tm_out; //предел времени ожидания ответа
uint16_t cnt_rx_out; //счетчик межбайтовый промежуток
uint16_t vol_rx_out; //предел межбайтового промежутка
}At_com;




extern struct
{
unsigned char ok :1;//требуемый ответ
unsigned char err :1;//ошибочный, нетребуемый ответ
unsigned char tm_out :1;//отсутствие ответа
unsigned char tx_en :1;//послать комманду
unsigned char rx_en :1;//принимать ответы
unsigned char rx_rec :1;//принят ответ
//unsigned char tm_out_en:1;//разрешение анализа по превышению времени ожидания ответа
}
fl_at_com;
 
extern unsigned char pap_act_buf,PAP_state,PAP_event;
extern unsigned char LCP_state, LCP_event,LCP_act_buf[LCP_ACT_VOL];
extern unsigned char IPCP_state, IPCP_event,IPCP_act_buf[IPCP_ACT_VOL];
extern unsigned char uk_in_act_LCP,uk_out_act_LCP,uk_in_act_IPCP,uk_out_act_IPCP;

extern unsigned char cnt_rst_lcp,vol_cnt_rst_lcp;
extern unsigned char cnt_rst_ipcp,vol_cnt_rst_ipcp;                       
extern unsigned char cnt_rst_pap,vol_cnt_rst_pap;       
                      
                        
extern uint16_t cnt_lcp_tm_out,cnt_ipcp_tm_out;
extern uint16_t cnt_pap_tm_out;
extern uint32_t cnt_ip_tm_cntr;
extern unsigned char rcvd_protokol;

extern unsigned char  i_scr_lcp,i_scan_lcp;
extern unsigned char  i_scj_lcp;//идентификаторы lcp
extern unsigned char  i_str_lcp,i_sta_lcp;
 
extern unsigned char  i_scr_ipcp,i_scan_ipcp;
extern unsigned char  i_scj_ipcp;//идентификаторы ipcp
extern unsigned char  i_str_ipcp,i_sta_ipcp; 
 
  
extern unsigned char  i_scr_pap;
 
 
 unsigned char ip_self[4];
  //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

 
 
 extern unsigned char Rs485_1_buf_rx_tx[MAX_BUF_RS485_1];
 extern unsigned char Rs485_2_buf_rx_tx[MAX_BUF_RS485_2];
 extern unsigned char Rs232_2_buf_rx_tx[MAX_BUF_RS232_2];
 
 
  extern struct                   //структура описывающая работу порта "RS485_1" 
  {
  uint16_t  cnt_bt_rx_tx;       // счетчик байтов на прием-передачу
  uint16_t  cnt_tm_tx_out;    // счетчик времени на удержание rts после передачи
  uint16_t  cnt_tm_pre_tx;    // счетчик времени на удержание rts перед передачи
  uint16_t  vol_tm_tx_out;    // предел счетчик времени на удержание rts после передачи
  uint16_t  cnt_tm_rx_out;   //  счетчик времени на определение конца приема
  uint16_t  vol_tm_rx_out;   //  предел времени на определение конца приема
  uint16_t  cnt_tm_out;     // счетчик времени на прием 
  uint16_t  vol_tm_out;     // предел счетчика времени на прием 
  unsigned char *p_data485;   // указатель на буфер передачи
  }Rs485_1,Rs485_2,Rs232_2;
 
 

 
 extern unsigned char state_led_md;
 

uint32_t summa_temperatura;         
uint16_t temperatura;
uint16_t count_summa_temperatura; 
 
 
uint32_t cnt_outcom,cnt_incom;
unsigned char time_cnt_ch_time;
struct
 {  
  unsigned char wr :1;
 }fl_truffic;


void CopyMass(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);
 
unsigned char sel_modul,count_ppp_down;
 
// что то пошло не так
// надо обнулить все переменные длястека протоколов
// переинициализировать СОМ порт
// прерывания должны иметь один приоритет
 void DESI_(void)
 {
   HAL_NVIC_DisableIRQ(SysTick_IRQn);
   HAL_NVIC_DisableIRQ(USART3_IRQn); 

 }
 
  void ENI_(void)
 {
   HAL_NVIC_EnableIRQ(SysTick_IRQn);
   HAL_NVIC_EnableIRQ(USART3_IRQn); 

 }
 

/*
надо разбить на два куска
обнуление переменных задачи
переинициализация порта

*/

void clr_all_stack(void)
{
  /*
 HAL_NVIC_DisableIRQ(USART3_IRQn); 
 HAL_UART_DeInit(&huart3);  
  */
Control.link_no =FALSE; //связи нет
Control.link_waits=FALSE; //ожидание квитка на контроль связи
Control.link=FALSE;      //контроль связи    
Control.nat=FALSE;      //контроль nat 
Control.cnt_link=0; // счетчик
Control.cnt_nat=0; // счетчик
Control.vol_link=0; // значение  
Control.vol_nat=0; // //значение 
Control.vol_waits=0; // значение времени квитка ожидания контроля связи
Control.cnt_try=0; // счетчик попыток получить квиток
Control.vol_try=0; // rjkbxtcndj попыток получить квиток
Control.vol_nat_r=0; // счетчик
  


fl_wdt.clr_wdt=0;
fl_wdt.fl_out=0;
fl_wdt.from_timer0=0;
fl_wdt.fl_main=0;
fl_wdt.from_timer3=0;
fl_wdt.from_timer2=0; 
 
 



fl_rewrite.cnt_reset=0;
cnt_stm_tm1=0;
cnt_stm_tm2=0;
Control.cnt_link=0;
fl_ip.ip_tm_cntr_en=0;
ip_change=0;
fl_appl_seq1.enable=0;
Control.link=FALSE; 
Control.link_waits=FALSE;
Control.cnt_try=0;
fl_ip.ip_tm_nat_en=0;

Control.cnt_nat=0;

Control.nat=FALSE;
cnt_supervosor=0;
time_cnt_ch_time=0;
cnt_clr_wdt=0;
cnt_flag_out=0;
fl_wdt.fl_out=0;
fl_cts_232.on=0;

fl_cts_232_ignor=FALSE;
count_tx_ppp=0;

cnt_no_link=0;
cnt_link_res=0;

cnt_cts_off=0;

Buf1_rx_ppp.check_busy=FALSE;
Buf1_rx_ppp.busy=FALSE;
Buf1_rx_ppp.rec=FALSE;

Buf2_rx_ppp.check_busy=FALSE;
Buf2_rx_ppp.busy=FALSE;
Buf2_rx_ppp.rec=FALSE;
/*
__HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);
__HAL_UART_DISABLE_IT(&huart3,UART_IT_TXE);

 __HAL_UART_FLUSH_DRREGISTER(&huart3);
      __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
*/

At_com.cnt_tm_out=0;
At_com.cnt_com=0;

At_com.cnt_rx_out=0;
     fl_at_com.tm_out=0;
     fl_at_com.rx_rec=0;
     fl_at_com.rx_en=0;
     fl_at_com.err=0;
     fl_at_com.ok=0;
     
     At_com.cnt_rx_out=0;
     
    cnt_lcp_tm_out=0;
    fl_lcp.lcp_tm_out_en=0;
    
    fl_lcp.t0_pl=0;
    
     fl_pap.pap_tm_out_en=0;
     cnt_pap_tm_out=0;
     fl_pap.t0_pl=0;
     
     fl_ipcp.ipcp_tm_out_en=0;
     cnt_ipcp_tm_out=0;
     fl_ipcp.t0_pl=0;
     
     event_modem=EVM_NONE;
     
   
    
    command_AT=FALSE;
    
   At_com.cnt_rx=0;
   
   fl_rx_ppp.switcher=0;
   
   fl_cts_232_ignor=FALSE;
   
   cnt_outcom=0;
   cnt_incom=0;
   
   count_tx_ppp=0;
   At_com.cnt_tx=0;
   fl_reg3.add_byte=0;
   
    fl_rx_ppp.fl_7e=0;
 fl_rx_ppp.fl_7d=0;
 fl_rx_ppp.switcher=0;
 
  fl_lcp.act_lcp_end=0;
  fl_lcp.lcp_tm_out_en=0;
  fl_lcp.t0_pl=0;
  fl_lcp.t0_mi=0;

  
   fl_ipcp.act_ipcp_end=0; 
  fl_ipcp.ipcp_tm_out_en=0;
  fl_ipcp.t0_pl=0;
  fl_ipcp.t0_mi=0;
 
  
  fl_ip.act_ip_end=0;
  fl_ip.ip_tm_cntr_en=0;
  fl_ip.ip_tm_nat_en=0;
  
  
  fl_pap.act_pap_end=0; 
  fl_pap.pap_tm_out_en=0;
  fl_pap.t0_pl=0;
  fl_pap.t0_mi=0;
  fl_pap.up=0;
 
 fl_lcp1.up=0; 
 fl_lcp1.down=0;
 
 fl_ipcp1.up=0; 
 fl_ipcp1.down=0;
 
 fl_lcp2.nac_rej=0;
 fl_ipcp2.nac_rej=0;
 
 
 
 pap_act_buf=0;PAP_state=0;PAP_event=0;
 LCP_state=0; LCP_event=0;
 IPCP_state=0; IPCP_event=0;
 uk_in_act_LCP=0;uk_out_act_LCP=0;uk_in_act_IPCP=0;uk_out_act_IPCP=0;
 
 cnt_rst_lcp=0;vol_cnt_rst_lcp=0;
 cnt_rst_ipcp=0;vol_cnt_rst_ipcp=0;                       
cnt_rst_pap=0;vol_cnt_rst_pap=0;

                       
cnt_lcp_tm_out=0;cnt_ipcp_tm_out=0;
cnt_pap_tm_out=0;
cnt_ip_tm_cntr=0;
rcvd_protokol=0;


i_scr_lcp=0;i_scan_lcp=0;
i_scj_lcp=0;//идентификаторы lcp
i_str_lcp=0;i_sta_lcp=0;


i_scr_ipcp=0;i_scan_ipcp=0;
i_scj_ipcp=0;//идентификаторы ipcp
i_str_ipcp=0;i_sta_ipcp=0; 
 
  
i_scr_pap=0;



At_com.cnt_tx=0;
At_com.cnt_rx=0;

At_com.cnt_com=0; //счетчик команд
At_com.cnt_tm_out=0; //счетчик времени ожидания ответа
At_com.vol_tm_out=0; //предел времени ожидания ответа
At_com.cnt_rx_out=0; //счетчик межбайтовый промежуток
At_com.vol_rx_out=0; //предел межбайтового промежутка


fl_at_com.ok=0; //требуемый ответ
fl_at_com.err=0; //ошибочный, нетребуемый ответ
fl_at_com.tm_out=0; //отсутствие ответа
fl_at_com.tx_en=0; //послать комманду
fl_at_com.rx_en=0; //принимать ответы
fl_at_com.rx_rec=0; //принят ответ



  fl_rewrite.ip=0;
  fl_rewrite.udp=0;
  fl_rewrite.num_self=0;
  fl_rewrite.cnt_reset=0;
  
  compress_off=0;
  /*
  MX_USART3_UART_Init();
  HAL_NVIC_EnableIRQ(USART3_IRQn);
 */

  PreparaeSim800Work();
     
  
  
}

void reset_uart_prep_sim800(void)
{
  
 // HAL_NVIC_DisableIRQ(USART3_IRQn);
   HAL_UART_DeInit(&huart3); 
   MX_USART3_UART_Init();
   HAL_UART_Receive_IT(&huart3,&sim800Data,1); // Каждый раз принимать один байт   
 // HAL_NVIC_EnableIRQ(USART3_IRQn);
  
   
}
 
int16_t crc_sum(unsigned char *ff,unsigned char kol)
{
  int16_t sum_crc;
  char i;
  sum_crc=0;
               
               for (i=0;i<kol;i++)
                                {
                                sum_crc+= *ff;                                
                                *ff++ ;
                                 }     
  
  return(sum_crc);
}

/// эта кфункция будет эквивалентна сбросу задачи модуля ( автомат перевести в исходное состояние)
void lock_it(void)
                {
                  ResetTaskSim800=1;
                  S1_YL();
               
                    }      


/*
void delay_on_contr(void)
{
unsigned char delay_count;

delay_count=18;

S1_RD();


                    
                      do
                          {   
                              __watchdog_reset();
                              delay(60000);
                              delay(60000);
                              delay(60000);
                              delay(60000);
                              delay(60000);
                              __watchdog_reset();
                              S1_CH();
                              
                              delay_count--;
                           } while(delay_count!=0);

S1_OFF();

}

*/

unsigned char crc_485(unsigned char num, unsigned char *p)
  {
  char i,ds_crc1;
    ds_crc1=0;
               for (i=0;i<num;i++)
                                {
                                ds_crc1=tab_crc[ds_crc1 ^ *p];
                                p++;
                                //ds_crc=tuta();
                                 }
  return(ds_crc1);
         }


 uint16_t crc_m1(unsigned char *ka,uint16_t num,uint16_t crc)   

     {
      unsigned char uchCRCHi,uIndex;
      unsigned char uchCRCLo;
      uint16_t ii;      
      uchCRCLo=crc;  
      uchCRCHi=crc>>8;
   
      
                   for (ii=0;ii<num;ii++)
                  {
    
       uIndex = uchCRCHi ^ *ka++;

       uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];

       uchCRCLo = auchCRCLo[uIndex];
       
      }

     return (uchCRCHi << 8  | uchCRCLo);

     }



unsigned char check_cts(void)
{ 
 // если линия CTS в пассиве возвращает 1  
  // fl_cts_232.on=1; признак линия в пассиве
  //
  // для версии 3 вольтовой все инверсно 0 -это активный уровень !!!
  //
  
  
     if(HAL_GPIO_ReadPin(GPIOD, CTS0))                // для 3 х вольтовой версии
 //  if(HAL_GPIO_ReadPin(DIO_PORTE, CTS0)==0)
 // if ((PINE & CTS0)==0)     
  {
  fl_cts_232.on=1;
  return(1);
  } //proverka CTS
  return(0);  
}  








uint16_t crc_ozu(void)
   

     {
      unsigned char uchCRCHi;
      unsigned char uchCRCLo;
      unsigned char temp;
      
      uchCRCLo=0xff;  
      uchCRCHi=0xff;
         
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ ip_ls[0]];
      uchCRCLo = auchCRCLo[uchCRCHi ^ ip_ls[0]];
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ ip_ls[1]];
      uchCRCLo = auchCRCLo[uchCRCHi ^ ip_ls[1]];
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ ip_ls[2]];
      uchCRCLo = auchCRCLo[uchCRCHi ^ ip_ls[2]];
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ ip_ls[3]];
      uchCRCLo = auchCRCLo[uchCRCHi ^ ip_ls[3]];
      
      temp=port_udp;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=port_udp>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=num_self;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=num_self>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=Control.vol_nat;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=Control.vol_nat>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=Control.vol_nat_r;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=Control.vol_nat_r>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=Control.vol_link;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=Control.vol_link>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=Control.vol_waits;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=Control.vol_waits>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=Control.vol_try;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=Control.vol_try>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      
      //!!!!!!!!!!!!!!!!!! dobavka
      temp=vol_cnt_no_link;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=vol_cnt_no_link>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=vol_cnt_no_link>>16;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=vol_cnt_no_link>>24;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=vol_cnt_link_res;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=vol_cnt_link_res>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=vol_cnt_link_res>>16;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=vol_cnt_link_res>>24;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      //EEEEEEEEEEEEEEEEEEEEEEEEE dobavka
      
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ Appl_seq_des];
      uchCRCLo = auchCRCLo[uchCRCHi ^ Appl_seq_des];
      
     
      
      
      temp=num_seq_cl;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=num_seq_cl>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      temp=Appl_seq.vol_tm_vzat;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=Appl_seq.vol_tm_vzat>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
     
      temp=Appl_seq.vol_cntr_cl;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      temp=Appl_seq.vol_cntr_cl>>8;
      uchCRCHi = uchCRCLo ^ auchCRCHi[uchCRCHi ^ temp];
      uchCRCLo = auchCRCLo[uchCRCHi ^ temp];
      
      return (uchCRCHi << 8  | uchCRCLo);

     } 






void load_par_from_memory_no(void)
{
unsigned char i, buf[20];

for(i=0;i<(L_KEYS-2);i++) keys[i]=keys_def[i];
  

framRead(A_IP_PAR,&buf[0],L_IP_PAR);

  {  
  
   ip_ls[0]=buf[OFS_IP];
   ip_ls[1]=buf[OFS_IP+1];
   ip_ls[2]=buf[OFS_IP+2];
   ip_ls[3]=buf[OFS_IP+3];
   port_udp=*(uint16_t*)&buf[OFS_PORT];
   num_self=*(uint16_t*)&buf[OFS_NUM];
  }


  
framRead(A_C_PAR,&buf[0],L_C_PAR);

  {
   
   Control.vol_nat=*(uint16_t*)&buf[OFS_NAT];
   Control.vol_link=*(uint16_t*)&buf[OFS_CCH];
   Control.vol_waits=DEF_CNTR_LINK_WAITS;
   Control.vol_try=*(uint16_t*)&buf[OFS_TM_CH]/Control.vol_waits;
   
   vol_cnt_no_link=*(uint16_t*)&buf[OFS_TM_NO_LINK]*60;   //dobavka
   vol_cnt_link_res=*(uint16_t*)&buf[OFS_TM_LINK_RES]*60; //dobavka
   
   Control.vol_nat_r=*(uint16_t*)&buf[OFS_NAT_R];
  }


// для настройки асинхронных массивов
framRead(A_SEQ_PAR,&buf[0],L_SEQ_PAR);

  {
 
  Appl_seq_des=buf[OFS_DES_SEQ];
  //Appl_seq_des=1; //пока запрет
  num_seq_cl=*(uint16_t*)&buf[OFS_NUM_CL];
  Appl_seq.vol_tm_vzat=*(uint16_t*)&buf[OFS_TM_VZ];
  Appl_seq.vol_cntr_cl=*(uint16_t*)&buf[OFS_TM_CL];
  
  }
 
 

framRead(A_TRAF,&buf[0],L_TRAF);
if(*(uint16_t*)&buf[L_TRAF-2]!=crc_m1(&buf[0],L_TRAF-2,0xffff))
  {      
   *(uint32_t*)&buf[0]=burst_ds_r();
   *(uint32_t*)&buf[4]=0;
   *(uint32_t*)&buf[8]=0;
   *(uint16_t*)&buf[L_TRAF-2]=crc_m1(&buf[0],L_TRAF-2,0xffff);
   framWriteSim800(A_TRAF, &buf[0],L_TRAF);
   
   cnt_outcom=0;
   cnt_incom=0;
  }
  else
  {  
  
   cnt_outcom=*(uint32_t*)&buf[4];
   cnt_incom=*(uint32_t*)&buf[8];
  }





}



void load_par_from_memory(void)
{
unsigned char i, buf[20];

for(i=0;i<(L_KEYS-2);i++) keys[i]=keys_def[i];
  

framRead(A_IP_PAR,&buf[0],L_IP_PAR);
if(*(uint16_t*)&buf[OFS_IP_PAR_CRC]!=crc_m1(&buf[0],L_IP_PAR-2,0xffff))
  {
   
   ip_ls[0]=DEF_IP_SRV_0;
   ip_ls[1]=DEF_IP_SRV_1;
   ip_ls[2]=DEF_IP_SRV_2;
   ip_ls[3]=DEF_IP_SRV_3;
   num_self=DEF_NUM_SELF;
   port_udp=DEF_UDP_PORT;
  }
  else
  {  
  
   ip_ls[0]=buf[OFS_IP];
   ip_ls[1]=buf[OFS_IP+1];
   ip_ls[2]=buf[OFS_IP+2];
   ip_ls[3]=buf[OFS_IP+3];
   port_udp=*(uint16_t*)&buf[OFS_PORT];
   num_self=*(uint16_t*)&buf[OFS_NUM];
  }


  
framRead(A_C_PAR,&buf[0],L_C_PAR);
if(*(uint16_t*)&buf[OFS_C_PAR_CRC]!=crc_m1(&buf[0],L_C_PAR-2,0xffff))
  {
  
   Control.vol_nat=DEF_CNTR_NAT;
   Control.vol_nat_r=DEF_CNTR_NAT_R;
   Control.vol_link=DEF_CNTR_LINK;
   Control.vol_waits=DEF_CNTR_LINK_WAITS;
   Control.vol_try=DEF_TM_CNTR_LINK/Control.vol_waits;  
   
   vol_cnt_no_link=DEF_TM_NO_LINK * 60;  //dobavka
   vol_cnt_link_res=DEF_TM_LINK_RES * 60;  //dobavka
   
  }
  else
  {
   
   Control.vol_nat=*(uint16_t*)&buf[OFS_NAT];
   Control.vol_link=*(uint16_t*)&buf[OFS_CCH];
   Control.vol_waits=DEF_CNTR_LINK_WAITS;
   Control.vol_try=*(uint16_t*)&buf[OFS_TM_CH]/Control.vol_waits;
   
   vol_cnt_no_link=*(uint16_t*)&buf[OFS_TM_NO_LINK]*60;   //dobavka
   vol_cnt_link_res=*(uint16_t*)&buf[OFS_TM_LINK_RES]*60; //dobavka
   
   Control.vol_nat_r=*(uint16_t*)&buf[OFS_NAT_R];
  }



framRead(A_SEQ_PAR,&buf[0],L_SEQ_PAR);
if(*(uint16_t*)&buf[OFS_SEQ_PAR_CRC]!=crc_m1(&buf[0],L_SEQ_PAR-2,0xffff))
  {
   
   Appl_seq_des=~DEF_DES_SEQ;
   num_seq_cl=DEF_NUM_DST_SEQ;
   Appl_seq.vol_tm_vzat=DEF_VOL_TM_VZAT;
   Appl_seq.vol_cntr_cl=DEF_TM_CNTR_CL;
   
  }
  else
  {
 
  Appl_seq_des=buf[OFS_DES_SEQ];
  num_seq_cl=*(uint16_t*)&buf[OFS_NUM_CL];
  Appl_seq.vol_tm_vzat=*(uint16_t*)&buf[OFS_TM_VZ];
  Appl_seq.vol_cntr_cl=*(uint16_t*)&buf[OFS_TM_CL];
  
  }
 

framRead(A_TRAF,&buf[0],L_TRAF);
if(*(uint16_t*)&buf[L_TRAF-2]!=crc_m1(&buf[0],L_TRAF-2,0xffff))
  {      
   *(uint32_t*)&buf[0]=burst_ds_r();
   *(uint32_t*)&buf[4]=0;
   *(uint32_t*)&buf[8]=0;
   *(uint16_t*)&buf[L_TRAF-2]=crc_m1(&buf[0],L_TRAF-2,0xffff);
   framWriteSim800(A_TRAF, &buf[0],L_TRAF);
   
   cnt_outcom=0;
   cnt_incom=0;
  }
  else
  {  
  
   cnt_outcom=*(uint32_t*)&buf[4];
   cnt_incom=*(uint32_t*)&buf[8];
  }







//RdFromFleshToArr(A_SEG8,(unsigned char *)&c_config_tc[0],SEG8*2);
//RdFromFleshToArr(A_SEG9,(unsigned char *)&ust_vhod,SEG9*2);
//RdFromFleshToArrInt(A_KD, &c_config_kd[0],2);
//ust_vhod=c_config_kd[AD_TM_VHOD] & 0x1f;
//ust_vhod=ust_vhod*1000;

}







// не понял зачем все эти манипуляции с ногами
// може это связан о спецификой включения выключения питания на SIM800 ?
void init_pins_out_to_hiz(void)
{
  // порт запрещался и ноги RTS DTR  на вход в третье состояние
  CLR_RTS0();
  CLR_DTR0();
}
void init_pins_hiz_to_out(void)
{
  // порт разрещался и ноги RTS DTR  на выход
  SET_RTS0();
  SET_DTR0();
}

void start_func_stm_off(void)
{
  //S2_OFF;//
  
  clr_all_stack(); 
  init_pins_out_to_hiz();
  
//  if(Regim!=RG_DEBAG){S3_OFF;S4_OFF;S5_OFF;}
  
  state_led_md=LED_MD_OFF;
  command_AT=TRUE;// включен коммандный режим
  CLR_PWR();   // исходное состояние
  SET_PWRK();
  state_modem=STM_OFF;
  cnt_stm_tm1=DEF_TM_STM_OFF;
  event_modem=EVM_NONE;
  write_log_info(ST_MODEM,MDM_OFF);
  send_info(sizeof(stm_pwr_off),stm_pwr_off,0,0);
}   



void start_func_stm_koff(void)
{
  
  clr_all_stack(); 
  
  init_pins_out_to_hiz();
  
 
  
  state_led_md=LED_MD_OFF;
  command_AT=TRUE;// включен коммандный режим
  CLR_PWR();  // только через сброс питания
  SET_PWRK();
  state_modem=STM_KOFF;
////  cnt_stm_tm1=DEF_TM_STM_KOFF;
  cnt_stm_tm1=DEF_TM_STM_OFF;
  
  cnt_stm_tm2=DEF_TM2_STM_OFF;
  
 // cnt_key_off=DEF_TM_KEY_OFF;  2016  !!!??
  
  event_modem=EVM_NONE;
  write_log_info(ST_MODEM,MDM_KOFF);
  send_info(sizeof(stm_pwr_koff),stm_pwr_koff,0,0);
}   




void start_func_stm_on(void)
{
  //S2_OFF;//
  
 // if(Regim!=RG_DEBAG){S3_OFF;S4_OFF;S5_OFF;}
 
   
  
   // оставляем только одну симку
       simka=SIM_BASE;
       //SET_SIM1;
       write_log_info(ST_MODEM,BASE_SIM);
       send_info(sizeof(stm_base_sim),stm_base_sim,0,0);   
   
  ///EEEEEEEEEEEE dobavka
  

  state_led_md=LED_MD_NO_CREG;
  command_AT=TRUE;// включен коммандный режим
  SET_RTS0();
  SET_DTR0();
  SET_PWR();
  CLR_PWRK();
  state_modem=STM_ON;
  cnt_stm_tm1=DEF_TM_STM_ON;
  cnt_stm_tm2=DEF_TM2_STM_ON;
  event_modem=EVM_NONE; 
  // write_log_info(ST_MODEM,MDM_ON);
  send_info(sizeof(stm_pwr_on),stm_pwr_on,0,0);    //ot dobavka   
}  

void start_func_stm_kon(void)
{ 
  SET_PWRK();
  state_modem=STM_KON;
  cnt_stm_tm1=DEF_TM_STM_KON;
  event_modem=EVM_NONE; 

}  

void start_func_stm_pwk(void)
{ 
  CLR_PWRK();
  state_modem=STM_PWK;
  cnt_stm_tm1=DEF_TM_STM_PWK;
  event_modem=EVM_NONE;
  send_info(sizeof(stm_pwr_kon),stm_pwr_kon,0,0);
}  



void start_func_stm_scp(void)
{
   unsigned char temp;
   clr_var_ppp_rx();
/*
   
   UCSR0B=UCSR0B & ~RXEN;
   UCSR0B=UCSR0B & ~RXCIE;
    UCSR0A=UCSR0A | TXC;
  */  
 //******************************************************************
  
   
   
 
   command_AT=FALSE;// выключен коммандный режим
 
   clr_var_ppp();
  
   count_tx_ppp=0; 
 // для активации установки соединения PPP 
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  fl_lcp1.up=1;
  fl_lcp1.down=0;
 //конфигурация соединения  
 
  //конфигурация канала
  
  layer_PPP=LAYER_LCP;
  LCP_state=START_ST; 
  
  IPCP_event=NO_EVENT;
  LCP_event=NO_EVENT;
  PAP_event=NO_EVENT;
 
  uk_in_act_IPCP=0;
  uk_out_act_IPCP=0;
  
  uk_in_act_LCP=0;
  uk_out_act_LCP=0;
  
  cnt_lcp_tm_out=VOL_LCP_TM_OUT;
  fl_lcp.act_lcp_end=1;           //действия все закончены
  fl_pap.act_pap_end=1;     
  fl_ipcp.act_ipcp_end=1;
  fl_ip.act_ip_end=1;
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  
  /*
  UCSR0B=UCSR0B | RXEN; 
  temp=UDR0;
  UCSR0B=UCSR0B | RXCIE;  
  */
   
   //   аналог это сброс регистра данных и разрешить прерывния от приема
      __HAL_UART_FLUSH_DRREGISTER(&huart3);
      __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
      
      
        
  cnt_stm_tm1=DEF_TM_STM_SCP; 
  cnt_stm_tm2=DEF_TM2_STM_SCP; 
}     









 //подготовка буфера к отправке по PPP 
 
void form_buf_tx_ppp(void) 
{   
uint16_t i;
unsigned char nopp,j;     
unsigned char *pointer; 
union
 {
 unsigned char bytes[2]; 
 uint16_t word;
 } temp;

 
 buf_tx_232[PPP_BEG]=0x7e;               
 buf_tx_232[PPP_PROT]=0x21;
                  
           //заголовок IP
  buf_tx_232[IP_VLEN]=0x45;               
  buf_tx_232[IP_TYPE]=0;               
  
   
  
  //общая длина IP 
  //20 -IP
  //8 -UDP
  //12 - Заголовок Данных
  //+ Длина буфера опций
  //+ Заполнитель
  //+ Длина данных
  nopp=0;
  
  // Расчет заполнителя
  if(Obj_ppp_tx.prozr==FALSE)
  {
   if((Obj_ppp_tx.l_opt<=4)&&(Obj_ppp_tx.l_opt>0))nopp=4-Obj_ppp_tx.l_opt;   
   if(Obj_ppp_tx.l_opt%4!=0) nopp=4-Obj_ppp_tx.l_opt%4;
   temp.word=20+8+12+Obj_ppp_tx.l_opt+nopp+Obj_ppp_tx.l_data;
  }else temp.word=20+8+Obj_ppp_tx.l_data;
   
  buf_tx_232[IP_LN]=temp.bytes[1]; //длина IP пакета         ///////////////!!!!!!!!!!!!!!!!!      
  buf_tx_232[IP_LN+1]=temp.bytes[0];                         ///////////////!!!!!!!!!!!!!!!!!
      
  buf_tx_232[IP_ID]=0;               
  buf_tx_232[IP_ID+1]=0;
  
  buf_tx_232[IP_FUK]=0;               
  buf_tx_232[IP_FUK+1]=0; 
  buf_tx_232[IP_TIME]=0x80;    
  buf_tx_232[IP_PROT]=0x11;     //UDP
  
 // buf_tx_232[IP_CRC]=0;     
 // buf_tx_232[IP_CRC+1]=0; //IP - CRC
  *(uint16_t*)&buf_tx_232[IP_CRC]=0;
     
  buf_tx_232[IP_SRC]=ip_self[0];
  buf_tx_232[IP_SRC+1]=ip_self[1]; 
  buf_tx_232[IP_SRC+2]=ip_self[2];    
  buf_tx_232[IP_SRC+3]=ip_self[3];
  
  buf_tx_232[IP_DST]=ip_ls[0]; 
  buf_tx_232[IP_DST+1]=ip_ls[1];    
  buf_tx_232[IP_DST+2]=ip_ls[2]; 
  buf_tx_232[IP_DST+3]=ip_ls[3];    
                                
 // buf_tx_232[18]=read_eeprom(ADR_IP_SRV);
 // buf_tx_232[19]=read_eeprom(ADR_IP_SRV+1);
 // buf_tx_232[20]=read_eeprom(ADR_IP_SRV+2);
  //buf_tx_232[21]=read_eeprom(ADR_IP_SRV+3);
                 
                                 
 //заголовок UDP
  //buf_tx_232[22]=UDP_PORT_H;
  //buf_tx_232[23]=UDP_PORT_L;
  //buf_tx_232[24]=UDP_PORT_H;
  //buf_tx_232[25]=UDP_PORT_L;       
  /*
  buf_tx_232[UDP_PSRC]=read_eeprom(ADR_UDP_PORT_HI);
  buf_tx_232[UDP_PSRC+1]=read_eeprom(ADR_UDP_PORT_LO);
  buf_tx_232[UDP_PDST]=read_eeprom(ADR_UDP_PORT_HI);
  buf_tx_232[UDP_PDST+1]=read_eeprom(ADR_UDP_PORT_LO);
  */
  buf_tx_232[UDP_PSRC]=port_udp>>8;
  buf_tx_232[UDP_PSRC+1]=port_udp;
  buf_tx_232[UDP_PDST]=port_udp>>8;
  buf_tx_232[UDP_PDST+1]=port_udp;
  
  
    //общая длина UDP 
  
 // 8 -UDP
 // 12 - Заголовок Данных
 // + Длина буфера опций
 // + Заполнитель
 // + Длина данных
   
 
   temp.word=temp.word-20;
  
  buf_tx_232[UDP_LN]=temp.bytes[1]; ///////////////длина UDP 
  buf_tx_232[UDP_LN+1]=temp.bytes[0];        ////////////   
    
  //buf_tx_232[UDP_CRC]=0;        //СRС UDP пакета
  //buf_tx_232[UDP_CRC+1]=0;
  *(uint16_t*)&buf_tx_232[UDP_CRC]=0; 
    
 if(Obj_ppp_tx.prozr==FALSE)
  {  
 //Транспортный заголовок
 //buf_tx_232[30]=0;        //CRC
 //buf_tx_232[31]=0; 
 //*(uint16_t*)&buf_tx_232[TR_CRC]=0;
                             
 temp.word=temp.word-8;                             
 //buf_tx_232[32]=temp.bytes[1];        //полная длина
 //buf_tx_232[33]=temp.bytes[0];                 
 *(uint16_t*)&buf_tx_232[TR_LEN]=temp.word;
 
// buf_tx_232[34]=0;        //шифр
// buf_tx_232[35]=0;              
 
  *(uint16_t*)&buf_tx_232[TR_KOD]=0;        
          
 buf_tx_232[TR_V]=Obj_ppp_tx.version;                                     //версия
 buf_tx_232[TR_V]=(buf_tx_232[TR_V]<<2) | Obj_ppp_tx.type_pac;              // тип
 buf_tx_232[TR_V]=(buf_tx_232[TR_V]<<4) | ((12+Obj_ppp_tx.l_opt+nopp)/4);   // длина траспортного заголовка
    
 
 buf_tx_232[TR_ID]=Obj_ppp_tx.id_pac; 
 
   *(uint16_t*)&buf_tx_232[TR_SRC]=Obj_ppp_tx.num_src;   //номер отправителя
   *(uint16_t*)&buf_tx_232[TR_DST]=Obj_ppp_tx.num_dst;   //номер получателя 
 
           
 pointer=&buf_tx_232[TR_OP_DATA];         
  
 //Забивка опции и заполнителя
 if(Obj_ppp_tx.l_opt!=0)
 {
  for(j=0;j<Obj_ppp_tx.l_opt;j++)
      {
       *pointer=*Obj_ppp_tx.p_opt;
       pointer++;
       Obj_ppp_tx.p_opt++;
      }   
  if(nopp!=0)
     for(j=0;j<nopp;j++)
        {
        *pointer=NOPPER;
        pointer++; 
        }     
 }
   
 //Забивка данных   
 for(i=0;i<Obj_ppp_tx.l_data;i++)
     {
       *pointer=*Obj_ppp_tx.p_data;
       pointer++;
       Obj_ppp_tx.p_data++;
      }   
 
   //crc
  *(uint16_t*)&buf_tx_232[TR_KOD]=0;
 
  *(uint16_t*)&buf_tx_232[TR_CRC]=0;
  *(uint16_t*)&buf_tx_232[TR_CRC]=crc_m1(&buf_tx_232[TR_SRC],*(uint16_t*)&buf_tx_232[TR_LEN],0xffff);
 

   kodirovka(*(uint16_t*)&buf_tx_232[TR_LEN]);
  
   
  }
  else 
  {
  
  pointer=&buf_tx_232[TR_SRC];   
  for(i=0;i<Obj_ppp_tx.l_data;i++)
     {
       *pointer=*Obj_ppp_tx.p_data;
       pointer++;
       Obj_ppp_tx.p_data++;
      }   
   } 
    
   
   
 // temp.word=calc_crc_ip(&buf_tx_232[2],20);
 // buf_tx_232[12]=temp.bytes[0]; 
 // buf_tx_232[13]=temp.bytes[1];  
  
 
  
  
 
  *(uint16_t*)&buf_tx_232[IP_CRC]=calc_crc_ip(&buf_tx_232[IP_VLEN],20); 
  

  temp.bytes[1]=buf_tx_232[UDP_LN]; ///////////////длина UDP 
  temp.bytes[0]=buf_tx_232[UDP_LN+1];    
  
 // *(uint16_t*)&buf_tx_232[UDP_CRC]=calc_crc_udp(&buf_tx_232[UDP_PSRC],temp.word);
  *(uint16_t*)&buf_tx_232[UDP_CRC]=calc_crc_udp_2(&buf_tx_232[0],temp.word);

  
  i=buf_tx_232[IP_LN];
  i=(i<<8)+buf_tx_232[IP_LN+1];  //длина IP пакета         ///////////////!!!!!!!!!!!!!!!!!  
  i=i+5;                  //длина PPP пакета         ///////////////!!!!!!!!!!!!!!!!! 
 
  vol_tx_ppp=i;
  temp.word=vol_tx_ppp-4;
 
  if(compress_off==1)
                 {
                   for (i=0;i < (vol_tx_ppp-1);i++)buf_tx_232[vol_tx_ppp+2-i]=buf_tx_232[vol_tx_ppp-1-i];
                   buf_tx_232[1]=0xff;
                   buf_tx_232[2]=0x03;
                   buf_tx_232[3]=0x00;
                  // temp.word=temp.word+2;
                   temp.word=temp.word+3;
                   vol_tx_ppp=temp.word+4;
                 }
  
  temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;
  buf_tx_232[vol_tx_ppp-3]=temp.bytes[0];
  buf_tx_232[vol_tx_ppp-2]=temp.bytes[1];                 
  buf_tx_232[vol_tx_ppp-1]=0x7e;  
  count_tx_ppp=0;
  
  
  /* 
  
  temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[PPP_PROT],i-4) ^ 0xffff;    // //длина IP+1 т.е   
  buf_tx_232[i-3]=temp.bytes[0];
  buf_tx_232[i-2]=temp.bytes[1];                 
  buf_tx_232[i-1]=0x7e;  
  
  vol_tx_ppp=i;
  count_tx_ppp=0;                
 */ 
   
  
  /*
  
                 temp.word=temp.word+2;
                 vol_tx_ppp=temp.word+4;
                  
                 if(compress_off==1)
                 {
                   for (k=0;k < (vol_tx_ppp-1);k++)buf_tx_232[vol_tx_ppp+1-k]=buf_tx_232[vol_tx_ppp-1-k];
                   buf_tx_232[1]=0xff;
                   buf_tx_232[2]=0x03;
                   temp.word=temp.word+2;
                   vol_tx_ppp=temp.word+4;
                 }
                 
                  

                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;                  
                 buf_tx_232[vol_tx_ppp-3]=temp.bytes[0];
                 buf_tx_232[vol_tx_ppp-2]=temp.bytes[1];
                 
                 buf_tx_232[vol_tx_ppp-1]=0x7e; 
  */
  
  
  
} 














  void clr_cntr_link_2(void)
 {
 Control.link_waits=FALSE;
 Control.link=FALSE;
 Control.cnt_link=Control.vol_link;
 Control.cnt_try=0;
 Control.link_no=FALSE;
 } 
 
 
 
 void clr_cntr_link(void)
 {
 if(Control.link_waits==FALSE)
 {
 cnt_no_link=vol_cnt_no_link;//dobavka
 Control.link=FALSE;
 Control.cnt_link=Control.vol_link;
 }
 } 
  
  


/*
void monitor_rst(void)
{
unsigned char buf[9];


time_cnt_ch_time=CHECK_TIME;

if((MCUSR & 0x01)==0x01)
{


simka=SIM_BASE; //dobavka
//SET_SIM1;
  
send_info(sizeof(evc_pwr),evc_pwr,0,0); 
write_log_info(ST_PIT_ON,RESET_POWER);

if (Appl_seq_des==1){MCUSR=0;return;}// блокировка 

buf[0]=COOL_RESET;
*(uint16_t*)&buf[1]=L_COOL_RESET;
*(uint32_t*)&buf[3]=burst_ds_r();




if(write_to_buf(&Appl_seq_buf[0],&Appl_seq.p_in,&Appl_seq.p_out,&Appl_seq.p_out_kv,&Appl_seq.crc,&buf[0],L_COOL_RESET+3, sizeof(Appl_seq_buf))==1)
   {
   fl_appl_seq1.over_buf=1;
   }

}
else
{

  
  
if(simka>1)  {simka=SIM_BASE;
//SET_SIM1;
}  //dobavka
  
if((MCUSR & 0x02)==0x02){write_log_info(ST_PIT_ON,RESET_EXT);send_info(sizeof(evc_rst),evc_rst,0,0);}
if((MCUSR & 0x04)==0x04){write_log_info(ST_PIT_ON,RESET_BROWN);send_info(sizeof(evc_bod),evc_bod,0,0);}
if((MCUSR & 0x08)==0x08){write_log_info(ST_PIT_ON,RESET_WDR);send_info(sizeof(evc_wdr),evc_wdr,0,0);}

if (Appl_seq_des==1){MCUSR=0;return;}// блокировка 

buf[0]=HOT_RESET;
*(uint16_t*)&buf[1]=L_HOT_RESET;
*(uint32_t*)&buf[3]=burst_ds_r();
if((MCUSR & 0x02)==0x02)buf[7]=RST_EXT;
if((MCUSR & 0x04)==0x04)buf[7]=RST_BROWN;
if((MCUSR & 0x08)==0x08)buf[7]=RST_WDR;
buf[8]=0;
if(write_to_buf(&Appl_seq_buf[0],&Appl_seq.p_in,&Appl_seq.p_out,&Appl_seq.p_out_kv,&Appl_seq.crc,&buf[0],L_HOT_RESET+3, sizeof(Appl_seq_buf))==1)
   {
   fl_appl_seq1.over_buf=1;
   }
}

MCUSR=0;
} 


*/

/*
void monitor_over_net(void)
   {        
     unsigned char buf[9];
    
     if (Appl_seq_des==1)return;// блокировка опроса охранки     
     if(count_block_net!=0)return;//блокировка монитора   
     if (((PINA & TCC2)&&(byte_state_net==YES_NET))|| (((PINA & TCC2)==0)&&(byte_state_net==NO_NET)))
          {
            if(byte_state_net==YES_NET) {byte_state_net=NO_NET;send_info(sizeof(evc_feed_akb),evc_feed_akb,0,0);}
            else {byte_state_net=YES_NET;send_info(sizeof(evc_feed_220),evc_feed_220,0,0);}
            count_block_net=3;
            buf[0]=AKB_SET;
            *(uint16_t*)&buf[1]=L_AKB_SET;
            *(uint32_t*)&buf[3]=burst_ds_r();
            buf[7]=byte_state_net;
            buf[8]=0; 
            if(write_to_buf(&Appl_seq_buf[0],&Appl_seq.p_in,&Appl_seq.p_out,&Appl_seq.p_out_kv,&Appl_seq.crc,&buf[0],L_AKB_SET+3, sizeof(Appl_seq_buf))==1)
                         {
                         fl_appl_seq1.over_buf=1;
                         } 
           
          }
   
   }

*/


 
 


void clr_cntr_nat(void)
 {
 Control.nat=FALSE;
 Control.cnt_nat=Control.vol_nat;
 //if(simka==SIM_RES)Control.cnt_nat=Control.vol_nat_r;else Control.cnt_nat=Control.vol_nat;
 } 


void clr_var_ppp_rx(void)
{
// UCSR0B=UCSR0B & ~RXEN;
// UCSR0B=UCSR0B & ~RXCIE;
 
__HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);
 
 fl_rx_ppp.switcher=0;
 fl_rx_ppp.fl_7e=0;
 fl_rx_ppp.fl_7d=0;
 Buf1_rx_ppp.ln_data=0;
 Buf1_rx_ppp.rec=FALSE;
 Buf1_rx_ppp.busy=FALSE;
 Buf2_rx_ppp.ln_data=0;
 Buf2_rx_ppp.rec=FALSE;
 Buf2_rx_ppp.busy=FALSE;  
 
  fl_ip.ip_tm_cntr_en=0;   //21.01.2008
  fl_ip.ip_tm_nat_en=0;    //21.01.2008
  Control.cnt_link=0;      //21.01.2008
  Control.link_waits=FALSE;//21.01.2008
  Control.cnt_try=0;       //21.01.2008
}  


void clr_var_ppp(void)
{
//UCSR0B=UCSR0B & ~TXEN; !!!!!!
//UCSR0B=UCSR0B & ~TXCIE; !!!!!

count_tx_ppp=0;

fl_reg3.add_byte=0;
fl_lcp.act_lcp_end=1;
fl_pap.act_pap_end=1;
fl_ipcp.act_ipcp_end=1;
fl_ip.act_ip_end=1;

cnt_rst_lcp=0;
cnt_rst_ipcp=0;                      
cnt_rst_pap=0;                     
                        
cnt_lcp_tm_out=0;
cnt_ipcp_tm_out=0;
cnt_pap_tm_out=0;
cnt_ip_tm_cntr=0;

rcvd_protokol=0;

  i_scr_lcp=0;
  i_scan_lcp=0;
  i_scj_lcp=0;
  i_str_lcp=0;
  i_sta_lcp=0;
  
  i_scr_ipcp=0;
  i_scan_ipcp=0;
  i_scj_ipcp=0;
  i_str_ipcp=0;
  i_sta_ipcp=0; 
  
  i_scr_pap=0;
  
  ip_self[0]=0;
  ip_self[1]=0;
  ip_self[2]=0;
  ip_self[3]=0;
 
  ip_change=0;
  
  fl_lcp.lcp_tm_out_en=0;
  fl_lcp.t0_pl=0;
  fl_lcp.t0_mi=0;

  fl_ipcp.ipcp_tm_out_en=0;
  fl_ipcp.t0_pl=0;
  fl_ipcp.t0_mi=0;
  
  fl_ip.ip_tm_cntr_en=0; 
  fl_ip.ip_tm_nat_en=0;
  
  fl_pap.pap_tm_out_en=0;
  fl_pap.t0_pl=0;
  fl_pap.t0_mi=0;
  fl_pap.up=0;
  
  fl_lcp1.up=0;
  fl_lcp1.down=0;
  
  fl_ipcp1.up=0;
  fl_ipcp1.down=0;
  
  fl_lcp2.nac_rej=0;
  
  fl_ipcp2.nac_rej=0;
  fl_cts_232.on=0;
  fl_cts_232_ignor=FALSE;
  
  Control.cnt_link=0;
  Control.cnt_nat=0;
  Control.link_no=FALSE;
  Control.link_waits=FALSE;
  Control.cnt_link=0;
  Control.cnt_nat=0;
  Control.cnt_try=0;
  
  Appl_RS485_1.fl_data_buf=FALSE;
  Appl_RS485_2.fl_data_buf=FALSE;
  Appl_RS485_1.id_tek=0;
  Appl_RS485_2.id_tek=0;
  fl_tx485.mon1=0;
  
  Appl_seq.cnt_waits=0;
  Appl_seq.cnt_try=0;
  fl_appl_seq.en_tx=0;
  fl_appl_seq.kv_waits=0;
  fl_appl_seq.en_povtor=0;
  Appl_seq.l_data=0;
  
  fl_appl_seq1.enable=0; //21.01.2008
  
  Appl_seq.cntr_cl=FALSE;
  Appl_seq.en_cntr_cl=FALSE;
  Appl_seq.cnt_cntr_cl=0;
  
  cnt_cts_off=0;
}




 void monitor_cntr_ch(void)
 {
   
   
  if((fl_lcp.act_lcp_end!=1)&&(fl_pap.act_pap_end!=1)||(fl_ipcp.act_ipcp_end!=1)||(fl_ip.act_ip_end!=1))return;
   
  if(Control.link_waits==TRUE) return; 
  if(Control.link==FALSE) return;  
  Control.link=FALSE;
  
  
 // {modbus_mem1[0x15e-0x150]++;} 
  clr_cntr_nat();
  
  fl_ip.act_ip_end=0;
  
  Obj_ppp_tx.prozr=FALSE;      //версия       
  Obj_ppp_tx.version=VER2;      //версия 
  Obj_ppp_tx.type_pac=SOOB;     //тип пакета
  Obj_ppp_tx.num_src=num_self; // номер отправителя
  Obj_ppp_tx.num_dst=NUM_SRV_LINK; // номер получателя
  Obj_ppp_tx.id_pac=ID_CC; // идентификатор пакета  
  Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций  
  
  Obj_ppp_tx.l_opt=0;// длина буфера опций   
  buf_opt_tr[0]=1;
  buf_opt_tr[1]=2;
  buf_opt_tr[2]=3;  
  buf_opt_tr[3]=4;
  buf_opt_tr[4]=5;
  buf_opt_tr[5]=6;
  buf_opt_tr[6]=7;
  buf_opt_tr[7]=8;
  buf_opt_tr[8]=9;  
  buf_opt_tr[9]=10;   
  buf_opt_tr[10]=11;  
  
  Obj_ppp_tx.kol_opt=0;// количество опций   
  
  Obj_ppp_tx.l_data=0;   // длина данных
  Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA];  //указатель буфера данных
  
  
  send_info(sizeof(msg_ctrl_ch),msg_ctrl_ch,1,Obj_ppp_tx.id_pac);
 
  form_buf_tx_ppp();                            

  Control.cnt_link=Control.vol_waits;
  Control.link_waits=TRUE;
  /*
  UCSR0A=UCSR0A | TXC;                            
  UCSR0B=UCSR0B | TXEN;
  UCSR0B=UCSR0B | TXCIE;
  */
  if(check_cts()==1) return;     
          
 // UDR0=buf_tx_232[0];
  
 // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
  

  return;     
 
 }  
  
  
 
 void monitor_cntr_nat(void)
 {
 unsigned char temp=NOPPER;
  if((fl_lcp.act_lcp_end!=1)&&(fl_pap.act_pap_end!=1)||(fl_ipcp.act_ipcp_end!=1)||(fl_ip.act_ip_end!=1))return; 
   
  if(Control.nat==FALSE) return;    
  clr_cntr_nat();
  fl_ip.act_ip_end=0;
 
  Obj_ppp_tx.prozr=TRUE;           
  Obj_ppp_tx.l_data=1;   // длина данных
  Obj_ppp_tx.p_data=&temp;  //указатель буфера данных
  
  send_info(sizeof(msg_ctrl_nat),msg_ctrl_nat,0,0);
  
  form_buf_tx_ppp(); 
  /*
  UCSR0A=UCSR0A | TXC;                            
  UCSR0B=UCSR0B | TXEN;
  UCSR0B=UCSR0B | TXCIE;
  */
  if(check_cts()==1) return;                
   
 // UDR0=buf_tx_232[0];
  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
  
  return; 
 }
 
 
  void monitor_cntr_cl_seq(void)
 {

  if((fl_lcp.act_lcp_end!=1)&&(fl_pap.act_pap_end!=1)||(fl_ipcp.act_ipcp_end!=1)||(fl_ip.act_ip_end!=1))return;
  if(Appl_seq.cntr_cl==FALSE) return;  
  if(Appl_seq.en_cntr_cl==FALSE) return; 
  Appl_seq.cnt_cntr_cl=Appl_seq.vol_cntr_cl; 
  Appl_seq.en_cntr_cl=FALSE;

  if (Appl_seq_des==1)return;// блокировка 
  
  clr_cntr_nat();
  fl_ip.act_ip_end=0;
  Obj_ppp_tx.prozr=FALSE;      //версия       
  Obj_ppp_tx.version=VER2;      //версия 
  Obj_ppp_tx.type_pac=ZAPR;     //тип пакета
  Obj_ppp_tx.num_src=num_self; // номер отправителя
  Obj_ppp_tx.num_dst=NUM_SRV_LINK; // номер получателя
  Obj_ppp_tx.id_pac=0; // идентификатор пакета  
  Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций  
  Obj_ppp_tx.l_opt=0;// длина буфера опций   
  buf_opt_tr[0]=PROT_SL_LS;
  buf_opt_tr[1]=COM_CNTR_CL;
  *(uint16_t*)&buf_opt_tr[2]=L_COM_CNTR_CL;  
  *(uint16_t*)&buf_opt_tr[4]=num_seq_cl;
  Obj_ppp_tx.kol_opt=0;// количество опций     
  Obj_ppp_tx.l_data=6;   // длина данных
  Obj_ppp_tx.p_data=&buf_opt_tr[0];  //указатель буфера данных    

   send_info(sizeof(req_out_ctrl_cl),req_out_ctrl_cl,1,Obj_ppp_tx.id_pac);
  
  form_buf_tx_ppp(); 
  /*
  UCSR0A=UCSR0A | TXC;                            
  UCSR0B=UCSR0B | TXEN;
  UCSR0B=UCSR0B | TXCIE;
  */
  if(check_cts()==1) return; 
  
  //UDR0=buf_tx_232[0];
  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
  
  return;      
 }  

/*

void monitor_change_state(void)
{

unsigned char buf[10];                                                                           

sv1.tc=struct_tcc1.real_tc;

if (Appl_seq_des==1)return;// блокировка опроса охранки


if(fl_appl_seq1.send_state==1)
{
fl_appl_seq1.send_state=0;
buf[0]=SEQ_STATE;
*(uint16_t*)&buf[1]=L_STATE;
*(uint32_t*)&buf[3]=burst_ds_r();
buf[7]=state_seq;
buf[8]=sv1.tc & 0x01;
//buf[9]=0;
buf[9]=byte_state_net & 0x01;
if(write_to_buf(&Appl_seq_buf[0],&Appl_seq.p_in,&Appl_seq.p_out,&Appl_seq.p_out_kv,&Appl_seq.crc,&buf[0],L_STATE+3, sizeof(Appl_seq_buf))==1)
  {
   fl_appl_seq1.over_buf=1;
  }
}





if( (sv1.tc & 0x01)!=(sv1.tc_old & 0x01))
  {
  if(sv1.tc & 0x01)Appl_seq.event=EV_TC_NORMA; else Appl_seq.event=EV_TC_ALARM;
  sv1.tc_old=sv1.tc;
  }


switch(Appl_seq.state)
{
case ST_ALARM:
                        
             switch(Appl_seq.event)
             {
             case(EV_TC_NORMA):                   
                         Appl_seq.tm_vzat=Appl_seq.vol_tm_vzat;
                         Appl_seq.state=ST_VZATIE;
                         break;
             case(EV_TM_OUT):
                         Appl_seq.tm_vzat=Appl_seq.vol_tm_vzat;
                         break;            
             default: break;
             }
     break;
case ST_VZAT:


             switch(Appl_seq.event)
             {
             case(EV_TC_ALARM):
                         send_info(sizeof(evc_alarm),evc_alarm,0,0);
                        
                         Appl_seq.tm_vzat=Appl_seq.vol_tm_vzat;
                         Appl_seq.state=ST_ALARM;
                         state_seq=ALARM;
                         buf[0]=ALARM;
                         *(uint16_t*)&buf[1]=L_ALARM;
                         *(uint32_t*)&buf[3]=burst_ds_r();
                         buf[7]=sv1.tc & 0x01;
                         buf[8]=0;
                         if(write_to_buf(&Appl_seq_buf[0],&Appl_seq.p_in,&Appl_seq.p_out,&Appl_seq.p_out_kv,&Appl_seq.crc,&buf[0],L_ALARM+3, sizeof(Appl_seq_buf))==1)
                         {
                         fl_appl_seq1.over_buf=1;
                         }
                         break;
             default: break;
             }
     break;     
case ST_VZATIE:
             switch(Appl_seq.event)
             {
             case(EV_TC_ALARM):
                         Appl_seq.tm_vzat=Appl_seq.vol_tm_vzat;
                         Appl_seq.state=ST_ALARM;
                         break;
             case(EV_TM_OUT):
                         
                         send_info(sizeof(evc_taken),evc_taken,0,0);
                         
                         Appl_seq.state=ST_VZAT;
                          state_seq=VZAT;
                          buf[0]=VZAT;
                         *(uint16_t*)&buf[1]=L_VZAT;
                         *(uint32_t*)&buf[3]=burst_ds_r();
                         buf[7]=sv1.tc & 0x01;
                         buf[8]=0;
                         if(write_to_buf(&Appl_seq_buf[0],&Appl_seq.p_in,&Appl_seq.p_out,&Appl_seq.p_out_kv,&Appl_seq.crc,&buf[0],L_VZAT+3, sizeof(Appl_seq_buf))==1)
                         {
                         fl_appl_seq1.over_buf=1;
                         }
                         break;       
             default: break;
             }
     break;          
     
default: break;
}

} 
 
 
*/




 
void Application_SEQ_tx(void)
{
 if(fl_appl_seq1.enable==0)return;
  
 if (Appl_seq_des==1)return;// блокировка сообщений
 
 if(Appl_seq.cntr_cl==TRUE) return;
 
 if((fl_lcp.act_lcp_end!=1)&&(fl_pap.act_pap_end!=1)||(fl_ipcp.act_ipcp_end!=1)||(fl_ip.act_ip_end!=1))return;
      //проверять здесь готово ли для отправки буфер PPP 
      
//if ( sim800Settings.ind_kvit_asinhr == tapsUpdateCnt.recordNum) return;  // это указатель на архивы ТИИ
if ( sim800Settings.ind_kvit_asinhr > *eventRecordPtr) return;  // это указатель на архивы событий
 
 
/*
надо еще завести указатель на архивы событий
признак с какого буфера былоо передано событие   0- с буфера ТИИ  1 - с буфера сообщений
unsigned char Appl_seq.type_arhiv;          // признак какой архив мы асинхронно передаем
соответственноо при квитировании мы должны наращивать sim800Settings.ind_kvit_soob_asinhr
*/

//if(*eventRecordPtr>=EVENT_LOG_RECORDS_NUM)*eventRecordPtr=0;    // это указатель на архивы событий
//  bkpSRAM_Read(EVENT_LOG_ADDR+ sim800Settings.ind_kvit_soob_asinhr*EVENT_LOG_RECORD_SIZE,(uint8_t*)&buf_tx_232[TR_OP_DATA + C1_DATA + 3],EVENT_LOG_RECORD_SIZE); 
   
  // после получения квитанции надо нарастить индекс отправленных архивов 
 
         
              // приращение идентификатора
              if(Appl_seq.l_data!=17)
                 {
                 if(fl_appl_seq.en_povtor ==0 )Appl_seq.id++;
                 }else return;
               
buf_tx_232[TR_OP_DATA+C1_DATA]=0x86;
buf_tx_232[TR_OP_DATA+C1_DATA+1]=14;
buf_tx_232[TR_OP_DATA+C1_DATA+2]=0;


bkpSRAM_Read(2 + EVENT_LOG_ADDR+ sim800Settings.ind_kvit_asinhr*EVENT_LOG_RECORD_SIZE,(uint8_t*)&buf_tx_232[TR_OP_DATA + C1_DATA + 3],EVENT_LOG_RECORD_SIZE);
//  framRead(TAPS_ARCHIVE_FRAM_ADDR + sim800Settings.ind_kvit_asinhr*TAPS_ARCHIVE_RECORD_SIZE,(uint8_t*)&buf_tx_232[TR_OP_DATA + C1_DATA + 3],TAPS_ARCHIVE_RECORD_SIZE);
 

Obj_ppp_tx.l_data=17;
               
               Appl_seq.cnt_waits=Control.vol_waits;              
               fl_appl_seq.kv_waits=1;      
               fl_appl_seq.en_povtor=0;  
               Appl_seq.l_data=Obj_ppp_tx.l_data;
             
             
             //формирование на передачу в буффер
               clr_cntr_nat();
              
              // clr_cntr_link_2(); // proverka
               
               fl_ip.act_ip_end=0;
  
               Obj_ppp_tx.prozr=FALSE;            
               Obj_ppp_tx.version=VER2;      //версия 
               Obj_ppp_tx.type_pac=SOOB;     //тип пакета
               Obj_ppp_tx.num_src=num_self; // номер отправителя
               Obj_ppp_tx.num_dst=num_seq_cl; // номер получателя // потом установить подумать не отошлется ли это другим котнроллерам
               Obj_ppp_tx.id_pac=Appl_seq.id; // идентификатор пакета  
               Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций  
               Obj_ppp_tx.l_opt=0;// длина буфера опций   
               Obj_ppp_tx.kol_opt=0;// количество опций 
  

               buf_tx_232[TR_OP_DATA+C1_PROT]=PROT_C1;
               buf_tx_232[TR_OP_DATA+C1_PORT]=PORT_SEQ;
               
               Obj_ppp_tx.l_data=2+Obj_ppp_tx.l_data;   // длина данных
               Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных 
 
               send_info(sizeof(msg_evc),msg_evc,1,Obj_ppp_tx.id_pac);
               
               form_buf_tx_ppp();    
               
               /*                 
               UCSR0A=UCSR0A | TXC;                            
               UCSR0B=UCSR0B | TXEN;
               UCSR0B=UCSR0B | TXCIE;
               */
               if(check_cts()==1) return;                
              
        //       UDR0=buf_tx_232[0];
               
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
           

 
 }


 void check_wdt(void)
 {
  // unsigned char i;
   
 fl_wdt.clr_wdt=0;  
  
 if (fl_wdt.fl_out==1)
     {                 
     
       
         
                 
     if((fl_wdt.from_timer0!=1)||(fl_wdt.fl_main!=1)||
        (fl_wdt.from_timer3!=1)||
        (fl_wdt.from_timer2!=1))
       
        {
        write_log_info(ST_ERROR,ERR1);
        lock_it();
        }

     if (prov_ozu!=crc_ozu())
         {
         write_log_info(ST_ERROR,ERR2);
         lock_it();
         }
     /*
     if(*(uint16_t*)&keys[L_KEYS-2]!=crc_m1(&keys[0],L_KEYS-2,0xffff))
     {
       write_log_info(ST_ERROR,ERR3);
       lock_it();
     }
     */
     
     fl_wdt.from_timer0=0; 
     fl_wdt.from_timer3=0; 
     fl_wdt.from_timer2=0;
     fl_wdt.fl_main=0;    
     fl_wdt.fl_out=0;  
     }  
  //__watchdog_reset();   
 }  



void energy_plus(void)
{



if(cnt_sec<DEF_CNT_SEC)return;
cnt_sec=0;

// тут должна вызывается односекундная процедура, которая под timer3 скрывается
one_secund_proc();

burst_ds_r();
}



void modem_engine(void)  
{
  
  if (ResetTaskSim800) 
  {
    write_log_info(ST_ERROR,ERR10);
    ResetTaskSim800=0;
    start_func_stm_off();
  } 
  //подача питания-включение модема в рабочем режиме
  // логично начать со сброса питания start_func_stm_koff()
 //  это и есть обработчик sIM800 задачи
 //
         {
            
            
          /* 
           if(fl_wdt.clr_wdt==1) check_wdt();
           fl_wdt.fl_main=1; 
          */ 
           
           supervisorwdt();
           
            
         

            energy_plus();
            
     
            
            monitor_wr_truffic();
            
            
            
           
           
            
           //  if ( (PINE & RI0 )==0 ) S3_GR;
           //  if ( PINE & RI0 ) S3_RD;
           //  if ( PINE & RI0 ) {init_modem_call_inside();}
             
            
            switch(state_modem)
               {
                 case STM_ON: 
                              
                              switch(event_modem)
                                     {
                                     case EVM_TM1:
                                                  
                                                  start_func_stm_kon();
                                                  break;  
                                    
                                     case EVM_NONE:
                                       if(cnt_stm_tm1==0)  {event_modem=EVM_TM1;break; }
                                                   break;
                                     
                                                     
                                     default:      {event_modem=EVM_NONE;break; } 
                                     /*
                                                     write_log_info(ST_ERROR,0x30);
                                                     lock_it();        
                                                     return;
                                     */
                                     }
                  break;
                  
                 case STM_KON: 
                              
                              switch(event_modem)
                                     {
                                     case EVM_TM1:
                                                  start_func_stm_pwk();
                                                  break;
                                                  
                                      case EVM_NONE:
                                       if(cnt_stm_tm1==0)  {event_modem=EVM_TM1;break; }
                                                   break;
                                     
                                                     
                                     default:      {event_modem=EVM_NONE;break; }  
                                      
                                      /*
                                                     write_log_info(ST_ERROR,0x31);
                                                    lock_it();        
                                                     return;
                                      */
                                     }
                  break;  
               
                case STM_PWK: 
                              
                              switch(event_modem)
                                     {
                                     case EVM_TM1:
                                                reset_uart_prep_sim800();
                                                  init_pins_hiz_to_out();
                                                  send_info(sizeof(stm_si),stm_si,0,0);
                                                  state_modem=STM_SI;
                                                  init_scen_stm_si();
                                                  cnt_stm_tm2=DEF_TM2_STM_SI;
                                                  event_modem=EVM_NONE;
                                                  command_AT=TRUE;// включен коммандный режим
                                     break;
                                     
                                                  
                                    case EVM_NONE:
                                       if(cnt_stm_tm1==0)  {event_modem=EVM_TM1;break; }
                                                   break;
                                     
                                                     
                                     default:      {event_modem=EVM_NONE;break; }  
                                     
                                     }
                  break; 
               
               case STM_SI:
                             // выполнение процедур - сценарий
                              at_com_tx(cnt_com);
                              at_com_rx(cnt_com);
                              at_com_scen_init(&cnt_com,&rep);
     
                              switch(event_modem)
                                     {
                                     case EVM_AT_OK:
                                                  
                                                   send_info(sizeof(stm_sc),stm_sc,0,0);
                                                   write_log_info(ST_MODEM,MDM_INIT); 
                                                   state_modem=STM_SC;
                                                   init_scen_stm_sc();
                                                   cnt_stm_tm2=DEF_TM2_STM_SC;
                                                   event_modem=EVM_NONE;
                                                   command_AT=TRUE;// включен коммандный режим
                                     break;
                                     case EVM_AT_ERR:
                                                    send_info(sizeof(evm_at_err),evm_at_err,0,0); 
                                                    start_func_stm_koff();

                                     break;
                                     
                                     
                                                  
                                    case EVM_NONE:
                                                    
                                                     break;
                                                     
                                      default: 
                                               write_log_info(ST_ERROR,0x33);
                                               lock_it();
                                               return;
                                     
                                     }
                  break;
                  
               
               case STM_SC:
                                   // выполнение процедур - сценарий
                              
                 
                                      
                                        //   if ( PINE & RI0 ) S3_RD;
                                        //   if ( PINE & RI0 ) {init_modem_call_inside();}
                 
                              at_com_tx(cnt_com);
                              at_com_rx(cnt_com);
                              at_com_scen_stm_sc(&cnt_com,&rep);                                   
                              switch(event_modem)
                                     {
                                     case EVM_AT_OK:
                                                   send_info(sizeof(stm_scp),stm_scp,0,0);
                                                   write_log_info(ST_MODEM,MDM_CON); 
                                                   state_modem=STM_SCP;
                                                   start_func_stm_scp();
                                                   event_modem=EVM_NONE;
 
                                     break;
                                     case EVM_AT_ERR:
                                                    send_info(sizeof(evm_at_err),evm_at_err,0,0);
                                                    start_func_stm_koff();
                                     break;
                                     
                                     
                                                  
                                     case EVM_NONE: break;
                                       /*    15 10 2018          
                                      default: write_log_info(ST_ERROR,0x34);
                                               lock_it();
                                              return;
                                     */
                                     default:event_modem=EVM_NONE;break;
                                     
                                     
                                     }
                  break;
                  
              
               case STM_SCP:
                                   
                 
                                      // выполнение процедур - сценарий
                            analiz_ppp_rx(); 
                           if (ResetTaskSim800) return;  // произошел сбой
                            
                    
                          
                           
                             if (layer_PPP!=LAYER_HW) 
                                             {
                                              monitor_event_LCP();   
                                              monitor_act_LCP();  
                                              }  
                             
                          
                                        if (layer_PPP==LAYER_PAP) 
                                             {
    
                                               monitor_event_PAP();   
                                               monitor_act_PAP();  
                                             }  

                                   
                
                                        if ((layer_PPP!=LAYER_HW)&&(layer_PPP!=LAYER_PAP)) 
                                             {
                                                
                                                monitor_event_IPCP();   
                                                monitor_act_IPCP();  
                                             } 
                                        
                                       
          
                   
                                          
                                        
                        //                 if((PINE & DCD0)==0)event_modem=EVM_DCD_ERR;
                        //    if(HAL_GPIO_ReadPin(DIO_PORTE, DCD0)==0) event_modem=EVM_DCD_ERR;
                                         
                               if(HAL_GPIO_ReadPin(GPIOD, DCD0)) event_modem=EVM_DCD_ERR;   // для 3-х вольтоой логики      
                                        
                                
                                        
                  
                               switch(event_modem)
                                     {                                  
                                     case EVM_NONE: if( cnt_stm_tm1==0) {event_modem=EVM_TM1; break;}
                                                    else break;
                                                    
                                     case EVM_PPP_OK:
                                                   cnt_stm_tm2=0;
                                                   send_info(sizeof(stm_ppp),stm_ppp,0,0);
                                                   write_log_info(ST_SET_PPP,SET_PPP_OK); 
                                                   
                                                   //S5_OFF;//
                                                   
                                                   state_led_md=LED_MD_PPP;
                                                   cnt_stm_tm1=0;
                                                   state_modem=STM_PPP;
                                                   event_modem=EVM_NONE;
                                                   command_AT=FALSE;// выключен коммандный режим
                                                   
                                                   
                                                   //EEEEEEEEE dobavka
                                                   
                                                   break;
                                     case EVM_TM1: 
                                                  send_info(sizeof(evm_tm),evm_tm,0,0);
                                                  write_log_info(ST_SET_PPP,SET_PPP_TM); 
                                                  goto err_set_ppp;
                                     case EVM_PPP_ERR: 
                                                  send_info(sizeof(evm_ppp_err),evm_ppp_err,0,0);
                                                  write_log_info(ST_SET_PPP,SET_PPP_RJ);
                                                  goto err_set_ppp;   
                                     case EVM_DCD_ERR:
                                                   send_info(sizeof(evm_dcd_err),evm_dcd_err,0,0);
                                                   write_log_info(ST_SET_PPP,SET_PPP_DCD);
                                                   goto err_set_ppp;
                                     case EVM_CTS_ERR:
                                                   send_info(sizeof(evm_cts_err),evm_cts_err,0,0);
                                                   write_log_info(ST_SET_PPP,SET_PPP_CTS); 
                                       err_set_ppp:  
                                            
                                                   
                                                      //  st_gprs_att();
                                                      cnt_cts_off=0;             
                                                      fl_cts_232_ignor=TRUE;
                                                      state_modem=STM_DCP; 
                                                      clr_var_ppp_rx();//обнулить все что касается приему по GPRS и запретить прием
                                                      cnt_stm_tm2=DEF_TM2_STM_DCP;
                                                      event_modem=EVM_DCP;
                                                      command_AT=FALSE;// выключен коммандный режим
                                                      
                                     break;
                                     
                                     
                                     default:  write_log_info(ST_ERROR,0x35);
                                               lock_it();
                                               return;
                                     
                                     }
                                     
                                     
                                     
                  break;
                  
                  case STM_PPP:
                                              
                                                
                                                analiz_ppp_rx();  
                                               
                                           
                                                
                                                monitor_event_IPCP();   
                                                monitor_act_IPCP();  
                                                
                                                
                                           //     monitor1_tx_rs232_2();   пока заблокирую
                                               
                                           //    #if ( RS232_handler_ON==1)  
                                           //      monitor1_tx_rs232_2();   //GPRS-штатный RS232_2
                                           //      monitor1_rec_rs232_2();
                                           //      monitor1_tm_rs232_2();
                                           //      #endif
                                                 

                                                Application_SEQ_tx();// монитор передачи сообщений охраны              
                                             
                                                monitor_cntr_ch();
                                                monitor_cntr_cl_seq();
                                                monitor_cntr_nat(); 
                                      
                                               
                              
                             
                              
                              

                           //   if((PINE & DCD0)==0)event_modem=EVM_DCD_ERR;
                          // if(HAL_GPIO_ReadPin(DIO_PORTE, DCD0)==0) event_modem=EVM_DCD_ERR;
                            if(HAL_GPIO_ReadPin(GPIOD, DCD0)) event_modem=EVM_DCD_ERR;   // для 3-х вольтоой логики  

                  
                               switch(event_modem)
                                     {                                  

                                     case EVM_PPP_ERR:
                                                      send_info(sizeof(evm_ppp_err),evm_ppp_err,0,0);
                                                      write_log_info(ST_PPP,PPP_RJ); 
                                                      goto err_ppp;
                                     case EVM_DCD_ERR:
                                                   send_info(sizeof(evm_dcd_err),evm_dcd_err,0,0);
                                                   write_log_info(ST_PPP,PPP_DCD);
                                                   goto err_ppp;  
                                                      
                                     case EVM_MS_LMT:
                                                      send_info(sizeof(evm_no_rec),evm_no_rec,0,0);
                                                      write_log_info(ST_PPP,PPP_LS); 
                                                      goto err_ppp; 
                                     case EVM_CTS_ERR:
                                                      send_info(sizeof(evm_cts_err),evm_cts_err,0,0);
                                                      write_log_info(ST_PPP,PPP_CTS);       
                                               err_ppp:      
                                        //              st_gprs_att();
                                                      cnt_cts_off=0;
                                                      fl_cts_232_ignor=TRUE;
                                                      state_modem=STM_DCP; 
                                                      clr_var_ppp_rx();//обнулить все что касается приему по GPRS и запретить прием
                                                      cnt_stm_tm2=DEF_TM2_STM_DCP;
                                                      event_modem=EVM_DCP;
                                                      command_AT=FALSE;// выключен коммандный режим
                                                      
                                                      
                                                     
                                     break;
                                 
                                     }
                  break;
                  
                  case STM_DCP:
                                               

                  
                    
                                                //GPRS-штатный RS232 
                                             //  #if ( RS232_handler_ON==1) 
                                             //   monitor1_rec_rs232_2();
                                             //   monitor1_tm_rs232_2();  
                                             //  #endif
                                                
                                                
                                                
                               
                               switch(event_modem)
                                     {

                                      
                                     
                                     case EVM_DCP:
                                                 if((fl_tx485.mon1==0)&&(fl_tx485.mon2==0)&&(fl_tx485.mon232_2==0)&&(fl_ip.act_ip_end==1)&&(fl_pap.act_pap_end==1)&&(fl_lcp.act_lcp_end==1)&&(fl_ipcp.act_ipcp_end==1))
                                                   {
                                                     send_terminate_lcp();
                                                     event_modem=EVM_RDC;
                                                   } 
                                       
                                                   break;
                                     
                                     
                                     case EVM_RDC:
                                                  if(fl_lcp.act_lcp_end==0)break;
                                                  count_ppp_down++;
                                                  
                                                  if (count_ppp_down < 7) { 
                                                    /*
                                                      // попробовать в свете нового понимания -кво попыток
                                                    write_log_info(ST_CLR_PPP,CLR_PPP_RDY); 
                                                 
                                                   state_modem=STM_DC;
                                                   cnt_stm_tm2=DEF_TM2_STM_DC;
                                                       //обнуление ppp переменных
                                                       //и запртить передачу по GPRS
                                                   
                                                 //  UCSR0B=UCSR0B & ~TXEN;
                                                 //  UCSR0B=UCSR0B & ~TXCIE;
                                                   
                                                   
                                                __HAL_UART_DISABLE_IT(&huart3,UART_IT_TXE);
                                                  
                                                   
                                                   clr_var_ppp(); 
                                                    
                                                 //   clr_all_stack();
                                                 //   state_modem=STM_DC;
                                                 //  cnt_stm_tm2=DEF_TM2_STM_DC;
                                                 //  reset_uart_prep_sim800();
                                                 // init_pins_hiz_to_out(); 
                                                    
                                                   init_scen_stm_dc_1();
                                                   event_modem=EVM_NONE;
                                                   command_AT=TRUE;// включен коммандный режим
                                                   */
                                                    
                                                  
                                                  count_ppp_down=0; 
                                                   write_log_info(ST_CLR_PPP,CLR_PPP_ERR); 
                                                   state_modem=STM_OFF;
                                                   start_func_stm_off();
                                                   event_modem=EVM_NONE;
                                                   command_AT=TRUE;// включен коммандный режим  
                                                   
                                                  }
                                                  
                                                  
                                                  
                                    /////////////////////  09 07 14 добавили кусок               
                                    // если PPP сессия часто рвется , то сбросить питание              
                                                  //выключаем модем 
                                                   
                                                  
                                                  
                                                  
                                                   //SVD2_1_OFF;!!!!!
                                                   //SVD2_2_OFF;!!!!!!!!!
                                                  else { 
                                    /////////////////////  09 07 14 добавили кусок               
                                    // если PPP сессия часто рвется , то сбросить питание              
                                                  //выключаем модем 
                                                    count_ppp_down=0; 
                                                   write_log_info(ST_CLR_PPP,CLR_PPP_ERR); 
                                                   state_modem=STM_OFF;
                                                   start_func_stm_off();
                                                   event_modem=EVM_NONE;
                                                   command_AT=TRUE;// включен коммандный режим
                                                  
                                                  } 
                                     break;
                                     }
                  
                  break;
                  
                  case STM_DC:
                              
                              at_com_tx(cnt_com);
                              at_com_rx(cnt_com);
                              at_com_scen_stm_dc_1(&cnt_com,&rep);  
                               
                               
                               switch(event_modem)
                                     {
                                     case EVM_AT_OK:                                                  
                                                   write_log_info(ST_CLR_PPP,CLR_PPP_OK);
                                                   send_info(sizeof(stm_sc),stm_sc,0,0);
                                                   state_modem=STM_SC;
                                                   init_scen_stm_sc();
                                                   cnt_stm_tm2=DEF_TM2_STM_SC;
                                                   event_modem=EVM_NONE;
                                                   command_AT=TRUE;// включен коммандный режим
                                                                        
                                                        
                                     break;
                                     case EVM_AT_ERR:
                                                   write_log_info(ST_CLR_PPP,CLR_PPP_ERR);
                                                   state_modem=STM_KOFF;
                                                   start_func_stm_koff();
                                                   event_modem=EVM_NONE;
                                                   command_AT=TRUE;// включен коммандный режим
                                                   
                                     break;
                                     
                                     default:   break;
                                     
                                     }
                  break;
                  
                  
                  case STM_KOFF:
                               switch(event_modem)
                                     {
                                     case EVM_TM1:
                                                 
                                                  start_func_stm_off();
                                                  event_modem=EVM_NONE;
                                                  break;
                                     
                                      case EVM_NONE:
                                       if(cnt_stm_tm1==0)  {event_modem=EVM_TM1;break; }
                                                   break;
                                     
                                                     
                                     default:      {event_modem=EVM_NONE;break; }    
                                     }
                  
                  break;
                  case STM_OFF:
                               switch(event_modem)
                                     {
                                     case EVM_TM1:
                                                 
                                                  start_func_stm_on();
                                                  event_modem=EVM_NONE;
                                                  break;
                                     
                                      case EVM_NONE:
                                       if(cnt_stm_tm1==0)  {event_modem=EVM_TM1;break; }
                                                   break;
                                     
                                                     
                                     default:      {event_modem=EVM_NONE;break; }        
                                     }
                  break;
         
                  
                 default: 
                   write_log_info(ST_ERROR,ERR9);    
                   lock_it();
                   return;
                  
               }
          } 
  
}



void send_err485(unsigned char port,unsigned char err,unsigned char id,uint16_t dst)
    {
       if(fl_ip.act_ip_end!=1)return;
          fl_ip.act_ip_end=0;      
          clr_cntr_nat();
          clr_cntr_link();
          Obj_ppp_tx.prozr=FALSE;            
          Obj_ppp_tx.version=VER2;      //версия 
          Obj_ppp_tx.type_pac=OTV;     //тип пакета
          Obj_ppp_tx.num_src=num_self; // номер отправителя
          Obj_ppp_tx.num_dst=dst; // номер получателя
          Obj_ppp_tx.id_pac=id; // идентификатор пакета  
          Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций  
          Obj_ppp_tx.l_opt=0;// длина буфера опций   
          Obj_ppp_tx.kol_opt=0;// количество опций   
         
         buf_tx_232[TR_OP_DATA+C1_PROT]=PROT_C1;
         buf_tx_232[TR_OP_DATA+C1_PORT]=PORT_SYS;

         buf_tx_232[TR_OP_DATA+C1_DATA]=RS485_ERR;
         *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1]=LN_RS485_ERR;
         buf_tx_232[TR_OP_DATA+C1_DATA+3]=port;
         buf_tx_232[TR_OP_DATA+C1_DATA+4]=err;
         Obj_ppp_tx.l_data=5+LN_RS485_ERR;   // длина данных
         Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных
        
         form_buf_tx_ppp();  
         /*
         UCSR0A=UCSR0A | TXC;                            
         UCSR0B=UCSR0B | TXEN;
         UCSR0B=UCSR0B | TXCIE;
         */
         if(check_cts()==1) return;                
         
         //UDR0=buf_tx_232[0];
         
         // Посылаем всегда по одному байту
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
         
         return;     
    }



/////////////////////////////////// stm


unsigned char check_cont_485_1 (unsigned char *pointer)
{
   
         
                    switch (*pointer)
                            {
                              case B2400:
                              case B4800:
                              case B9600:
                              case B38400:
                              case B19200:break;
                              default: return(1);
                            }
         
         
                switch ((*(pointer+1)) & 0x07)
                           {
                           case NON:
                           case ODD:
                           case EVEN: break;
                           default: return(1);
                           }
         

                    switch (((*(pointer+1))>>3)& 0x03)
                           {
                           case STOP1:
                           case STOP2:break;
                           default: return(1);
                           }
            
                    
                    switch (((*(pointer+1))>>5)& 0x03)
                           {
                           case INF7:
                           case INF8:break;
                           default: return(1);
                           }



                  if((*(uint16_t*)(pointer+2))>10000)return(1);
                  
                  if((*(uint16_t*)(pointer+4))==0)return(1);
                  if((*(uint16_t*)(pointer+4))>10000)return(1);
                  
                  if((*(uint16_t*)(pointer+6))<3)return(1);
                  
                  return(0);
}


/*

unsigned char run_cont_485_1 (unsigned char *pointer)
{
    
   
                switch ((*(pointer+1)) & 0x07)
                           {
                           case NON:  UCSR3C=UCSR3C & ~0x30;
                                 break;
                           case ODD:  UCSR3C=UCSR3C | 0x30;
                                 break;
                           case EVEN: UCSR3C=UCSR3C & ~0x30;
                                 UCSR3C=UCSR3C | 0x20;
                                 break;
                           default: return(1);
                           }
         

                    switch (((*(pointer+1))>>3)& 0x03)
                           {
                           case STOP1:UCSR3C=UCSR3C & ~0x08; 
                                 break;
                           case STOP2:
                                 UCSR3C=UCSR3C | 0x08; 
                                 break;
                           default: return(1);
                           }
            
            
                    switch (((*(pointer+1))>>5)& 0x03)
                           {
                           case INF7:
                                UCSR3C=UCSR3C & ~0x06;
                                UCSR3C=UCSR3C | 0x04;
                                break;
                           case INF8:
                                UCSR3C=UCSR3C | 0x06;
                                break;
                           default: return(1);
                           }


                  if((*(uint16_t*)(pointer+2))>10000)return(1);
                  Rs485_1.cnt_tm_tx_out=0;
                  Rs485_1.vol_tm_tx_out=*(uint16_t*)(pointer+2);
                  
                  if((*(uint16_t*)(pointer+4))==0)return(1);
                  if((*(uint16_t*)(pointer+4))>10000)return(1);
                  Rs485_1.cnt_tm_rx_out=0;
                  Rs485_1.vol_tm_rx_out=*(uint16_t*)(pointer+4);
                  
                  
                  if((*(uint16_t*)(pointer+6))<3)return(1);
                  Rs485_1.cnt_tm_out=0;
                  Rs485_1.vol_tm_out=*(uint16_t*)(pointer+6); // 1000 миллисекунд
                   
                   
                   
                 switch (*pointer)
                            {
                              case B2400:
                                     UBRR3H=R2400_H;
                                     UBRR3L=R2400_L;
                                     
                                     
                                     if(Appl_RS485_1.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                          
                                        //  __disable_interrupt();
                                          SET_RTS1;
                                        //  __enable_interrupt();
                                          
                                          
                                          Rs485_1.cnt_tm_pre_tx=Appl_RS485_1.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_1.cnt_tm_pre_tx=17;// 17 миллисекунд
                                    
                                  //   Rs485_1.vol_tm_tx_out=Rs485_1.vol_tm_tx_out+5;
                                     break;
                              case B4800:
                                     UBRR3H=R4800_H;
                                     UBRR3L=R4800_L;
                                     
                                     if(Appl_RS485_1.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                        //  __disable_interrupt();
                                         SET_RTS1;
                                       //   __enable_interrupt();
                                          
                                          Rs485_1.cnt_tm_pre_tx=Appl_RS485_1.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_1.cnt_tm_pre_tx=9;// 9 миллисекунд
                                        
                               //      Rs485_1.vol_tm_tx_out=Rs485_1.vol_tm_tx_out+3;
                                     break;
                              case B9600:
                                     UBRR3H=R9600_H;
                                     UBRR3L=R9600_L;
                                                                          
                                     if(Appl_RS485_1.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                         // __disable_interrupt();
                                          SET_RTS1;
                                        //  __enable_interrupt();
                                          
                                          Rs485_1.cnt_tm_pre_tx=Appl_RS485_1.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_1.cnt_tm_pre_tx=4;// 4 миллисекунд
                                        
                               //      Rs485_1.vol_tm_tx_out=Rs485_1.vol_tm_tx_out+2;
                                     break;
                              case B19200:
                                     UBRR3H=R19200_H;
                                     UBRR3L=R19200_L;
                                                                          
                                     if(Appl_RS485_1.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                           
                                          // __disable_interrupt();
                                          SET_RTS1;
                                         // __enable_interrupt();
                                          
                                          Rs485_1.cnt_tm_pre_tx=Appl_RS485_1.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_1.cnt_tm_pre_tx=2;// 2 миллисекунд
                                        
                               //      Rs485_1.vol_tm_tx_out=Rs485_1.vol_tm_tx_out+1;
                                     break;
                                     
                               case B38400:
                                     UBRR3H=R38400_H;
                                     UBRR3L=R38400_L;
                                                                          
                                     if(Appl_RS485_1.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                           
                                          // __disable_interrupt();
                                          SET_RTS1;
                                         // __enable_interrupt();
                                          
                                          Rs485_1.cnt_tm_pre_tx=Appl_RS485_1.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_1.cnt_tm_pre_tx=2;// 2 миллисекунд
                                        
                               //      Rs485_1.vol_tm_tx_out=Rs485_1.vol_tm_tx_out+1;
                                     break;       
                              default: return(1);
                            }  
                   
                   
                   
                   
return(0);
}



unsigned char run_cont_485_2 (unsigned char *pointer)
{
    
   
                switch ((*(pointer+1)) & 0x07)
                           {
                           case NON:  UCSR1C=UCSR1C & ~0x30;
                                 break;
                           case ODD:  UCSR1C=UCSR1C | 0x30;
                                 break;
                           case EVEN: UCSR1C=UCSR1C & ~0x30;
                                 UCSR1C=UCSR1C | 0x20;
                                 break;
                           default: return(1);
                           }
         

                    switch (((*(pointer+1))>>3)& 0x03)
                           {
                           case STOP1:UCSR1C=UCSR1C & ~0x08; 
                                 break;
                           case STOP2:
                                 UCSR1C=UCSR1C | 0x08; 
                                 break;
                           default: return(1);
                           }
            
            
                    switch (((*(pointer+1))>>5)& 0x03)
                           {
                           case INF7:
                                UCSR1C=UCSR1C & ~0x06;
                                UCSR1C=UCSR1C | 0x04;
                                break;
                           case INF8:
                                UCSR1C=UCSR1C | 0x06;
                                break;
                           default: return(1);
                           }


                  if((*(uint16_t*)(pointer+2))>10000)return(1);
                  Rs485_2.cnt_tm_tx_out=0;
                  Rs485_2.vol_tm_tx_out=*(uint16_t*)(pointer+2);
                  
                  if((*(uint16_t*)(pointer+4))==0)return(1);
                  if((*(uint16_t*)(pointer+4))>10000)return(1);
                  Rs485_2.cnt_tm_rx_out=0;
                  Rs485_2.vol_tm_rx_out=*(uint16_t*)(pointer+4);
                  
                  
                  if((*(uint16_t*)(pointer+6))<3)return(1);
                  Rs485_2.cnt_tm_out=0;
                  Rs485_2.vol_tm_out=*(uint16_t*)(pointer+6); // 1000 миллисекунд
                   
                   
                   
                 switch (*pointer)
                            {
                              case B2400:
                                     UBRR1H=R2400_H;
                                     UBRR1L=R2400_L;
                                     
                                     
                                     if(Appl_RS485_2.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                          
                                        //  __disable_interrupt();
                                          SET_RTS3;
                                        //  __enable_interrupt();
                                          
                                          
                                          Rs485_2.cnt_tm_pre_tx=Appl_RS485_2.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_2.cnt_tm_pre_tx=17;// 17 миллисекунд
                                    
                                //     Rs485_2.vol_tm_tx_out=Rs485_2.vol_tm_tx_out+5;
                                     break;
                              case B4800:
                                     UBRR1H=R4800_H;
                                     UBRR1L=R4800_L;
                                     
                                     if(Appl_RS485_2.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                        //  __disable_interrupt();
                                         SET_RTS3;
                                       //   __enable_interrupt();
                                          
                                          Rs485_2.cnt_tm_pre_tx=Appl_RS485_2.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_2.cnt_tm_pre_tx=9;// 9 миллисекунд
                                        
                                 //    Rs485_2.vol_tm_tx_out=Rs485_2.vol_tm_tx_out+3;
                                     break;
                              case B9600:
                                     UBRR1H=R9600_H;
                                     UBRR1L=R9600_L;
                                                                          
                                     if(Appl_RS485_2.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                         // __disable_interrupt();
                                          SET_RTS3;
                                        //  __enable_interrupt();
                                          
                                          Rs485_2.cnt_tm_pre_tx=Appl_RS485_2.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_2.cnt_tm_pre_tx=4;// 4 миллисекунд
                                        
                                //     Rs485_2.vol_tm_tx_out=Rs485_2.vol_tm_tx_out+2;
                                     break;
                              case B19200:
                                     UBRR1H=R19200_H;
                                     UBRR1L=R19200_L;
                                                                          
                                     if(Appl_RS485_2.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                           
                                          // __disable_interrupt();
                                          SET_RTS3;
                                         // __enable_interrupt();
                                          
                                          Rs485_2.cnt_tm_pre_tx=Appl_RS485_2.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_2.cnt_tm_pre_tx=2;// 2 миллисекунд
                                        
                                //     Rs485_2.vol_tm_tx_out=Rs485_2.vol_tm_tx_out+1;
                                     break;
                                     
                                case B38400:
                                     UBRR1H=R38400_H;
                                     UBRR1L=R38400_L;
                                                                          
                                     if(Appl_RS485_2.pre_tx!=0)
                                          {
                                          // перехват порта для лафкина
                                           
                                          // __disable_interrupt();
                                          SET_RTS3;
                                         // __enable_interrupt();
                                          
                                          Rs485_2.cnt_tm_pre_tx=Appl_RS485_2.pre_tx;//задержка перед передачей
                                          }
                                        else Rs485_2.cnt_tm_pre_tx=2;// 2 миллисекунд
                                        
                                //     Rs485_2.vol_tm_tx_out=Rs485_2.vol_tm_tx_out+1;
                                     break;      
                                     
                              default: return(1);
                            }  
                   
                   
                   
                   
return(0);
}


*/


// настройка порта по контексту из зароса
unsigned char run_cont_232_2 (unsigned char *pointer)
{
  uint32_t baud,wordlength,stopbit,parity;
//  void MX_USART2_UART_Init_Custom(uint32_t baud, uint32_t wordlength,uint32_t stopbit, uint32_t parity)
    
   
                switch ((*(pointer+1)) & 0x07)
                           {
                           case NON:  parity=UART_PARITY_NONE;
                                 break;
                           case ODD:  parity=UART_PARITY_ODD;
                                 break;
                           case EVEN: parity=UART_PARITY_EVEN;
                                 
                                 break;
                           default: return(1);
                           }
         

                    switch (((*(pointer+1))>>3)& 0x03)
                           {
                           case STOP1:  stopbit=UART_STOPBITS_1; 
                                 break;
                           case STOP2:
                                 stopbit=UART_STOPBITS_2; 
                                 break;
                           default: return(1);
                           }
            
            
                    switch (((*(pointer+1))>>5)& 0x03)
                           {
                           case INF7:
                                return(1);  // 7 битный формат не поддерживается
                           case INF8:
                               wordlength=UART_WORDLENGTH_8B;
                                break;
                           default: return(1);
                           }


                  if((*(uint16_t*)(pointer+2))>10000)return(1);
                  Rs232_2.cnt_tm_tx_out=0;
                  Rs232_2.vol_tm_tx_out=*(uint16_t*)(pointer+2);
                  
                  if((*(uint16_t*)(pointer+4))==0)return(1);
                  if((*(uint16_t*)(pointer+4))>10000)return(1);
                  Rs232_2.cnt_tm_rx_out=0;
                  Rs232_2.vol_tm_rx_out=*(uint16_t*)(pointer+4);
                  
                  
                  if((*(uint16_t*)(pointer+6))<3)return(1);
                  Rs232_2.cnt_tm_out=0;
                  Rs232_2.vol_tm_out=*(uint16_t*)(pointer+6); // 1000 миллисекунд
                   
                   
                   
                 switch (*pointer)
                            {
                              case B2400:     baud=2400;break;
                              case B4800:     baud=4800;break;     
                             case B9600:      baud=9600;break;
                             
                              case B19200:   baud=19200;break;
                                    
                               case B38400:   baud=38400;break;
                                    
                              default: return(1);
                            }  
                   
   HAL_UART_DeInit(&huart2);               
   MX_USART2_UART_Init_Custom(baud, wordlength,stopbit, parity);
                   
return(0);
}









 unsigned char check_ln_conf(unsigned char *buf_rx_ppp,unsigned char offset,uint16_t count_rx_ppp)
 {
 uint16_t ind;

 ind=offset+C1_DATA;
 next_check_ln: 
               ind=ind+3+*(uint16_t*)&buf_rx_ppp[ind+1]; 
               if(ind>count_rx_ppp-3)return(1);
               if(ind==count_rx_ppp-3)return(0);
               goto next_check_ln;
 } 



uint16_t proc_config(unsigned char *buf_rx_ppp,unsigned char offset,uint16_t count_rx_ppp)
 {

 //unsigned char flag_monitor_beg=0;
 
 uint16_t ind,temp,ofs,i;
 uint16_t cnt_bt;
 unsigned char buf[20],*p;
 uint32_t old_unix; 
 cnt_bt=0;
 ind=offset+C1_DATA;
 next_parametr: 
               
               switch(buf_rx_ppp[ind]&0x7f)
                 {
                 case CONF_VERS:  if((buf_rx_ppp[ind] & 0x80)==0x80)goto bad_com;
                                    else
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_RD_CONF_VERS)goto bad_com;   
                                    }
                                   buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  
                                   buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                   buf[0]=ret_version(0,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt]);
                                   *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=buf[0]+1;
                                  cnt_bt=cnt_bt+buf[0]+1+3;
                                  break;
                 
                 case CONF_MAP:  if((buf_rx_ppp[ind] & 0x80)==0x80)goto bad_com;
                                    else
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_RD_CONF_MAP)goto bad_com;   
                                    }
                                   buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  
                                   buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                   buf[0]=ret_version(1,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt]);
                                   *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=buf[0]+1;
                                  cnt_bt=cnt_bt+buf[0]+1+3;
                                  break;
                 
                 
                case CONF_OZU:  if((buf_rx_ppp[ind] & 0x80)==0x80)goto bad_com;
                                    else
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_RD_CONF_OZU)goto bad_com;   
                                    }
                                   buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  
                                   buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                   buf[0]=ret_version(2,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt]);
                                   *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=buf[0]+1;
                                  cnt_bt=cnt_bt+buf[0]+1+3;
                                  break;
                
                 case CONF_TM_NAT:  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_NAT)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                    
                                    if (temp<10){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                    
                                    
                                    Control.vol_nat=temp;
                                    CopyMass(A_C_PAR,&buf[0],L_C_PAR); 
                                    *(uint16_t*)&buf[OFS_NAT]=temp;
                                    *(uint16_t*)&buf[OFS_C_PAR_CRC]=crc_m1(&buf[0],L_C_PAR-2,0xffff);
                                    framWriteSim800(A_C_PAR, &buf[0],L_C_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_NAT-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_NAT+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_C_PAR+OFS_NAT,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_NAT+1+3;
                                  break;
                   
                   case CONF_TM_CC:  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_CC)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                    
                                    if (temp<10){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                    
                                    Control.vol_link=temp;
                                    CopyMass(A_C_PAR,&buf[0],L_C_PAR); 
                                    *(uint16_t*)&buf[OFS_CCH]=temp;
                                    *(uint16_t*)&buf[OFS_C_PAR_CRC]=crc_m1(&buf[0],L_C_PAR-2,0xffff);
                                    framWriteSim800(A_C_PAR, &buf[0],L_C_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_CC-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_CC+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_C_PAR+OFS_CCH,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_CC+1+3;
                                  break;
                                  
                                  
                    case CONF_TM_WT :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_WT)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                     
                                    if (temp<5){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}

                                    Control.vol_try=temp/Control.vol_waits;
                                   
                                    CopyMass(A_C_PAR,&buf[0],L_C_PAR); 
                                    *(uint16_t*)&buf[OFS_TM_CH]=temp;
                                    *(uint16_t*)&buf[OFS_C_PAR_CRC]=crc_m1(&buf[0],L_C_PAR-2,0xffff);
                                    framWriteSim800(A_C_PAR, &buf[0],L_C_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_WT-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_WT+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_C_PAR+OFS_TM_CH,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_WT+1+3;
                                  break;              


               case CONF_NUM_CL :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_NUM_CL)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                     
                                    if (temp<3){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}

                                    num_seq_cl=temp;
                                    
                                    CopyMass(A_SEQ_PAR,&buf[0],L_SEQ_PAR); 
                                    *(uint16_t*)&buf[OFS_NUM_CL]=temp;
                                    *(uint16_t*)&buf[OFS_SEQ_PAR_CRC]=crc_m1(&buf[0],L_SEQ_PAR-2,0xffff);
                                    framWriteSim800(A_SEQ_PAR, &buf[0],L_SEQ_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_NUM_CL-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_NUM_CL+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_SEQ_PAR+OFS_NUM_CL,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_NUM_CL+1+3;
                                  break;   
               case CONF_TM_VZ :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_VZ)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                     
                                    if (temp<1){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}

                                    Appl_seq.vol_tm_vzat=temp;
                                    
                                    CopyMass(A_SEQ_PAR,&buf[0],L_SEQ_PAR); 
                                    *(uint16_t*)&buf[OFS_TM_VZ]=temp;
                                    *(uint16_t*)&buf[OFS_SEQ_PAR_CRC]=crc_m1(&buf[0],L_SEQ_PAR-2,0xffff);
                                    framWriteSim800(A_SEQ_PAR, &buf[0],L_SEQ_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_VZ-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_VZ+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_SEQ_PAR+OFS_TM_VZ,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_VZ+1+3;
                                  break;   

               case CONF_TM_CL :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_CL)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                     
                                    if (temp<5){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}

                                    Appl_seq.vol_cntr_cl=temp;
                                    
                                    CopyMass(A_SEQ_PAR,&buf[0],L_SEQ_PAR); 
                                    *(uint16_t*)&buf[OFS_TM_CL]=temp;
                                    *(uint16_t*)&buf[OFS_SEQ_PAR_CRC]=crc_m1(&buf[0],L_SEQ_PAR-2,0xffff);
                                    framWriteSim800(A_SEQ_PAR, &buf[0],L_SEQ_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_CL-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_CL+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_SEQ_PAR+OFS_TM_CL,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_CL+1+3;
                                  break;   
             
             case CONF_DES_OHR :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_DES_OHR)goto bad_com;
                                    //запись
                                    temp=buf_rx_ppp[ind+3];
                                     
                                    if (temp>2){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}

                                    Appl_seq_des=temp;
                                    
                                    CopyMass(A_SEQ_PAR,&buf[0],L_SEQ_PAR); 
                                    buf[OFS_DES_SEQ]=temp;
                                    *(uint16_t*)&buf[OFS_SEQ_PAR_CRC]=crc_m1(&buf[0],L_SEQ_PAR-2,0xffff);
                                    framWriteSim800(A_SEQ_PAR, &buf[0],L_SEQ_PAR);
                                    prov_ozu=crc_ozu();
                                    
                                    
                                    
                                      
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_DES_OHR-1))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_DES_OHR+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_SEQ_PAR+OFS_DES_SEQ,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],1);
                                    
                                    cnt_bt=cnt_bt+L_CONF_DES_OHR+1+3;
                                  break;  


                 case CONF_NUM_SELF :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_NUM_SELF)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                     
                                    if (temp<3){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                     
                                    fl_rewrite.num_self=1;  
                                    
                                    //WrArrayToFlesh(ABUF_NUM,&buf_rx_ppp[ind+3],L_CONF_NUM_SELF,0,0);                                  
                                   // RdFromFleshToArr(ABUF_NUM,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                                                                              
                                    CopyMass(A_IP_PAR,&buf[0],L_IP_PAR); 
                                    *(uint16_t*)&buf[OFS_NUM]=temp;
                                    *(uint16_t*)&buf[OFS_IP_PAR_CRC]=crc_m1(&buf[0],L_IP_PAR-2,0xffff);
                                    framWriteSim800(A_IP_PAR, &buf[0],L_IP_PAR);
                                  //  prov_ozu=crc_ozu();
                                    
                                    }
                                    else if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_NUM_SELF-2))goto bad_com;
                                         

                                    CopyMass(A_IP_PAR+OFS_NUM,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_NUM_SELF+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    
                                    
                                    cnt_bt=cnt_bt+L_CONF_NUM_SELF+1+3;
                                  break;   

                  case CONF_UDP :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_UDP)goto bad_com;
                                   
                                   
                                    //запись
                                    fl_rewrite.udp=1;
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                   // WrArrayToFlesh(ABUF_PORT, &buf_rx_ppp[ind+3],L_CONF_UDP,0,0);
                                   // RdFromFleshToArr(ABUF_PORT,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    CopyMass(A_IP_PAR,&buf[0],L_IP_PAR); 
                                    *(uint16_t*)&buf[OFS_PORT]=temp;
                                    *(uint16_t*)&buf[OFS_IP_PAR_CRC]=crc_m1(&buf[0],L_IP_PAR-2,0xffff);
                                    framWriteSim800(A_IP_PAR, &buf[0],L_IP_PAR);
                                    
                                    }
                                    else if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_UDP-2))goto bad_com;
                                      

                                     
                                    CopyMass(A_IP_PAR+OFS_PORT,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_UDP+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    
                                    
                                    cnt_bt=cnt_bt+L_CONF_UDP+1+3;
                                  break;   
                   
                   case CONF_IP :  
                                                   
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_IP)goto bad_com;
                                      
                                    fl_rewrite.ip=1;
                                    
                                    //запись
                                   // WrArrayToFlesh(ABUF_IP, &buf_rx_ppp[ind+3],L_CONF_IP,0,0);
                                   // RdFromFleshToArr(ABUF_IP,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],4);
                                    
                              
                                    CopyMass(A_IP_PAR,&buf[0],L_IP_PAR); 
                                    buf[OFS_IP]=buf_rx_ppp[ind+3];
                                    buf[OFS_IP+1]=buf_rx_ppp[ind+4];
                                    buf[OFS_IP+2]=buf_rx_ppp[ind+5];
                                    buf[OFS_IP+3]=buf_rx_ppp[ind+6];
                                    *(uint16_t*)&buf[OFS_IP_PAR_CRC]=crc_m1(&buf[0],L_IP_PAR-2,0xffff);
                                    framWriteSim800(A_IP_PAR, &buf[0],L_IP_PAR);
                                    //prov_ozu=crc_ozu();
                                    
                                    }
                                    else if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_IP-4))goto bad_com;
                                        
                                    CopyMass(A_IP_PAR+OFS_IP,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],4);  
                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_IP+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  

                                    cnt_bt=cnt_bt+L_CONF_IP+1+3;
                                  break;   


                   case CONF_PDP :                                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    goto bad_com;
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=0)goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение                                    
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                  
                                    CopyMass(A_C_GPRS,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],L_C_GPRS_MAX);
                                    
                 same_1:            
                   
                                    
                                    p=&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt];
                                    ofs=0;
                   while(*p) {p++;ofs++;}
                   p++;ofs++;
                   while(*p) {p++;ofs++;}
                   p++;ofs++;
                   while(*p) {p++;ofs++;}
                   p++;ofs++;
                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=ofs+1;   
                                    cnt_bt=cnt_bt+ofs+1+3;
                                  
                                      break;   
                                    
                   /*
                                    ofs=TR_OP_DATA+C1_DATA+4+cnt_bt;
                                    temp=buf_tx_232[ofs];
                                    for(i=0;i<temp;i++)buf_tx_232[ofs+i]=buf_tx_232[ofs+i+1];
                                    buf_tx_232[ofs+i]=0;
                              
                                    
                                    ofs=ofs+temp+1;
                                    temp=buf_tx_232[ofs];
                                    for(i=0;i<temp;i++)buf_tx_232[ofs+i]=buf_tx_232[ofs+i+1];
                                    buf_tx_232[ofs+i]=0;

                                 
                                    ofs=ofs+temp+1;
                                    temp=buf_tx_232[ofs];
                                    for(i=0;i<temp;i++)buf_tx_232[ofs+i]=buf_tx_232[ofs+i+1];
                                    buf_tx_232[ofs+i]=0;
                                    
                                    ofs=ofs+temp+1;
                                    ofs=ofs-(TR_OP_DATA+C1_DATA+4+cnt_bt);
                                 
 
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=ofs+1;   
                                    cnt_bt=cnt_bt+ofs+1+3;
                                  
                                      break;   
                 
                            */                 
               case CONF_RESET:          
                                 if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=0)goto bad_com;
                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    cnt_bt=cnt_bt+1+3;
                                    fl_rewrite.cnt_reset=4;
                                    break;
                                    }
                                    else goto bad_com;
                 
                 
                 case CONF_PDP_DOP :                                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+1];
                                    
                                    
                                    
                                    if((temp==1)||(temp==2))goto bad_com;
                                    if(temp>L_CONF_PDP)goto bad_com;
                                   
                                    if(temp==0)
                                       {
                                        
                                      //   temp=L_CONF_PDP;                                
                                      //   WrArrayToFlesh(A_C_GPRS,0,temp,0x01,0x00);      // Отменил операцию очистки 
                                        
                                       }
                                       
                                       else
                                         { 
                                        //запись
                                        framWriteSim800(A_C_GPRS,&buf_rx_ppp[ind+3],temp);                                  
                                       
                                         }
                                    }
                                    else {
                                            if(*(uint16_t*)&buf_rx_ppp[ind+1]!=0)goto bad_com;
                                            temp=L_CONF_PDP;
                                          }
                                     
                                    CopyMass(A_C_GPRS,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],temp); 
                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind]; 
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=temp+1;                                  
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK 
                                     

                                    cnt_bt=cnt_bt+temp+1+3;
                                     
                                  break;   
 
                                 
               
                                    

                 case CONF_TIME:
                                 if((buf_rx_ppp[ind] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_WR_CONF_TIME)goto bad_com;
                                    //запись времени 
                                    old_unix=unix;
                                   // set_rlt(2,&buf_rx_ppp[ind+3]);
                                    
                                    
                                    RTC_TimeStructure.Seconds=buf_rx_ppp[ind+3];
                                    RTC_TimeStructure.Minutes=buf_rx_ppp[ind+4];
                                    RTC_TimeStructure.Hours=buf_rx_ppp[ind+5];
                                    
                                    RTC_DateStructure.Date=buf_rx_ppp[ind+6];
                                    RTC_DateStructure.Month=buf_rx_ppp[ind+7];
                                    RTC_DateStructure.Year=buf_rx_ppp[ind+8];
                                    
                                    HAL_RTC_SetTime(&hrtc,&RTC_TimeStructure,RTC_FORMAT_BIN);
                                    HAL_RTC_SetDate(&hrtc,&RTC_DateStructure,RTC_FORMAT_BIN);
                                    
                                    
                                    
                                    
                                    burst_ds_r();
                                    
                                    }
                                    else
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_RD_CONF_TIME)goto bad_com;   
                                    }
                                    
                                  buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];
                                  *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_WR_CONF_TIME+1;
                                  
                                  buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK                                  
                                  burst_ds_r();                                
                                  buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt]=real_time.r_sec;
                                  buf_tx_232[TR_OP_DATA+C1_DATA+5+cnt_bt]=real_time.r_min;
                                  buf_tx_232[TR_OP_DATA+C1_DATA+6+cnt_bt]=real_time.r_hor;
                                  buf_tx_232[TR_OP_DATA+C1_DATA+7+cnt_bt]=real_time.r_date;
                                  buf_tx_232[TR_OP_DATA+C1_DATA+8+cnt_bt]=real_time.r_month;
                                  buf_tx_232[TR_OP_DATA+C1_DATA+9+cnt_bt]=real_time.r_year;
                                  cnt_bt=cnt_bt+L_WR_CONF_TIME+1+3;
                 
                      break;
 
                      
                 case CONF_PDP_R :                                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    goto bad_com;
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=0)goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение                                    
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                  
                                    CopyMass(A_CR_GPRS,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],L_CR_GPRS_MAX);
                                    
                                    goto same_1;
                                    
                                  
                                  //  break;                
                 
                                    
                 case CONF_PDP_DOP_R :                                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+1];
                                    
                                    
                                    
                                    if((temp==1)||(temp==2))goto bad_com;
                                    if(temp>L_CONF_PDP_R)goto bad_com;
                                   
                                    if(temp==0)
                                       {
                                         
                                        // temp=L_CONF_PDP_R;                                
                                        // WrArrayToFlesh(A_CR_GPRS,0,temp,0x01,0x00);  // Отменил операцию очистки  
                                        
                                       }
                                       
                                       else
                                         { 
                                        //запись
                                        framWriteSim800(A_CR_GPRS,&buf_rx_ppp[ind+3],temp);                                  
                                        //записать во флешь признак перезаписи контекста
                                         
                                         }
                                    }
                                    else {
                                            if(*(uint16_t*)&buf_rx_ppp[ind+1]!=0)goto bad_com;
                                            temp=L_CONF_PDP_R;
                                          }
                                     
                                    CopyMass(A_CR_GPRS,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],temp); 
                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind]; 
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=temp+1;                                  
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK 
     
                                    cnt_bt=cnt_bt+temp+1+3;
                                     
                                  break;                       
                      
                      
                 case CONF_KEYS:
                               
                                
                                if((buf_rx_ppp[ind] & 0x80)!=0x80)goto bad_com;
                                if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_WR_CONF_KEYS)goto bad_com;   
                                
                               
                                for(i=0;i<L_KEYS-2;i++)buf[i]=buf_rx_ppp[ind+3+i];
                                *(uint16_t*)&buf[L_KEYS-2]=crc_m1(&buf[0],L_KEYS-2,0xffff);
                                
                                framWriteSim800(A_KEYS,&buf[0],L_KEYS);
                                
           
                                
                                 buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind]; 
                                 *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=1;                                  
                                 buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK 
                                 cnt_bt=cnt_bt+1+3;
                                 
                                 CopyMass(A_KEYS,&keys[0],L_KEYS); // новый ключ
                                 

                      break;
 
                       //!!!!!!!!!!dobavka
                      case CONF_TM_NO_LINK:
                            
                        if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_NO_LINK)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                    
                                    if(temp!=0)
                                      if(temp <= (Control.vol_link/60)){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                    
                                    
                                    vol_cnt_no_link=temp;
                                    vol_cnt_no_link=vol_cnt_no_link*60;
                                    
                                    CopyMass(A_C_PAR,&buf[0],L_C_PAR); 
                                    *(uint16_t*)&buf[OFS_TM_NO_LINK]=temp;
                                    *(uint16_t*)&buf[OFS_C_PAR_CRC]=crc_m1(&buf[0],L_C_PAR-2,0xffff);
                                    framWriteSim800(A_C_PAR, &buf[0],L_C_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_NO_LINK-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_NO_LINK+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_C_PAR+OFS_TM_NO_LINK,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_NO_LINK+1+3;
                         break;
                        
                       case CONF_TM_LINK_RES:
                            
                        if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_LINK_RES)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                    
                                    if(temp < MIN_TM_LINK_RES){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                    
                                    
                                    vol_cnt_link_res=temp;
                                    vol_cnt_link_res=vol_cnt_link_res*60;
                                    
                                    CopyMass(A_C_PAR,&buf[0],L_C_PAR); 
                                    *(uint16_t*)&buf[OFS_TM_LINK_RES]=temp;
                                    *(uint16_t*)&buf[OFS_C_PAR_CRC]=crc_m1(&buf[0],L_C_PAR-2,0xffff);
                                    framWriteSim800(A_C_PAR, &buf[0],L_C_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_LINK_RES-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_LINK_RES+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_C_PAR+OFS_TM_LINK_RES,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_LINK_RES+1+3;
                         break;    
                 
                         
                case CONF_TP_TS:   
                            if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TP_TS)goto bad_com;
                                    //запись
                                    
                                    CopyMass(A_SEQ_PAR,&buf[0],L_SEQ_PAR);
                                    
                                    temp=buf_rx_ppp[ind+3];
                                    if((temp!=0xff)&&(temp>2)){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                    if(temp!=0xff)
                                     {
                                      buf[OFS_TP_TS]=temp; 
                                      
                                      }
                                    
                                    temp=buf_rx_ppp[ind+4];
                                    if ((temp!=0xff)&&(temp>1)){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                    if(temp!=0xff)
                                      {
                                      buf[OFS_TP_TS+1]=temp; 
                                      
                                      }
 
                                    
                                    *(uint16_t*)&buf[OFS_SEQ_PAR_CRC]=crc_m1(&buf[0],L_SEQ_PAR-2,0xffff);
                                    framWriteSim800(A_SEQ_PAR, &buf[0],L_SEQ_PAR);
                                    prov_ozu=crc_ozu();
                                    
                                  
                                    
                                    }
                            
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TP_TS-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TP_TS+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_SEQ_PAR+OFS_TP_TS,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TP_TS+1+3;
                              
                           break;
                         
                 case CONF_TM_NAT_R:  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TM_NAT_R)goto bad_com;
                                    //запись
                                    temp=*(uint16_t*)&buf_rx_ppp[ind+3];
                                    
                                    if (temp<10){buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x01;goto nocorr_com;}
                                    
                                    
                                    Control.vol_nat_r=temp;
                                    CopyMass(A_C_PAR,&buf[0],L_C_PAR); 
                                    *(uint16_t*)&buf[OFS_NAT_R]=temp;
                                    *(uint16_t*)&buf[OFS_C_PAR_CRC]=crc_m1(&buf[0],L_C_PAR-2,0xffff);
                                    framWriteSim800(A_C_PAR, &buf[0],L_C_PAR);
                                    prov_ozu=crc_ozu();
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=(L_CONF_TM_NAT_R-2))goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=L_CONF_TM_NAT_R+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_C_PAR+OFS_NAT_R,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],2);
                                    
                                    cnt_bt=cnt_bt+L_CONF_TM_NAT_R+1+3;
                                  break;        
                  
                case CONF_TRAF :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TRAF)goto bad_com;
                                    //запись
                                    
                                    
                                    cnt_incom=0;
                                    cnt_outcom=0;
                                    *(uint32_t*)&buf[0]=burst_ds_r();
                                    *(uint32_t*)&buf[4]=cnt_outcom;
                                    *(uint32_t*)&buf[8]=cnt_incom;
                                    *(uint16_t*)&buf[L_TRAF-2]=crc_m1(&buf[0],L_TRAF-2,0xffff);
                                    framWriteSim800(A_TRAF, &buf[0],L_TRAF);
                                    }
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TRAF)goto bad_com;}
                                     

                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=(L_TRAF-2)+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    CopyMass(A_TRAF,&buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt],L_TRAF-2);
                                    
                                    cnt_bt=cnt_bt+(L_TRAF-2)+1+3;
                                  break;                     
                  
                  case CONF_TEMPER :  
                 
                                  
                                  if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)goto bad_com;
                                    
                                    else {if(*(uint16_t*)&buf_rx_ppp[ind+1]!=L_CONF_TEMPER)goto bad_com;}
                                   
                                    
                                    buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];  //чтение
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=1+1;   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x00; //OK  
                                    buf_tx_232[TR_OP_DATA+C1_DATA+4+cnt_bt]=tempSensor[0].curTemp=-100;    // temp_to_grad(temperatura); пока закоментарим
                                    cnt_bt=cnt_bt+1+1+3;
                                    
                                  break;                       
                                  
                                  
                 default:     
                  bad_com:buf_tx_232[TR_OP_DATA+C1_DATA+3+cnt_bt]=0x02; //не поддерживается   
                  nocorr_com: 
                         buf_tx_232[TR_OP_DATA+C1_DATA+cnt_bt]=buf_rx_ppp[ind];
                         *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1+cnt_bt]=1;                        
                         cnt_bt=cnt_bt+4;  
                      break;                  
                 }
               
               
 ind=ind+3+*(uint16_t*)&buf_rx_ppp[ind+1]; 
 if(ind>=count_rx_ppp-3)return(cnt_bt);
 goto next_parametr;
 } 








void monitor1_tx_rs232_2(void)
{
  /*
  uint16_t i;
  if(fl_232_2.busy==1)return;
  if((fl_232_2.tx==1)||(Appl_RS232_2.fl_data_buf==TRUE))
    {
    if(fl_232_2.tx==1)fl_232_2.tx=0;
    if(Appl_RS232_2.fl_data_buf==TRUE)Appl_RS232_2.fl_data_buf=FALSE;
    fl_232_2.busy=1;
    fl_tx485.mon232_2=1;
    Appl_RS232_2.dst_tek=Appl_RS232_2.dst_buf;
    Appl_RS232_2.id_tek=Appl_RS232_2.id_buf;
    Rs232_2.cnt_bt_rx_tx=Appl_RS232_2.ln_data_buf;
    for(i=0;i<Rs232_2.cnt_bt_rx_tx;i++)Rs232_2_buf_rx_tx[i]=Appl_RS232_2_data_buf[i];
    Rs232_2.p_data485=&Rs232_2_buf_rx_tx[0];
    //здесь должна быть функция настройки порта по контексту.
    
  
  
    if (run_cont_232_2(&Appl_RS232_2.cont_buf[0])==1)
       {
       fl_232_2.busy=0;
       fl_tx485.mon232_2=0;
       return;
       }
   */ 
  
  // HAL_UART_DeInit(&huart2);
  //    MX_USART2_UART_Init();
  //     HAL_UART_Receive_IT(&huart2, &rs485InputByte, 1); // Каждый раз принимать один байт  
  
  
  if(fl_232_2.tx==0) return;
    fl_232_2.tx=0;
    RS485_DIR(1);
      
      HAL_UART_Transmit_DMA(&huart2,(uint8_t*)&Appl_RS232_2_data_buf,Appl_RS232_2.ln_data_buf);
      LED1_RED(1);  
    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    rs485RxPtr=0;
    rs485DataReady=0;     
     
    
}



///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Работа с портом RS232-2  stm block
/*

отдаю Рустему  заполненную структуру с установленными флагами
fl_232_2.tx==1
fl_232_2.busy=1;
fl_tx485.mon232_2=1;
Приложение Рустема
дожен анализировать флаг fl_232_2.tx==1
если он установлен его нужно сбросить 
проверить контекст если он правильный послать данные в порт
если неправильный 
то
обнулить флаги
fl_232_2.busy=0;
fl_tx485.mon232_2=0;

если данные от устройства пришли то должен взвести флаг
fl_232_2.rec=1
Appl_RS232_2_data_buf[i];
Appl_RS232_2.ln_data_buf;

если не пришли
fl_232_2.tm_out=1


я анализирую флаг
fl_232_2.rec
fl_232_2.tm_out


void monitor1_tx_rs232_2(void)
{
  uint16_t i;
  if(fl_232_2.busy==1)return;
  if((fl_232_2.tx==1)||(Appl_RS232_2.fl_data_buf==TRUE))
    {
    if(fl_232_2.tx==1)fl_232_2.tx=0;
    if(Appl_RS232_2.fl_data_buf==TRUE)Appl_RS232_2.fl_data_buf=FALSE;
    fl_232_2.busy=1;
    fl_tx485.mon232_2=1;
    Appl_RS232_2.dst_tek=Appl_RS232_2.dst_buf;
    Appl_RS232_2.id_tek=Appl_RS232_2.id_buf;
    Rs232_2.cnt_bt_rx_tx=Appl_RS232_2.ln_data_buf;
    for(i=0;i<Rs232_2.cnt_bt_rx_tx;i++)Rs232_2_buf_rx_tx[i]=Appl_RS232_2_data_buf[i];
    Rs232_2.p_data485=&Rs232_2_buf_rx_tx[0];
    //здесь должна быть функция настройки порта по контексту.
    
    if (run_cont_232_2(&Appl_RS232_2.cont_buf[0])==1)
       {
       fl_232_2.busy=0;
       fl_tx485.mon232_2=0;
       return;
       }
     
    RS485_DIR(1);
      
      HAL_UART_Transmit_DMA(&huart2,(uint8_t*)&Appl_RS232_2_data_buf,Appl_RS232_2.ln_data_buf);
      LED1_RED(1);  
    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    rs485RxPtr=0;
    rs485DataReady=0;     
     
    }
}

// эту функцию вызываю если был таймаут
void monitor1_tm_rs232_2(void)
  {
  if(fl_tx485.mon232_2==0)return;
  if(fl_ip.act_ip_end!=1)return; 
  if(fl_232_2.tm_out==0)return;
   
  // __disable_interrupt();
  CLR_RTS2;
  //__enable_interrupt();
  
  clr_cntr_nat();
  clr_cntr_link();
  
  fl_232_2.tm_out=0; 
  if(PINE & DCD0) send_err485(NUM_RS232_2,RS_NO_LINK,Appl_RS232_2.id_tek,Appl_RS232_2.dst_tek);// игнорировать передачу если нет GPRS канала
  fl_232_2.busy=0; 
  fl_tx485.mon232_2=0;
  }



void monitor1_rec_rs232_2(void)
  {
  uint16_t i;
  if(fl_tx485.mon232_2==0)return;
  if(fl_ip.act_ip_end!=1)return; 
  if(fl_232_2.rec==0)return;

  fl_232_2.rec=0;
  
 // __disable_interrupt();
  CLR_RTS2;
 // __enable_interrupt();
  
  clr_cntr_nat();
  clr_cntr_link();
  
  
  if((PINE & DCD0)==0)
  {
     fl_232_2.over=0;         // игнорировать передачу если нет GPRS канала
     Rs232_2.cnt_bt_rx_tx=0;
     fl_232_2.busy=0;
     fl_tx485.mon232_2=0; 
     return; 
  }
  
  
  
  if(fl_232_2.over==1)
     {
     fl_232_2.over=0;
     send_err485(NUM_RS232_2,RS_OVER_BUF_RX,Appl_RS232_2.id_tek,Appl_RS232_2.dst_tek);
     Rs232_2.cnt_bt_rx_tx=0;
     fl_232_2.busy=0;
     fl_tx485.mon232_2=0; 
     return; 
     }
    
    //!!!!!!!!!!!!!! здесь бы надо проверк на длину буфера для PPP
   if(Rs232_2.cnt_bt_rx_tx+70>VOL_TX_PPP)
      {
      send_err485(NUM_RS232_2,RS_OVER_BUF_PPP_TX,Appl_RS232_2.id_tek,Appl_RS232_2.dst_tek);
      Rs232_2.cnt_bt_rx_tx=0;
      fl_232_2.busy=0;
      fl_tx485.mon232_2=0; 
      return; 
      }
    
  fl_ip.act_ip_end=0;
  
  Obj_ppp_tx.prozr=FALSE;            
  Obj_ppp_tx.version=VER2;      //версия 
  Obj_ppp_tx.type_pac=OTV;     //тип пакета
  Obj_ppp_tx.num_src=num_self; // номер отправителя
  Obj_ppp_tx.num_dst=Appl_RS232_2.dst_tek; // номер получателя
  Obj_ppp_tx.id_pac=Appl_RS232_2.id_tek; // идентификатор пакета  
  Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций  
  Obj_ppp_tx.l_opt=0;// длина буфера опций   
  Obj_ppp_tx.kol_opt=0;// количество опций 
  

    buf_tx_232[TR_OP_DATA+C1_PROT]=PROT_C1;
    buf_tx_232[TR_OP_DATA+C1_PORT]=PORT232_2;
   
   

    for(i=0;i<Rs232_2.cnt_bt_rx_tx;i++)buf_tx_232[TR_OP_DATA+C1_DATA+i]=Rs232_2_buf_rx_tx[i];
    Obj_ppp_tx.l_data=2+Rs232_2.cnt_bt_rx_tx;   // длина данных
    Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных 
    Rs232_2.cnt_bt_rx_tx=0;
    
  form_buf_tx_ppp();                            
  UCSR0A=UCSR0A | TXC;                            
  UCSR0B=UCSR0B | TXEN;
  UCSR0B=UCSR0B | TXCIE; 
  fl_232_2.busy=0;
  fl_tx485.mon232_2=0;
  if(check_cts()==1) return;                
  //S2_RD;//
  UDR0=buf_tx_232[0]; 
  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart9,&buf_tx_232[0],1);
    
   // 
  
  return;     
  
  }


*/

///EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE!Работа с портом RS232-2
void monitor_wr_truffic(void)
{
  unsigned char buf[14];
 if(time_cnt_ch_time!=0)return;
 time_cnt_ch_time=CHECK_TIME;  
 burst_ds_r();                                                                
 if ((real_time.r_min==0)&&(fl_truffic.wr==0))
    {
      fl_truffic.wr=1;
     // *(uint32_t*)&buf[0]=burst_ds_r();
      CopyMass(A_TRAF,&buf[0],4);
      *(uint32_t*)&buf[4]=cnt_outcom;
      *(uint32_t*)&buf[8]=cnt_incom;
      *(uint16_t*)&buf[L_TRAF-2]=crc_m1(&buf[0],L_TRAF-2,0xffff);
      framWriteSim800(A_TRAF, &buf[0],L_TRAF);
      return;
    }
 if((real_time.r_min!=0)&&(fl_truffic.wr==1))fl_truffic.wr=0;
}


/*
int proc_temp_to_grad(char t,uint16_t t_cod,uint16_t offset_cod,unsigned char delta_t,unsigned char delta_cod)
 {
   return(t-(t_cod-offset_cod)*delta_t/delta_cod);
 }

char temp_to_grad(uint16_t t)
{

 if(t>230 && t<=319) return(proc_temp_to_grad(60,t,207,5,23));
 if(t>319 && t<=393) return(proc_temp_to_grad(40,t,320,10,73));
 if(t>393 && t<=477) return(proc_temp_to_grad(30,t,394,10,83));
 if(t>477 && t<=706) return(proc_temp_to_grad(20,t,478,25,228));
 if(t>706 && t<=828) return(proc_temp_to_grad(-5,t,707,15,121));
 if(t>828 && t<=891) return(proc_temp_to_grad(-20,t,829,10,62));
 if(t>891 && t<=939) return(proc_temp_to_grad(-30,t,892,10,57));
  return(0x80);
}

*/

/*
char temp_to_grad(uint16_t t)
{
 
  
//  if(t>=207 && t<=230) return(60-(t-207)*5 /23);

 // if(t>230 && t<=287) return(55-(t-231)*10 /56);
 // if(t>287 && t<=319) return(45-(t-288)*5 /31);
  
  if(t>230 && t<=319) return(55-(t-231)*15 /90);
  
 // if(t>319 && t<=355) return(40-(t-320)*5 /35);
 // if(t>355 && t<=393) return(35-(t-356)*5 /37);
    
  if(t>319 && t<=393) return(40-(t-320)*10 /73);
  
  
  //if(t>393 && t<=434) return(30-(t-394)*5 /40);
  //if(t>434 && t<=477) return(25-(t-435)*5 /42);
  
  if(t>393 && t<=477) return(30-(t-394)*10 /83);
  
  if(t>477 && t<=706) return(20-(t-478)*25 /228);

//  if(t>707 && t<=749) return(-5-(t-707)*5 /42);
//  if(t>749 && t<=790) return(-10-(t-750)*5 /40);
//  if(t>790 && t<=828) return(-15-(t-791)*5 /37);
  
  if(t>706 && t<=828) return(-5-(t-707)*15 /121);
  
 
 // if(t>828 && t<=861) return(-20-(t-829)*5 /32);
 // if(t>861 && t<=891) return(-25-(t-862)*5 /29);
  if(t>828 && t<=891) return(-20-(t-829)*10 /62);
   
   
 // if(t>891 && t<=917) return(-30-(t-892)*5 /25);
 // if(t>917 && t<=939) return(-35-(t-918)*5 /21);
  if(t>891 && t<=939) return(-30-(t-892)*10 /57);
 
 // if(t>939 && t<=958) return(-40-(t-940)*5 /18);
  
  
  return(0x80);
}

*/


void send_info(char size,char const *p,unsigned char fl_id,unsigned char id)
{
/*
  char massiv[6];
  
  if(Regim!=RG_DEBAG)return;
           burst_ds_r();
           sprintf(massiv,"%d\0",real_time.r_date);
           mov_massiv(strlen(massiv),massiv);
           s_port('.');
           sprintf(massiv,"%d\0",real_time.r_month);
           mov_massiv(strlen(massiv),massiv);
           s_port('.');
           sprintf(massiv,"%d\0",real_time.r_year);
           mov_massiv(strlen(massiv),massiv);
           s_port(' ');
           sprintf(massiv,"%d\0",real_time.r_hor);
           mov_massiv(strlen(massiv),massiv);
           s_port(':');
           sprintf(massiv,"%d\0",real_time.r_min);
           mov_massiv(strlen(massiv),massiv);
           s_port(':');
           sprintf(massiv,"%d\0",real_time.r_sec);
           mov_massiv(strlen(massiv),massiv);
           s_port(' ');
           
           while(size--) s_port(*p++);
           
           if(fl_id!=0)
           { 
             s_port(' ');
             s_port('I');
             s_port('D');
             s_port(':');
            
             sprintf(massiv,"%d\0",id);
             mov_massiv(strlen(massiv),massiv);
           }
           
           s_port(0x0d);
           s_port(0x0a);
  */
}


