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
#include "ozu_map.h"
#include "def_at.h"
#include "def_link.h"
#include "dfcnst.h"
#include "def_log.h"
#include "map_mbus.h"
#include "fram.h"
#include "sim800.h"
#include "archive.h"


//адоптированный обрезанный стек PPP 14.09.07

#define C_GLUK1    0x55   
#define C_GLUK2    0xaa

extern uint16_t *eventRecordPtr;
extern UART_HandleTypeDef huart3;
extern RTC_TimeTypeDef  RTC_TimeStructure;
extern RTC_DateTypeDef RTC_DateStructure; 

unsigned char zapr_from_sim800;
uint32_t toUnix(RTC_DateTypeDef RTC_DateStructure, RTC_TimeTypeDef RTC_TimeStructure);
uint8_t modbusSlaveTask(uint8_t *inputBuf, uint16_t inputBytesNum,uint8_t *outputBuf, uint16_t *bytesToTransmit);
void    mov_buf (char size,unsigned char * p);
extern unsigned char A_IP_PAR_array[L_IP_PAR];

char MyoutBuffer[5000],MyinBuffer[5000], *p2p;
uint16_t byteSize;
void CopyMass(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);
char MymodbusRegistersEdit( char *mb_req, char *mb_ans, char b,  char *size_ans);
char ext_mbus(unsigned char *buf_rx_ppp,unsigned char offset_in,uint16_t start_adres, uint16_t size,char kod_comand);

extern union {
       unsigned char mb[SEG3*2];//байтовый массив
       } c2_byte; //1я страница конфигурации 

extern union {
       unsigned char mb[SEG99*2];//байтовый массив 
       } c1_byte; // страница конфигурации ГЗУ 

uint16_t swap(uint16_t a)
{
  return (a << 8) | (a >> 8);
}

extern unsigned char obmen_full;
extern char bit_registr1;
extern unsigned char nomer_tab;
extern uint16_t sost;
extern char zadan_ys;




void refresh_a(char m);
//void RdFromFleshToArrInt(uint16_t adres_flesh, uint16_t *adres_ozu,uint16_t num);
//void WrArrayToFleshInt(uint16_t adres_flesh,  uint16_t *adres_ozu, uint16_t num,unsigned char flag,uint16_t znach); //запись конфигурации кп во флеш

unsigned char crc_485(unsigned char num, unsigned char *p);

extern unsigned char bufer[2];

void lock_it(void);
extern void write_log_info(unsigned char sost,unsigned char mesto);


extern void set_rlt(unsigned char address,unsigned char * data);

void  transform_buf(unsigned char *p,uint16_t kol_byte,uint16_t s_rand);


void boot(void);
void r_p_flash(uint16_t adress);
void reload_apl (void);

void send_info(char size,char const *p,unsigned char fl_id,unsigned char id);


//const char gluk[]= {'g','l','u','k'};
//const char gluk1[]= {'g','l','u','k','1'};


const char evc_rld[]=            {'E','V','C',':','r','l','d'};

const char rec_ctrl_ch[]=             {'R','E','C',':','c','t','r','l',' ','c','h'};
const char rec_evc[]=             {'R','E','C',':','e','v','c'};

const char ans_in_ctrl_cl[]={'A','N','S','-','<','c','t','r','l',' ','c','l'};


/*
const char req_in_485_1[]={'R','E','Q','-','<','4','8','5','_','1'};
const char req_in_485_2[]={'R','E','Q','-','<','4','8','5','_','2'};
const char req_in_232[]={'R','E','Q','-','<','2','3','2'};
const char req_in_st_contr[]={'R','E','Q','-','<','s','t',' ','c','o','n','t','r'};
const char req_in_config[]={'R','E','Q','-','<','c','o','n','f','i','g'};
const char req_in_prog[]={'R','E','Q','-','<','p','r','o','g'};
*/

const char ans_out_485_1[]={'A','N','S','-','>','4','8','5','_','1'};
const char ans_out_485_2[]={'A','N','S','-','>','4','8','5','_','2'};
const char ans_out_232[]={'A','N','S','-','>','2','3','2'};
const char ans_out_st_contr[]={'A','N','S','-','>','s','t',' ','c','o','n','t','r'};
const char ans_out_config[]={'A','N','S','-','>','c','o','n','f','i','g'};
const char ans_out_prog[]={'A','N','S','-','>','p','r','o','g'};
const char ans_out_mbus[]={'A','N','S','-','>','m','b','u','s'};

//const char ans_out_ping[]={'A','N','S','-','>','p','i','n','g'};

//const char prov1[]={'P','R','O','V','1'};
//const char prov2[]={'P','R','O','V','2'};

/*
#define NO_MODEL  0x2
#define NO_MAP    0x7
#define DATA_MONTH 0x06
#define DATA_DATA  0x1a
#define GOD       0x11

const  unsigned char new_version[]={0x06,0xcd,0x00,0x07,0x00,NO_MODEL,0x00,NO_MAP,DATA_MONTH,DATA_DATA,0x00,GOD};
*/




extern unsigned char sel_modul;

extern char bit_level,bit_level_psm,sost_psm;
extern uint32_t unix;


//#include "sec.h"

//enum t_version {VER1 = 1 ,VER2,VER3,VER4};
//enum t_type {ZAPR,OTV,SOOB,KVIT};
extern unsigned char simka; //dobavka

extern uint16_t num_self,num_seq_cl,port_udp;
extern unsigned char ip_ls[4]; 



extern uint32_t cnt_no_link, vol_cnt_no_link;//dobavka

void delay(uint16_t period);
extern uint32_t burst_ds_r(void);
uint16_t proc_config(unsigned char *buf_rx_ppp,unsigned char offset,uint16_t count_rx_ppp);
unsigned char check_ln_conf(unsigned char *buf_rx_ppp,unsigned char offset,uint16_t count_rx_ppp);
unsigned char check_cont_485_1 (unsigned char *pointer);
unsigned char check_cont_485_2 (unsigned char *pointer);

extern unsigned char Regim;
unsigned char proc_udp_data(unsigned char *buf_rx_ppp,uint16_t count_rx_ppp);
uint16_t crc_m1(unsigned char *ka,uint16_t num,uint16_t crc);
uint16_t crc_ozu(void);
extern uint16_t prov_ozu;

uint16_t proc_modbus(unsigned char *pnt_buf,unsigned char length);



void clr_cntr_nat(void);
void form_buf_tx_ppp(void);

unsigned char check_cts(void);
void clr_cntr_link(void);

//void framRead(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);
//void WrArrayToFlesh(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num,unsigned char flag,unsigned char znach);

void send_err485(unsigned char port,unsigned char err,unsigned char id,uint16_t dst);


unsigned char compress_off;

extern uint32_t cnt_outcom,cnt_incom;

extern unsigned char point_log_buf;

  struct
  {
  unsigned char  ip         :1;
  unsigned char  udp        :1;
  unsigned char  num_self   :1;
  unsigned char cnt_reset;
  }fl_rewrite;

///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!для РРР
extern  struct
 {
  unsigned char data[VOL_RX_PPP];//сам буфер
  uint16_t ln_data;  // длина данных
  enum bool rec; // пакет принят
  enum bool busy;// буфер занят
  enum bool check_busy;// проверка занят ли буфер
 }Buf1_rx_ppp,Buf2_rx_ppp; // 



extern struct
{
  unsigned char tc:1;
  unsigned char tc_old :1;
}sv1,sv2;




extern enum t_event_modem event_modem;

extern unsigned char buf_tx_232[VOL_TX_PPP];
extern uint16_t count_tx_ppp,vol_tx_ppp;
extern unsigned char layer_PPP;

struct
 {
  unsigned char act_lcp_end :1;  
  unsigned char lcp_tm_out_en :1;
  unsigned char t0_pl:1;
  unsigned char t0_mi:1;
 }fl_lcp; 
 

struct
 {
  unsigned char act_ipcp_end :1;  
  unsigned char ipcp_tm_out_en :1;
  unsigned char t0_pl:1;
  unsigned char t0_mi:1;
 }fl_ipcp; 

struct
 {
  unsigned char act_ip_end :1; 
  unsigned char ip_tm_cntr_en :1; 
  unsigned char ip_tm_nat_en :1;
 }fl_ip;



  
struct
 {
  unsigned char act_pap_end :1;  
  unsigned char pap_tm_out_en:1;
  unsigned char t0_pl:1;
  unsigned char t0_mi:1;  
  unsigned char up: 1; 
 }fl_pap; 


 struct
 {
  unsigned char up :1;  
  unsigned char down :1;
 }fl_lcp1; 


struct
 {
  unsigned char up :1;  
  unsigned char down :1;
 }fl_ipcp1;
 
 struct
 {
  unsigned char nac_rej :1; // reject - 1 
 }fl_lcp2;
   
 struct
 {
  unsigned char nac_rej :1; // reject - 1 
 }fl_ipcp2;
 
 
struct
 {
  unsigned char time_485_1;
  uint16_t client_485_1;
  unsigned char start_485_1;
  unsigned char time_485_2;
  uint16_t client_485_2;
  unsigned char start_485_2;
  unsigned char time_232_2;
  uint16_t client_232_2;
  unsigned char start_232_2;
 }Life; 
 

unsigned char pap_act_buf,PAP_state,PAP_event;
unsigned char LCP_state, LCP_event,LCP_act_buf[LCP_ACT_VOL];
unsigned char IPCP_state, IPCP_event,IPCP_act_buf[IPCP_ACT_VOL];
unsigned char uk_in_act_LCP,uk_out_act_LCP,uk_in_act_IPCP,uk_out_act_IPCP;

unsigned char cnt_rst_lcp,vol_cnt_rst_lcp;
unsigned char cnt_rst_ipcp,vol_cnt_rst_ipcp;                       
unsigned char cnt_rst_pap,vol_cnt_rst_pap;       
                      
                        
uint16_t cnt_lcp_tm_out,cnt_ipcp_tm_out;
uint16_t cnt_pap_tm_out;
uint32_t cnt_ip_tm_cntr;
unsigned char rcvd_protokol;

unsigned char  i_scr_lcp,i_scan_lcp;
unsigned char  i_scj_lcp;//идентификаторы lcp
unsigned char  i_str_lcp,i_sta_lcp;
 
unsigned char  i_scr_ipcp,i_scan_ipcp;
unsigned char  i_scj_ipcp;//идентификаторы ipcp
unsigned char  i_str_ipcp,i_sta_ipcp; 
 
  
unsigned char  i_scr_pap;


unsigned char buf_rejc_opt_lcp[MAX_DL_LCP-6];
unsigned char buf_ack_opt_lcp[MAX_DL_LCP-12];
unsigned char buf_nak_opt_lcp[MAX_DL_LCP-12];
unsigned char buf_rej_opt_lcp[MAX_DL_LCP-12];

unsigned char buf_rejc_opt_ipcp[MAX_DL_IPCP-4];
unsigned char buf_ack_opt_ipcp[MAX_DL_IPCP-10];
unsigned char buf_nak_opt_ipcp[MAX_DL_IPCP-10];
unsigned char buf_rej_opt_ipcp[MAX_DL_IPCP-10];
 

unsigned char ln_rejc_lcp,ln_rej_lcp,ln_ack_lcp,ln_nak_lcp;

unsigned char ln_rejc_ipcp,ln_rej_ipcp,ln_ack_ipcp,ln_nak_ipcp;




extern struct                   //структура, описывающая объект передачи по PPP 
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
 char ip_change;
 extern unsigned char ip_self[4];
 unsigned char ip_pri_dns[4],ip_sec_dns[4]; 
  //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE 

 
 
 
   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!структуры приложения
extern struct
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
}Appl_seq;

extern unsigned char Appl_seq_des;
extern unsigned char state_seq;



extern struct
{
unsigned char en_tx :1;
unsigned char kv_waits :1;
unsigned char en_povtor:1;
}fl_appl_seq;

extern struct
{
unsigned char over_buf :1;
unsigned char send_state :1;
unsigned char enable:1; 
}fl_appl_seq1; 




extern unsigned char cnt_tu1, cnt_tu2;

extern uint16_t modbus_mem1[SEG1];


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


//extern unsigned char byte_state_net;

  //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
 
 uint16_t  delay_pre_tx;


unsigned char reboot_byte;
 
 struct
 {
 unsigned char  end_pg      :1;// флаг последней страницы
 unsigned char  ch_crc      :1;// флаг проверки crc
 unsigned char  send_kv     :1;//флаг отсылки квитка
 unsigned char  cor_pg      :1;// флаг корректной страницы дальше
 unsigned char  first_pg    :1;//флаг первой страницы
 unsigned char  hold_pg     :1;// флаг захвата данных
 unsigned char  send_crc     :1;// флаг посылки опции СРС
 //unsigned char  yes_crc     :1;// верный crc
 }fl_pg_out;
 
 uint16_t pg_crc; //значение crc куска данных
 uint16_t cnt_pg;// реальный счетчик страниц
 unsigned char id_pg_kp;//циклический идентификатор страниц контроллера
 unsigned char id_pg_appl;//циклический идентификатор страниц верхнего приложения
 
 //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
 
 
extern unsigned char Appl_RS485_1_data_buf[LN_BUF_485_1];// сам буфер
extern unsigned char Appl_RS485_2_data_buf[LN_BUF_485_2];// сам буфер
extern unsigned char Appl_RS232_2_data_buf[LN_BUF_232_2];// сам буфер
 
extern struct                   //структура описывающая работу приложения "RS485_1" 
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

/*  
extern struct
 {
 unsigned char mon1 : 1;
 unsigned char mon2 : 1;
 } fl_tx485;
*/ 

extern struct
 {
 unsigned char busy : 1;
 unsigned char rec : 1;
 unsigned char tm_out : 1;
 unsigned char tx : 1;
 unsigned char over : 1;
 unsigned char buffed : 1;
 }fl_485_1,fl_485_2,fl_232_2;

 
 



 struct                   //структура, описывающая объект передачи по PPP 
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

 extern unsigned char buf_opt_tr[20]; 
 
 
 const uint16_t fcstab[256] = {
      0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
      0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
      0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
      0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
      0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
      0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
      0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
      0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
      0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
      0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
      0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
      0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
      0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
      0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
      0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
      0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
      0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
      0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
      0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
      0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
      0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
      0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
      0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
      0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
      0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
      0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
      0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
      0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
      0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
      0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
      0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
      0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
   };

 
 
 
 
 
 

 
 void prov_uk_in(void)
   {        
          unsigned char temp;
          
          uk_in_act_LCP++;
          if(uk_in_act_LCP>=LCP_ACT_VOL)uk_in_act_LCP=0;         
          if(uk_out_act_LCP==0)temp=LCP_ACT_VOL-1;else temp=uk_out_act_LCP-1;
       //   if(uk_in_act_LCP==temp){write_log_info(ST_ERROR,ERR3);lock_it();}//переполнение буфера
         if(uk_in_act_LCP==temp)event_modem=EVM_PPP_ERR;//переполнение буфера
    }  
   
   
   void prov_uk_in_ipcp(void)
   {        
          unsigned char temp;
          
          uk_in_act_IPCP++;
          if(uk_in_act_IPCP>=IPCP_ACT_VOL)uk_in_act_IPCP=0;         
          if(uk_out_act_IPCP==0)temp=IPCP_ACT_VOL-1;else temp=uk_out_act_IPCP-1;
        //  if(uk_in_act_IPCP==temp){write_log_info(ST_ERROR,ERR4);lock_it();}//переполнение буфера
          if(uk_in_act_IPCP==temp)event_modem=EVM_PPP_ERR;//переполнение буфера
         
    }   
 
 
 
 
uint16_t pppfcs16(uint16_t fcs,unsigned char *cp,uint16_t len)

   {

       while (len--)
           fcs=(fcs >> 8) ^ fcstab[(fcs ^ *cp++) & 0xff];
       return (fcs);
   }


 void send_terminate_lcp(void)
 
 {
     union
     {
      unsigned char bytes[2]; 
      uint16_t word;
     }temp;              
                 
                 fl_lcp.act_lcp_end=0;
                 buf_tx_232[0]=0x7e; 
                 buf_tx_232[1]=0xff;
                 buf_tx_232[2]=0x03;
                 buf_tx_232[3]=0xc0;
                 buf_tx_232[4]=0x21;
                 
                 buf_tx_232[5]=TERM_REQ; 
                 buf_tx_232[6]=0;   
                 
                 buf_tx_232[7]=0; 
                 buf_tx_232[8]=0x04;
                 
                 
                 
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],buf_tx_232[8]+4) ^ 0xffff;                
                 buf_tx_232[9]=temp.bytes[0];
                 buf_tx_232[10]=temp.bytes[1];
                 
                 buf_tx_232[11]=0x7e; 
                 
                
                 count_tx_ppp=0;
                 vol_tx_ppp=buf_tx_232[8]+8;               
                
              //   UCSR0A=UCSR0A | TXC; 
              //   UCSR0B=UCSR0B | TXEN;
              //   UCSR0B=UCSR0B | TXCIE;  
                 //S2_RD;//
                // UDR0=buf_tx_232[0];
                 
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                 
                 
}
 
 
 
void run_lcp_act(void)
      {  
       unsigned char i;
       
       union
     {
      unsigned char bytes[2]; 
      uint16_t word;
     }temp;
      
       
       switch (LCP_act_buf[uk_out_act_LCP])
          {
           case TLU:    
                   
                fl_pap.up=1;
                fl_lcp.act_lcp_end=0;
                PAP_state=START_ST;
                layer_PPP=LAYER_PAP;      
                fl_lcp.act_lcp_end=1;
              break;
           case TLD:  
                fl_lcp.act_lcp_end=0;
                layer_PPP=LAYER_LCP;  
                fl_lcp.act_lcp_end=1; 
                break;
           
           case IRC:                   
                fl_lcp.act_lcp_end=0;
                cnt_lcp_tm_out=VOL_LCP_TM_OUT; 
              //  cnt_rst_lcp=vol_cnt_rst_lcp;
                fl_lcp.lcp_tm_out_en=1; 
                fl_lcp.act_lcp_end=1; 
                     
                break;
           
           case SCR:    

                                            
                 fl_lcp.act_lcp_end=0;
                 buf_tx_232[0]=0x7e; 
                 buf_tx_232[1]=0xff;
                 buf_tx_232[2]=0x03;
                 buf_tx_232[3]=0xc0;
                 buf_tx_232[4]=0x21;
                 
                 buf_tx_232[5]=CONF_REQ; 
                 buf_tx_232[6]=i_scr_lcp;   

      //           compress_off=1;               
                 
                 
if(compress_off==0) 
                  {
                   buf_tx_232[7]=0; 
                   buf_tx_232[8]=0x0e;
                  }
                else
                {
                   buf_tx_232[7]=0; 
                   buf_tx_232[8]=0x0a; 
                }
                 
                 buf_tx_232[9]=2; 
                 buf_tx_232[10]=6;
                
                 buf_tx_232[11]=0; 
                 buf_tx_232[12]=0x0a; 
                // buf_tx_232[12]=0; 
                 buf_tx_232[13]=0;
                 buf_tx_232[14]=0;
                
              
  if(compress_off==0) 
      {
                 
                 buf_tx_232[15]=7;
                 buf_tx_232[16]=2;
                 buf_tx_232[17]=8;
                 buf_tx_232[18]=2;       
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],buf_tx_232[8]+4) ^ 0xffff;                
                 buf_tx_232[19]=temp.bytes[0];
                 buf_tx_232[20]=temp.bytes[1];  
                 buf_tx_232[21]=0x7e; 
          
      }
                 
          else
          {
                 
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],buf_tx_232[8]+4) ^ 0xffff;                
                 buf_tx_232[15]=temp.bytes[0];
                 buf_tx_232[16]=temp.bytes[1];
                 buf_tx_232[17]=0x7e;               
          }       
                 
                 
                 
                 count_tx_ppp=0;
                 vol_tx_ppp=buf_tx_232[8]+8;     
                 
        
              
                 
                /*
                 UCSR0A=UCSR0A | TXC; 
                 UCSR0B=UCSR0B | TXEN;
                 UCSR0B=UCSR0B | TXCIE;
                 */
                 if(check_cts()==1) return;  
                
               //  UDR0=buf_tx_232[0];
                  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
             if(check_cts()==1) return;     
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                 
                 
                
                    
                break;
           
          case SCA: 
                 fl_lcp.act_lcp_end=0;
                 buf_tx_232[0]=0x7e; 
                 buf_tx_232[1]=0xff;
                 buf_tx_232[2]=0x03;
                 buf_tx_232[3]=0xc0;
                 buf_tx_232[4]=0x21;  
                 
                 buf_tx_232[5]=CONF_ACK;
                 buf_tx_232[6]=i_scan_lcp;
                 
                 temp.word=ln_ack_lcp+4;
                 buf_tx_232[7]=temp.bytes[1]; 
                 buf_tx_232[8]=temp.bytes[0];
                
                 temp.word=temp.word+4;         
                 vol_tx_ppp=temp.word+4;  
                   
                 for(i=0;i<ln_ack_lcp;i++)buf_tx_232[9+i]=buf_ack_opt_lcp[i];
                 
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;                  
                 buf_tx_232[9+ln_ack_lcp]=temp.bytes[0];
                 buf_tx_232[10+ln_ack_lcp]=temp.bytes[1];
                 buf_tx_232[11+ln_ack_lcp]=0x7e;
                 
                 count_tx_ppp=0;   
                 /*
                 UCSR0A=UCSR0A | TXC;                            
                 UCSR0B=UCSR0B | TXEN;
                 UCSR0B=UCSR0B | TXCIE;
                 if(check_cts()==1) return;   
                 //S2_RD;//
                 UDR0=buf_tx_232[0]; 
                  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart9,&buf_tx_232[0],1);
    
    */
                 if(check_cts()==1) return;     
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
               
                 
                
                  
                break;
           case SCN:  

                 fl_lcp.act_lcp_end=0;
                 buf_tx_232[0]=0x7e; 
                 buf_tx_232[1]=0xff;
                 buf_tx_232[2]=0x03;
                 buf_tx_232[3]=0xc0;
                 buf_tx_232[4]=0x21;   
                 
                 if (fl_lcp2.nac_rej==0) 
                     {
                     buf_tx_232[5]=CONF_NAK;                       
                     
                     buf_tx_232[6]=i_scan_lcp;
                 
                     temp.word=ln_nak_lcp+4;
                     buf_tx_232[7]=temp.bytes[1]; 
                     buf_tx_232[8]=temp.bytes[0];
                    
                    temp.word=temp.word+4;         
                    vol_tx_ppp=temp.word+4;
                     

                      
                     for(i=0;i<ln_nak_lcp;i++)buf_tx_232[9+i]=buf_nak_opt_lcp[i];
                     temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;                  
                     buf_tx_232[9+ln_nak_lcp]=temp.bytes[0];
                     buf_tx_232[10+ln_nak_lcp]=temp.bytes[1];
                     buf_tx_232[11+ln_nak_lcp]=0x7e;
                     
                     count_tx_ppp=0;                             
                   //  UCSR0A=UCSR0A | TXC;                            
                   //  UCSR0B=UCSR0B | TXEN;
                   //  UCSR0B=UCSR0B | TXCIE;
                     if(check_cts()==1) return;   
                    
                     //UDR0=buf_tx_232[0];
                      
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                     
                     
                     }  
                     else
                     { 
                     fl_lcp2.nac_rej=0;
                     buf_tx_232[5]=CONF_REJ;  
                     buf_tx_232[6]=i_scan_lcp;
                 
                     temp.word=ln_rej_lcp+4;
                     buf_tx_232[7]=temp.bytes[1]; 
                     buf_tx_232[8]=temp.bytes[0];
                                 
                    temp.word=temp.word+4;         
                    vol_tx_ppp=temp.word+4; 
                     
                     for(i=0;i<ln_rej_lcp;i++)buf_tx_232[9+i]=buf_rej_opt_lcp[i];
                     temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;                  
                     buf_tx_232[9+ln_rej_lcp]=temp.bytes[0];
                     buf_tx_232[10+ln_rej_lcp]=temp.bytes[1];                     
                     buf_tx_232[11+ln_rej_lcp]=0x7e;                     
                    
                     count_tx_ppp=0;                             
                   //  UCSR0A=UCSR0A | TXC;                            
                   //  UCSR0B=UCSR0B | TXEN;
                   //  UCSR0B=UCSR0B | TXCIE;
                     if(check_cts()==1) return;   
                     //S2_RD;//
                     //UDR0=buf_tx_232[0];
                      
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                     
                     }   
                     

                     
                break; 
           
          }                  
      }



void monitor_act_PAP(void)
      {  
       
       unsigned char k;
       unsigned char temp1;
       char buf[70],*p,*p_name,*p_psw,i;
       
       union
     {
      unsigned char bytes[2]; 
      uint16_t word;
     }temp;
      
       
       if((pap_act_buf==SCR)&&(fl_pap.act_pap_end==1)&&(fl_lcp.act_lcp_end==1)&&(fl_ipcp.act_ipcp_end==1)&&(fl_ip.act_ip_end==1))
        {        
               
                
                
                 pap_act_buf=NO_EVENT;
                  
                 fl_pap.act_pap_end=0;
                 buf_tx_232[0]=0x7e; 
                 buf_tx_232[1]=0xc0;
                 buf_tx_232[2]=0x23;
                 
                 buf_tx_232[3]=CONF_REQ; 
                 
                 i_scr_pap++;
                 buf_tx_232[4]=i_scr_pap;         
                 
                 
                 
                 if(simka==SIM_BASE)
                 {
                 CopyMass(A_C_GPRS,(unsigned char *)&buf[0],L_C_GPRS_MAX);
                 p=&buf[0];
                 temp1=0;
                 while(*p) {p++;temp1++;}
                 p++;
                 p_name=p;
                 
                 temp.bytes[0]=0;
                 while(*p) {p++;temp.bytes[0]++;}
                 p++;
                 p_psw=p;
                 
                  temp.bytes[1]=0;
                  while(*p) {p++;temp.bytes[1]++;}
                  
                  
                 /* 
                 RdFromFleshToArr(A_C_GPRS,&temp1,1);// количество байт APN                 
                 RdFromFleshToArr(A_C_GPRS+temp1+1,&temp.bytes[0],1);// количество логина
                 RdFromFleshToArr(A_C_GPRS+temp1+temp.bytes[0]+2,&temp.bytes[1],1);// количество пароля
                 */
              
                 if((temp.bytes[0]==0)||(temp.bytes[0]>MAX_VOL_US_NAME)) buf_tx_232[7]=0;
                    else {buf_tx_232[7]=temp.bytes[0];
                         // RdFromFleshToArr(A_C_GPRS+temp1+2,&buf_tx_232[8],temp.bytes[0]);
                          for(i=0;i<temp.bytes[0];i++) buf_tx_232[8+i]=*p_name++;
                          }
               
                 if((temp.bytes[1]==0)||(temp.bytes[1]>MAX_VOL_PSW)) buf_tx_232[8+buf_tx_232[7]]=0;
                    else {buf_tx_232[8+buf_tx_232[7]]=temp.bytes[1];
                    //RdFromFleshToArr(A_C_GPRS+temp1+temp.bytes[0]+3,&buf_tx_232[9+buf_tx_232[7]],temp.bytes[1]);
                          for(i=0;i<temp.bytes[1];i++) buf_tx_232[i + 9 + buf_tx_232[7]]=*p_psw++;
                       }
                 }
                 else   //!!!!!!!!!!dobavka
                 {
                    CopyMass(A_CR_GPRS,(unsigned char *)&buf[0],L_CR_GPRS_MAX);
                 p=&buf[0];
                 temp1=0;
                 while(*p) {p++;temp1++;}
                 p++;
                 p_name=p;
                 
                 temp.bytes[0]=0;
                 while(*p) {p++;temp.bytes[0]++;}
                 p++;
                 p_psw=p;
                 
                  temp.bytes[1]=0;
                  while(*p) {p++;temp.bytes[1]++;}
                 /*  
                 RdFromFleshToArr(A_CR_GPRS,&temp1,1);// количество байт APN                 
                 RdFromFleshToArr(A_CR_GPRS+temp1+1,&temp.bytes[0],1);// количество логина
                 RdFromFleshToArr(A_CR_GPRS+temp1+temp.bytes[0]+2,&temp.bytes[1],1);// количество пароля
                 */
              
                 if((temp.bytes[0]==0)||(temp.bytes[0]>MAX_VOL_US_NAME)) buf_tx_232[7]=0;
                    else {buf_tx_232[7]=temp.bytes[0];
                   // RdFromFleshToArr(A_CR_GPRS+temp1+2,&buf_tx_232[8],temp.bytes[0]);
                    for(i=0;i<temp.bytes[0];i++) buf_tx_232[8+i]=*p_name++;
                    }
               
                 if((temp.bytes[1]==0)||(temp.bytes[1]>MAX_VOL_PSW)) buf_tx_232[8+buf_tx_232[7]]=0;
                    else {buf_tx_232[8+buf_tx_232[7]]=temp.bytes[1];
                    //RdFromFleshToArr(A_CR_GPRS+temp1+temp.bytes[0]+3,&buf_tx_232[9+buf_tx_232[7]],temp.bytes[1]);
                    for(i=0;i<temp.bytes[1];i++) buf_tx_232[i + 9 + buf_tx_232[7]]=*p_psw++;
                    }
                 }
                  //!!!!!!!!!!dobavka
                   
                 
                 
                 temp.word=6+buf_tx_232[7]+buf_tx_232[8+buf_tx_232[7]];
                 
                
                  
                 buf_tx_232[5]=temp.bytes[1]; 
                 buf_tx_232[6]=temp.bytes[0]; 
                 
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
                 
                

      
                 count_tx_ppp=0;                            
               //  UCSR0A=UCSR0A | TXC;                            
               //  UCSR0B=UCSR0B | TXEN;
               //  UCSR0B=UCSR0B | TXCIE;
                 if(check_cts()==1) return;    
                 
                 
                 //UDR0=buf_tx_232[0]; 
                  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                 
                

        }
      }




void monitor_event_PAP(void)
    {
     if (fl_pap.up==1)      //UP
         {
         fl_pap.up=0; 
           switch(PAP_state)
               {
                case START_ST:                                                                
                            vol_cnt_rst_pap=VOL_PAP_MC;  
                           
                           // IRC;   //!!!!!!!!!!!!!!111                                            
                            cnt_pap_tm_out=VOL_PAP_TM_OUT; 
                           // cnt_rst_pap=vol_cnt_rst_pap;
                            fl_pap.pap_tm_out_en=1; 
                            fl_pap.act_pap_end=1;  
                            pap_act_buf=SCR;                    
                            PAP_state=SEND_REQ; 
                return;
               }
            
         return;
         }
     
    
     
     if(fl_pap.t0_pl==1)        //TO_PL
        {
         fl_pap.t0_pl=0; 
         switch(PAP_state)
               {                     
                               
                case SEND_REQ:
                           // vol_cnt_rst_pap=VOL_PAP_MC;                                                    
                            cnt_pap_tm_out=VOL_PAP_TM_OUT; 
                            pap_act_buf=SCR;               
                            PAP_state=SEND_REQ;
                            return;
                                                                            
               } 
            
         return;
        }       
       
         
     
     switch (PAP_event)
        {
       
               
       
        case RCA:  
           
        
            PAP_event=NO_EVENT;
            switch(PAP_state)
               {                     

              
                case SEND_REQ:                                                                                                     
                            //TLU;  
                            
                            fl_pap.up=0;  
                            PAP_state=OPEN_ST; 
                            IPCP_state=START_ST;  
                            fl_ipcp.act_ipcp_end=1; 
                            fl_ipcp1.up=1;
                            fl_ipcp1.down=0;
                            layer_PPP=LAYER_IPCP;

                            return;
                                                                                 
               }  
             return; 
        
    
    }
 }




void monitor_act_LCP(void)
      {  
       
       if((uk_in_act_LCP!=uk_out_act_LCP)&&(fl_lcp.act_lcp_end==1)&&(fl_pap.act_pap_end==1)&&(fl_ipcp.act_ipcp_end==1)&&(fl_ip.act_ip_end==1))
        {   
        run_lcp_act();
        uk_out_act_LCP++;  
        if (uk_out_act_LCP>=LCP_ACT_VOL)uk_out_act_LCP=0; 
        }
      }
 
 
  
void monitor_event_LCP(void)
    {
     if (fl_lcp1.up==1)      //UP
         {  
         
          
         fl_lcp1.up=0; 
           switch(LCP_state)
               {
                case START_ST:                                                                

                           // vol_cnt_rst_lcp=VOL_LCP_MC;  
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();                    
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                    
                            LCP_state=SEND_REQ;  
                            
             
                           
                return;
               }
            
         return;
         }
     
    
     
     if(fl_lcp.t0_pl==1)        //TO_PL
        {
         fl_lcp.t0_pl=0; 
         switch(LCP_state)
               {                     
                               
                
                case SEND_REQ:
                          //  vol_cnt_rst_lcp=VOL_LCP_MC;                                                    
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                 
                            LCP_state=SEND_REQ;
                            return;
                case REC_ACK:      
                           // vol_cnt_rst_lcp=VOL_LCP_MC;                                              
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                 
                            LCP_state=SEND_REQ;
                            return;
                case SEND_ACK: 
                           // vol_cnt_rst_lcp=VOL_LCP_MC;                                                   
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                 
                            LCP_state=SEND_ACK;
                            return;                                                              
               } 
            
         return;
        }  
     
     switch (LCP_event)
        {
        case RCR_PL:  
            LCP_event=NO_EVENT; 
            switch(LCP_state)
               {                     
               
                case SEND_REQ:                                                                
                            LCP_act_buf[uk_in_act_LCP]=SCA;  
                            prov_uk_in();                 
                            LCP_state=SEND_ACK;   

                            return;
                case REC_ACK:
        
                            LCP_act_buf[uk_in_act_LCP]=SCA;  
                            prov_uk_in();       
                            LCP_act_buf[uk_in_act_LCP]=TLU;  
                            prov_uk_in();          
                            LCP_state=OPEN_ST;  
                           
                            return;
                case SEND_ACK:                                                                
                            LCP_act_buf[uk_in_act_LCP]=SCA;  
                            prov_uk_in();                 
                            LCP_state=SEND_ACK; 
                   
                            return;
                case OPEN_ST:      
                            LCP_act_buf[uk_in_act_LCP]=TLD;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                        //    vol_cnt_rst_lcp=VOL_LCP_MC; 
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                                                                
                            LCP_act_buf[uk_in_act_LCP]=SCA;  
                            prov_uk_in();                 
                            LCP_state=SEND_ACK; 
                   
                            return;                                                                    
               }  
             return; 
        case RCR_MI: 
            
            LCP_event=NO_EVENT;
            switch(LCP_state)
               {                     

               
                case SEND_REQ:                                                                
                            
                       
                            LCP_act_buf[uk_in_act_LCP]=SCN;  
                            prov_uk_in();                 
                            LCP_state=SEND_REQ;    
                     
                            return;
                case REC_ACK:                                                                
                            LCP_act_buf[uk_in_act_LCP]=SCN;  
                            prov_uk_in();              
                            LCP_state=REC_ACK;     
                           
                            return;
                case SEND_ACK:                                                                
                            LCP_act_buf[uk_in_act_LCP]=SCN;  
                            prov_uk_in();                 
                            LCP_state=SEND_REQ;
                         
                            return;
                case OPEN_ST:      
                            LCP_act_buf[uk_in_act_LCP]=TLD;  
                            prov_uk_in();   
                       //     vol_cnt_rst_lcp=VOL_LCP_MC;
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                                                                
                            LCP_act_buf[uk_in_act_LCP]=SCN;  
                            prov_uk_in();                 
                            LCP_state=SEND_REQ;
                          
                            return;                                                                    
               }  
             return; 
        case RCA:    
 
  
  
            LCP_event=NO_EVENT;
            switch(LCP_state)
               {                     
                
              
                case SEND_REQ: 
                            
                          //  vol_cnt_rst_lcp=VOL_LCP_MC;  
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();                                        
                            LCP_state=REC_ACK; 
                     
                            return;
                case REC_ACK:       
                        //    vol_cnt_rst_lcp=VOL_LCP_MC;                                             
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                 
                            LCP_state=SEND_REQ;
                            return;
                case SEND_ACK:                                                                
                         //   vol_cnt_rst_lcp=VOL_LCP_MC;  
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();                                        
                            LCP_act_buf[uk_in_act_LCP]=TLU;  
                            prov_uk_in();
                            LCP_state=OPEN_ST;

                            return;
                case OPEN_ST:
                        //    vol_cnt_rst_lcp=VOL_LCP_MC;
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=TLD;  
                            prov_uk_in();   
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                                                                               
                            LCP_state=SEND_REQ;
                         
                            return;                                                                    
               }  
             return; 
        case RCN: 
                LCP_event=NO_EVENT;
        switch(LCP_state)
               {                     

                
                case SEND_REQ:                                                                
                         //   vol_cnt_rst_lcp=VOL_LCP_MC;  
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in(); 
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                                       
                            LCP_state=SEND_REQ;
                          
                            return;
                case REC_ACK:     
                        //    vol_cnt_rst_lcp=VOL_LCP_MC;                                               
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                 
                            LCP_state=SEND_REQ;
                          
                            return;
                case SEND_ACK:                                                                
                        //    vol_cnt_rst_lcp=VOL_LCP_MC;  
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();                                        
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();
                            LCP_state=SEND_ACK;
                        
                            return;
                case OPEN_ST:      
                            LCP_act_buf[uk_in_act_LCP]=TLD;  
                            prov_uk_in(); 
                       //     vol_cnt_rst_lcp=VOL_LCP_MC;  
                            LCP_act_buf[uk_in_act_LCP]=IRC;  
                            prov_uk_in();
                            LCP_act_buf[uk_in_act_LCP]=SCR;  
                            prov_uk_in();                                                                               
                            LCP_state=SEND_REQ;
                      
                            return;                                                                    
               }  
                                      
        }
    
    }
  
  
  ////////////////мониторы IPCP        
  
  void run_ipcp_act(void)
      {  
       unsigned char i;
       
       union
     {
      unsigned char bytes[2]; 
      uint16_t word;
     }temp;
      
       
       switch (IPCP_act_buf[uk_out_act_IPCP])
          {
           case TLU:     

                event_modem=EVM_PPP_OK;
 
                layer_PPP=LAYER_IP;   
                fl_ip.act_ip_end=1;   
                fl_ipcp.act_ipcp_end=1; 
             
                  Control.link_waits=FALSE; 
                  Control.link=FALSE;  
                  Control.nat=FALSE;
                
               if(Control.vol_link!=0)fl_ip.ip_tm_cntr_en=1;
               if(Control.vol_nat!=0)fl_ip.ip_tm_nat_en=1;
               Control.cnt_try=0;
               Control.cnt_link=BEG_CNTR_LINK;
               clr_cntr_nat();
               
               Control.cnt_nat=BEG_CNTR_LINK-1; // пустой пакет!!!!
               
              break;
           
          
         
           case IRC:                   
                fl_ipcp.act_ipcp_end=0;
                cnt_ipcp_tm_out=VOL_IPCP_TM_OUT; 
                cnt_rst_ipcp=vol_cnt_rst_ipcp;
                fl_ipcp.ipcp_tm_out_en=1; 
                fl_ipcp.act_ipcp_end=1; 
                     
                break;
           
           case SCR:    

                 fl_ipcp.act_ipcp_end=0;
                 
                 
                 
                 
                 buf_tx_232[0]=0x7e;               
                 
                 
                 if(compress_off==0)
                 {
                 buf_tx_232[1]=0x80;
                 buf_tx_232[2]=0x21;
                 
                 buf_tx_232[3]=CONF_REQ; 
                 buf_tx_232[4]=i_scr_ipcp;   
 
                 
                 buf_tx_232[5]=0;
                 buf_tx_232[6]=10;//без DNS
                  
               //  buf_tx_232[6]=22;//c DNS 
                              
                 buf_tx_232[7]=3;      //запрос IP контроллера
                 buf_tx_232[8]=6;
                 buf_tx_232[9]=ip_self[0]; 
                 buf_tx_232[10]=ip_self[1]; 
                 buf_tx_232[11]=ip_self[2];
                 buf_tx_232[12]=ip_self[3];
                 
                 
               //  buf_tx_232[13]=0x81;      //запрос IP_PRI_DNS
               //  buf_tx_232[14]=6;
               //  buf_tx_232[15]=ip_pri_dns[0]; 
               //  buf_tx_232[16]=ip_pri_dns[1]; 
                // buf_tx_232[17]=ip_pri_dns[2];
                // buf_tx_232[18]=ip_pri_dns[3];
                 
                // buf_tx_232[19]=0x83;      //запрос IP_SEC_DNS
                // buf_tx_232[20]=6;
                // buf_tx_232[21]=ip_sec_dns[0]; 
                // buf_tx_232[22]=ip_sec_dns[1]; 
                // buf_tx_232[23]=ip_sec_dns[2];
                // buf_tx_232[24]=ip_sec_dns[3];
                                  
                 
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],buf_tx_232[6]+2) ^ 0xffff;                  
                 buf_tx_232[13]=temp.bytes[0];//без DNS
                 buf_tx_232[14]=temp.bytes[1];//без DNS
                 
                 buf_tx_232[15]=0x7e; //без DNS
               //  buf_tx_232[25]=temp.bytes[0];//c DNS 
               //  buf_tx_232[26]=temp.bytes[1]; //c DNS              
               //  buf_tx_232[27]=0x7e; //c DNS 
                
               
                 vol_tx_ppp=buf_tx_232[6]+6;               
                 }
                 
                 
                 else
                 {
                 buf_tx_232[1]=0xff;
                 buf_tx_232[2]=0x03;
                 buf_tx_232[3]=0x80;
                 buf_tx_232[4]=0x21;
                 
                 buf_tx_232[5]=CONF_REQ; 
                 buf_tx_232[6]=i_scr_ipcp;   
 
                 
                 buf_tx_232[7]=0;
                 buf_tx_232[8]=10;//без DNS
                  
               //  buf_tx_232[6]=22;//c DNS 
                              
                 buf_tx_232[9]=3;      //запрос IP контроллера
                 buf_tx_232[10]=6;
                 buf_tx_232[11]=ip_self[0]; 
                 buf_tx_232[12]=ip_self[1]; 
                 buf_tx_232[13]=ip_self[2];
                 buf_tx_232[14]=ip_self[3];
                 
                 
               //  buf_tx_232[13]=0x81;      //запрос IP_PRI_DNS
               //  buf_tx_232[14]=6;
               //  buf_tx_232[15]=ip_pri_dns[0]; 
               //  buf_tx_232[16]=ip_pri_dns[1]; 
                // buf_tx_232[17]=ip_pri_dns[2];
                // buf_tx_232[18]=ip_pri_dns[3];
                 
                // buf_tx_232[19]=0x83;      //запрос IP_SEC_DNS
                // buf_tx_232[20]=6;
                // buf_tx_232[21]=ip_sec_dns[0]; 
                // buf_tx_232[22]=ip_sec_dns[1]; 
                // buf_tx_232[23]=ip_sec_dns[2];
                // buf_tx_232[24]=ip_sec_dns[3];
                                  
                 
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],buf_tx_232[8]+4) ^ 0xffff;                  
                 buf_tx_232[15]=temp.bytes[0];//без DNS
                 buf_tx_232[16]=temp.bytes[1];//без DNS
                 
                 buf_tx_232[17]=0x7e; //без DNS
               //  buf_tx_232[25]=temp.bytes[0];//c DNS 
               //  buf_tx_232[26]=temp.bytes[1]; //c DNS              
               //  buf_tx_232[27]=0x7e; //c DNS 
               
                 vol_tx_ppp=buf_tx_232[8]+8;               
                 }
                 
                 
                  count_tx_ppp=0;
                 
               //  UCSR0A=UCSR0A | TXC;                            
               //  UCSR0B=UCSR0B | TXEN;
               //  UCSR0B=UCSR0B | TXCIE;
                 if(check_cts()==1) return;   
                 
               //  UDR0=buf_tx_232[0]; 
                  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                 
               
                 
               
                break; 
                 
           case SCA: 
                 fl_ipcp.act_ipcp_end=0;
                 buf_tx_232[0]=0x7e; 
                 buf_tx_232[1]=0x80;
                 buf_tx_232[2]=0x21;
                                 
                 buf_tx_232[3]=CONF_ACK;
                 buf_tx_232[4]=i_scan_ipcp;
                 
                 temp.word=ln_ack_ipcp+4;
                 buf_tx_232[5]=temp.bytes[1]; 
                 buf_tx_232[6]=temp.bytes[0];
                
                 temp.word=temp.word+2;         
                 vol_tx_ppp=temp.word+4;  
                  
                 for(i=0;i<ln_ack_ipcp;i++)buf_tx_232[7+i]=buf_ack_opt_ipcp[i];
                 
                 
                 
                  if(compress_off==1)
                 {
                   for (i=0;i < (vol_tx_ppp-1);i++)buf_tx_232[vol_tx_ppp+1-i]=buf_tx_232[vol_tx_ppp-1-i];
                   buf_tx_232[1]=0xff;
                   buf_tx_232[2]=0x03;
                   temp.word=temp.word+2;
                   vol_tx_ppp=temp.word+4;
                 }
                 
                 
                 
                 
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;                  
                 
                 
                 /*
                 buf_tx_232[7+ln_ack_ipcp]=temp.bytes[0];
                 buf_tx_232[8+ln_ack_ipcp]=temp.bytes[1];
                 buf_tx_232[9+ln_ack_ipcp]=0x7e;
                 */
               
                 buf_tx_232[vol_tx_ppp-3]=temp.bytes[0];
                 buf_tx_232[vol_tx_ppp-2]=temp.bytes[1];
                 buf_tx_232[vol_tx_ppp-1]=0x7e;
             
                 
                 
                 count_tx_ppp=0;                             
                   //  UCSR0A=UCSR0A | TXC;                            
                   //  UCSR0B=UCSR0B | TXEN;
                   //  UCSR0B=UCSR0B | TXCIE;
                     if(check_cts()==1) return;
                
                // UDR0=buf_tx_232[0]; 
                  
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                 
               
                
                  
                break;
           case SCN:  
                
                

                 fl_ipcp.act_ipcp_end=0;
                 buf_tx_232[0]=0x7e; 
                 buf_tx_232[1]=0x80;
                 buf_tx_232[2]=0x21;

                 
                 if (fl_ipcp2.nac_rej==0) 
                     {
                     buf_tx_232[3]=CONF_NAK;                       
                     
                     buf_tx_232[4]=i_scan_ipcp;
                 
                     temp.word=ln_nak_ipcp+4;
                     buf_tx_232[5]=temp.bytes[1]; 
                     buf_tx_232[6]=temp.bytes[0];
                    
                    temp.word=temp.word+2;         
                    vol_tx_ppp=temp.word+4;
                     

                      
                     for(i=0;i<ln_nak_ipcp;i++)buf_tx_232[7+i]=buf_nak_opt_ipcp[i];
                     temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;                  
                     buf_tx_232[7+ln_nak_ipcp]=temp.bytes[0];
                     buf_tx_232[8+ln_nak_ipcp]=temp.bytes[1];
                     buf_tx_232[9+ln_nak_ipcp]=0x7e;
                     
                     count_tx_ppp=0;                             
                  //   UCSR0A=UCSR0A | TXC;                            
                  //   UCSR0B=UCSR0B | TXEN;
                  //   UCSR0B=UCSR0B | TXCIE;
                     if(check_cts()==1) return;
                     
                     //UDR0=buf_tx_232[0];
                      
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                     
                     
                     }  
                     else
                     { 
                     fl_ipcp2.nac_rej=0;
                     buf_tx_232[3]=CONF_REJ;  
                     buf_tx_232[4]=i_scan_ipcp;
                 
                     temp.word=ln_rej_ipcp+4;
                     buf_tx_232[5]=temp.bytes[1]; 
                     buf_tx_232[6]=temp.bytes[0];
                                 
                    temp.word=temp.word+2;         
                    vol_tx_ppp=temp.word+4; 
                     
                     for(i=0;i<ln_rej_ipcp;i++)buf_tx_232[7+i]=buf_rej_opt_ipcp[i];
                     
                     
                  if(compress_off==1)
                 {
                   for (i=0;i < (vol_tx_ppp-1);i++)buf_tx_232[vol_tx_ppp+1-i]=buf_tx_232[vol_tx_ppp-1-i];
                   buf_tx_232[1]=0xff;
                   buf_tx_232[2]=0x03;
                   temp.word=temp.word+2;
                   vol_tx_ppp=temp.word+4;
                 }
                 
                 
                 
                 
                 temp.word=pppfcs16(PPPINITFCS16,&buf_tx_232[1],temp.word) ^ 0xffff;                  
                 
                 
                 /*
                     buf_tx_232[7+ln_rej_ipcp]=temp.bytes[0];
                     buf_tx_232[8+ln_rej_ipcp]=temp.bytes[1];                     
                     buf_tx_232[9+ln_rej_ipcp]=0x7e;        
                 */
               
                 buf_tx_232[vol_tx_ppp-3]=temp.bytes[0];
                 buf_tx_232[vol_tx_ppp-2]=temp.bytes[1];
                 buf_tx_232[vol_tx_ppp-1]=0x7e;
                     
                            
                              
                    
                     count_tx_ppp=0;                             
                  //   UCSR0A=UCSR0A | TXC;                            
                  //   UCSR0B=UCSR0B | TXEN;
                  //   UCSR0B=UCSR0B | TXCIE;
                     if(check_cts()==1) return;                 
                     
                    // UDR0=buf_tx_232[0]; 
                      
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                     
                     }   
                     

                     
                break; 
          
                     
          
          }                  
      }
  
  

  
   
  void monitor_act_IPCP(void)
      {  
       
       if((uk_in_act_IPCP!=uk_out_act_IPCP)&&(fl_lcp.act_lcp_end==1)&&(fl_pap.act_pap_end==1)&&(fl_ipcp.act_ipcp_end==1)&&(fl_ip.act_ip_end==1))
        {   
        run_ipcp_act();
        uk_out_act_IPCP++;  
        if (uk_out_act_IPCP>=IPCP_ACT_VOL)uk_out_act_IPCP=0; 
        }
      }
 
 
  
void monitor_event_IPCP(void)
    {
     if (fl_ipcp1.up==1)      //UP
         {  
         
          
         fl_ipcp1.up=0; 
           switch(IPCP_state)
               {
                case START_ST:                                                                
                            //vol_cnt_rst_ipcp=VOL_IPCP_MC;  
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp();                    
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                    
                            IPCP_state=SEND_REQ; 
                           
                return;
               }
            
         return;
         }
     
    
     
     if(fl_ipcp.t0_pl==1)        //TO_PL
        {
         fl_ipcp.t0_pl=0; 
         switch(IPCP_state)
               {                     
                               
                
                case SEND_REQ:
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;                                                    
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp();                
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_REQ;
                            return;
                case REC_ACK:      
                            //vol_cnt_rst_ipcp=VOL_IPCP_MC;                                              
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp();                
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_REQ;
                            return;
                case SEND_ACK: 
                          //  vol_cnt_rst_ipcp=VOL_IPCP_MC;                                                   
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp();                 
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_ACK;
                            return;                                                              
               } 
            
         return;
        }  
   
     switch (IPCP_event)
        {
        case RCR_PL:  
            IPCP_event=NO_EVENT; 
            switch(IPCP_state)
               {                     

               
                case SEND_REQ:                                                                
                            IPCP_act_buf[uk_in_act_IPCP]=SCA;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_ACK;   
                         
                            return;
                case REC_ACK:
        
                            IPCP_act_buf[uk_in_act_IPCP]=SCA;  
                            prov_uk_in_ipcp();       
                            IPCP_act_buf[uk_in_act_IPCP]=TLU;  
                            prov_uk_in_ipcp();          
                            IPCP_state=OPEN_ST;  
                           
                            return;
                case SEND_ACK:                                                                
                            IPCP_act_buf[uk_in_act_IPCP]=SCA;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_ACK; 
                   
                            return;
                case OPEN_ST:      
                                                                              
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp(); 
                            IPCP_act_buf[uk_in_act_IPCP]=SCA;  
                            prov_uk_in_ipcp();                 
                   
                            return;                                                                    
               }  
             return; 
        case RCR_MI: 
           
            
            
            IPCP_event=NO_EVENT;
            switch(IPCP_state)
               {                     

               
                case SEND_REQ:                                                                
                            
                       
                            IPCP_act_buf[uk_in_act_IPCP]=SCN;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_REQ;    
                     
                            return;
                case REC_ACK:                                                                
                            IPCP_act_buf[uk_in_act_IPCP]=SCN;  
                            prov_uk_in_ipcp();              
                            IPCP_state=REC_ACK;     
                           
                            return;
                case SEND_ACK:                                                                
                            IPCP_act_buf[uk_in_act_IPCP]=SCN;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_REQ;
                         
                            return;
                
               /*
               case OPEN_ST:      
                          
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp(); 
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                                                                
                            IPCP_act_buf[uk_in_act_IPCP]=SCN;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_REQ;
               
                            
                            return;  
                 */           
               }  
             return; 
        case RCA:  
            
            
           
            
            IPCP_event=NO_EVENT;
            switch(IPCP_state)
               {                     

                
                case SEND_REQ: 
                            
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;  
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp();                                        
                            IPCP_state=REC_ACK; 
                     
                            return;
                case REC_ACK:       
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp(); 
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;                                             
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_REQ;
                     
                            return;
                case SEND_ACK:                                                                

                            
                            vol_cnt_rst_ipcp=VOL_IPCP_MC;  
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp();                                        
                            IPCP_act_buf[uk_in_act_IPCP]=TLU;  
                            prov_uk_in_ipcp();
                            IPCP_state=OPEN_ST;
                  
                            return;
               /*
               case OPEN_ST:
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;
              //подумать    ?????????????????????????????
                           
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp(); 
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                                                                               
                            IPCP_state=SEND_REQ;
                         
                            
                            return;
                            */
                            
               }  
             return; 
        case RCN: 
                IPCP_event=NO_EVENT;
        switch(IPCP_state)
               {                     

                
                case SEND_REQ:                                                                
                            vol_cnt_rst_ipcp=VOL_IPCP_MC;  
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp(); 
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                                       
                            IPCP_state=SEND_REQ;
                          
                            return;
                case REC_ACK:     
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;                                               
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp(); 
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                 
                            IPCP_state=SEND_REQ;
                          
                            return;
                case SEND_ACK:                                                                
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;   
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp();                                        
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();
                            IPCP_state=SEND_ACK;
                        
                            return;
                
               case OPEN_ST:      
                            IPCP_act_buf[uk_in_act_IPCP]=IRC;  
                            prov_uk_in_ipcp(); 
                           // vol_cnt_rst_ipcp=VOL_IPCP_MC;  
                            IPCP_act_buf[uk_in_act_IPCP]=SCR;  
                            prov_uk_in_ipcp();                                                                               
                            IPCP_state=SEND_REQ;
                      
                            return;  
                            
                            
               }  
             return; 
       
       
                            
        }
    
    }
  
  
  ////////////////  конец мониторы IPCP 

  uint16_t calc_crc_ip(unsigned char *p,uint16_t count)
      {      
      
          union          
          {
           unsigned char bytes[2]; 
           uint16_t word;
          }temp1; 
         
          union
          {
           unsigned char bytes[4]; 
           uint32_t long_word;
          }temp2; 
       
         temp1.word=0;
         temp2.long_word=0;

        while( count > 1 )  {
           /*  This is the inner loop */  
               temp1.bytes[0]=*p;
               temp1.bytes[1]=*(p+1); 
               temp2.long_word=temp2.long_word+temp1.word; 
               p=p+2;         
               count -= 2;
                            }

           /*  Add left-over byte, if any */
       if( count > 0 )   
               {
               temp1.bytes[0]=0;
               temp1.bytes[1]=*p; 
               temp2.long_word=temp2.long_word+temp1.word; 
               } 
          
    
           /*  Fold 32-bit sum to 16 bits */
       while (temp2.long_word>>16)
           temp2.long_word = (temp2.long_word & 0xffff) + (temp2.long_word >> 16);

       temp1.bytes[0]=~temp2.bytes[0];
       temp1.bytes[1]=~temp2.bytes[1];  
       
  
          
       return(temp1.word);
      }
       



 
 uint16_t calc_crc_udp_2(unsigned char *buf_rx_ppp,uint16_t count)
      {      
          unsigned char *p;
      
          union          
          {
           unsigned char bytes[2]; 
           uint16_t word;
          }temp1; 
         
          union
          {
           unsigned char bytes[4]; 
           uint32_t long_word;
          }temp2; 
         p=buf_rx_ppp+22;
         temp1.word=0;
         temp2.long_word=0;

        while( count > 1 )  {
           /*  This is the inner loop */  
               temp1.bytes[0]=*p;
               temp1.bytes[1]=*(p+1); 
               temp2.long_word=temp2.long_word+temp1.word; 
               p=p+2;         
               count -= 2;
                            }

           /*  Add left-over byte, if any */
       if( count > 0 )   
               {
               //temp1.bytes[0]=0;
               //temp1.bytes[1]=*p; 
               temp1.bytes[0]=*p; 
               temp1.bytes[1]=0;
               temp2.long_word=temp2.long_word+temp1.word; 
               } 
          
          
          
          temp1.bytes[0]=buf_rx_ppp[14];
          temp1.bytes[1]=buf_rx_ppp[15]; 
          temp2.long_word=temp2.long_word+temp1.word;  
          temp1.bytes[0]=buf_rx_ppp[16];
          temp1.bytes[1]=buf_rx_ppp[17]; 
          temp2.long_word=temp2.long_word+temp1.word;  
          
          temp1.bytes[0]=buf_rx_ppp[18];
          temp1.bytes[1]=buf_rx_ppp[19]; 
          temp2.long_word=temp2.long_word+temp1.word;  
          temp1.bytes[0]=buf_rx_ppp[20];
          temp1.bytes[1]=buf_rx_ppp[21]; 
          temp2.long_word=temp2.long_word+temp1.word;  
          
                     
      
          temp1.bytes[0]=0;
          temp1.bytes[1]=0x11; 
          temp2.long_word=temp2.long_word+temp1.word;  
           
          temp1.bytes[0]=buf_rx_ppp[26];                                //длина
          temp1.bytes[1]=buf_rx_ppp[27]; 
          temp2.long_word=temp2.long_word+temp1.word; 
         
      
          
         
          
    
           /*  Fold 32-bit sum to 16 bits */
       while (temp2.long_word>>16)
           temp2.long_word = (temp2.long_word & 0xffff) + (temp2.long_word >> 16);

      temp1.bytes[0]=~temp2.bytes[0];
      temp1.bytes[1]=~temp2.bytes[1]; 
          
       return(temp1.word);
      }
 
 
 
 
 unsigned char proc_ppp_packet(unsigned char *buf_rx_ppp,uint16_t count_rx_ppp)
 {                
    uint16_t i;
    unsigned char vol_length_opt;
    unsigned char ind_opt,count_length_opt;
     
     struct
 {
  unsigned char mru :1;  
  unsigned char accm :1; 
  unsigned char aut_prot :1; 
  unsigned char pfc :1;
  unsigned char acfc :1;
  unsigned char zap_opt:1;
  unsigned char ip_self:1;
  unsigned char ip_pri_dns:1;
 }fl_opt;     
                           
  
  
  union
 {
 unsigned char bytes[2]; 
 uint16_t word;
 } Crc_out,temp,length;
    
    
   
 
  if(count_rx_ppp<=5)return(1); 
   
    //подсчет FCS      
  if (pppfcs16(PPPINITFCS16,&buf_rx_ppp[1],count_rx_ppp-2)!=PPPGOODFCS16)return(1);       
 
 
  if ((buf_rx_ppp[1]==0xff)&&(buf_rx_ppp[2]==0x03)&&(buf_rx_ppp[3]==0xc0)&&(buf_rx_ppp[4]==0x21)&&(layer_PPP!=LAYER_HW))rcvd_protokol=PR_LCP; 
  
  
  if(compress_off==1)
  {
    if ((buf_rx_ppp[1]==0xff)&&(buf_rx_ppp[2]==0x03)&&(buf_rx_ppp[3]==0xc0)&&(buf_rx_ppp[4]==0x23)&&(layer_PPP==LAYER_PAP))
    {  
    rcvd_protokol=PR_PAP;
    for(i=0;i<(count_rx_ppp-3);i++) buf_rx_ppp[1+i]= buf_rx_ppp[3+i];
    count_rx_ppp=count_rx_ppp-2;
    }
    
   if ((buf_rx_ppp[1]==0xff)&&(buf_rx_ppp[2]==0x03)&&(buf_rx_ppp[3]==0x80)&&(buf_rx_ppp[4]==0x21)&&((layer_PPP==LAYER_IP)||(layer_PPP==LAYER_IPCP)))
    {  
    rcvd_protokol=PR_IPCP; 
    for(i=0;i<(count_rx_ppp-3);i++) buf_rx_ppp[1+i]= buf_rx_ppp[3+i];
    count_rx_ppp=count_rx_ppp-2;
    }
    
    if ((buf_rx_ppp[1]==0xff)&&(buf_rx_ppp[2]==0x03)&&(buf_rx_ppp[3]==0x00)&&(buf_rx_ppp[4]==0x21)&&(layer_PPP==LAYER_IP))
    {
      rcvd_protokol=PR_IP;
      for(i=0;i<(count_rx_ppp-4);i++) buf_rx_ppp[1+i]= buf_rx_ppp[4+i];
      count_rx_ppp=count_rx_ppp-3;
    }
    
  }
  else
     {
     if ((buf_rx_ppp[1]==0xc0)&&(buf_rx_ppp[2]==0x23)&&(layer_PPP==LAYER_PAP))rcvd_protokol=PR_PAP;
     if ((buf_rx_ppp[1]==0x80)&&(buf_rx_ppp[2]==0x21)&&((layer_PPP==LAYER_IP)||(layer_PPP==LAYER_IPCP)))rcvd_protokol=PR_IPCP; 
     if ((buf_rx_ppp[1]==0x21)&&(layer_PPP==LAYER_IP))rcvd_protokol=PR_IP;     
     }
  
  
  
  
  

 
  
     
     switch (rcvd_protokol) 
      {
       case PR_LCP:
 
           
              // здесь  проверка общей длины     
              length.bytes[1]=buf_rx_ppp[7];
              length.bytes[0]=buf_rx_ppp[8];
              if (length.word!=count_rx_ppp-8)return(1);
              if (count_rx_ppp>MAX_DL_LCP)return(1);//не воспринимать длинные пакеты                           
              
              vol_length_opt=count_rx_ppp-12;
              count_length_opt=0;
              ind_opt=9;  
             
              ln_rejc_lcp=0;
              ln_ack_lcp=0;
              ln_nak_lcp=0;
              ln_rej_lcp=0; 
              
                                  
             
                           
              switch (buf_rx_ppp[5])      
                  {
                  case CONF_REQ :          
                 // (если вообще не те опции то сброс кода дать) 
                      //проверка предложенной конфигурации от сервера    
                      //если нужная конфигурация то событие RCR+                  
                      //если нет то RCR-;                       
                    
             /*
                   for(i=0;i<vol_length_opt;i++)              //соглашение на любую конфигурацию
                       {
                       buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt];
                       ln_ack_lcp++;
                       //count_length_opt++;
                       ind_opt++;
                       }
                  
                  LCP_event=RCR_PL;
                  i_scan_lcp=buf_rx_ppp[6];    // присвоение идентификатора                   
                  return(1); 
                     */
                
                    fl_opt.zap_opt=0;  
                
next_opt_lcp_11:  if(count_length_opt==vol_length_opt) goto end_pcp_opt_11;
                                                                //обработаны все опции
        switch(buf_rx_ppp[ind_opt])
        {
           case AUT_PROT:

                if ((buf_rx_ppp[ind_opt+2]!=0xc0)||(buf_rx_ppp[ind_opt+3]!=0x23))
                {
                  fl_opt.zap_opt=1;
                  
                                          
                  buf_nak_opt_lcp[ln_nak_lcp]=3;
                  ln_nak_lcp++;                                        
                  buf_nak_opt_lcp[ln_nak_lcp]=4;
                  ln_nak_lcp++;
                  buf_nak_opt_lcp[ln_nak_lcp]=0xc0; 
                  ln_nak_lcp++;                     
                  buf_nak_opt_lcp[ln_nak_lcp]=0x23; 
                  ln_nak_lcp++;        
                 
                //  count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                //  ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];      
                }
                else for(i=0;i<buf_rx_ppp[ind_opt+1];i++){buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+i];ln_ack_lcp++;}
                
            count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
            ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];     
            goto next_opt_lcp_11;                                  
           
             default:             
             for(i=0;i<buf_rx_ppp[ind_opt+1];i++){buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+i];ln_ack_lcp++;}
             for(i=0;i<buf_rx_ppp[ind_opt+1];i++){buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+i];ln_nak_lcp++;}
             count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
             ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];   
             goto next_opt_lcp_11; 
        }
                         
   end_pcp_opt_11:                     
                
                  if(fl_opt.zap_opt==0)LCP_event=RCR_PL;else LCP_event=RCR_MI;
                  fl_opt.zap_opt=0;
                  i_scan_lcp=buf_rx_ppp[6];    // присвоение идентификатора                   
                  return(1); 
                     
     /*              
                  
       next_opt_lcp_1:  if(count_length_opt==vol_length_opt) { 
    
                                                             goto end_pcp_opt_1;
                                                             }   //обработаны все опции
                       
                       switch(buf_rx_ppp[ind_opt])
                          {
                            case MRU: 
   
                                 if (buf_rx_ppp[ind_opt+1]!=4)
                                      {                                        
                                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);
                                        } 
                                 if ((buf_rx_ppp[ind_opt+2]==DL_PPP_IN_HI)&&(buf_rx_ppp[ind_opt+3]==DL_PPP_IN_LO)) 
                                       {    
                                        fl_opt.mru=1;
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt];
                                        ln_ack_lcp++;
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+1];
                                        ln_ack_lcp++;
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+2]; 
                                        ln_ack_lcp++;  
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+3]; 
                                        ln_ack_lcp++; 
                                       }
                                     else   
                                        {
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt];
                                        ln_nak_lcp++;
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+1];
                                        ln_nak_lcp++;
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+2]; 
                                        ln_nak_lcp++;  
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+3]; 
                                        ln_nak_lcp++; 
                                        }  
                                 count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                 ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];
                                 goto next_opt_lcp_1;   
                                 
                            case ACCM:       
                                  if (buf_rx_ppp[ind_opt+1]!=6)
                                      {
                                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);
                                        } 
                                    
                                    if ( 
                                           (buf_rx_ppp[ind_opt+2]==0)&&
                                           (buf_rx_ppp[ind_opt+3]==0)&& 
                                           (buf_rx_ppp[ind_opt+4]==0)&&
                                           (buf_rx_ppp[ind_opt+5]==0) 
                                        ) 
                                       
                                       {   
                                       
                                        fl_opt.accm=1;
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt];
                                        ln_ack_lcp++;
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+1];
                                        ln_ack_lcp++;
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+2]; 
                                        ln_ack_lcp++;  
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+3]; 
                                        ln_ack_lcp++; 
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+4]; 
                                        ln_ack_lcp++;
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+5]; 
                                        ln_ack_lcp++;
                                       }
                                     else   
                                        {
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt];
                                        ln_nak_lcp++;
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+1];
                                        ln_nak_lcp++;
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+2]; 
                                        ln_nak_lcp++;  
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+3]; 
                                        ln_nak_lcp++;    
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+4]; 
                                        ln_nak_lcp++;  
                                        
                                        buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+5]; 
                                        ln_nak_lcp++; 
                                        }  
                                 count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                 ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];
                                 goto next_opt_lcp_1;                                       

                                     
                            
                            case AUT_PROT:
                                      if (((buf_rx_ppp[ind_opt+1]-2)%2)!=0) 
                                          {
                                           for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                           LCP_event=RUC;
                                           return(1);
                                          }  
                                      for(i=0;i<(buf_rx_ppp[ind_opt+1]-2);i=i+2)               
                                         {
                                         if ((buf_rx_ppp[ind_opt+2+i]==0xc0)&&(buf_rx_ppp[ind_opt+3+i]==0x23)) 
                                              {
                                               
                                               fl_opt.aut_prot=1;
                                               
                                               buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt];
                                               ln_ack_lcp++;
                                   
                                               buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+1];
                                               ln_ack_lcp++;
                                        
                                               buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+2+i]; 
                                               ln_ack_lcp++;  
                                        
                                               buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+3+i]; 
                                               ln_ack_lcp++; 
                                              }
                                            else   
                                             {
                                             
                                             if (fl_opt.zap_opt==0)
                                                 {
                                                  fl_opt.zap_opt=1;                                            
                                                  buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt];
                                                  ln_nak_lcp++;                                        
                                                  buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+1];
                                                  ln_nak_lcp++;
                                                  }
                                             buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+2+i]; 
                                             ln_nak_lcp++;                     
                                              buf_nak_opt_lcp[ln_nak_lcp]=buf_rx_ppp[ind_opt+3+i]; 
                                             ln_nak_lcp++;                                             
                                            }              
                                            
                                           }  
                                 count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                 ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];          
                                 goto next_opt_lcp_1;                   
                                  
                            case PFC:
                                  if (buf_rx_ppp[ind_opt+1]!=2)
                                      {                                        
                                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);
                                        } 


                                        fl_opt.pfc=1;
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt];
                                        ln_ack_lcp++;
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+1];
                                        ln_ack_lcp++;                                        
                                        
                                        count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                        ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];          
                                        goto next_opt_lcp_1;   
                                         
                                  
                            case ACFC:      
                                   if (buf_rx_ppp[ind_opt+1]!=2)
                                      {                                        
                                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);
                                        } 
                                        
                                        
                                        fl_opt.acfc=1;
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt];
                                        ln_ack_lcp++;
                                        
                                        buf_ack_opt_lcp[ln_ack_lcp]=buf_rx_ppp[ind_opt+1];
                                        ln_ack_lcp++;                                        
                                        
                                        count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                        ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];          
                                        goto next_opt_lcp_1;   
                            
                            default:

                                        for(i=0;i<buf_rx_ppp[ind_opt+1];i++){buf_rej_opt_lcp[ln_rej_lcp]=buf_rx_ppp[ind_opt+i];ln_rej_lcp++;}
                                        count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                        ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];      
                                  goto next_opt_lcp_1;        
                          }
                                            
 end_pcp_opt_1:        
                       
                   
                                                                                
                       
                       if (ln_rej_lcp!=0)                    
                         {
     
                         LCP_event=RCR_MI;
                         fl_lcp2.nac_rej=1;
                         return(1); 
                         }
                       
                       
                      
                           
                      if(fl_opt.accm==0) 
                            {  
                            buf_nak_opt_lcp[ln_nak_lcp]=ACCM;
                            ln_nak_lcp++;                                        
                            buf_nak_opt_lcp[ln_nak_lcp]=6;
                            ln_nak_lcp++;
                            buf_nak_opt_lcp[ln_nak_lcp]=0; 
                            ln_nak_lcp++;  
                            buf_nak_opt_lcp[ln_nak_lcp]=0; 
                            ln_nak_lcp++;             
                            buf_nak_opt_lcp[ln_nak_lcp]=0; 
                            ln_nak_lcp++;                              
                            buf_nak_opt_lcp[ln_nak_lcp]=0; 
                            ln_nak_lcp++; 
                            }
                                     
                      if(fl_opt.aut_prot==0) 
                            {    
                              
                      
                         
                             
                            buf_nak_opt_lcp[ln_nak_lcp]=AUT_PROT;
                            ln_nak_lcp++;
                                        
                            buf_nak_opt_lcp[ln_nak_lcp]=4;
                            ln_nak_lcp++;
                                        
                            buf_nak_opt_lcp[ln_nak_lcp]=0xc0; 
                            ln_nak_lcp++;  
                            
                            buf_nak_opt_lcp[ln_nak_lcp]=0x23; 
                            ln_nak_lcp++;                    
                            }   
                      if(fl_opt.pfc==0) 
                            { 
                            buf_nak_opt_lcp[ln_nak_lcp]=PFC;
                            ln_nak_lcp++;
                                        
                            buf_nak_opt_lcp[ln_nak_lcp]=2;
                            ln_nak_lcp++; 
                            }                  
                     if(fl_opt.acfc==0) 
                            { 
                             buf_nak_opt_lcp[ln_nak_lcp]=ACFC;
                            ln_nak_lcp++;
                                        
                            buf_nak_opt_lcp[ln_nak_lcp]=2;
                            ln_nak_lcp++; 
                            }                   
                         
                
                 
                                         
                         if (ln_nak_lcp!=0)                    
                         { 
       
                         LCP_event=RCR_MI;
                         fl_lcp2.nac_rej=0;  
                         i_scan_lcp=buf_rx_ppp[6];     // присвоение идентификатора                 
                         return(1); 
                         } 
                       
                         if (ln_ack_lcp!=0)                    
                         {
                         
                         
                         LCP_event=RCR_PL;
                         i_scan_lcp=buf_rx_ppp[6];    // присвоение идентификатора                   
                         return(1); 
                         }  
                      
                      
                        
                      break; 
                      */
                       
                  case CONF_ACK : 
                      //проверка опций (если вообще не те опции то сброс кода дать)
                      //опции та, то событие RCA
                    
                    if(buf_rx_ppp[6]!=i_scr_lcp)return(1); //проверка идентификатора
                    //не обрабатывать опции
                    //сразу RCA
                    LCP_event=RCA;
                    i_scr_lcp++;//приращение идентификатора                      
                    return(1);  
                    
                    /*
   next_opt_lcp_2:  if(count_length_opt==vol_length_opt) goto end_pcp_opt_2;   //обработаны все опции
                       
                       switch(buf_rx_ppp[ind_opt])
                          {
                            case ACCM:       
                                  if (buf_rx_ppp[ind_opt+1]!=6)
                                      {
                                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);
                                        } 
                                    
                                    if ( 
                                           (buf_rx_ppp[ind_opt+2]==0)&&
                                           (buf_rx_ppp[ind_opt+3]==0x0a)&& 
                                           (buf_rx_ppp[ind_opt+4]==0)&&
                                           (buf_rx_ppp[ind_opt+5]==0) 
                                        )  fl_opt.accm=1;                                        
                                     
                                       
                                     
                                      
                                 count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                 ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];
                                 goto next_opt_lcp_2;     
                              
                              case PFC:
                                  if (buf_rx_ppp[ind_opt+1]!=2)
                                      {                                        
                                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);
                                        } 

                                        fl_opt.pfc=1;
                                        count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                        ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];          
                                        goto next_opt_lcp_2;   
                                         
                                  
                            case ACFC:      
                                   if (buf_rx_ppp[ind_opt+1]!=2)
                                      {                                        
                                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);
                                        } 

                                        fl_opt.acfc=1;
                                                                            
                                        
                                        count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                        ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];          
                                        goto next_opt_lcp_2;      
                            
                            default:    for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                                        LCP_event=RUC;
                                        return(1);    
                                  
                          }
                      
                      
  end_pcp_opt_2:      if((fl_opt.accm==1)&&(fl_opt.acfc==1)&&(fl_opt.pfc==1))
                        {       
                                  
                         LCP_event=RCA;
                         i_scr_lcp++;//приращение идентификатора                      
                         return(1);      
                        } 
                        else
                        {
                        for(i=5;i<=count_rx_ppp-4;i++){buf_rejc_opt_lcp[ln_rejc_lcp]=buf_rx_ppp[i];ln_rejc_lcp++;}
                        LCP_event=RUC;
                        return(1);               
                        }
                        */        
                       
                  case CONF_NAK : 
                          //!!!!! подумать                    
                        // то RCN но все равно запрос тех же опций  
                      if(buf_rx_ppp[6]!=i_scr_lcp)return(1);                        //проверка идентификатора
                      i_scr_lcp++;//приращение идентификатора
                      LCP_event=RCN; 
                      
                      if((buf_rx_ppp[9]==7)&&(buf_rx_ppp[10]==2)&&(buf_rx_ppp[11]==8)&&(buf_rx_ppp[12]==2))compress_off=1;
                      
                      
                      return(1); 
                  case CONF_REJ :
                        // !!!!! подумать!!!!!!!!
                       // то RCN но все равно запрос тех же опций 
                      if(buf_rx_ppp[6]!=i_scr_lcp)return(1);                        //проверка идентификатора
                      i_scr_lcp++;//приращение идентификатора
                      LCP_event=RCN;
                      if((buf_rx_ppp[9]==7)&&(buf_rx_ppp[10]==2)&&(buf_rx_ppp[11]==8)&&(buf_rx_ppp[12]==2))compress_off=1;
                      return(1);          
                                 
                  } 
                    
       break;
       
       case PR_PAP:   
            
            
            
          
  
               
            // здесь  проверка общей длины     
              length.bytes[1]=buf_rx_ppp[5];
              length.bytes[0]=buf_rx_ppp[6];
              if (length.word!=count_rx_ppp-6)return(1);
              if (count_rx_ppp>MAX_DL_PAP)return(1);//не воспринимать длинные пакеты                           
                

                           
              switch (buf_rx_ppp[3])      
                  {
                  case CONF_ACK :
                       if(buf_rx_ppp[4]!=i_scr_pap)return(1);     //проверка идентификатора
                       PAP_event=RCA; 
                       return(1);                 
                  } 
        
       break;
       
       
       case PR_IPCP: 
       
             
              
              // здесь  проверка общей длины     
              length.bytes[1]=buf_rx_ppp[5];
              length.bytes[0]=buf_rx_ppp[6];
              if (length.word!=count_rx_ppp-6)return(1);
              if (count_rx_ppp>MAX_DL_IPCP)return(1);//не воспринимать длинные пакеты                           
              
              vol_length_opt=count_rx_ppp-10;
              count_length_opt=0;
              ind_opt=7;  
             
              ln_rejc_ipcp=0;
              ln_ack_ipcp=0;
              ln_nak_ipcp=0;
              ln_rej_ipcp=0; 
              
             
                           
              switch (buf_rx_ppp[3])      
                  {
                  case CONF_REQ : 
                         
                           
                 // (если вообще не те опции то сброс кода дать) 
                      //проверка предложенной конфигурации от сервера    
                      //если нужная конфигурация то событие RCR+                  
                      //если нет то RCR-;                       
                     
                    for(i=0;i<vol_length_opt;i++){buf_ack_opt_ipcp[ln_ack_ipcp]=buf_rx_ppp[ind_opt+i];ln_ack_ipcp++;}
                    IPCP_event=RCR_PL;
                    i_scan_ipcp=buf_rx_ppp[4];    // присвоение идентификатора                   
                    return(1);  
                    
                    //Подтверждать все!!!!!!!!!!
                    

                       
                  case CONF_ACK : 
                      //проверка опций (если вообще не те опции то сброс кода дать)
                      //опции та, то событие RCA
                  //  led2_on;    //пока
                  //  lock_it();    // пока
                    
                   
                                         
                    
                    
                    if(buf_rx_ppp[4]!=i_scr_ipcp)return(1);                        //проверка идентификатора
                      
                     IPCP_event=RCA;
                     i_scr_ipcp++;//приращение идентификатора                      
                     return(1);  
                     //НИЧЕГО ДАЛЬШЕ НЕ РАЗБИРАТЬ!!!!!!!!!
              
                       
                  case CONF_NAK :                     
                        // то RCN но все равно запрос тех же опций  

                      if(buf_rx_ppp[4]!=i_scr_ipcp)return(1);                        //проверка идентификатора
                     
                     
    next_opt_ipcp_3:  if(count_length_opt==vol_length_opt) goto end_ipcp_opt_3;   //обработаны все опции
                       
                       switch(buf_rx_ppp[ind_opt])
                          { 
                          
                            case IP_SELF:       
                                  if (buf_rx_ppp[ind_opt+1]!=6)
                                      {
                                       // for(i=3;i<=count_rx_ppp-4;i++){buf_rejc_opt_ipcp[ln_rejc_ipcp]=buf_rx_ppp[i];ln_rejc_ipcp++;}
                                       // IPCP_event=RUC;
                                        return(1);
                                        } 
                                         
                                      //  {modbus_mem1[0x15f-0x150]++;} 
                                                                                  
                                          if (buf_rx_ppp[ind_opt+2] != ip_self[0] ) ip_change=1;
                                           if (buf_rx_ppp[ind_opt+3] != ip_self[1] ) ip_change=1;
                                            if (buf_rx_ppp[ind_opt+4] != ip_self[2] ) ip_change=1;
                                             if (buf_rx_ppp[ind_opt+5] != ip_self[3] ) ip_change=1;
                                         
                                          ip_self[0]=buf_rx_ppp[ind_opt+2];
                                          ip_self[1]=buf_rx_ppp[ind_opt+3];
                                          ip_self[2]=buf_rx_ppp[ind_opt+4];
                                          ip_self[3]=buf_rx_ppp[ind_opt+5]; 
                                          
                                          fl_opt.ip_self=1;                                        
                                      
                                 count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                 ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];
                                 goto next_opt_ipcp_3;  
                             
                             
                            case IP_PRI_DNS:       
                                  if (buf_rx_ppp[ind_opt+1]!=6)
                                      {
                                     //   for(i=3;i<=count_rx_ppp-4;i++){buf_rejc_opt_ipcp[ln_rejc_ipcp]=buf_rx_ppp[i];ln_rejc_ipcp++;}
                                     //   IPCP_event=RUC;
                                        return(1);
                                        } 
                                           
                                          ip_pri_dns[0]=buf_rx_ppp[ind_opt+2];
                                          ip_pri_dns[1]=buf_rx_ppp[ind_opt+3];
                                          ip_pri_dns[2]=buf_rx_ppp[ind_opt+4];
                                          ip_pri_dns[3]=buf_rx_ppp[ind_opt+5]; 
                  
                                          fl_opt.ip_pri_dns=1;                                        
                                      
                                 count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                 ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];
                                 goto next_opt_ipcp_3; 
                             
                            
                            case IP_SEC_DNS:       
                                  if (buf_rx_ppp[ind_opt+1]!=6)
                                      {
                                      //  for(i=3;i<=count_rx_ppp-4;i++){buf_rejc_opt_ipcp[ln_rejc_ipcp]=buf_rx_ppp[i];ln_rejc_ipcp++;}
                                      //  IPCP_event=RUC;
                                        return(1);
                                        } 
                                    
                                          ip_sec_dns[0]=buf_rx_ppp[ind_opt+2];
                                          ip_sec_dns[1]=buf_rx_ppp[ind_opt+3];
                                          ip_sec_dns[2]=buf_rx_ppp[ind_opt+4];
                                          ip_sec_dns[3]=buf_rx_ppp[ind_opt+5]; 
                                          fl_opt.mru=1;                                        
                                      
                                 count_length_opt=count_length_opt+buf_rx_ppp[ind_opt+1];        
                                 ind_opt=ind_opt+buf_rx_ppp[ind_opt+1];
                                 goto next_opt_ipcp_3; 
                             

                            
                            default:   // for(i=3;i<=count_rx_ppp-4;i++){buf_rejc_opt_ipcp[ln_rejc_ipcp]=buf_rx_ppp[i];ln_rejc_ipcp++;}
                                       // IPCP_event=RUC;
                                        return(1);    
                                  
                          }
                      
                      
  end_ipcp_opt_3:                                        
                      i_scr_ipcp++;//приращение идентификатора
                      IPCP_event=RCN;                                             
                      return(1);         
                       
                      
                  case CONF_REJ :
                       // то RCN но все равно запрос тех же опций 
                      if(buf_rx_ppp[4]!=i_scr_ipcp)return(1);                        //проверка идентификатора
                      i_scr_ipcp++;//приращение идентификатора
                      IPCP_event=RCN;

                      return(1);      
                  
                    
                  } 
                            
       break;        
           
        
       case PR_IP:            
   
   
  
  if(count_rx_ppp<(33+MIN_TR_HEAD))return(1); 
   
  
  Crc_out.bytes[0]=buf_rx_ppp[12];    //проверка CRC IP 
  Crc_out.bytes[1]=buf_rx_ppp[13];  
  buf_rx_ppp[12]=0; 
  buf_rx_ppp[13]=0; 
  temp.word=calc_crc_ip(&buf_rx_ppp[2],20);
  if(Crc_out.word!=temp.word)return(1);


  
  
  if((buf_rx_ppp[18]!=ip_self[0])||       //сверка со своим IP
   (buf_rx_ppp[19]!=ip_self[1])|| 
   (buf_rx_ppp[20]!=ip_self[2])|| 
   (buf_rx_ppp[21]!=ip_self[3]))return(1);
  
 
  
  if(buf_rx_ppp[11]!=0x11)return(1);  //сверка с протоколом UDP
  
  

  
  Crc_out.bytes[0]=buf_rx_ppp[28];    //проверка CRC UDP 
  Crc_out.bytes[1]=buf_rx_ppp[29];  
  buf_rx_ppp[28]=0; 
  buf_rx_ppp[29]=0; 
  temp.bytes[1]=buf_rx_ppp[26]; 
  temp.bytes[0]=buf_rx_ppp[27]; 
  temp.word=calc_crc_udp_2(&buf_rx_ppp[0],temp.word);
  if(Crc_out.word!=temp.word)return(1); 
   
  
   Crc_out.bytes[1]=port_udp>>8;
   Crc_out.bytes[0]=port_udp;
   if((buf_rx_ppp[24]!=Crc_out.bytes[1])||       //проверка UDP порта 
      (buf_rx_ppp[25]!=Crc_out.bytes[0]))return(1); 
  
                                                   //10.11.2010
    count_rx_ppp=buf_rx_ppp[4];                   //обрезание в РРР мусора, длина складывается из длины IP пакета и +5(начало и конец РРР)
    count_rx_ppp=(count_rx_ppp<<8)| buf_rx_ppp[5];//
    count_rx_ppp=count_rx_ppp+5;                  //
   
   if(proc_udp_data(buf_rx_ppp,count_rx_ppp)==0)return(1); 
 
                 
    break;
      } 
 
 return(1);  
 }
 
 
 

 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 
 
 
 
 void send_no_sinc(unsigned char id,uint16_t dst,unsigned char error)
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
          
          
        Obj_ppp_tx.l_opt=3;// длина буфера опций   
        buf_opt_tr[0]=KOD_OP_ERROR;
        buf_opt_tr[1]=3;
        buf_opt_tr[2]=error;
        
        //Obj_ppp_tx.kol_opt=1;// количество опций     
       
       
         Obj_ppp_tx.l_data=0;   // длина данных
         Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных
        
         form_buf_tx_ppp();                            
       //  UCSR0A=UCSR0A | TXC;                            
       //  UCSR0B=UCSR0B | TXEN;
       //  UCSR0B=UCSR0B | TXCIE;
         if(check_cts()==1) return;                
        
        // UDR0=buf_tx_232[0];
          
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
         
         return;     
    }


unsigned char proc_paging(void)
{ 
 if(fl_pg_out.hold_pg==1)
    {
        //send_info(sizeof(gluk),gluk,1,3);
    
         if((fl_pg_out.first_pg==1)&&(cnt_pg==0))
         { 
   //      send_info(sizeof(gluk),gluk,1,4);
         cnt_pg=0;
         fl_pg_out.first_pg=0;
         fl_pg_out.end_pg=0;
         fl_pg_out.hold_pg=0;
         fl_pg_out.ch_crc=0;
         fl_pg_out.cor_pg=0;
         // дать ошибку рассинхронизация 
         return(1);
         }
    
        if((fl_pg_out.first_pg==1)&&(cnt_pg!=0))
         {                                      // это в случае когда процесс прерван а потом заново возобновляется
  //        send_info(sizeof(gluk),gluk,1,5);
          fl_pg_out.hold_pg=1;
          fl_pg_out.first_pg=0;
          cnt_pg=0;
          id_pg_kp=id_pg_appl+1;
          fl_pg_out.cor_pg=1;
          goto check_end_pg;
         }
      
    
     
      
      
      if(id_pg_kp==id_pg_appl)
         {
  //        send_info(sizeof(gluk),gluk,1,6);
          cnt_pg++;
          id_pg_kp++;
          fl_pg_out.cor_pg=1;
          goto check_end_pg;
         }
         else
         {
   //       send_info(sizeof(gluk),gluk,1,7);
          fl_pg_out.cor_pg=0;
          if(id_pg_appl==(id_pg_kp-1))goto check_end_pg;
             else
             {
     //         send_info(sizeof(gluk),gluk,1,8);
              cnt_pg=0;
              fl_pg_out.first_pg=0;
              fl_pg_out.end_pg=0;
              fl_pg_out.hold_pg=0;
              fl_pg_out.ch_crc=0;
              fl_pg_out.cor_pg=0;
              // дать ошибку рассинхронизация 
              return(1);
             }
         }   
         
    }
    else
    {
      
    //  send_info(sizeof(gluk),gluk,1,9);
      if(fl_pg_out.first_pg==1)
         {
 //         send_info(sizeof(gluk),gluk,1,10);
          fl_pg_out.hold_pg=1;
          fl_pg_out.first_pg=0;
          cnt_pg=0;
          id_pg_kp=id_pg_appl+1;
          fl_pg_out.cor_pg=1;
          goto check_end_pg;
         }
         else
         {
  //       send_info(sizeof(gluk),gluk,1,11);
         cnt_pg=0;
         fl_pg_out.first_pg=0;
         fl_pg_out.end_pg=0;
         fl_pg_out.hold_pg=0;
         fl_pg_out.ch_crc=0;
         fl_pg_out.cor_pg=0;
         // дать ошибку рассинхронизация 
         return(1);
         }
    }
    
check_end_pg:
       
 //   send_info(sizeof(gluk),gluk,1,12);
    if(fl_pg_out.end_pg==1)
       {
 //        send_info(sizeof(gluk),gluk,1,13);
         fl_pg_out.hold_pg=0;
         fl_pg_out.first_pg=0;
       }
  
//send_info(sizeof(gluk),gluk,1,14);    
return(0);
}
 
 
 unsigned char proc_option(unsigned char *buf_rx_ppp,uint16_t ind,uint16_t count_rx_opt,unsigned char port)
 {

 unsigned char tt;
//return 0 ошибка, игнорировать, весь некорректный пакет
// return 1 - прочитаны все опции (при поступившем запросе) дальше выход и выдача ответ на запрос 
// return 2 - ответить сразу размером страницы
// return 3 - ошибка время жизни истекло
// return 4 - ошибка операция недоступна 
 
 unsigned char flag_life;
 uint16_t life_cl;
 unsigned char life_time;
 unsigned char life_start;
 uint16_t cnt;
 
 cnt=0;
 flag_life=0;
 life_cl=0;
 life_time=0;
 life_start=0;
 

 
 next_option: 
               
               switch(buf_rx_ppp[ind])
                 {
                 case KOD_OP_ERROR:  
                                  
                                  if(buf_rx_ppp[ind+1]!=L_ERR_NO_INTABL)return(0);   
                                  if(buf_rx_ppp[ind+2]!=ERR_NO_INTABL)return(0);
                                  if(((buf_rx_ppp[TR_V]>>4)&0x03)!=KVIT)return(0);
                                  if(*(uint16_t*)&buf_rx_ppp[ind+3]==num_seq_cl)
                                   {
                            
                                    Appl_seq.cnt_cntr_cl=0;
                                    Appl_seq.cntr_cl=TRUE;
                                    Appl_seq.en_cntr_cl=TRUE;
                                    Appl_seq.cnt_try=0;
                                    fl_appl_seq.en_povtor=0; 
                                    Appl_seq.l_data=0; 
                                    Appl_seq.cnt_waits=0;
                                    fl_appl_seq.kv_waits=0;
                                    }
                            
                                     cnt_no_link=vol_cnt_no_link;//dobavka
                                  
                                     Control.link_waits=FALSE;
                                     Control.cnt_link=Control.vol_link;                  
                                     Control.link=FALSE; 
                           
                                     Control.cnt_try=0;
                                     Control.link_no=FALSE;
                                     return(0);
                 
                 case KOD_OP_LUFKIN: 
                                     if(buf_rx_ppp[ind+1]!=L_OP_LUFKIN)return(0);  
                                     if(((buf_rx_ppp[TR_V]>>4)&0x03)!=ZAPR)return(0);
                                     delay_pre_tx=*(uint16_t*)&buf_rx_ppp[ind+2]; 
                                     if(delay_pre_tx>5000){delay_pre_tx=0;return(0); }//5 секунд                                       
                                     break;
                  
                 case KOD_OP_SIZE_PG: 
                                      if(buf_rx_ppp[ind+1]!=L_OP_SIZE_PG)return(0);   
                                      if(((buf_rx_ppp[TR_V]>>4)&0x03)!=ZAPR)return(0);                                                             
                                      return(2); 
                 
                 case KOD_WR_PG: 
                                      if(buf_rx_ppp[ind+1]!=L_OP_WR_PG)return(0);   
                                      if(((buf_rx_ppp[TR_V]>>4)&0x03)!=ZAPR)return(0);
                                      if (buf_rx_ppp[ind+2] & FIRST_PG) fl_pg_out.first_pg=1;else fl_pg_out.first_pg=0; 
                                      if (buf_rx_ppp[ind+2] & NO_END_PG) fl_pg_out.end_pg=0;else{  fl_pg_out.end_pg=1;}                                                        
                                      if (buf_rx_ppp[ind+2] & NADO_KV) { fl_pg_out.send_kv=1;}else fl_pg_out.send_kv=0;
                                      id_pg_appl=buf_rx_ppp[ind+3];
                                      break;
                                                  
                 case KOD_WR_PG_CRC: 
                                     
                                      fl_pg_out.ch_crc=0;
                                      if(buf_rx_ppp[ind+1]!=L_OP_WR_PG_CRC)return(0);   
                                      if(((buf_rx_ppp[TR_V]>>4)&0x03)!=ZAPR)return(0); 
                                      fl_pg_out.ch_crc=1;
                                      pg_crc=*(uint16_t*)&buf_rx_ppp[ind+2];                                                            
      
                       //               send_info(sizeof(gluk),gluk,1,2);
                                      
                                      break;
                 
                 case KOD_OP_LIFE:   
                                      switch(port)
                                      {
                                      case PORT485_1:
                                           life_time=Life.time_485_1;
                                           life_cl=Life.client_485_1;
                                           life_start=Life.start_485_1;
                                           break;
                                      case PORT485_2:
                                           life_time=Life.time_485_2;
                                           life_cl=Life.client_485_2;
                                           life_start=Life.start_485_2;
                                           break;
                                      case PORT232_2:
                                           life_time=Life.time_232_2;
                                           life_cl=Life.client_232_2;
                                           life_start=Life.start_232_2;
                                           break;
                                      default:return(0);
                                      }
                                      
                                      
                                      if(buf_rx_ppp[ind+2]==0)
                                      {
                                        
                                
                                        if(life_cl==*(uint16_t*)&buf_rx_ppp[TR_SRC])
                                        {
                                          if(life_time==0)flag_life=2;                                         
                                             else 
                                                  {
                                                  switch(port)
                                                     {
                                                       case PORT485_1:
                                                            Life.time_485_1=0;
                                                            Life.start_485_1=0;
                                                            break;
                                                       case PORT485_2:
                                                            Life.time_485_2=0;
                                                            Life.start_485_2=0;
                                                            break;
                                                       case PORT232_2:
                                                            Life.time_232_2=0;
                                                            Life.start_232_2=0;
                                                             break;     
                                                        default :write_log_info(ST_ERROR,ERR6);
                                                                 lock_it(); // сбой озу
                                                                 return(0);  // 11 10 2018
                                                      }
                                                   flag_life=1;    
                                                   }
                                        }
                                        else flag_life=3;
                                      }
                                      else
                                      {
                                        if(life_time==0)
                                        {
                                          switch(port)
                                                {
                                                 case PORT485_1:
                                                       Life.time_485_1=buf_rx_ppp[ind+2];
                                                       Life.client_485_1=*(uint16_t*)&buf_rx_ppp[TR_SRC];
                                                       Life.start_485_1=1;
                                                       break;
                                                 case PORT485_2:
                                                       Life.time_485_2=buf_rx_ppp[ind+2];
                                                       Life.client_485_2=*(uint16_t*)&buf_rx_ppp[TR_SRC];
                                                       Life.start_485_2=1;
                                                       break;
                                                  case PORT232_2:
                                                       Life.time_232_2=buf_rx_ppp[ind+2];
                                                       Life.client_232_2=*(uint16_t*)&buf_rx_ppp[TR_SRC];
                                                       Life.start_232_2=1;
                                                       break;     
                                                  default :write_log_info(ST_ERROR,ERR5);
                                                           lock_it(); // сбой озу
                                                           return(0);   // 11 10 2018
                                                  }
                                          flag_life=1;
                                        }
                                        else 
                                        {
                                          
                                          if((life_cl==*(uint16_t*)&buf_rx_ppp[TR_SRC]) || (life_start==1))
                                          {
                                            switch(port)
                                                {
                                                case PORT485_1:
                                                       Life.time_485_1=buf_rx_ppp[ind+2];
                                                       Life.client_485_1=*(uint16_t*)&buf_rx_ppp[TR_SRC];
                                                       Life.start_485_1=1;
                                                       break;
                                                 case PORT485_2:
                                                       Life.time_485_2=buf_rx_ppp[ind+2];
                                                       Life.client_485_2=*(uint16_t*)&buf_rx_ppp[TR_SRC];
                                                       Life.start_485_2=1;
                                                       break;
                                                  case PORT232_2:
                                                       Life.time_232_2=buf_rx_ppp[ind+2];
                                                       Life.client_232_2=*(uint16_t*)&buf_rx_ppp[TR_SRC];
                                                       Life.start_232_2=1;
                                                       break;   
                                                  default :write_log_info(ST_ERROR,ERR5);
                                                           lock_it(); // сбой озу
                                                            return(0);   // 11 10 2018
                                                  }
                                          flag_life=1;
                                          }
                                          else flag_life=3;
                                        }  
                                          
                                      }
                                      
                                      
                                      
                                      break;
                                    
                                  
                 default:  return(0);   
                              
                 }
 
 //здесь высчитывается длина заполнителя              
 tt=4-((buf_rx_ppp[ind+1]+4)%4);
 if(tt==4)tt=0;           
 ///////////////
 
 cnt=cnt+buf_rx_ppp[ind+1]+tt;
 if(cnt>=count_rx_opt)
 {
   if(flag_life==0){return(1);}//все опции просчитаны
     else
     {
       if(flag_life==1)return(1);
       if(flag_life==2)return(3);
       if(flag_life==3)return(4);
       write_log_info(ST_ERROR,ERR4);
       lock_it(); // сбой озу
        return(0);   // 11 10 2018
     }
 }  
   
 ind=ind+buf_rx_ppp[ind+1]+tt;               
 //if(ind>=count_rx_opt)return(1);
 goto next_option;
 } 
 
 
 
 void send_size_pg(unsigned char id,uint16_t dst)
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
          
          
        Obj_ppp_tx.l_opt=4;// длина буфера опций   
        buf_opt_tr[0]=KOD_OP_SIZE_PG;
        buf_opt_tr[1]=L_OP_SIZE_PG_OTV;
        *(uint16_t*)&buf_opt_tr[2]=256;  
        
        //Obj_ppp_tx.kol_opt=1;// количество опций     
       
       
         Obj_ppp_tx.l_data=0;   // длина данных
         Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных
        
         form_buf_tx_ppp();                            
       //  UCSR0A=UCSR0A | TXC;                            
       //  UCSR0B=UCSR0B | TXEN;
       //  UCSR0B=UCSR0B | TXCIE;
         if(check_cts()==1) return;                
         //S2_RD;//
         //UDR0=buf_tx_232[0];
          
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
         
         return;     
    }
 
/* 
 void send_otv_ping(unsigned char id,uint16_t dst)
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
              
          Obj_ppp_tx.kol_opt=0;// количество опций   
          Obj_ppp_tx.l_opt=0;// длина буфера опций   
        
          Obj_ppp_tx.l_data=0;   // длина данных
         
          
         form_buf_tx_ppp();                            
         UCSR0A=UCSR0A | TXC;                            
         UCSR0B=UCSR0B | TXEN;
         UCSR0B=UCSR0B | TXCIE;
         if(check_cts()==1) return;                
         S2_RD;
         UDR0=buf_tx_232[0];
         
         return;     
    }
 
 */
 
  unsigned char proc_udp_data(unsigned char *buf_rx_ppp,uint16_t count_rx_ppp)
               {
               uint16_t length,crc;
              // unsigned char *p_data;
               unsigned char length_head;
               unsigned char offset;
               uint16_t ii,kol,i;
               
             
              
               
               
               //проверка длины данных UDP 
               length=*(uint16_t*)&buf_rx_ppp[UDP_LN];
               if(length<MIN_TR_HEAD)return(0);
               
               // проверка длины транспорта
               length=*(uint16_t*)&buf_rx_ppp[TR_LEN];
               if (length<MIN_TR_HEAD) return(0);
               if (length>count_rx_ppp-5-20-8)return(0);
           
               transform_buf(&buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_LEN]-7,*(uint16_t*)&buf_rx_ppp[TR_KOD]);
               
               *(uint16_t*)&buf_rx_ppp[TR_KOD]=0;
                
               crc=*(uint16_t*)&buf_rx_ppp[TR_CRC];
               *(uint16_t*)&buf_rx_ppp[TR_CRC]=0;
               
               //проверка crc
                if(crc!=crc_m1(&buf_rx_ppp[TR_SRC],*(uint16_t*)&buf_rx_ppp[TR_LEN],0xffff))return(0);         
                         
                
                

               if((buf_rx_ppp[TR_V]>>6)==VER2)// версия протокола VER2
               { 
               
               
               if(*(uint16_t*)&buf_rx_ppp[TR_DST]!=num_self)return(0); 
               
               
               
               length_head=(buf_rx_ppp[TR_V]&0x0f)*4;
               
               
               if(length_head<MIN_TR_HEAD)return(0);
               if(length_head>50)return(0);// пока ограничил
               
         //      send_info(sizeof(prov1),prov1,0,0);
              
              
            
               
               length=count_rx_ppp-5-20-8-length_head;
               
               
             
               if(length==0) //если длина данных 0 то контроль связи и проверка
                  {
                   
                  if(length_head>MIN_TR_HEAD)
                    {
                     if (proc_option(buf_rx_ppp,TR_OP_DATA,length_head-MIN_TR_HEAD,PORT_NONE)==2)
                       {
                        send_size_pg(buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                       }
                      return(0);
                    }
                  
//                  send_info(sizeof(prov2),prov2,0,0);
                
                  /*
                  if(((buf_rx_ppp[TR_V]>>4)&0x03)==ZAPR)
                  { 
                    send_otv_ping(buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                    send_info(sizeof(ans_out_ping),ans_out_ping,1,buf_rx_ppp[TR_ID]);
                    return(0); 
                  }
                  
                  */

                   if(*(uint16_t*)&buf_rx_ppp[TR_SRC]!=NUM_SRV_LINK)return(0);
//                  if(buf_rx_ppp[TR_ID]!=ID_CC)return(0);

                  if(((buf_rx_ppp[TR_V]>>4)&0x03)!=KVIT)return(0); 
                   
                  send_info(sizeof(rec_ctrl_ch),rec_ctrl_ch,1,buf_rx_ppp[TR_ID]);
                  
                  if(Control.link_waits==TRUE)
                     {   
                     cnt_no_link=vol_cnt_no_link;//dobavka  
                     Control.link_waits=FALSE;
                     Control.cnt_link=Control.vol_link;                   
                     Control.cnt_try=0;
                     Control.link_no=FALSE;
                     return(1); 
                     }
                  return(0);
                  } 
                
                 
              
              
              
                // дальнейший разбор
                 
                 
                 clr_cntr_nat();
                 
                 offset=2+20+8+length_head;
              
                
                
                
                if(buf_rx_ppp[offset+SL_LS_PROT]==PROT_SL_LS)
                  {
                    //разбор команд
                    //пока одна команда - наличие отсутствие клиента в таблице маршрутизации
                    if(((buf_rx_ppp[TR_V]>>4)&0x03)!=OTV)return(0);
                    if(buf_rx_ppp[offset+SL_LS_PROT+1]!=COM_CNTR_CL)return(0);
                    if(*(uint16_t*)&buf_rx_ppp[offset+SL_LS_PROT+2]!=L_OTV_CNTR_CL)return(0);
                    if(*(uint16_t*)&buf_rx_ppp[offset+SL_LS_PROT+4]!=num_seq_cl)return(0);
                    if(buf_rx_ppp[offset+SL_LS_PROT+6]!=0)
                      {// присутствие в таблице
                       
                       Appl_seq.en_cntr_cl=FALSE;
                       Appl_seq.cntr_cl=FALSE;
                       Appl_seq.cnt_cntr_cl=0;

                     // присутствие в таблице
                       }
                   cnt_no_link=vol_cnt_no_link;//dobavka
                   Control.link_waits=FALSE;
                   Control.cnt_link=Control.vol_link;                  
                   Control.link=FALSE;                           
                   Control.cnt_try=0;
                   Control.link_no=FALSE;    
                  
                  send_info(sizeof(ans_in_ctrl_cl),ans_in_ctrl_cl,1,buf_rx_ppp[TR_ID]);
                  return(1);
                  }
                
                
                
                
                 if(buf_rx_ppp[offset+C1_PROT]!=PROT_C1)return(0);
                  
                 
                 
                  
                  
                  switch(buf_rx_ppp[offset+C1_PORT])
                      {
                       case PORT485_1:
   
                            
                          
                                // можно отправиь сообщение об ошибке "слишком длинные данные для приложения"
                                   send_info(sizeof(ans_out_485_1),ans_out_485_1,1,buf_rx_ppp[TR_ID]);
                                   send_err485(NUM_RS485_1,RS_BUSY,buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                                   return(0);
                                  
                         
                            
                           
                       case PORT485_2:
                         
                                // этот порт мы не поддерживаем !!!!
                                // можно отправиь сообщение об ошибке "слишком длинные данные для приложения"
                                   send_info(sizeof(ans_out_485_2),ans_out_485_2,1,buf_rx_ppp[TR_ID]);
                                   send_err485(NUM_RS485_2,RS_BUSY,buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                                   return(0);
                                  
                       
                            
                     case PORT232_2:
                         
   
// обслуживать порт или нет пока
                         
                           // то можно послать служебное сообщение порт не поддерживается
                               {
                               
                               send_info(sizeof(ans_out_232),ans_out_232,1,buf_rx_ppp[TR_ID]);
                                 send_err485(NUM_RS232_2,RS_BUSY,buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                               return(0);
                               }
                       


 /*                      
                       
                                if(length<11) 
                                  {
                                    // можно отправиь сообщение об ошибке "некорректные данные"
                                    send_info(sizeof(ans_out_232),ans_out_232,1,buf_rx_ppp[TR_ID]);
                                    send_err485(NUM_RS232_2,RS_DATA_ERR,buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                                   return(0);
                                  }                 
                  
                             if(length> LN_BUF_232_2+10) 
                                 {
                                // можно отправиь сообщение об ошибке "слишком длинные данные для приложения"
                                   send_info(sizeof(ans_out_232),ans_out_232,1,buf_rx_ppp[TR_ID]);
                                   send_err485(NUM_RS232_2,RS_OVER_BUF_TX,buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                                   return(0);
                                  } 
                            
                            
                            // здесь должна быть проверка контекста, если все нормально то дальше забит контекст
                            // то можно послать служебное сообщение
                                
                             if (check_cont_485_1 (&buf_rx_ppp[offset+C1_CONT])==1)
                                 {
                                   send_info(sizeof(ans_out_232),ans_out_232,1,buf_rx_ppp[TR_ID]);
                                   send_err485(NUM_RS232_2,RS_NO_CONT,buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                                   return(0);
                                  } 
                            
                            
                            if(Appl_RS232_2.fl_data_buf==TRUE)
                               {
                               // то можно послать служебное сообщение что буфер переполнен
                               send_info(sizeof(ans_out_232),ans_out_232,1,buf_rx_ppp[TR_ID]);
                                 send_err485(NUM_RS232_2,RS_BUSY,buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC]);
                               break;
                               }
                               
                              
                               
                               
                               Appl_RS232_2.pre_tx=0;
                                if(length_head>MIN_TR_HEAD)
                                  {
                                    i=proc_option(buf_rx_ppp,TR_OP_DATA,length_head-MIN_TR_HEAD,PORT232_2);
                                    
                                    
                                    if(i==3)//время жизни истекло
                                    {  
                                      send_no_sinc(buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC],OUT_LIFE);
                                      return(0);
                                    }
                                    
                                    if(i==4) //операция недоступна
                                      {
                                      send_no_sinc(buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC],DES_OPERATE);
                                      return(0);
                                      }
                                    
                                    if (i!=1)return(0);                                                                       
                                    Appl_RS232_2.pre_tx=delay_pre_tx;//задержка перед передачей
                                  } 
                               
                               
                               if(fl_232_2.busy==1)Appl_RS232_2.fl_data_buf=TRUE;
                               else fl_232_2.tx=1;
                               Appl_RS232_2.dst_buf=*(uint16_t*)&buf_rx_ppp[TR_SRC];
                               Appl_RS232_2.id_buf=buf_rx_ppp[TR_ID];
                               for(i=0;i<8;i++)Appl_RS232_2.cont_buf[i]=buf_rx_ppp[offset+C1_CONT+i];
                               Appl_RS232_2.ln_data_buf=length-10;
                               for(i=0;i<Appl_RS232_2.ln_data_buf;i++)Appl_RS232_2_data_buf[i]=buf_rx_ppp[offset+C1_DATA_RS+i];
                               
                               
                            break;
                            
   */                  
                            

                       case PORT_SEQ:
                            if(length<5)return(0);
                           // if(*(uint16_t*)&buf_rx_ppp[TR_SRC]!=num_seq_cl)return(0);
                               
                            if(((buf_rx_ppp[TR_V]>>4)&0x03)==KVIT)
                                {
                                if(*(uint16_t*)&buf_rx_ppp[TR_SRC]!=num_seq_cl)return(0);   
                                
                                if(length!=5)return(0);
                                if(*(uint16_t*)&buf_rx_ppp[offset+C1_DATA+1]!=L_KVITOK)return(0);
                                
                               if(buf_rx_ppp[offset+C1_DATA]==KVITOK_SEQ_10)         // подтверждаем архивы
                               {
                                        if(fl_appl_seq.kv_waits==1)
                                          {   
                                           if(Appl_seq.id!=buf_rx_ppp[TR_ID])return(0);// проверка идентификатора
                                 
                                             Appl_seq.cnt_waits=0;
                                             fl_appl_seq.kv_waits=0;
                                 
                                             cnt_no_link=vol_cnt_no_link;//dobavka
                                             Control.link_waits=FALSE;
                                             Control.cnt_link=Control.vol_link;                   
                                             Control.cnt_try=0;
                                             Control.link_no=FALSE;
                                 
                                             fl_appl_seq.en_povtor=0;
                                             // КВИТИРОВАНИЕ очередного архива
                                                
                                                Appl_seq.l_data=0;
                                                
                                                if ( sim800Settings.ind_kvit_asinhr > *eventRecordPtr) return(0);
                                                
                                                sim800Settings.ind_kvit_asinhr++;
                                              
                                                if ( sim800Settings.ind_kvit_asinhr> TAPS_ARCHIVE_RECORDS_NUM) sim800Settings.ind_kvit_asinhr=0;
                                                
                                                framWrite(A_IND_DNS_KVIT,(uint8_t*)&sim800Settings.ind_kvit_asinhr,2); 
                                               
                                                    send_info(sizeof(rec_evc),rec_evc,1,buf_rx_ppp[TR_ID]);
                                                  return(1); 
                                           }  else return(0);   
                                   }
                               else return(0);
                                  
                                }
                            
                            
                            if(((buf_rx_ppp[TR_V]>>4)&0x03)==ZAPR)
                               {
                               if(buf_rx_ppp[offset+C1_DATA]!=ZAPR_TC)return(0);
                               if(length!=5)return(0);
                               if(*(uint16_t*)&buf_rx_ppp[offset+C1_DATA+1]!=L_ZAPR_TC)return(0);
                                
                               if(fl_ip.act_ip_end!=1)return(0);
                               fl_ip.act_ip_end=0; 
                               clr_cntr_nat();
                               clr_cntr_link();
                               Obj_ppp_tx.prozr=FALSE;            
                               Obj_ppp_tx.version=VER2;      //версия 
                               Obj_ppp_tx.type_pac=OTV;     //тип пакета
                               Obj_ppp_tx.num_src=num_self; // номер отправителя
                               Obj_ppp_tx.num_dst=*(uint16_t*)&buf_rx_ppp[TR_SRC]; // номер получателя
                               Obj_ppp_tx.id_pac=buf_rx_ppp[TR_ID]; // идентификатор пакета  
                               Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций          
                               Obj_ppp_tx.l_opt=0;// длина буфера опций   
                               buf_tx_232[TR_OP_DATA+C1_PROT]=PROT_C1;
                               buf_tx_232[TR_OP_DATA+C1_PORT]=PORT_SEQ;
                               buf_tx_232[TR_OP_DATA+C1_DATA]=SEQ_STATE;
                                *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1]=L_STATE;
                                *(uint32_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3]=unix;
                              
                                
                                
                              
                                   buf_tx_232[TR_OP_DATA+C1_DATA+7]=DES_SEQ;
                                   buf_tx_232[TR_OP_DATA+C1_DATA+8]=DES_SEQ;
                                   buf_tx_232[TR_OP_DATA+C1_DATA+9]=DES_SEQ;
                               
                             
                               
                          
                                
                                Obj_ppp_tx.l_data=5+L_STATE;   // длина данных
                                Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных
                             
                               send_info(sizeof(ans_out_st_contr),ans_out_st_contr,1,buf_rx_ppp[TR_ID]);
                              
                               form_buf_tx_ppp();                            
                            //   UCSR0A=UCSR0A | TXC;                            
                            //   UCSR0B=UCSR0B | TXEN;
                            //   UCSR0B=UCSR0B | TXCIE;
                               if(check_cts()==1) return(1);                
                               //S2_RD//
                              // UDR0=buf_tx_232[0]; 
                                
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                               
                               return(1);
                               } 
                            
                           /* 
                            if(fl_appl_seq.kv_waits==1)
                                {  
                                 if(((buf_rx_ppp[TR_V]>>4)&0x03)!=KVIT)return(0);  
                                 if(Appl_seq.id!=buf_rx_ppp[TR_ID])return(0);// проверка идентификатора
                                

                                 Appl_seq.cnt_waits=0;
                                 fl_appl_seq.kv_waits=0;
                                 
                                 cnt_no_link=vol_cnt_no_link;//dobavka
                                 Control.link_waits=FALSE;
                                 Control.cnt_link=Control.vol_link;                   
                                 Control.cnt_try=0;
                                 Control.link_no=FALSE;
                                 
                                 fl_appl_seq.en_povtor=0;
                                 // КВИТИРОВАНИЕ
                                 Appl_seq.p_out=Appl_seq.p_out_kv;
                                 Appl_seq.l_data=0;
                                 if(fl_appl_seq1.over_buf==1)
                                    {
                                    fl_appl_seq1.over_buf=0;
                                    fl_appl_seq1.send_state=1;
                                    }
                                 send_info(sizeof(rec_evc),rec_evc,1,buf_rx_ppp[TR_ID]);
                                 return(1); 
                                 }
                              */   
                                 
                            break;     
                       case PORT_SYS:
                            break;
                        case PORT_CONF:
                            if(length<5)return(0);               
                            if(((buf_rx_ppp[TR_V]>>4)&0x03)!=ZAPR)return(0);
                            
                            if (check_ln_conf(&buf_rx_ppp[0],offset,count_rx_ppp)==1)return(0);
                            

                            
                            if((buf_rx_ppp[offset+C1_DATA] & 0x7f)==CONF_LOG)
                            {
                                 if(length>5)return(0);
                                 if(*(uint16_t*)&buf_rx_ppp[offset+C1_DATA+1]!=L_CONF_LOG)return(0);
                                 if((buf_rx_ppp[offset+C1_DATA] & 0x80)==0x80)
                                    {
                                    //стирание лог файла          
                                    //__watchdog_reset();
                                   //  WrArrayToFlesh(BEG_BUF_LOG,0,(L_LOG*6),0x01,0x00);  // Отменил операцию очистки 
                                   // __watchdog_reset();
                                    point_log_buf=0;
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1]=L_CONF_LOG+1;
                                     Obj_ppp_tx.l_data=5+L_CONF_LOG+1;
                                  //  ii=0;
                                    }
                                    else
                                    {
                                    //чтение 
                                    CopyMass(BEG_BUF_LOG,&buf_tx_232[TR_OP_DATA+C1_DATA+4],(L_LOG*6));
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+1]=L_CONF_LOG+(L_LOG*6)+1;
                                     Obj_ppp_tx.l_data=5+L_CONF_LOG+(L_LOG*6)+1;
                                   // ii=;
                                    }
                                
                                   buf_tx_232[TR_OP_DATA+C1_DATA]=buf_rx_ppp[offset+C1_DATA];
                                   buf_tx_232[TR_OP_DATA+C1_DATA+3]=0x00; //OK
                                   goto otv_conf;                              
                              
                            }
                            
                             
                             
                           Obj_ppp_tx.l_data=proc_config(&buf_rx_ppp[0],offset,count_rx_ppp);
                           Obj_ppp_tx.l_data=Obj_ppp_tx.l_data+2;     
                          
  
  
  otv_conf:                        
                           clr_cntr_nat();                                  
                           if(fl_ip.act_ip_end!=1)return(0);
                           fl_ip.act_ip_end=0;      
                           clr_cntr_link();
                                  Obj_ppp_tx.prozr=FALSE;            
                                  Obj_ppp_tx.version=VER2;      //версия 
                                  Obj_ppp_tx.type_pac=OTV;     //тип пакета
                                  Obj_ppp_tx.num_src=num_self; // номер отправителя
                                  Obj_ppp_tx.num_dst=*(uint16_t*)&buf_rx_ppp[TR_SRC]; // номер получателя
                                  Obj_ppp_tx.id_pac=buf_rx_ppp[TR_ID]; // идентификатор пакета  
                                  Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций  
                                  Obj_ppp_tx.l_opt=0;// длина буфера опций   
                                  Obj_ppp_tx.kol_opt=0;// количество опций   
     
                                  buf_tx_232[TR_OP_DATA+C1_PROT]=PROT_C1;
                                  buf_tx_232[TR_OP_DATA+C1_PORT]=PORT_CONF;
 
                                
                                 Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных
        
                                 
                                send_info(sizeof(ans_out_config),ans_out_config,1,buf_rx_ppp[TR_ID]); 
                                form_buf_tx_ppp();   
                                
                                if((fl_rewrite.ip==1)||(fl_rewrite.udp==1)||(fl_rewrite.num_self==1))
                                   {
                                     
                                 //     UCSR2C=0x06;
                                 //   UBRR2H=R9600_H;
                                 //   UBRR2L=R9600_L;
                                  //  mov_buf(L_IP_PAR,&A_IP_PAR_array[0]); // для контроля
                                    CopyMass(A_IP_PAR,&buf_rx_ppp[0],L_IP_PAR);
                                    
                                 //   UCSR2C=0x06;
                                 //   UBRR2H=R9600_H;
                                 //   UBRR2L=R9600_L;
                                  //  mov_buf(L_IP_PAR,&buf_rx_ppp[0]); // для контроля
                                    
                                    
                                    
                                    if(fl_rewrite.num_self==1)num_self=*(uint16_t*)&buf_rx_ppp[OFS_NUM];
                                    if(fl_rewrite.udp==1)port_udp=*(uint16_t*)&buf_rx_ppp[OFS_PORT];
                                    if(fl_rewrite.ip==1)
                                          {
                                          ip_ls[0]=buf_rx_ppp[OFS_IP];
                                          ip_ls[1]=buf_rx_ppp[OFS_IP+1];
                                          ip_ls[2]=buf_rx_ppp[OFS_IP+2];
                                          ip_ls[3]=buf_rx_ppp[OFS_IP+3];
                                          }
                                   fl_rewrite.ip=0;
                                   fl_rewrite.udp=0;
                                   fl_rewrite.num_self=0;
                                   prov_ozu=crc_ozu();
                                   
                                   Control.cnt_link=3;
                                   Control.link_waits=FALSE;
                                   Control.cnt_try=0;
                                   Control.link_no=FALSE;
                                   }
                               
                                                         
                             //   UCSR0A=UCSR0A | TXC;                            
                             //   UCSR0B=UCSR0B | TXEN;
                             //   UCSR0B=UCSR0B | TXCIE;
                                if(check_cts()==1) return(1);   
                                //S2_RD //           
                               // UDR0=buf_tx_232[0];
                                 
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                                
 
                             return(1);  
                             
                                 
                           // break;
                       
                       
                       
                       
                       
                       case PORT_PROG:
                         
                    /*
                         функцию программирования оставить через КП конфиг
                         она будет формировать пакет в формате, который будет идти по модбасу
                         для программирования
                         В ПЧ текущей версии внутрисхемное программирование не поддерживается херим код
                         */
                            
                            
                               {
 
                               send_no_sinc(buf_rx_ppp[TR_ID],*(uint16_t*)&buf_rx_ppp[TR_SRC],NO_SINC_PG);
                               return(0);
                               }
                            
                                  
                      
          
              case PORT_MBUS:
                                 if(((buf_rx_ppp[TR_V]>>4)&0x03)==ZAPR)
                               {
                                 
                                 if(length>259)return(0);
                                 
                                 
                                 
                                 //////////////////////каминтел
                                
                            //     if(sel_modul==1)
                                 {
                                 buf_tx_232[TR_OP_DATA+C1_DATA]=buf_rx_ppp[offset+C1_DATA+8];//функция                       
                                 buf_tx_232[TR_OP_DATA+C1_DATA+1]=buf_rx_ppp[offset+C1_DATA+9];//адрес
                                 
                                 }
                                ///////////////////////// конец каминтел 
                                 
                            //     if((length<18)||(length>259))return(0);
                                 
                                 if(length<18)return(0);
                                
                                 
                                 // проверку не делать на допустимость в это задача ПЧ
                               //  ii=proc_modbus(&buf_rx_ppp[offset+C1_DATA+8],length-10);
                               //  if(ii==0)return(0);
                                 
                             
                               
                               if(fl_ip.act_ip_end!=1)return(0);
                               fl_ip.act_ip_end=0;
                               
                               clr_cntr_nat();
                               clr_cntr_link();
                               Obj_ppp_tx.prozr=FALSE;            
                               Obj_ppp_tx.version=VER2;      //версия 
                               Obj_ppp_tx.type_pac=OTV;     //тип пакета
                               Obj_ppp_tx.num_src=num_self; // номер отправителя
                               Obj_ppp_tx.num_dst=*(uint16_t*)&buf_rx_ppp[TR_SRC]; // номер получателя
                               Obj_ppp_tx.id_pac=buf_rx_ppp[TR_ID]; // идентификатор пакета  
                               Obj_ppp_tx.p_opt=&buf_opt_tr[0]; // //указатель буфера опций          
                               Obj_ppp_tx.l_opt=0;// длина буфера опций  
                               Obj_ppp_tx.kol_opt=0;// количество опций 
                               buf_tx_232[TR_OP_DATA+C1_PROT]=PROT_C1;
                               buf_tx_232[TR_OP_DATA+C1_PORT]=PORT_MBUS;
                               
                               
                              buf_tx_232[TR_OP_DATA+C1_DATA]=buf_rx_ppp[offset+C1_DATA+8];//адрес  !!! оставляем                              
                              buf_tx_232[TR_OP_DATA+C1_DATA+1]=buf_rx_ppp[offset+C1_DATA+9];//функция  !!! оставляем
                                                        
                              crc=swap(*(uint16_t*)&buf_rx_ppp[offset+C1_DATA+10]);//здесь временно старт адрес 
                              kol=swap(*(uint16_t*)&buf_rx_ppp[offset+C1_DATA+12]); //здесь временно количество регистров
                              // 
                              // работаем с рустемовской функцмей
        
                              MyoutBuffer[0] = buf_rx_ppp[offset+C1_DATA+9];//функция  !!! оставляем
                              MyoutBuffer[1] = buf_rx_ppp[offset+C1_DATA+10];
                              MyoutBuffer[2] = buf_rx_ppp[offset+C1_DATA+11];  //старт адрес 
                              MyoutBuffer[3] = 0; 
                               MyoutBuffer[4] = buf_rx_ppp[offset+C1_DATA+13];  // к-во регистров
                           //////////////////////////// это для ПЧ   
                               // скопировать пришедший модбас пакет во входной буфер MyoutBuffer[5000]
                               // выходные данные с функции скопировать в промежуточный массив чтобы не испортить массив
                               // если выходных данных больше 1000 то я эту посылку не отсылаю
                               for(i=0;i< (length-10);i++) MyoutBuffer[i]= buf_rx_ppp[offset+C1_DATA+ 8 +i]; 
                               zapr_from_sim800=1; // признак что запрос идет состорны gsm канала
                               // при этом операция распознавания не работает и опрерации с установкой адреса модбас должны работать
                               if(modbusSlaveTask(&MyoutBuffer[0], length-10, &MyinBuffer[0], &byteSize)==0) {zapr_from_sim800=0;return(0);}  // Нет данных
                               zapr_from_sim800=0;
                              // возвращает готовый пакет с СРС размером byteSize
                              if ( byteSize > 1000) return(0);  // слишком много данных не полезет в UDP
                               p2p=(char *)&buf_tx_232[TR_OP_DATA+C1_DATA];
                              for(i=0;i<byteSize;i++) *p2p++=MyinBuffer[i];     // копируем ответ в UDP пакет
                              Obj_ppp_tx.l_data=byteSize+2;     // 2 это смещение до указателя C1_PROT
                               //////////////////////////// это для ПЧ 
 
/*                              
                              if (MymodbusRegistersEdit(MyoutBuffer, MyinBuffer, 1, &byteSize) != 0) return(0);  // Нет данных
                              
                              p2p=(char *)&buf_tx_232[TR_OP_DATA+C1_DATA+1];
                              for(i=0;i<byteSize;i++) *p2p++=MyinBuffer[1+i];     // к-во данных без срс и без адреса устройства
                              Obj_ppp_tx.l_data=byteSize+1;

    
                                *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+Obj_ppp_tx.l_data]=crc_m1(&buf_tx_232[TR_OP_DATA+C1_DATA],Obj_ppp_tx.l_data,0xffff);       //crc                               
                               i=buf_tx_232[TR_OP_DATA+C1_DATA+Obj_ppp_tx.l_data];
                               buf_tx_232[TR_OP_DATA+C1_DATA+Obj_ppp_tx.l_data]=buf_tx_232[TR_OP_DATA+C1_DATA+Obj_ppp_tx.l_data+1];
                               buf_tx_232[TR_OP_DATA+C1_DATA+Obj_ppp_tx.l_data+1]=i;
                                Obj_ppp_tx.l_data=Obj_ppp_tx.l_data+4;   // длина данных
                               
metka_send_mbus: 
*/  
  
  
                                Obj_ppp_tx.p_data=&buf_tx_232[TR_OP_DATA+C1_PROT];  //указатель буфера данных
                                
                                
                                
                                
                                
                               send_info(sizeof(ans_out_mbus),ans_out_mbus,1,Obj_ppp_tx.id_pac);    
                               
                               
                               form_buf_tx_ppp();                            
                             //  UCSR0A=UCSR0A | TXC;                            
                             //  UCSR0B=UCSR0B | TXEN;
                             //  UCSR0B=UCSR0B | TXCIE;
                               if(check_cts()==1) return(1);                
                               //S2_RD//
                               //UDR0=buf_tx_232[0]; 
                                
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);
    
    
                               
                               return(1);
                               } 
                              break;                              
                      
                      
                      }
              
              
              
              
               }//для версии VER2
               
              
                  
               return(0);
               } 
 
 ////////////////EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
 
 
 
 
 
 
 

 
 
 
void analiz_ppp_rx(void)
{
   
  
  
  if(fl_ipcp.act_ipcp_end==0)return;
  if (fl_ip.act_ip_end==0)return;         
  
  if(Buf1_rx_ppp.rec==TRUE)
  {
   
  proc_ppp_packet(&Buf1_rx_ppp.data[0],Buf1_rx_ppp.ln_data); 
  Buf1_rx_ppp.ln_data=0;
  Buf1_rx_ppp.rec=FALSE;
  Buf1_rx_ppp.busy=FALSE;
  return;
  }
  
  if(Buf2_rx_ppp.rec==TRUE)
  {
  
 
  
  proc_ppp_packet(&Buf2_rx_ppp.data[0],Buf2_rx_ppp.ln_data); 
  Buf2_rx_ppp.ln_data=0;
  Buf2_rx_ppp.rec=FALSE;
  Buf2_rx_ppp.busy=FALSE;
  return;
  }        
          
}






 


uint16_t proc_modbus(unsigned char *pnt_buf,unsigned char length)
 {
   uint16_t temp;
   unsigned char i;

     i=pnt_buf[length-2];
     pnt_buf[length-2]=pnt_buf[length-1];
     pnt_buf[length-1]=i;                            
     if(*(uint16_t*)&pnt_buf[length-2]!=crc_m1(&pnt_buf[0],length-2,0xffff))return(0); //crc
   
   
   switch (pnt_buf[1])
   {
   case 1:if(length!=8)return(0);
        if(swap(*(uint16_t*)&pnt_buf[4])==0 || swap(*(uint16_t*)&pnt_buf[4])>2000){temp=pnt_buf[1];temp=temp<<8;return(temp | 0x8003);}
        if(swap(*(uint16_t*)&pnt_buf[4])>8)return(0);
        return(1);
   case 3:
   case 4:
      if(length!=8)return(0);
      if(swap(*(uint16_t*)&pnt_buf[4])==0 || swap(*(uint16_t*)&pnt_buf[4])>125){temp=pnt_buf[1];temp=temp<<8;return(temp | 0x8003);}
      if (( 0xffff - swap(*(uint16_t*)&pnt_buf[2])) < ( swap(*(uint16_t*)&pnt_buf[4])-1 )){temp=pnt_buf[1];temp=temp<<8;return(temp | 0x8002);}
      break;   
   case 5:
      if(length!=8)return(0);
      if(swap(*(uint16_t*)&pnt_buf[4])!=0 && swap(*(uint16_t*)&pnt_buf[4])!=0xff00){temp=pnt_buf[1];temp=temp<<8;return(0x8503);}
      return(5);
   case 6:
     if(length!=8)return(0);
     return(6);  
   case 16:
     if(length<11)return(0); 
     if(length>249)return(0);
     if(swap(*(uint16_t*)&pnt_buf[4])==0 || swap(*(uint16_t*)&pnt_buf[4])>120)return(0x9003);
     temp=pnt_buf[5];
     if(temp!=swap(*(uint16_t*)&pnt_buf[4]))return(0x9003);
     if ((0xffff -swap(*(uint16_t*)&pnt_buf[2])) < (swap( *(uint16_t*)&pnt_buf[4])-1 ))return(0x9002);
     return(16);
   default: return(0);     
   }
   
   return(pnt_buf[1]);
 }
 

                             
char ext_mbus(unsigned char *buf_rx_ppp,unsigned char offset_in,uint16_t start_adres, uint16_t size,char kod_comand)
{
              uint16_t ii;
              // char i,k;
              // unsigned char flag_time;
              // uint32_t old_unix;
               uint16_t length;
               
               //buf_tx_232[TR_OP_DATA+C1_DATA] - указатель, куда складывать результат
               // buf_rx_ppp[offset+C1_DATA+8]; - указатель на входной запрос модбас или ELAM
               // length-10 - количество данных в модбас пакете
               //  эти данные мы потом должны подставить в функцию Рустема
               
               buf_tx_232[TR_OP_DATA+C1_DATA]=  buf_rx_ppp[offset_in + C1_DATA+8];//адрес  !!! оставляем                              
               buf_tx_232[TR_OP_DATA+C1_DATA+1]=buf_rx_ppp[offset_in + C1_DATA+9];//функция  !!! оставляем
             
                              
                              
                                                             switch(kod_comand)
                               {
                                 case 1:
                                      buf_tx_232[TR_OP_DATA+C1_DATA+2]=1;
                                      buf_tx_232[TR_OP_DATA+C1_DATA+3]=0;
                                      
                                     
                                      
                                      Obj_ppp_tx.l_data=4;    
                                      break; 
                                        
                                 case 3:
                                 case 4:
                                   
                                    buf_tx_232[TR_OP_DATA+C1_DATA+2]=buf_rx_ppp[offset_in+C1_DATA+13] << 1;//byte_count
                                    
                                    *(uint32_t*)&modbus_mem1[AD_TIME]=unix;   // unix time
                                    length=modbus_mem1[AD_TIME+1];
                                    modbus_mem1[AD_TIME+1]=modbus_mem1[AD_TIME];
                                    modbus_mem1[AD_TIME]=length;
                                    
                                    /////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    /////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    for(ii=0;ii<size;ii++)
                                    { 
                                    *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=0;
                                   
                                    if((start_adres+ii)==0)//нулевой адрес там свои тс, для совместимости
                                    {
                                      buf_tx_232[TR_OP_DATA+C1_DATA+3]=0;
                                      buf_tx_232[TR_OP_DATA+C1_DATA+4]=(sv1.tc & 0x01) | ((sv2.tc & 0x01)<<1);
                                    }
                                    
               
                                   

                                  
     
                                    if((start_adres+ii)==MB_SEC)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_sec);
                                    if((start_adres+ii)==MB_MIN)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_min);
                                    if((start_adres+ii)==MB_HR)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_hor);
                                    if((start_adres+ii)==MB_DAY)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_date);
                                    if((start_adres+ii)==MB_MONTH)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_month);
                                    if((start_adres+ii)==MB_YEAR)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_year);

                                    
                                   
                                   
                                    
                                    
                                    }  // end 
                                    ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    
                                    
                                       
                                       
                                       Obj_ppp_tx.l_data=3+size*2; //без start_adres
                                       break;
                                 case 5:
                                        *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+2]=swap(start_adres);//start
                                        *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+4]=0;
                                      
                                      
                                        
                                        *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+4]=swap(size);

                                       //Сформировать registr value и конечное число отправляемых байт выдать команду
                                       
                                        Obj_ppp_tx.l_data=6;
                                       break;       
                                 case 6:
                                       
                                       *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+2]=swap(start_adres);//start
                                       *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+4]=swap(size);
                                       
                                       
                                       
                                      
                                        
                                       Obj_ppp_tx.l_data=6;
                                       
                                       break;
                                 case 16:
                                      
                                     
                                      
                                      *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+2]=swap(start_adres);//start
                                      *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+4]=swap(size);//start
                                       /////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    /////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                   
                                      
                                      
                                      
                                      
                                  
                                    ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                   
                                   
                                   
                                      
                                     
                                      Obj_ppp_tx.l_data=6; 
                                       
                                       break;
                               case 0x8302:      
                               case 0x8303:
                               case 0x8402:      
                               case 0x8403:      
                               case 0x8503:  
                               case 0x9002:
                               case 0x9003: 
                                          buf_tx_232[TR_OP_DATA+C1_DATA+1]=ii>>8;
                                          buf_tx_232[TR_OP_DATA+C1_DATA+2]=ii;
                                          Obj_ppp_tx.l_data=3;
                                          break;
                               default: return(0);  // не нашли такого кода команды
                               }  // end switch
                return(1);
}



char MymodbusRegistersEdit( char *mb_req, char *mb_ans, char b,  char *size_ans) 
// возвращает нолб если все нормально
//char ext_mbus(unsigned char *buf_rx_ppp,unsigned char offset_in,uint16_t start_adres, uint16_t size,char kod_comand)
{
              uint16_t ii,start_adres;
              char size;
              // char i,k;
              // unsigned char flag_time;
              // uint32_t old_unix;
               uint16_t length;
               
               //buf_tx_232[TR_OP_DATA+C1_DATA] - указатель, куда складывать результат
               // buf_rx_ppp[offset+C1_DATA+8]; - указатель на входной запрос модбас или ELAM
               // length-10 - количество данных в модбас пакете
               //  эти данные мы потом должны подставить в функцию Рустема
               
      //         buf_tx_232[TR_OP_DATA+C1_DATA]=  buf_rx_ppp[offset_in + C1_DATA+8];//адрес  !!! оставляем                              
      //         buf_tx_232[TR_OP_DATA+C1_DATA+1]=buf_rx_ppp[offset_in + C1_DATA+9];//функция  !!! оставляем
               
                           mb_ans++;
                           *mb_ans = *mb_req;   // номер команды
                           start_adres=swap(*(uint16_t*)(mb_req + 1));
                           size=*(mb_req + 4);  // к-во регистров
                              
                                                             switch(*mb_req)
                               {
                                 case 1:
                                      //buf_tx_232[TR_OP_DATA+C1_DATA+2]=1;
                                      //buf_tx_232[TR_OP_DATA+C1_DATA+3]=0;                                   
                                      //Obj_ppp_tx.l_data=4;    
                                      
                                      *(mb_ans+1)=1;
                                      *mb_ans++=0;
                                      *size_ans=3;
                                      
                                      break; 
                                        
                                 case 3:
                                 case 4:
                                   
                                   // buf_tx_232[TR_OP_DATA+C1_DATA+2]=buf_rx_ppp[offset_in+C1_DATA+13] << 1;//byte_count
                                    *(mb_ans+1)=size<<1;  // к-во байт
                                    
                                    *(uint32_t*)&modbus_mem1[AD_TIME]=unix;   // unix time
                                    length=modbus_mem1[AD_TIME+1];
                                    modbus_mem1[AD_TIME+1]=modbus_mem1[AD_TIME];
                                    modbus_mem1[AD_TIME]=length;
                                    
                                    /////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    /////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    for(ii=0;ii<size;ii++)
                                    { 
                                     *(uint16_t*)(mb_ans+2+(ii<<1) )=0;
                                     
                                  //  *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=0;
                                   
                                   
                                    
               
                                   

                                  
                                    /*
                                    if((start_adres+ii)==MB_SEC)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_sec);
                                    if((start_adres+ii)==MB_MIN)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_min);
                                    if((start_adres+ii)==MB_HR)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_hor);
                                    if((start_adres+ii)==MB_DAY)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_date);
                                    if((start_adres+ii)==MB_MONTH)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_month);
                                    if((start_adres+ii)==MB_YEAR)*(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+3+(ii<<1)]=swap(real_time.r_year);
                                   */
                                    
                                    if((start_adres+ii)==MB_SEC)  *(uint16_t*)(mb_ans+2+(ii<<1))=swap(real_time.r_sec);
                                    if((start_adres+ii)==MB_MIN)  *(uint16_t*)(mb_ans+2+(ii<<1))=swap(real_time.r_min);
                                    if((start_adres+ii)==MB_HR)   *(uint16_t*)(mb_ans+2+(ii<<1))=swap(real_time.r_hor);
                                    if((start_adres+ii)==MB_DAY)  *(uint16_t*)(mb_ans+2+(ii<<1))=swap(real_time.r_date);
                                    if((start_adres+ii)==MB_MONTH)*(uint16_t*)(mb_ans+2+(ii<<1))=swap(real_time.r_month);
                                    if((start_adres+ii)==MB_YEAR) *(uint16_t*)(mb_ans+2+(ii<<1))=swap(real_time.r_year);
                                   
                                   
                                    
                                    
                                    }  // end 
                                    ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    ///////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    
                                    
                                       
                                       
                                      //  Obj_ppp_tx.l_data=3+size*2; //без start_adres
                                    *size_ans=2+size*2;
                                       break;
                                 case 5:
                                        *(uint16_t*)(mb_ans+1)=swap(start_adres);//start
                                        *(uint16_t*)(mb_ans+3)=swap(size);
                                      
                                      
                                        
                                      //  *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+4]=swap(size);

                                       //Сформировать registr value и конечное число отправляемых байт выдать команду
                                       
                                        //Obj_ppp_tx.l_data=6;
                                        *size_ans=5;
                                       break;       
                                 case 6:
                                       
                                      // *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+2]=swap(start_adres);//start
                                      // *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+4]=swap(size);
                                        *(uint16_t*)(mb_ans+1)=swap(start_adres);//start
                                        *(uint16_t*)(mb_ans+3)=swap(size);
                                       
                                       
                                       
                                      
                                        
                                       //Obj_ppp_tx.l_data=6;
                                       *size_ans=5;
                                       break;
                                 case 16:
                                      
                                     
                                      
                                   //   *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+2]=swap(start_adres);//start
                                   //   *(uint16_t*)&buf_tx_232[TR_OP_DATA+C1_DATA+4]=swap(size);//start
                                       
                                        *(uint16_t*)(mb_ans+1)=swap(start_adres);//start
                                        *(uint16_t*)(mb_ans+3)=swap(size);
                                       
                                   
                                      
                                     
                                      //Obj_ppp_tx.l_data=6; 
                                      *size_ans=5; 
                                       break;
                              
                               default: return(1);  // не нашли такого кода команды
                               }  // end switch
                return(0);
}
