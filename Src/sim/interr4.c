#include "stm32f4xx_hal.h"
//#include <iom1280.h> 
//#include <inavr.h> 
//#include "dfpin.h"
#include "dfproc.h"
#include "ozu_map.h"
#include "dfcnst.h"
#include "def_at.h"
#include "def_link.h"
#include "sec.h"
#include "map_ef.h"
#include "map_mbus.h"
#include "main.h"

void interrupt_debug_tu(void);
void CLR_PWRK(void);
extern UART_HandleTypeDef huart3;
unsigned char sim800Data;

//void milisecund_handler_sim800(void);
//char HAL_GPIO_ReadPin( unsigned char port, char GPIO_Pin);
extern unsigned char trevoga;

char dabl,flag_arhiv_prihod;
unsigned char tempo_pac,contr_sym_from_sim800;


extern unsigned char cnt_snd;
extern char bit_registr1;

extern uint16_t cnt_sec;

extern uint16_t cnt_supervosor;

extern unsigned char simka; //dobavka

extern unsigned char layer_PPP;
                                              
extern uint32_t cnt_outcom,cnt_incom;
extern unsigned char time_cnt_ch_time;

uint16_t crc_m1(unsigned char *ka,uint16_t num,uint16_t crc);
void framWrite(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num);

void lock_it(void);

void fun_state_md(void);
extern void tx_rs485_1(void);
extern void tx_rs485_2(void);
extern void tx_rs232_2(void);
extern void tx_rs232_time(void);
extern void test_fram(void);
extern uint32_t burst_ds_r(void);

unsigned char check_cts(void);

//extern unsigned int func_crc_tii(void);
//extern __no_init unsigned int crc_tii;
//extern __no_init unsigned int cnt_tii[8];

extern char ip_change;
extern unsigned char Fl_ch_sim; ; //dobavka
extern uint32_t cnt_no_link, cnt_link_res; //dobavka

//extern unsigned char count_block_net;

extern struct
 {
 unsigned char  clr_wdt     :1;
 unsigned char  fl_out      :1;
 unsigned char  from_timer0 :1;
 unsigned char  fl_main     :1;
 unsigned char  from_timer3 :1;
 unsigned char  from_timer2 :1; 
 
 }fl_wdt;
unsigned char cnt_clr_wdt, cnt_flag_out;

extern unsigned char cnt_rst_lcp,vol_cnt_rst_lcp;
extern unsigned char cnt_rst_ipcp,vol_cnt_rst_ipcp;                       
extern unsigned char cnt_rst_pap,vol_cnt_rst_pap;  
extern uint16_t cnt_lcp_tm_out,cnt_ipcp_tm_out;
extern uint16_t cnt_pap_tm_out;

extern struct
{
unsigned char over_buf :1;
unsigned char send_state :1;
unsigned char enable:1; 
}fl_appl_seq1; 

 extern struct
  {
  unsigned char  ip         :1;
  unsigned char  udp        :1;
  unsigned char  num_self   :1;
  unsigned char cnt_reset;
  }fl_rewrite;
 


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


  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!структуры приложения
extern  struct
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

//unsigned char Appl_seq_des;

//unsigned char state_seq;

extern unsigned char Appl_seq_buf[MAX_BUF_SEQ]; 
unsigned char point_Head;



extern struct
{
unsigned char en_tx :1;
unsigned char kv_waits :1;
unsigned char en_povtor:1;
}fl_appl_seq;


extern struct
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


extern unsigned char proverka;
extern unsigned char fl_at_mom_232;

extern enum t_event_modem event_modem;


extern unsigned char buf_tx_232[VOL_TX_PPP];
extern uint16_t count_tx_ppp,vol_tx_ppp;


extern unsigned char Regim;
extern unsigned char cnt_stm_tm1,cnt_stm_tm2;
extern uint16_t cnt_key_off;

extern void sending_at_pac(void); 

extern  struct
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

extern enum bool command_AT;
extern enum bool fl_cts_232_ignor;
extern uint16_t cnt_cts_off;

extern struct
 {
 unsigned char on: 1;
 } fl_cts_232;

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


//#pragma dataseg=MY_SEG
unsigned char Rs485_1_buf_rx_tx[MAX_BUF_RS485_1];
unsigned char Rs485_2_buf_rx_tx[MAX_BUF_RS485_2];
unsigned char Rs232_2_buf_rx_tx[MAX_BUF_RS232_2];
//#pragma dataseg=default


struct                   //структура описывающая работу порта "RS485_1" 
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
  
struct
 {
 unsigned char busy : 1;
 unsigned char rec : 1;
 unsigned char tm_out : 1;
 unsigned char tx : 1;
 unsigned char over : 1;
 unsigned char buffed : 1;
 }fl_485_1,fl_485_2,fl_232_2;



extern  struct
 {
  unsigned char data[VOL_RX_PPP];//сам буфер
  uint16_t ln_data;  // длина данных
  enum bool rec; // пакет принят
  enum bool busy;// буфер занят
  enum bool check_busy;// проверка занят ли буфер
 }Buf1_rx_ppp,Buf2_rx_ppp; // 


 struct
 {
 unsigned char fl_7e : 1;//
 unsigned char fl_7d : 1;//
 unsigned char switcher : 1;//переключатель буферов
 } fl_rx_ppp;

///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!для РРР
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

extern struct
 {
  unsigned char add_byte :1; 
 }fl_reg3;
  //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE 
 

extern uint32_t summa_temperatura;         
extern uint16_t temperatura;
extern uint16_t count_summa_temperatura; 

uint16_t modbus_mem1[SEG1];
extern uint32_t arr_tii_32[16];




uint32_t summa_adc[4];         
unsigned char count_summa_adc[4];


///11111111111111111111111111111 функции

/*
 struct struct_ts
{
 
  unsigned char tek_pin :1;
  unsigned char old_pin :1;
  unsigned char fl_ch_tc :1;
//  unsigned char real_tc:1;
  unsigned char count_tc;
};
*/


///11111111111111111111111111111 функции
 
unsigned char cnt_tu1, cnt_tu2;

/*
void send_232_2(void)
{
UCSR2B=UCSR2B & ~(RXCIE | RXEN);   
UCSR2B=UCSR2B | TXCIE;  // enable transmit 485_2
UDR2=*Rs232_2.p_data485++;
Rs232_2.cnt_bt_rx_tx--; 
}

*/


/*

void sending_ppp_proverka(void)
{   
   if(count_tx_ppp>=20)
   {
    // UCSR0B=UCSR0B & ~TXEN;!!!!
    // UCSR0B=UCSR0B & ~TXCIE; !!!!
     //S2_OFF;//
     return;
   }
   
    UDR0=buf_tx_232[count_tx_ppp]; 
   
   
   
   count_tx_ppp++;
}

*/


  void sending_ppp_pac(void)
{   
  cnt_supervosor=0;
  
 if(count_tx_ppp>=(vol_tx_ppp-1))
     { 
  
     if (fl_lcp.act_lcp_end==0) fl_lcp.act_lcp_end=1;
     if (fl_pap.act_pap_end==0) fl_pap.act_pap_end=1;    
     if (fl_ipcp.act_ipcp_end==0) fl_ipcp.act_ipcp_end=1;  
     if (fl_ip.act_ip_end==0) fl_ip.act_ip_end=1; 
     fl_reg3.add_byte=0;   
   //  UCSR0B=UCSR0B & ~TXEN;!!!!!!!
   //  UCSR0B=UCSR0B & ~TXCIE; !!!!
     //S2_OFF;//
     
     
       
     __HAL_UART_DISABLE_IT(&huart3,UART_IT_TXE);
     
     
     return;
     }   
      
       
if (fl_reg3.add_byte==1) 
    {         
      fl_reg3.add_byte=0;
      if (buf_tx_232[count_tx_ppp]==0x7e){tempo_pac=0x5e;goto send_tempo_pac;}  
      if (buf_tx_232[count_tx_ppp]==0x7d){tempo_pac=0x5d;goto send_tempo_pac;} 
      tempo_pac=buf_tx_232[count_tx_ppp]+0x20;
      goto send_tempo_pac;
    }

                                                         
  count_tx_ppp++;                
   
  if(count_tx_ppp==(vol_tx_ppp-1))goto exit_tx;   
  
       
       if ((buf_tx_232[count_tx_ppp]==0x7e)||(buf_tx_232[count_tx_ppp]==0x7d))
            {
            fl_reg3.add_byte=1;
            tempo_pac=0x7d; 
            
            goto send_tempo_pac;
            } 
        
       if ((buf_tx_232[count_tx_ppp]==17)||(buf_tx_232[count_tx_ppp]==19))
            {
            fl_reg3.add_byte=1;
            tempo_pac=0x7d; 
            
            goto send_tempo_pac;
            }     
        
       if ((buf_tx_232[count_tx_ppp]<0x20)&&(fl_ip.act_ip_end==1)) 
            {
            fl_reg3.add_byte=1;
            tempo_pac=0x7d; 
            
            goto send_tempo_pac;
            }   
       
                  
exit_tx:  //UDR0=buf_tx_232[count_tx_ppp]; 
          tempo_pac=buf_tx_232[count_tx_ppp]; 

send_tempo_pac:
  
  //  UDR0=tempo_pac;  
    
     
    //Посылаем всегда по одному байту
    HAL_UART_Transmit_IT(&huart3,&tempo_pac,1);                 // используем передачу без DMA!!!
    
    
  
} 

 
 
 void recive_buf1(unsigned char temp)
  {
  
 // if(Regim==RG_DEBAG)UDR1=temp;  //!!!!!!!! poka
    
  if(fl_rx_ppp.fl_7e==1)
                  { 
                   
                   
                    if(Buf1_rx_ppp.ln_data>=VOL_RX_PPP)// проверка на переполнение первого буфера
                    {
                       Buf1_rx_ppp.ln_data=0;
                       fl_rx_ppp.fl_7e=0;
                       fl_rx_ppp.fl_7d=0;
                       //S2_OFF;//        
                       
                       //S3_GR;//!!!!proverka
                       return;
                    }
  
                     
                    // //////////укладка
                    if(temp==0x7d)
                       {  
                         fl_rx_ppp.fl_7d=1;
                         return;
                       }
                        else
                       {  
                         if (fl_rx_ppp.fl_7d==1)
                            {                 
                              
                              Buf1_rx_ppp.data[Buf1_rx_ppp.ln_data]=temp ^ 0x20;
                              Buf1_rx_ppp.ln_data++;
                              fl_rx_ppp.fl_7d=0;
                              /*
                              if((temp>=0x20)&&(temp<0x40)){Buf1_rx_ppp.data[Buf1_rx_ppp.ln_data]=temp-0x20;Buf1_rx_ppp.ln_data++;fl_rx_ppp.fl_7d=0;return;}                                
                              if(temp==0x5e){Buf1_rx_ppp.data[Buf1_rx_ppp.ln_data]=0x7e;Buf1_rx_ppp.ln_data++;fl_rx_ppp.fl_7d=0;return;}  
                              if(temp==0x5d){Buf1_rx_ppp.data[Buf1_rx_ppp.ln_data]=0x7d;Buf1_rx_ppp.ln_data++;fl_rx_ppp.fl_7d=0;return;} 
                              Buf1_rx_ppp.ln_data=0;
                              fl_rx_ppp.fl_7e=0;
                              fl_rx_ppp.fl_7d=0;             
                              S3_RD;//!!!!proverka
                              S2_OFF;
                              */
                              
                              return;
                             }  
                            else
                              {        
                                   Buf1_rx_ppp.data[Buf1_rx_ppp.ln_data] =temp;
                                   Buf1_rx_ppp.ln_data++;                                                  
                        
                                     if(temp==0x7e)         //нет переполнения
                                        {  
                                         if(Buf1_rx_ppp.ln_data<=2) //если подряд два 0x7e 
                                             {
                                              //  if(ppp_packet.rcvd==1)???????????????
                                              /*
                                              if((layer_PPP==LAYER_IP)&&(Buf1_rx_ppp.ln_data==2))
                                              {
                                               S4_RD;//proverka!!!!!
                                               while(1)__watchdog_reset();//proverka!!!!!
                                              }
                                              */
                                               
                                              fl_rx_ppp.fl_7e=1;   
                                              Buf1_rx_ppp.data[0]=temp;  
                                              Buf1_rx_ppp.ln_data=1; 
                                              return;
                                              }
                                        
                                                    
                                         //S2_OFF;//
                                         fl_rx_ppp.fl_7e=0;
                                         fl_rx_ppp.switcher=1;
                                         Buf1_rx_ppp.rec=TRUE; 
                                         Buf1_rx_ppp.busy=TRUE;
                                         if(Buf2_rx_ppp.busy==TRUE)
                                          {
                                        
                                           Buf2_rx_ppp.check_busy=TRUE;
                                           //UCSR0B=UCSR0B & ~RXEN;
                                           //UCSR0B=UCSR0B & ~RXCIE; 
                                           
                                            
     // аналог это запретить прерывния от приема
     
      __HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);
      
      
      
                                           
                                          }
                                         return;
                                        }  
                               }
                  
                        }
                                    
                    return;   
                  }         //к пекет бегин
                  else
                  {
                   if(temp==0x7e)
                          {
                        //  if(ppp_packet.rcvd==1)???????????????                          
                          fl_rx_ppp.fl_7e=1;   
                          Buf1_rx_ppp.data[0]=temp;  
                          Buf1_rx_ppp.ln_data=1;
                          //S2_GR;// 
                          }
                  }
  }
 



void recive_buf2(unsigned char temp)
  {
  
 // if(Regim==RG_DEBAG)UDR1=temp;  //!!!!!!!! poka
    
  if(fl_rx_ppp.fl_7e==1)
                  { 
                   
                   
                    if(Buf2_rx_ppp.ln_data>=VOL_RX_PPP)// проверка на переполнение первого буфера
                    {
                       Buf2_rx_ppp.ln_data=0;
                       fl_rx_ppp.fl_7e=0;
                       fl_rx_ppp.fl_7d=0;
                       //S2_OFF;//
                  //     S3_GR;//!!!!proverka
                       return;
                    }
  
                     
                    // //////////укладка
                    if(temp==0x7d)
                       {  
                         fl_rx_ppp.fl_7d=1;
                         return;
                       }
                        else
                       {  
                         if (fl_rx_ppp.fl_7d==1)
                            { 
                              Buf2_rx_ppp.data[Buf2_rx_ppp.ln_data]=temp ^ 0x20;
                              Buf2_rx_ppp.ln_data++;
                              fl_rx_ppp.fl_7d=0;
                              
                              /*                    
                              if((temp>=0x20)&&(temp<0x40)){Buf2_rx_ppp.data[Buf2_rx_ppp.ln_data]=temp-0x20;Buf2_rx_ppp.ln_data++;fl_rx_ppp.fl_7d=0;return;}                                
                              if(temp==0x5e){Buf2_rx_ppp.data[Buf2_rx_ppp.ln_data]=0x7e;Buf2_rx_ppp.ln_data++;fl_rx_ppp.fl_7d=0;return;}  
                              if(temp==0x5d){Buf2_rx_ppp.data[Buf2_rx_ppp.ln_data]=0x7d;Buf2_rx_ppp.ln_data++;fl_rx_ppp.fl_7d=0;return;} 
                              Buf2_rx_ppp.ln_data=0;
                              fl_rx_ppp.fl_7e=0;
                              fl_rx_ppp.fl_7d=0;
                              S3_RD;//!!!!proverka
                              S2_OFF;
                              */
                              return;
                             }  
                            else
                              {        
                                   Buf2_rx_ppp.data[Buf2_rx_ppp.ln_data]=temp;
                                   Buf2_rx_ppp.ln_data++;                                                  
                        
                                     if(temp==0x7e)         //нет переполнения
                                        {  
                                         if(Buf2_rx_ppp.ln_data<=2) //если подряд два 0x7e 
                                             {
                                              /*
                                               if((layer_PPP==LAYER_IP)&&(Buf2_rx_ppp.ln_data==2))
                                              {
                                               S5_RD;//proverka!!!!!
                                               while(1)__watchdog_reset();//proverka!!!!!
                                              }
                                              */
                                               
                                               //  if(ppp_packet.rcvd==1)???????????????
                                              fl_rx_ppp.fl_7e=1;   
                                              Buf2_rx_ppp.data[0]=temp;  
                                              Buf2_rx_ppp.ln_data=1; 
                                              return;
                                              }
  

                                         //S2_OFF;//
                                         fl_rx_ppp.fl_7e=0;
                                         fl_rx_ppp.switcher=0;
                                         Buf2_rx_ppp.rec=TRUE; 
                                         Buf2_rx_ppp.busy=TRUE;
                                         if(Buf1_rx_ppp.busy==TRUE)
                                          {
                                           
                                           Buf1_rx_ppp.check_busy=TRUE;
                                        //   UCSR0B=UCSR0B & ~RXEN;
                                        //   UCSR0B=UCSR0B & ~RXCIE; 
                                           
                                            
     // аналог это запретить прерывния от приема
     
      __HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);
      
      
      
                                           
                                           
                                          }
                                         return;
                                        }  
                               }
                  
                        }
                                    
                    return;   
                  }         //к пекет бегин
                  else
                  {
                   if(temp==0x7e)
                          {
                        //  if(ppp_packet.rcvd==1)???????????????
                          fl_rx_ppp.fl_7e=1;   
                          Buf2_rx_ppp.data[0]=temp;  
                          Buf2_rx_ppp.ln_data=1; 
                          //S2_GR;//
                          }        
                  }
  } 
 //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE






//////////////////////////////////////////////////////////////////////////////////////////////
/*
#pragma vector=TIMER0_COMPA_vect       
     __interrupt void TIMER0_COMPA_interrupt(void)
 {
   
   
   fl_wdt.from_timer0=1;
   
   // __enable_interrupt();
   
 
   
 }
*/
//////////////////////////////////////////////1/16000000///////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////
// эта процедура должна вызываться в SysTick милисекундном прерывании
       
void milisecund_handler_sim800(void)       
  {
    
  //////////////////////////////////////////  interrupt_debug_tu();
  
 //  unsigned char temp;
  
    fl_wdt.from_timer2=1;
    fl_wdt.from_timer0=1;
    
    cnt_clr_wdt++;
  if (cnt_clr_wdt>=250)
     {
     cnt_clr_wdt=0;
     fl_wdt.clr_wdt=1;
     cnt_flag_out++;
     if (cnt_flag_out>=50)
       {
        cnt_flag_out=0;
        fl_wdt.fl_out=1;
       } 
      }
    
   
    if(cnt_sec<DEF_CNT_SEC)cnt_sec++;
  
  /* Пробовал отлавливать управляющие символы в канале
  if( contr_sym_from_sim800==0) goto next_line;
  contr_sym_from_sim800--;
  if( contr_sym_from_sim800==0) {
    
     RS485_DIR(1);
      
      HAL_UART_Transmit_DMA(&huart2,&sim800Data,1);
  }
  next_line:
  */
  
    if(fl_cts_232.on==1)
    {
      if(fl_cts_232_ignor==TRUE)
            {
             fl_cts_232.on=0;
             if(count_tx_ppp==0)  HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);                                             //{UDR0=buf_tx_232[0];}
             
              /*
    Посылаем всегда по одному байту
    HAL_UART_Transmit_IT(&huart9,&buf_tx_232[0],1);
    
    */
             
                else sending_ppp_pac();
        ///     return;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             }
      
     if(HAL_GPIO_ReadPin(GPIOD, CTS0)==0)  // 3 вольта 
     // if (PINE & CTS0)  
      {
        cnt_cts_off=0;
        fl_cts_232.on=0; 
        if(count_tx_ppp==0) HAL_UART_Transmit_IT(&huart3,&buf_tx_232[0],1);                  //{UDR0=buf_tx_232[0];}
         /*
    Посылаем всегда по одному байту
    HAL_UART_Transmit_IT(&huart9,&buf_tx_232[0],1);
    
    */
        
           else sending_ppp_pac(); 
       } 
       else
        {
        cnt_cts_off++;
        if(cnt_cts_off>=10000)
           {
           cnt_cts_off=0;
           event_modem=EVM_CTS_ERR;
           }
        }
    }
  
  
   fun_state_md(); 
 
  
  if(Buf1_rx_ppp.check_busy==TRUE)
     {
      if(Buf1_rx_ppp.busy==FALSE)
      {
      Buf1_rx_ppp.check_busy=FALSE;
    //  temp=UDR0;                                     // очистить буфер на всякий случай
    //  UCSR0B=UCSR0B | RXEN;
    //  UCSR0B=UCSR0B | RXCIE; 
      
      
      //аналог это сброс регистра данных и разрешить прерывния от приема
      __HAL_UART_FLUSH_DRREGISTER(&huart3);
      __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
      
      
      
      
      }    
     }  
  
  if(Buf2_rx_ppp.check_busy==TRUE)
     {
      if(Buf2_rx_ppp.busy==FALSE)
      {
      Buf2_rx_ppp.check_busy=FALSE;
     // temp=UDR0;                                     // очистить буфер на всякий случай
     // UCSR0B=UCSR0B | RXEN;
     // UCSR0B=UCSR0B | RXCIE; 
  
  //    аналог это сброс регистра данных и разрешить прерывния от приема
      __HAL_UART_FLUSH_DRREGISTER(&huart3);
      __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
      
      
           
      
      
      }    
     }   
  
/* cnt_key_off нигде не используется
  
   if (cnt_key_off==0)goto  next_step00;
       cnt_key_off--;
       if (cnt_key_off==0) CLR_PWRK();
  
  next_step00:
*/             
             
     if (At_com.cnt_tm_out==0)goto next_step0; 
   {
   At_com.cnt_tm_out--;
   if(At_com.cnt_tm_out==0)
     {
  //   UCSR0B=UCSR0B & ~RXEN;
  //   UCSR0B=UCSR0B & ~RXCIE; 
     
      /*
      аналог это запретить прерывния от приема
       это какие-то рудименты
       таймауты работают только в режиме АТ команд
       достаточно взвести флаг fl_at_com.tm_out=1; fl_at_com.rx_rec=1;
       эти манипуляции с запрещением прерываний не нужны !
       от них STM сходит с ума
     */
       
     // __HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);
      
      
         
     
     At_com.cnt_rx_out=0;
     fl_at_com.tm_out=1;
     fl_at_com.rx_rec=0;
     fl_at_com.rx_en=0;
     fl_at_com.err=0;
     fl_at_com.ok=0;
     }
   }


   
   
next_step0:  

  if(At_com.cnt_rx_out==0)goto next_step1;
    At_com.cnt_rx_out--;
    if(At_com.cnt_rx_out==0)
    {
  //   UCSR0B=UCSR0B & ~RXEN;
  //   UCSR0B=UCSR0B & ~RXCIE;
     fl_at_com.rx_rec=1; 
     
     
      
    //  аналог это запретить прерывния от приема
     
     // __HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);
      
      
           
     
  
    } //счетчик межбайтовый промежуток
next_step1:
   
     
      
                
  
      ///таймер рестарта LCP
    if(fl_lcp.lcp_tm_out_en==1)
       {                 
            if(cnt_lcp_tm_out==0)goto lable_tm24;  
            cnt_lcp_tm_out--;
            if(cnt_lcp_tm_out==0)fl_lcp.t0_pl=1;
            /*
               cnt_lcp_tm_out--;
               if(cnt_lcp_tm_out==0)
                  {
                  cnt_lcp_tm_out=VOL_LCP_TM_OUT;
                  if(cnt_rst_lcp==0)
                     {   
                      cnt_lcp_tm_out=vol_cnt_rst_lcp;
                      fl_lcp.t0_mi=1;
                     }
                     else 
                       {
                       cnt_rst_lcp--;
                       fl_lcp.t0_pl=1;       
                       } 
                  } 
            */
         
       }
  lable_tm24:;
 //////////////////////////////////////////
 
 ///таймер рестарта PAP
    if(fl_pap.pap_tm_out_en==1)
       {        
               if(cnt_pap_tm_out==0)goto lable_tm25;
               cnt_pap_tm_out--;
               if(cnt_pap_tm_out==0)fl_pap.t0_pl=1; 
         /*      
                 if(cnt_pap_tm_out==0)
                  {
                  cnt_pap_tm_out=VOL_PAP_TM_OUT;
                  if(cnt_rst_pap==0)
                     {   
                      cnt_pap_tm_out=vol_cnt_rst_pap;
                      fl_pap.t0_mi=1;
                     }
                     else 
                       {
                       cnt_rst_pap--;
                       fl_pap.t0_pl=1; 
                       } 
                  } 
       */
                 
       }
  lable_tm25: ;
 //////////////////////////////////////////
 
 ///таймер рестарта IPCP
    if(fl_ipcp.ipcp_tm_out_en==1)
       {                 
               
               if(cnt_ipcp_tm_out==0)goto lable_tm26;
               cnt_ipcp_tm_out--;
               if(cnt_ipcp_tm_out==0)fl_ipcp.t0_pl=1; 
              /* 
               cnt_ipcp_tm_out--;
               if(cnt_ipcp_tm_out==0)
                  {
                  cnt_ipcp_tm_out=VOL_IPCP_TM_OUT;
                  if(cnt_rst_ipcp==0)
                     {   
                      cnt_ipcp_tm_out=vol_cnt_rst_ipcp;
                      fl_ipcp.t0_mi=1;
                     }
                     else 
                       {  
                       cnt_rst_ipcp--;
                       fl_ipcp.t0_pl=1; 
                       } 
                  } */
       
       }
                
  lable_tm26:

             return;





       }
 
///////////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////////////////////



//
// прерывание от таймера заменяем односекундной процедурой
//
 void one_secund_proc(void)      
    { 
 
      
         fl_wdt.from_timer3=1;
         
  
         
  
      
         
         
         
         if(time_cnt_ch_time!=0) time_cnt_ch_time--;
         
         cnt_supervosor++;
      
    /*
    if((PINB & CTS_232)==CTS_232)
                    { 
                    cnt_term++;
                    if(cnt_term>=4)lock_it();
                    }
                    else cnt_term=0;
   */ 
    
    
    //if(count_block_net!=0) count_block_net--;
     
             
         
    
   
      if(Appl_seq.cntr_cl==TRUE)
      {
      if(Appl_seq.cnt_cntr_cl!=0) 
         {
         Appl_seq.cnt_cntr_cl--;
         if(Appl_seq.cnt_cntr_cl==0) Appl_seq.en_cntr_cl=TRUE;
         } 
      }
   

if(cnt_stm_tm1!=0) cnt_stm_tm1--;

 /*  перенес обработку  в case    
      if(cnt_stm_tm1==0)goto next_tm00;  
         cnt_stm_tm1--;
         if(cnt_stm_tm1==0) event_modem=EVM_TM1;
 next_tm00:
 */ 
   
   
 /*   15 10 18 потом надо подумать что с этим счастьем делать*/
   if(cnt_stm_tm2==0)goto next_tm001;  
         cnt_stm_tm2--;
         if(cnt_stm_tm2==0) lock_it();
         

   
next_tm001:   
   
   
    
    if(fl_rewrite.cnt_reset==0)goto next_tm01; 
    fl_rewrite.cnt_reset--;
    if(fl_rewrite.cnt_reset==0)
       {
        /*   ничего не писать только взвести флаг чтобыл сброс
         __disable_interrupt();
         
        

        *(uint32_t*)&buf_tx_232[0]=burst_ds_r();
        *(uint32_t*)&buf_tx_232[4]=cnt_outcom;
        *(uint32_t*)&buf_tx_232[8]=cnt_incom;
        *(uint16_t*)&buf_tx_232[L_TRAF-2]=crc_m1(&buf_tx_232[0],L_TRAF-2,0xffff);
        framWriteSim800(A_TRAF, &buf_tx_232[0],L_TRAF); 
         */
        lock_it();   
       }       
next_tm01: 
  
  
      
   ///таймер контроля охранки убирается этих процессов нет вообще в природе
  /*
    if (Appl_seq.tm_vzat==0)goto next_tm0;
    Appl_seq.tm_vzat--;
    if (Appl_seq.tm_vzat==0)Appl_seq.event=EV_TM_OUT;
next_tm0:   
    */
    
    
        if(Appl_seq.cnt_waits==0)goto next_tm1;
        Appl_seq.cnt_waits--;
        if(Appl_seq.cnt_waits==0)
               { 
                  
                   //   Appl_seq.cnt_try++;
                      fl_appl_seq.kv_waits=0;
                      fl_appl_seq.en_povtor=1; 
                      Appl_seq.l_data=0;
                      
                      /*
                      if(Appl_seq.cnt_try>=2) 
                       {
                       Appl_seq.cnt_cntr_cl=0;
                       Appl_seq.cntr_cl=TRUE;
                       Appl_seq.en_cntr_cl=TRUE;
                       Appl_seq.cnt_try=0;
                       fl_appl_seq.en_povtor=0;                      
                       }
                       */
                       
                                      
               }    
 next_tm1:   
    
    if(fl_ip.ip_tm_cntr_en==1)
       {       
               
               if(Control.cnt_link==0) goto  next_tm2;
               
               Control.cnt_link--;
               if ( ip_change==1) { ip_change=0; Control.cnt_link=0;}   // если поменялся IP тут же послать контроль канала !!!
               if(Control.cnt_link==0)
               { 
               
               fl_appl_seq1.enable=1;
               
               //Control.cnt_link=Control.vol_link;
               Control.link=TRUE; 
               if(Control.link_waits==TRUE)
                  {
                 // if(Control.vol_try!=0)
                  //   {
                      Control.cnt_try++;
                      if(Control.cnt_try>=Control.vol_try)
                       {   
                       Control.link=FALSE;   
                       event_modem=EVM_MS_LMT;
                       Control.link_no=TRUE;//признак связи нет.
                       }
                    // }
                  }
                              
                Control.link_waits=FALSE;
                             
               }    
       
       }
       
 next_tm2:  
   
       
       if(fl_ip.ip_tm_nat_en==1)   ///таймер контроля NAT
       {       
               Control.cnt_nat--;
               if(Control.cnt_nat==0)
               { 
                 Control.cnt_nat=Control.vol_nat;
              //  if(simka==SIM_RES)Control.cnt_nat=Control.vol_nat_r;else Control.cnt_nat=Control.vol_nat;
                Control.nat=TRUE;
               }    
       
       }
  
   //!!!!!!!!!!!!!dobavka
      
          
 //EEEEEEEEEEEEEE dobavka
 
    if (Life.time_485_1==0)goto next_tm5;
        Life.time_485_1--;
    if (Life.time_485_1==0)Life.start_485_1=0;    
    next_tm5:
      
    if (Life.time_485_2==0)goto next_tm6;
        Life.time_485_2--;
    if (Life.time_485_2==0)Life.start_485_2=0;    
       
    next_tm6:
      
    if (Life.time_232_2==0)goto next_tm7;
        Life.time_232_2--;
    if (Life.time_232_2==0)Life.start_232_2=0;
    next_tm7:;
 ////////////////////////////////////////// 
     
    }


/*
  
  меговская процедура обработчика
#pragma vector=USART0_RX_vect
           
           __interrupt  void USART0_RX_interrupt(void)
                 
                 
                {
                unsigned char data;                
                data=UDR0;
                          
                
                // в терминальном режиме заполняем массив
                if (fl_at_mom_232==1) {
                                      
                                       Appl_seq_buf[point_Head]=data; 
                                       point_Head++;
                                       point_Head=point_Head & 0x3f;
                                       return;
                }
                
                
                
                
                //
              
                if(command_AT==TRUE)
                    {
                    if(fl_at_com.rx_en==0)return;  //dobavka 08.11.2007
                    
                    //S2_GR;//
                    At_com.cnt_rx_out=At_com.vol_rx_out;
                    At_com.cnt_tm_out=0;
                    if(At_com.cnt_rx < LN_BUF_AT)
                          {
                           At_com.buf[At_com.cnt_rx]=data;
                           At_com.cnt_rx++;
                           }
                    return;
                    }
               
                  cnt_incom++;
                  if(fl_rx_ppp.switcher==0)recive_buf1(data);else recive_buf2(data);
  
                 }
 /////////////////////////////////////////////////////////////////////////////////////////////////////  
*/
  
  
/*

AXTUNG !!!
HAL_UART_Receiv_IT(&huart3,&sim800Data,1) строка должна быть при инициализации в main 
вызов
// для STM   в процедуре HAL_UART__RxCpltCallBack(UART_HandlTypeDef *huart)

if(huart->Instance==UART6)
{
//принятые даные лежат в переменной  sim800Data
  RxSim800Handler();

*/
  void RxSim800Handler(void)
  {
 if(command_AT==TRUE)
                    {
                    if(fl_at_com.rx_en==0) goto razr;  //dobavka 08.11.2007
                    
                    //S2_GR;//
                    At_com.cnt_rx_out=At_com.vol_rx_out;
                    At_com.cnt_tm_out=0;
                    if(At_com.cnt_rx < LN_BUF_AT)
                          {
                           At_com.buf[At_com.cnt_rx]=sim800Data;
                           At_com.cnt_rx++;
                           }
                    goto razr;

                    }
               
                  cnt_incom++;
                  contr_sym_from_sim800=50;
                  if(fl_rx_ppp.switcher==0)recive_buf1(sim800Data);else recive_buf2(sim800Data);

razr:
                 HAL_UART_Receive_IT(&huart3,&sim800Data,1);


}










 //////////////////////////////////////////////////////////////////////////////////////////////////////////

//#pragma vector=USART0_TX_vect
//     __interrupt void USART0_TX_interrupt(void) 

/*
// для STM   в процедуре HAL_UART__TxCpltCallBack(UART_HandlTypeDef *huart)

if(huart->Instance==UART9)
TxSim800Handler();
{
*/
       void TxSim800Handler(void)

{ 
  
  if(command_AT==TRUE)sending_at_pac(); 
    
  else 
        {
        
        if(fl_cts_232_ignor==TRUE){sending_ppp_pac();return;}
        
        cnt_outcom++;
        
        if(check_cts()==1) return;
        sending_ppp_pac();
        } 
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////

/*
// для STM   в процедуре HAL_UART__TxCpltCallBack(UART_HandlTypeDef *huart)

if(huart->Instance==UART9)
{
 if(command_AT==TRUE)sending_at_pac(); 
    
  else 
        {
        
        if(fl_cts_232_ignor==TRUE){sending_ppp_pac();return;}
        
        cnt_outcom++;
        
        if(check_cts()==1) return;
        sending_ppp_pac();
        } 

}

*/




/*
/////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma vector=USART1_RX_vect
           
           __interrupt  void USART1_RX_interrupt(void)
                 
                 
                {
                unsigned char data;
                
                data=UDR1;
                S4_GR;
                Rs485_2.cnt_tm_rx_out=Rs485_2.vol_tm_rx_out;
                Rs485_2.cnt_tm_out=0;
                if(Rs485_2.cnt_bt_rx_tx < MAX_BUF_RS485_2)
                       {
                        Rs485_2_buf_rx_tx[Rs485_2.cnt_bt_rx_tx]=data;
                        Rs485_2.cnt_bt_rx_tx++;
                        }
                        else fl_485_2.over=1;
                }
 /////////////////////////////////////////////////////////////////////////////////////////////////////
 



/////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma vector=USART1_TX_vect
           
           __interrupt  void USART1_TX_interrupt(void)

{

 if(Rs485_2.cnt_bt_rx_tx==0) goto end_tx1;
                                                       
                UDR1=*Rs485_2.p_data485++;
                Rs485_2.cnt_bt_rx_tx--; 
                return;
end_tx1:
               UCSR1B=UCSR1B & ~TXCIE;
               Rs485_2.cnt_bt_rx_tx=0;
               Rs485_2.cnt_tm_tx_out=Rs485_2.vol_tm_tx_out;
               if(Rs485_2.cnt_tm_tx_out==0)
               {
                S4_OFF;
                Rs485_2.cnt_tm_out=Rs485_2.vol_tm_out;
                CLR_RTS3;  // togle to receiv mode  
                UCSR1B =UCSR1B | RXEN | RXCIE; // togle to receiv mode ;
               }
             

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

*/




/*
/////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma vector=USART3_RX_vect
           
           __interrupt  void USART3_RX_interrupt(void)
                 
                 
                {
                unsigned char data;
                
                data=UDR3;
                S3_GR;
                Rs485_1.cnt_tm_rx_out=Rs485_1.vol_tm_rx_out;
                Rs485_1.cnt_tm_out=0;
                if(Rs485_1.cnt_bt_rx_tx < MAX_BUF_RS485_1)
                       {
                        Rs485_1_buf_rx_tx[Rs485_1.cnt_bt_rx_tx]=data;
                        Rs485_1.cnt_bt_rx_tx++;
                        }
                        else fl_485_1.over=1;
                }
 /////////////////////////////////////////////////////////////////////////////////////////////////////
 



/////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma vector=USART3_TX_vect
           
           __interrupt  void USART3_TX_interrupt(void)

{

 if(Rs485_1.cnt_bt_rx_tx==0) goto end_tx2;
                                                       
                UDR3=*Rs485_1.p_data485++;
                Rs485_1.cnt_bt_rx_tx--; 
                return;
end_tx2:
               UCSR3B=UCSR3B & ~TXCIE;
               Rs485_1.cnt_bt_rx_tx=0;
               Rs485_1.cnt_tm_tx_out=Rs485_1.vol_tm_tx_out;
               if(Rs485_1.cnt_tm_tx_out==0)
               {
                S3_OFF;
                Rs485_1.cnt_tm_out=Rs485_1.vol_tm_out;
                CLR_RTS1;  // togle to receiv mode  
                UCSR3B =UCSR3B | RXEN | RXCIE; // togle to receiv mode ;
               }
              

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

*/

  














   


