#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_def.h"
#include "sram_rtc.h"
//#include <iom1280.h> 
//#include <inavr.h>
//#include "dfpin.h"
#include "ozu_map.h"
#include "map_ef.h"
#include "dfproc.h"



unsigned char A_C_GPRS_array[L_C_GPRS_MAX];
unsigned char A_CR_GPRS_array[L_CR_GPRS_MAX];
unsigned char A_IP_PAR_array[L_IP_PAR];
unsigned char A_C_PAR_array[L_C_PAR];
unsigned char A_SEQ_PAR_array[L_SEQ_PAR];
unsigned char A_KOD_SIM1_array[L_KOD_SIM1];
unsigned char A_KOD_SIM2_array[L_KOD_SIM2];
unsigned char A_TRAF_array[L_TRAF];
unsigned char A_KEYS_array[L_KEYS];
unsigned char BEG_BUF_LOG_array[L_LOG*6];

extern uint32_t unix;
extern unsigned char Rs232_2_buf_rx_tx[MAX_BUF_RS232_2];
extern void delay(uint16_t period);
void    mov_buf (char size,unsigned char * p);
void s_port(unsigned char ch);
//extern RTC_TimeStructure;
//extern RTC_DateStructure;

// unsigned char *ff_r,*src_r;

extern RTC_TimeTypeDef  RTC_TimeStructure;
extern RTC_DateTypeDef RTC_DateStructure; 

uint32_t toUnix(RTC_DateTypeDef RTC_DateStructure, RTC_TimeTypeDef RTC_TimeStructure);

struct
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


 unsigned char *ff_w,*fff_w;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 /*
char daa_to_hex(char b)
{
return( (b & 0xf) + ((b>>4)*10) );
}

char hex_to_daa(char b)
{
return( ((b/10)<<4) + (b%10) );
}   
 
*/

extern void framWrite(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num); 
  
  uint32_t burst_ds_r(void)
  {                                                   
   
   // при работе с STM заполняем свою структуру из системного времени STM 
   // пользуемся его функциями
   
   real_time.r_sec=RTC_TimeStructure.Seconds;
   real_time.r_min=RTC_TimeStructure.Minutes;
   real_time.r_hor=RTC_TimeStructure.Hours;
   
   real_time.r_date=RTC_DateStructure.Date;
   real_time.r_month=RTC_DateStructure.Month;
   real_time.r_year=RTC_DateStructure.Year;
   
   unix=toUnix(RTC_DateStructure,RTC_TimeStructure);
   
   return(unix);
   
   
  }
  
  



 
 


  
//
// копирование массива конфигурации в структуру
//  
void CopyMass(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num)   
{
  uint16_t i;
  uint16_t glu;
   unsigned char *src_r;
 
 // контсрукция математические выражения c вычитанием (adres_flesh - A_C_GPRS)  НЕ РАБОТАЕТ !!!!!
  
 
  
   if( (adres_flesh >= A_C_GPRS)   && (adres_flesh < (A_C_GPRS + L_C_GPRS_MAX )) ) {
    glu=A_C_GPRS;
    src_r=&A_C_GPRS_array[adres_flesh - glu];
    
    
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
                 }
  //  
  
  else if( (adres_flesh >= A_CR_GPRS)   && (adres_flesh < (A_CR_GPRS + L_CR_GPRS_MAX )) ) {
    glu=A_CR_GPRS;
    src_r=&A_CR_GPRS_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;           
                 } 
  
  
  else if( (adres_flesh >= A_IP_PAR)   && (adres_flesh < (A_IP_PAR + L_IP_PAR )) ) {
    glu=A_IP_PAR;
    src_r=&A_IP_PAR_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
    
                                 //   UCSR2C=0x06;
                                 //   UBRR2H=R9600_H;
                                 //   UBRR2L=R9600_L;
                                   // s_port(adres_flesh - A_IP_PAR);
                                  //  mov_buf(L_IP_PAR,&A_IP_PAR_array[0]); // для контроля
                                   
    
               
                 }    
  
  
  //  
    else  if( (adres_flesh >= A_C_PAR)   && (adres_flesh < (A_C_PAR + L_C_PAR )) ) {
      glu=A_C_PAR;
    src_r=&A_C_PAR_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
               
                 }  
  
    //  
     else if( (adres_flesh >= A_SEQ_PAR)   && (adres_flesh < (A_SEQ_PAR + L_SEQ_PAR )) ) {
       glu=A_SEQ_PAR;
    src_r=&A_SEQ_PAR_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
                
                 }  
  
    //  
     else if( (adres_flesh >= A_KOD_SIM1)   && (adres_flesh < (A_KOD_SIM1 + L_KOD_SIM1 )) ) {
       glu=A_KOD_SIM1;
    src_r=&A_KOD_SIM1_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
               
                 }
  
     //  
    else  if( (adres_flesh >= A_KOD_SIM2)   && (adres_flesh < (A_KOD_SIM2 + L_KOD_SIM2 )) ) {
      glu=A_KOD_SIM2;
      src_r=&A_KOD_SIM2_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
                
                 }
  
     //  
   else    if( (adres_flesh >= A_TRAF)   && (adres_flesh < (A_TRAF + L_TRAF )) ) {
       glu=A_TRAF;
       src_r=&A_TRAF_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
                
                 }
 
     //  
   else    if( (adres_flesh >= A_KEYS)   && (adres_flesh < (A_KEYS + L_KEYS )) ) {
       glu=A_KEYS;
       src_r=&A_KEYS_array[adres_flesh - glu];
    for (i=0;i<num;i++) *adres_ozu++=*src_r++;
               
                 }
  
    else  if( (adres_flesh >= BEG_BUF_LOG)   && (adres_flesh < (BEG_BUF_LOG + (L_LOG*6) )) ) {
       glu=BEG_BUF_LOG;
       src_r=&BEG_BUF_LOG_array[adres_flesh - glu];
        for (i=0;i<num;i++) *adres_ozu++=*src_r++;
                
                 }
  
    
  
}  
  
  
 
 
 
 
 
  
  // пишем во влеш и дублируем в структуру
  //
   // после записи во флеш нужно вызвать эту процедуру 
  void framWriteSim800(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num)  //запись конфигурации кп во флеш
  {
    uint16_t i;
    
    
   fff_w=adres_ozu; // запомнить
    
    
    
    switch(adres_flesh){
      
  case A_C_GPRS: {
    ff_w=&A_C_GPRS_array[0];
    for (i=0;i<num;i++) *ff_w++=*adres_ozu++;
                                               
                 break;
                 }
    
  case A_CR_GPRS: {
     ff_w=&A_CR_GPRS_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                 break;
                 }  
     
     
     //
      case A_IP_PAR: {
     ff_w=&A_IP_PAR_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                              
    
                 break;
                 }  
     
     
      //
      case A_C_PAR: {
     ff_w=&A_C_PAR_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                 break;
                 }  
     
      case A_SEQ_PAR: {
     ff_w=&A_SEQ_PAR_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                 break;
                 }  
     
      case A_KOD_SIM1: {
     ff_w=&A_KOD_SIM1_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                 break;
                 }  
     
      case A_KOD_SIM2: {
     ff_w=&A_KOD_SIM2_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                 break;
                 }  
     
      case A_TRAF: {
     ff_w=&A_TRAF_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                 break;
                 }  
    
     
      case A_KEYS: {
     ff_w=&A_KEYS_array[0];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                 break;
                 }
     
    
     
    default: break;
  }
  
  
   if( (adres_flesh >= BEG_BUF_LOG)   && (adres_flesh < (BEG_BUF_LOG + (L_LOG*6))) ) {
     ff_w=&BEG_BUF_LOG_array[adres_flesh - BEG_BUF_LOG];
    for (i=0;i<num;i++)*ff_w++=*adres_ozu++;
                
                 }
    
    framWrite(adres_flesh,  fff_w, num) ;
  
  
                                            
   }
  
  
  
//EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
 
 
 
 
 

