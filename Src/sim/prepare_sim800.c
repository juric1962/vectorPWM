//#include <iom1280.h>              
//#include <inavr.h>
#include <stdlib.h> 
#include <string.h>
//#include "dfpin.h"
#include "dfcnst.h"
#include "ozu_map.h"
#include "map_ef.h"
#include "stdint.h"
#include "sim800.h"
#include "fram.h"


#define BALDA 1

SIM800_SETTINGS_STRUCT sim800Settings;

void copymasNSwap(uint8_t *dst, uint8_t *src, uint16_t bytesNum);


void sim800StructInit(void) // Инициализация структуры sim800 для работы с модбас
{
  uint8_t temp[4];  
  framRead(A_IP_PAR,temp,4);  
  sim800Settings.ip0=temp[0];
  sim800Settings.ip1=temp[1];
  sim800Settings.ip2=temp[2];
  sim800Settings.ip3=temp[3];  
  framRead(A_PORT,(uint8_t*)&sim800Settings.port,2); 
  framRead(A_NUM,(uint8_t*)&sim800Settings.num,2);
  sim800Settings.num_contr=0;    // читается всегда нулем
  framRead(A_NAT,(uint8_t*)&sim800Settings.nat,2);   
  framRead(A_CCH,(uint8_t*)&sim800Settings.cch,2); 
  framRead(A_TM_CH,(uint8_t*)&sim800Settings.tm_ch,2); 
  
   framRead(A_TM_CL,(uint8_t*)&sim800Settings.tm_client_asinc,2); 
   framRead(A_NUM_CL,(uint8_t*)&sim800Settings.num_client_asinc,2); 
   framRead(A_DES_SEQ,(uint8_t*)&sim800Settings.des_asinc,1);
   framRead(A_IND_DNS_KVIT,(uint8_t*)&sim800Settings.ind_kvit_asinhr,2); 
    if ( sim800Settings.ind_kvit_asinhr> TAPS_ARCHIVE_RECORDS_NUM) sim800Settings.ind_kvit_asinhr=0;
   
    //
    // надо будет выдавать архивы по этому индексу
   
   
  framRead(A_C_GPRS,(uint8_t*)sim800Settings.a_c_gprs,58); 

  
  copymasNSwap((uint8_t*)sim800Settings.a_c_gprs,(uint8_t*)sim800Settings.a_c_gprs,58);
}

void sim800StructToFlash(void) // Инициализация структуры sim800 для работы с модбас
{
  uint8_t temp[4];  
  temp[0]=sim800Settings.ip0;
  temp[1]=sim800Settings.ip1;
  temp[2]=sim800Settings.ip2;
  temp[3]=sim800Settings.ip3;  
  copymasNSwap((uint8_t*)sim800Settings.a_c_gprs,(uint8_t*)sim800Settings.a_c_gprs,58);  
  framWrite(A_IP_PAR,temp,4);  
  framWrite(A_PORT,(uint8_t*)&sim800Settings.port,2);
  
  if ( sim800Settings.num_contr==0x55aa)  framWrite(A_NUM,(uint8_t*)&sim800Settings.num,2); 
  sim800Settings.num_contr=0;    // читается всегда нулем
  
  framWrite(A_NAT,(uint8_t*)&sim800Settings.nat,2);   
  framWrite(A_CCH,(uint8_t*)&sim800Settings.cch,2); 
  framWrite(A_TM_CH,(uint8_t*)&sim800Settings.tm_ch,2);
  
   framWrite(A_TM_CL,(uint8_t*)&sim800Settings.tm_client_asinc,2); 
   framWrite(A_NUM_CL,(uint8_t*)&sim800Settings.num_client_asinc,2); 
   framWrite(A_DES_SEQ,(uint8_t*)&sim800Settings.des_asinc,1); 
   framWrite(A_IND_DNS_KVIT,(uint8_t*)&sim800Settings.ind_kvit_asinhr,2); 
  
  framWrite(A_C_GPRS,(uint8_t*)sim800Settings.a_c_gprs,58);   
}





void CLR_PWR(void);
void SET_PWRK(void);
void CLR_RTS0(void);
void CLR_DTR0(void);

extern unsigned char ResetTaskSim800;

extern unsigned char A_C_GPRS_array[L_C_GPRS_MAX];
extern unsigned char A_CR_GPRS_array[L_CR_GPRS_MAX];
extern unsigned char A_IP_PAR_array[L_IP_PAR];
extern unsigned char A_C_PAR_array[L_C_PAR];
extern unsigned char A_SEQ_PAR_array[L_SEQ_PAR];
extern unsigned char A_KOD_SIM1_array[L_KOD_SIM1];
extern unsigned char A_KOD_SIM2_array[L_KOD_SIM2];
extern unsigned char A_TRAF_array[L_TRAF];
extern unsigned char A_KEYS_array[L_KEYS];
extern unsigned char BEG_BUF_LOG_array[L_LOG*6];




extern void s_port(unsigned char ch);
//extern void framRead(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);
unsigned char check_keys(void);

void begin_ts(void);
void load_par_first_flash(void);

void load_sel_modul(void);
void init_pins_mkd_mod(void);
void init_pins_ts_mod(void);

void begin_ts_mb(void);
void start_a(void);
void check_sim(void);

void init_pins(void);
void init_proc_state(void);
void delay(uint16_t period);
void long_delay(uint32_t period);
void delay_on_contr(void);
void init_proc_state(void);
void init_modem_only(void);
void start_time(void);
void WrArrayToFlesh(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num);
void tx_rs232_time(void);
void load_par_first(void);
void serch_point_log(void);
uint16_t crc_ozu(void);
void monitor_rst(void);

extern unsigned char Regim;
extern unsigned char Rs232_2_buf_rx_tx[MAX_BUF_RS232_2];
extern uint16_t prov_ozu;

unsigned char check_memory_map(void);
void vosstan_memory(void);
void vosstan_memory_no(void);
void monitor_terminal(void); // проверка терминала 
void load_par_from_memory(void);
void load_par_from_memory_no(void);
void load_rw_pdp(void);
void load_rw_pdp_r(void);
void monitor_beg_state_seq(unsigned char zad);
void modem_engine(void);
void init_ports_debug(void);
void init_modem_call(void);








// вызывается в секции инициализации STM
//
//  в вечном цикле вызывается только modem_engine()
//
//

void PreparaeSim800Work(void)
 {
 
 /*
    CLR_PWR();
    SET_PWRK();  // исходное состояние питание выключено RTS DTR (1) пассив
    CLR_RTS0();
    CLR_DTR0(); 
 */
   
 // только при первом включении надо по умолчанию восстановить параметры !!!!!!!!!
               vosstan_memory_no();
               
               
               // check_keys(); 
                serch_point_log();
                load_par_from_memory_no();
               
                prov_ozu=crc_ozu();
  



// заполнить структуры для работы задачи modem_engine()
// 
                
framRead( A_C_GPRS,&A_C_GPRS_array[0],L_C_GPRS_MAX);

framRead( A_CR_GPRS,&A_CR_GPRS_array[0],L_CR_GPRS_MAX);

framRead(A_IP_PAR,&A_IP_PAR_array[0],L_IP_PAR);

framRead(A_C_PAR,&A_C_PAR_array[0],L_C_PAR);

framRead(A_SEQ_PAR,&A_SEQ_PAR_array[0],L_SEQ_PAR);

framRead(A_KOD_SIM1,&A_KOD_SIM1_array[0],L_KOD_SIM1);

framRead(A_KOD_SIM2,&A_KOD_SIM2_array[0],L_KOD_SIM2);

framRead(A_TRAF,&A_TRAF_array[0],L_TRAF);

framRead(A_KEYS,&A_KEYS_array[0],L_KEYS);

framRead(BEG_BUF_LOG,&BEG_BUF_LOG_array[0],(L_LOG*6));





//modem_engine();
  
}
