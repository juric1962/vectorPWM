#include "def_log.h"
#include "map_ef.h"
#include "def_link.h"
#include "ozu_map.h"
#include "stdint.h"

extern uint32_t burst_ds_r(void);
extern void framWriteSim800(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num);   
extern void framRead(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);
extern unsigned char buf_tx_232[VOL_TX_PPP];

unsigned char point_log_buf;

 void write_log_info(unsigned char sost,unsigned char mesto)
{

unsigned char buf[6];

*(uint32_t*)&buf[0]=burst_ds_r();             
buf[4]=sost;
buf[5]=mesto;
framWriteSim800(BEG_BUF_LOG+point_log_buf*6,&buf[0],6);

point_log_buf++;
if(point_log_buf>=L_LOG)point_log_buf=0;
}


void serch_point_log (void)
{

uint16_t i;
union
     {
      unsigned char bytes[4]; 
      uint32_t word;
     }temp_long,temp_long1;     


framRead(BEG_BUF_LOG,&buf_tx_232[0],(L_LOG*6));

temp_long.bytes[0]=buf_tx_232[0];
temp_long.bytes[1]=buf_tx_232[1];
temp_long.bytes[2]=buf_tx_232[2];
temp_long.bytes[3]=buf_tx_232[3];
if(temp_long.word==0)
   {
   point_log_buf=0;
   return;
   }
   
for(i=0;i<(L_LOG-1);i++)
  {
  temp_long.bytes[0]=buf_tx_232[i*6];
  temp_long.bytes[1]=buf_tx_232[(i*6)+1];
  temp_long.bytes[2]=buf_tx_232[(i*6)+2];
  temp_long.bytes[3]=buf_tx_232[(i*6)+3];
  
  temp_long1.bytes[0]=buf_tx_232[(i*6)+6];
  temp_long1.bytes[1]=buf_tx_232[(i*6)+6+1];
  temp_long1.bytes[2]=buf_tx_232[(i*6)+6+2];
  temp_long1.bytes[3]=buf_tx_232[(i*6)+6+3];
  
  if(temp_long1.word<temp_long.word) 
     {
     point_log_buf=i+1; 
     if(point_log_buf>=L_LOG)point_log_buf=0; 
     return;
     }
  }
  
 point_log_buf=0; 
}  



