
//#include <iom1280.h> 
//#include "dfpin.h"
#include "dfcnst.h"
#include "stdint.h"
//#include "my_dinogram.h"

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
//void CLR_DEBUG_TU4(void);
//void SET_DEBUG_TU4(void);

unsigned char state_led_md; 
uint16_t count_led;

extern unsigned char simka; //dobavka

//DEBUG_DP debug_tu;

//
//debug_tu.period_ck
//debug_tu.impuls_dp должны автодекрементироваться в sys-tic прерывании

/*
void interrupt_debug_tu(void)
{
 if (debug_tu.period_ck !=0) debug_tu.period_ck--; 
 if (debug_tu.impuls_dp !=0) debug_tu.impuls_dp--; 
}


void fun_debug_tu(void)
{
 
  switch(debug_tu.state_debug_tu)
  {
  case 0: {debug_tu.period_ck=10000; debug_tu.state_debug_tu=1; CLR_DEBUG_TU4(); break;}
  case 1:  if(debug_tu.period_ck !=0 ) break;
           debug_tu.impuls_dp=100;
           SET_DEBUG_TU4();
           debug_tu.state_debug_tu=2;
           break;
           
  case 2: if(debug_tu.impuls_dp !=0 ) break;
           
           CLR_DEBUG_TU4();
           debug_tu.state_debug_tu=0;
           break;
    
  default : { debug_tu.state_debug_tu=0;break;}
     
  }
 
}

*/


void fun_state_md(void)
{
//  fun_debug_tu();
  
  count_led++;
  switch(state_led_md)
  {
  case LED_MD_OFF: S1_OFF();  break;
  case LED_MD_NO_CREG:  if(count_led<500){if(simka==SIM_BASE){S1_GR();}else {S1_RD();}}else {S1_YL();}break; //dobavka
  case LED_MD_YES_CREG: if(count_led<100){if(simka==SIM_BASE){S1_GR();}else {S1_RD();}}else {S1_OFF();}break;//dobavka
  case LED_MD_GPRS:if(count_led<500){if(simka==SIM_BASE){S1_GR();}else {S1_RD();}}else {S1_OFF();}break;//dobavka
  case LED_MD_PPP: if(simka==SIM_BASE){S1_GR();}else {S1_RD();};break;                                //dobavka
     
  }
  if(count_led>=1000)count_led=0;
}



