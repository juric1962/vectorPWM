#include "stm32f4xx_hal.h"
//#include <iom1280.h>              
//#include <inavr.h>
#include <string.h> 
//#include "dfpin.h"
#include "dfproc.h"
#include "def_at.h"
#include "map_ef.h"
#include "dfcnst.h"

 extern unsigned char state_led_md;
//char HAL_GPIO_ReadPin( unsigned char port, char GPIO_Pin);
//23.03.07 включение PAP для сименса

//void st_gprs_atcom(void);
//void st_gprs_att(void);

void init_pins_out_to_hiz(void);
void reset_uart_prep_sim800(void);
extern UART_HandleTypeDef huart3;
void lock_it(void);


const char stm_pin1_ok[]=        {'S','T','M',':','p','i','n',' ','s','i','m','1',' ','o','k'};
const char stm_pin2_ok[]=        {'S','T','M',':','p','i','n',' ','s','i','m','2',' ','o','k'};
const char stm_pin1_err[]=       {'S','T','M',':','p','i','n',' ','s','i','m','1',' ','e','r','r'};
const char stm_pin2_err[]=       {'S','T','M',':','p','i','n',' ','s','i','m','2',' ','e','r','r'};

void send_info(char size,char const *p,unsigned char fl_id,unsigned char id);
void CopyMass(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);

extern void framRead(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);
extern void delay(uint16_t period);
unsigned char cnt_com; 
unsigned char rep;

unsigned char speed_modem;
void sending_at_pac(void); 

 void init_modem_only(void);

extern enum t_event_modem event_modem;

extern unsigned char simka; //dobavka

struct
{
unsigned char eho:1;//состояние эхо ответа
unsigned char kod :1;//состояние ответ-код 
}
fl_conf_modem;

char emei[40];

struct
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


struct
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



unsigned char Cnt_modem_on_off,flag;

extern unsigned char Regim;




 void sending_at_pac(void)
{   
 if(At_com.cnt_tx>=(At_com.ln_buf-1))
     { 
     At_com.cnt_tx=0;
     At_com.cnt_tm_out=At_com.vol_tm_out;     // в этом режиме после посылки последнего байта надо обязательно дождаться ответа !!!
    // UCSR0B=UCSR0B | RXEN;
    // UCSR0B=UCSR0B | RXCIE;  
    
     
     
     
     __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
     
     return;
     }   
    

    At_com.cnt_tx++;                   
   // UDR0=At_com.buf[At_com.cnt_tx];
    
    
    //Посылаем всегда по одному байту
    HAL_UART_Transmit_IT(&huart3,(unsigned char *)&At_com.buf[At_com.cnt_tx],1);
    
    
    
  

    
   
} 







void at_com_tx(unsigned char cnt)
{
char *p;

if(cnt>=VOL_LIST)return;
if(fl_at_com.tx_en==0)return;

memset(At_com.buf,0,LN_BUF_AT);

switch(At_com.list_com[cnt])
    {
      case cATCIMI: 
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
          // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "AT+CIMI");             // информация о SIM карте
           break;   
      
     case cAT: 
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
          // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "AT");
           break;  
     case cATE0:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
          // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "ate0");
           break; 
     case cATV0:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "atv0");
           break;  
     case cATE0V0:
           fl_conf_modem.kod=1;
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "ATE0V0");
           break;  
     case cATIPR:
             switch(speed_modem)
                   {
                    case S9600:strcpy(At_com.buf, "at+ipr=9600");break;
                    case S19200:strcpy(At_com.buf, "at+ipr=19200");break;
                    case S57600:strcpy(At_com.buf, "at+ipr=57600");break;
                    case S115200:strcpy(At_com.buf, "at+ipr=115200");break;
                    default:strcpy(At_com.buf, "at+ipr=115200");break;
                   }
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ        
           // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           break; 
     case cATIFC:
          At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
          // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
          strcpy(At_com.buf, "at+ifc=2,2");
          break;
     case cATC:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "at&c");     
           break;
     case cATD:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=3000;
           strcpy(At_com.buf, "at&d");    
           break;
     case cATW:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "at&w"); 
           break; 
     case cATCREG:
           At_com.vol_rx_out=1000;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=1200;
           strcpy(At_com.buf, "at+creg?");
           break; 
     case cATCGATT:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=5000;
           strcpy(At_com.buf, "at+cgatt=1");
           break;    
     case cATCGDCONT:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
 //          At_com.vol_tm_out=200;
           At_com.vol_tm_out=250;
           strcpy(At_com.buf, "at+cgdcont=1,");
              
             
    
           if(simka==SIM_BASE) CopyMass(A_C_GPRS,(unsigned char *)&At_com.buf[19],LN_BUF_AT-19); //dobavka
             else CopyMass(A_CR_GPRS,(unsigned char *)&At_com.buf[19],LN_BUF_AT-19);              //dobavka
           
          p=&At_com.buf[19];
          while(*p)p++;
          *p++='"';
          *p=0;
             
             
              At_com.buf[13]='"';
              At_com.buf[14]='I';
              At_com.buf[15]='P';
              At_com.buf[16]='"';
              At_com.buf[17]=',';
              At_com.buf[18]='"';
             
            //  At_com.buf[18+(unsigned char)At_com.buf[18]+1]='"';
            //  At_com.buf[18+(unsigned char)At_com.buf[18]+2]=0;
          
           /*      
           At_com.buf[18]='"';
           At_com.buf[19]='i';
           At_com.buf[20]='n';
           At_com.buf[21]='t';
           At_com.buf[22]='e';
           At_com.buf[23]='r';
           At_com.buf[24]='n';
           At_com.buf[25]='e';
           At_com.buf[26]='t';
           At_com.buf[27]='.';
           At_com.buf[28]='m';
           At_com.buf[29]='t';
           At_com.buf[30]='s';
           At_com.buf[31]='.';
           At_com.buf[32]='r';
           At_com.buf[33]='u';
           At_com.buf[34]='"';
           At_com.buf[35]=0;
           */
           
           break; 
     
     case cATCREG0:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "at+creg=0");
           break;        
           
     case cATDGPRS:
          // At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
          // At_com.vol_rx_out=5;   //ОТ СКОРОСТИ
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=10000; 
           strcpy(At_com.buf, "atd*99***1#");
           break;
    case cATH:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=3000;
           strcpy(At_com.buf, "ath"); 
           break;  
   
   case cATCGATT0:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=500;
           strcpy(At_com.buf, "at+cgatt=0");
           break;  
   
    case cATCPIN:
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=2000;
           strcpy(At_com.buf, "at+cpin=");
              
           if(simka==SIM_BASE) CopyMass(A_PIN_SIM1,(unsigned char *)&At_com.buf[8],4); 
             else CopyMass(A_PIN_SIM2,(unsigned char *)&At_com.buf[8],4);              
           At_com.buf[12]=0;
             
          break;
     
     case PLUS:
           At_com.cnt_tm_out=0;
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=2300;  
           strcpy(At_com.buf, "+++");
           fl_at_com.rx_rec=0;
           fl_at_com.ok=0;
           fl_at_com.err=0;
           fl_at_com.tm_out=0;
           fl_at_com.tx_en=0;
           fl_at_com.rx_en=1;
       //   fl_at_com.tm_out_en=1;
           At_com.ln_buf=strlen(At_com.buf);
           At_com.cnt_tx=0;
           At_com.cnt_rx=0;
           
           //UCSR0B=UCSR0B & ~RXEN;  
           //UCSR0B=UCSR0B & ~RXCIE; 
        
           //UCSR0B=UCSR0B | TXEN;
           //UCSR0B=UCSR0B | TXCIE;
           
           
           //UDR0=At_com.buf[0];
           
           
   // Посылаем всегда по одному байту
            
         //  __HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);   // мутное действие 30 11 2018
           
           
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,(unsigned char *)&At_com.buf[0],1);
    
    
           
           
           return;
           
   case PAUSA_REG:At_com.vol_tm_out=5000;
               goto yyy;
   
    case PAUSA1:
               At_com.vol_tm_out=3000;
               goto yyy;
   
    case PAUSA2:At_com.vol_tm_out=5000;
               goto yyy;
    case PAUSA5:
               At_com.vol_tm_out=2000;
               goto yyy;
    case PAUSA4:At_com.vol_tm_out=2000;
          yyy:
          
           At_com.cnt_tm_out=At_com.vol_tm_out;  
           fl_at_com.rx_rec=0;
           fl_at_com.ok=0;
           fl_at_com.err=0;
           fl_at_com.tm_out=0;
           fl_at_com.tx_en=0;
           fl_at_com.rx_en=1;
           At_com.cnt_tx=0;
           At_com.cnt_rx=0;
           
 //__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);  //   3 12 2018         
           
           return;   
 
 case PAUSA3: 
           At_com.vol_tm_out=20000;
           At_com.cnt_tm_out=At_com.vol_tm_out;  
           fl_at_com.rx_rec=0;
           fl_at_com.ok=0;
           fl_at_com.err=0;
           fl_at_com.tm_out=0;
           fl_at_com.tx_en=0;
           fl_at_com.rx_en=1;
           At_com.cnt_tx=0;
           At_com.cnt_rx=0;
           
 //__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);  //   3 12 2018              
           return;

case cATSGAUTH: 
           At_com.vol_rx_out=50;   //ОТ СКОРОСТИ
           // At_com.vol_tm_out=100;
           At_com.vol_tm_out=120;
           strcpy(At_com.buf, "at^sgauth=1");
        //   strcpy(At_com.buf, "at%cgpco=1");
           break; 
 
case cATCGREG:
           At_com.vol_rx_out=1000;   //ОТ СКОРОСТИ
           At_com.vol_tm_out=1200;
           strcpy(At_com.buf, "at+cgreg?");           
           break;
           
     default: return;      
    }
    fl_at_com.rx_rec=0;
    fl_at_com.ok=0;
    fl_at_com.err=0;
    fl_at_com.tm_out=0;
    fl_at_com.tx_en=0;
    fl_at_com.rx_en=1;
    //fl_at_com.tm_out_en=1;
    At_com.cnt_tm_out=0;
    At_com.ln_buf=strlen(At_com.buf)+1;
    At_com.buf[At_com.ln_buf-1]=0x0d;
    At_com.cnt_tx=0;
    At_com.cnt_rx=0;
    //UCSR0B=UCSR0B | TXEN;
    //UCSR0B=UCSR0B | TXCIE;
    

    //UDR0=At_com.buf[0]; 
     
   // Посылаем всегда по одному байту
            
         
         //  автоматически разрешает прерывание от передатчика
           HAL_UART_Transmit_IT(&huart3,(unsigned char *)&At_com.buf[0],1);
    
    

  

    
    
    
}



void check_rx_ok(void)
{
 if(fl_conf_modem.kod==0)
                 {
                  
                  if (
                     (At_com.buf[At_com.cnt_rx-3]=='O')&&
                     (At_com.buf[At_com.cnt_rx-2]=='K')
                     )
                      fl_at_com.ok=1;
                     else fl_at_com.err=1; 
                 }
                 else
                 {
                 if(At_com.buf[At_com.cnt_rx-2]==KOD_OK)fl_at_com.ok=1; 
                     else fl_at_com.err=1;
                 }  
}




void at_com_rx(unsigned char cnt)
{

char i,*you;
if(fl_at_com.tx_en==1)return;
if(fl_at_com.rx_en==0)return;
if(fl_at_com.rx_rec==0)return;


At_com.cnt_tm_out=0;
fl_at_com.rx_en=0;
fl_at_com.rx_rec=0;
fl_at_com.tx_en=0;
//fl_at_com.tm_out_en=0;



if(cnt>=VOL_LIST){fl_at_com.err=1;return;}



//if(At_com.buf[At_com.cnt_rx-1]!=0xd){fl_at_com.err=1;return;}
if((fl_conf_modem.kod==1)&&(At_com.cnt_rx<2)){fl_at_com.err=1;return;}
if((fl_conf_modem.kod==0)&&(At_com.cnt_rx<3)){fl_at_com.err=1;return;}


//At_com.buf[At_com.cnt_rx-1]=0;

// Если ответ с последни кодом 0xa то затереть его? это специфика sim800
if( At_com.buf[At_com.cnt_rx-1]==0xa) At_com.cnt_rx--;

switch(At_com.list_com[cnt])
    {
     case cAT:    
     case cATE0:
     case cATV0:
     case cATE0V0:
     case cATIPR: 
     case cATIFC:
     case cATC: 
     case cATD: 
     case cATW:
     case cATCGATT:
     case cATCGDCONT:
     case cATSGAUTH: 
     case cATCREG0: 
     case cATH:  

                if(At_com.buf[At_com.cnt_rx-1]!=0xd){fl_at_com.err=1;return;}
                check_rx_ok();
                break;  
    case cATCREG:
                     if(At_com.buf[At_com.cnt_rx-1]!=0xd){fl_at_com.err=1;return;}
                  //   if(At_com.buf[At_com.cnt_rx-2]=='2')fl_at_com.ok=1;  
                     if(At_com.buf[At_com.cnt_rx-5]=='1'){fl_at_com.ok=1; state_led_md=LED_MD_YES_CREG;}   
                     else fl_at_com.err=1; 
           break;  
    
   case cATCGREG:
                     if(At_com.buf[At_com.cnt_rx-1]!=0xd){fl_at_com.err=1;return;}
                  //   if(At_com.buf[At_com.cnt_rx-2]=='2')fl_at_com.ok=1;  
                     if(At_com.buf[At_com.cnt_rx-5]=='1'){fl_at_com.ok=1;state_led_md=LED_MD_GPRS;}    
                     else fl_at_com.err=1; 
           break;          
    
   case cATCPIN:
                      if((At_com.buf[0]==0x30)&&(At_com.buf[1]==0xd))
                             { 
                               if(simka==SIM_BASE) send_info(sizeof(stm_pin1_ok),stm_pin1_ok,0,0);
                               else send_info(sizeof(stm_pin2_ok),stm_pin2_ok,0,0);
                               
                               //S5_GR;//
                               fl_at_com.ok=1;
                             }
                      else
                          { 
                            if(simka==SIM_BASE) send_info(sizeof(stm_pin1_err),stm_pin1_err,0,0);
                               else send_info(sizeof(stm_pin2_err),stm_pin2_err,0,0);
                            fl_at_com.err=1;
                          }
                      
           break;            
           
    case cATDGPRS:
               /*
               led_sost_s_on;
               if(fl_conf_modem.kod==0){fl_at_com.ok=1; SVD2_1_ON; while(1);}//proverkafl_at_com.ok=1;
                 else
                 {
                 SVD1_1_ON; while(1);//proverka
                  if((At_com.buf[At_com.cnt_rx-2]==KOD_CONNECT)&&(At_com.buf[At_com.cnt_rx-1]==0xd))fl_at_com.ok=1; 
                  if((PIND & DCD)==0)fl_at_com.ok=1;else fl_at_com.err=1;
                 //if(At_com.buf[At_com.cnt_rx-2]==KOD_CONNECT)fl_at_com.ok=1; было
                 //    else fl_at_com.err=1;
                 }  
                 */
                 
                  if((At_com.buf[At_com.cnt_rx-2]==KOD_CONNECT)&&(At_com.buf[At_com.cnt_rx-1]==0xd))fl_at_com.ok=1;
                  else fl_at_com.err=1;
                  
                  
                //  if((PINE & DCD0)==0)fl_at_com.ok=1;else fl_at_com.err=1;
                                             
           break;
     case PLUS:
           return;
           
           
    case cATCIMI:
     // if(At_com.cnt_rx >28 ) {fl_at_com.err=1;break;}
     // if((At_com.buf[At_com.cnt_rx-1]==0xd))
      you=&At_com.buf[0];
      while(*you <=0xd) you++;
      
      
      {
       
       for(i=0;i<At_com.cnt_rx;i++) {
         if(*you==0xd) {emei[0]=i;return;}
         else emei[i+1]=*you++;
       }
      }
   //  emei[0] =At_com.cnt_rx-ii-1;   // в первом байте лежит к-во байт emei  
     break;      
           
     default:fl_at_com.err=1; At_com.cnt_rx=0;return;      
    }
   
    
}

unsigned char at_com_scen_init(unsigned char *cnt,unsigned char *rp)
{
  if(fl_at_com.tx_en==1)return(0);
  if(fl_at_com.rx_en==1)return(0);
  if(fl_at_com.rx_rec==1)return(0);
  
  

  switch(At_com.list_com[*cnt])
  {
  case cATE0V0:
           
           
           (*cnt)++;*rp=0;
           fl_at_com.tx_en=1;
           
           break;            
  case cAT: 
  
  case cATCREG:
           if(fl_at_com.ok==1){(*cnt)++;*rp=0;fl_at_com.tx_en=1;}
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
           (*rp)++;
           if(*rp>=60){event_modem=EVM_AT_ERR;return(2);}//было 40
           fl_at_com.tx_en=1;
           }
       break; 

       
  
  case cATCGDCONT:
  case cATCREG0:  
           if(fl_at_com.ok==1){(*cnt)++;*rp=0;fl_at_com.tx_en=1;}
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
           (*rp)++;
           if(*rp>=3){event_modem=EVM_AT_ERR;return(2);}
           fl_at_com.tx_en=1;
           }
       break; 
 
 case cATSGAUTH: 

           (*cnt)++;*rp=0;
           fl_at_com.tx_en=1;
           break;
           
  case cATCGATT:
      // if(fl_at_com.ok==1){event_modem=EVM_AT_OK;return(1);}
       if(fl_at_com.ok==1){(*cnt)++;*rp=0;fl_at_com.tx_en=1;}
       if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
           (*rp)++;
         //  if(*rp>=1){event_modem=EVM_AT_ERR;return(3);}
           if(*rp>=6){event_modem=EVM_AT_ERR;return(3);}
           fl_at_com.tx_en=1;
           }
       break; 
       
 case cATCGREG:
           //if(fl_at_com.ok==1){(*cnt)++;*rp=0;fl_at_com.tx_en=1;}
           if(fl_at_com.ok==1){event_modem=EVM_AT_OK;return(1);}
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
           (*rp)++;
           if(*rp>=20){event_modem=EVM_AT_ERR;return(3);}
           fl_at_com.tx_en=1;
           }
       break;  
       
case cATCPIN:
           if(fl_at_com.ok==1){(*cnt)++;*rp=0;fl_at_com.tx_en=1;}        
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1)){event_modem=EVM_AT_ERR;return(3);}
           
       break;         
       
 
 case PAUSA_REG:
 case PAUSA5:  
          (*cnt)++;*rp=0;
           fl_at_com.tx_en=1; 
           break; 
       
  case cATCIMI:  
          (*cnt)++;*rp=0;
           fl_at_com.tx_en=1; 
           break;    
           
           
  }
  
 fl_at_com.ok=0; 
 fl_at_com.tm_out=0;
 fl_at_com.err=0;
 return(0);
}



void init_scen_stm_si(void)
{

 unsigned char buf[4];
 unsigned char i;
 
 
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
  
 
 
 i=0;
 //st_gprs_atcom();

cnt_com=0;
rep=0;

At_com.list_com[0]=cATE0V0;
At_com.list_com[1]=cATE0V0;
At_com.list_com[2]=cATE0V0;
At_com.list_com[3]=PAUSA5;
At_com.list_com[4]=cAT;
At_com.list_com[5]=cATCIMI;
At_com.list_com[6]=cATCREG0;





 if(simka==SIM_BASE) CopyMass(A_PIN_SIM1,&buf[0],4); 
                else CopyMass(A_PIN_SIM2,&buf[0],4);              
if
   ((buf[0]>=0x30)&&(buf[0]<=0x39)&&
   (buf[1]>=0x30)&&(buf[1]<=0x39)&&
   (buf[2]>=0x30)&&(buf[2]<=0x39)&&
//     (buf[3]>=0x30)&&(buf[3]<=0x39)){At_com.list_com[7]=cATCPIN;i=1; S5_RD;} else {S5_YL;}
       (buf[3]>=0x30)&&(buf[3]<=0x39)){At_com.list_com[7]=cATCPIN;i=1; }   
     
     
At_com.list_com[7+i]=cATCREG;
At_com.list_com[8+i]=PAUSA_REG;//5секунд
At_com.list_com[9+i]=cATCGDCONT;
At_com.list_com[10+i]=cATSGAUTH;  
At_com.list_com[11+i]=cATCGATT;
At_com.list_com[12+i]=cATCGREG;



At_com.ln_list=13+i;
At_com.cnt_rx=0;
At_com.cnt_tm_out=0;
fl_at_com.rx_en=0;
fl_at_com.tx_en=1;



}



void init_scen_stm_sc(void)
{
//st_gprs_att();



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
  
  
  
cnt_com=0;
rep=0;
At_com.list_com[0]=cATDGPRS;
At_com.ln_list=1;
At_com.cnt_rx=0;
At_com.cnt_tm_out=0;
fl_at_com.rx_en=0;
fl_at_com.tx_en=1;
}


unsigned char at_com_scen_stm_sc(unsigned char *cnt,unsigned char *rp)
{
  if(fl_at_com.tx_en==1)return(0);
  if(fl_at_com.rx_en==1)return(0);
  if(fl_at_com.rx_rec==1)return(0);
  
  switch(At_com.list_com[*cnt])
  {

  case cATDGPRS:
    /*       
    if(fl_at_com.ok==1){event_modem=EVM_AT_OK;return(1);}
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
           (*rp)++;
           if(*rp>=4){event_modem=EVM_AT_ERR;return(2);}
           fl_at_com.tx_en=1;
           }
           */
           
   // if(fl_at_com.ok==1){event_modem=EVM_AT_OK;return(1);}
         
    if((fl_at_com.err==1)||(fl_at_com.tm_out==1)||(fl_at_com.ok==1))
           {
           
             
           //  if((PINE & DCD0)||(fl_at_com.ok==1))   // линия DCD активна !!!
             
            if( ((HAL_GPIO_ReadPin(GPIOD, DCD0)==0) ||(fl_at_com.ok==1)) )
             {
               event_modem=EVM_AT_OK;
               return(1);
             }
         else 
             {
             (*rp)++;
              if(*rp>=4){event_modem=EVM_AT_ERR;return(2);}
              fl_at_com.tx_en=1;
             }
           }
    
       break; 
  }
  
 fl_at_com.ok=0; 
 fl_at_com.tm_out=0;
 fl_at_com.err=0;
 return(0);
}


/*
void init_scen_stm_dc(void)
{

cnt_com=0;
rep=0;

At_com.list_com[0]=PAUSA1;
At_com.list_com[1]=PLUS;
At_com.list_com[2]=PAUSA2;
At_com.list_com[3]=cATH;
At_com.ln_list=4;
At_com.cnt_rx=0;
At_com.cnt_tm_out=0;
fl_at_com.rx_en=0;
fl_at_com.tx_en=1;
}


unsigned char at_com_scen_stm_dc(unsigned char *cnt,unsigned char *rp)
{
if(fl_at_com.tx_en==1)return(0);
  if(fl_at_com.rx_en==1)return(0);
  if(fl_at_com.rx_rec==1)return(0);
  
  switch(At_com.list_com[*cnt])
  {
  case PAUSA1:
  case PLUS:
  case PAUSA2:
          (*cnt)++;*rp=0;
           fl_at_com.tx_en=1; 
           break;
        
  case cATH:
       if(fl_at_com.ok==1){event_modem=EVM_AT_OK;return(1);}
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
           (*rp)++;
           if(*rp>=6){event_modem=EVM_AT_ERR;return(2);}
           fl_at_com.tx_en=1;
           }
       break; 
  }
  
 fl_at_com.ok=0; 
 fl_at_com.tm_out=0;
 fl_at_com.err=0;
 return(0);
}

*/

void init_scen_stm_dc_1(void)
{



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
  
  
cnt_com=0;
rep=0;

At_com.list_com[0]=PAUSA1;
At_com.list_com[1]=PLUS;
At_com.list_com[2]=PAUSA2;      
At_com.list_com[3]=cATH;
//At_com.list_com[4]=PAUSA4;
//At_com.list_com[5]=cATCGATT0;
//At_com.list_com[6]=PAUSA3;
//At_com.list_com[7]=cATCGATT;
At_com.ln_list=4;
At_com.cnt_rx=0;
At_com.cnt_tm_out=0;
fl_at_com.rx_en=0;
fl_at_com.tx_en=1;
}

unsigned char at_com_scen_stm_dc_1(unsigned char *cnt,unsigned char *rp)
{
if(fl_at_com.tx_en==1)return(0);
  if(fl_at_com.rx_en==1)return(0);
  if(fl_at_com.rx_rec==1)return(0);
  
  switch(At_com.list_com[*cnt])
  {
  case PAUSA1:
  case PLUS:
  case PAUSA2:
  case PAUSA3:
          (*cnt)++;*rp=0;
           fl_at_com.tx_en=1; 
           
           
        
           
           
           
           break;
  
  case PAUSA4:
          // if((PINE & DCD0)==0){event_modem=EVM_AT_OK;return(1);}else{event_modem=EVM_AT_ERR;return(3);}
    
        //   if(HAL_GPIO_ReadPin(DIO_PORTE, DCD0)==0) {event_modem=EVM_AT_OK;return(1);}else{event_modem=EVM_AT_ERR;return(3);}
            if(HAL_GPIO_ReadPin(GPIOD, DCD0)) 
            {
              init_pins_out_to_hiz();
              event_modem=EVM_AT_OK;
              return(1);
            }
            else
            {
              init_pins_out_to_hiz();
              event_modem=EVM_AT_ERR;
               return(3);
            } // 3 вольта
           
         //  (*cnt)++;*rp=0;
         //  fl_at_com.tx_en=1; 
        //   break;
   /*     
  case cATH:
          
         if((fl_at_com.err==1)||(fl_at_com.tm_out==1)||fl_at_com.ok==1)
           {
           (*rp)++;
           if(*rp>=5)
                     {
                       
                     
                     (*cnt)++;*rp=0;
                    // if)(PIND & DCD=)==0){event_modem=EVM_AT_OK;return(1);}
                    
                    // if((PIND & DCD)==0){SVD1_1_ON;*cnt=*cnt+2; *rp=0;}
                    //   else {(*cnt)++;*rp=0;}
                     }
           fl_at_com.tx_en=1;
           }
       break; */
 
 case cATH:
          
   //  if(fl_at_com.ok==1){(*cnt)++;*rp=0;fl_at_com.tx_en=1;}
                
         if(fl_at_com.ok==1){event_modem=EVM_AT_OK;return(1);}
         
         // сомнительная вещь ее нужно делать безвсяких условий иначе появляется неопределенность
         //
         //  if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
         else
           {
           (*rp)++;
           if(*rp>=6){event_modem=EVM_AT_ERR;return(2);}
           fl_at_com.tx_en=1;
           }
         
       break;       
       
 case cATCGATT0:
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
            (*cnt)++;
            fl_at_com.tx_en=1; 
           }
       break;
       
  case cATCGATT:
       if(fl_at_com.ok==1){event_modem=EVM_AT_OK;return(1);}
           if((fl_at_com.err==1)||(fl_at_com.tm_out==1))
           {
           (*rp)++;
           //if(*rp>=1){event_modem=EVM_AT_ERR;return(3);}
           if(*rp>=2){event_modem=EVM_AT_ERR;return(3);}
           fl_at_com.tx_en=1;
           }
       break;              
  }
  
 fl_at_com.ok=0; 
 fl_at_com.tm_out=0;
 fl_at_com.err=0;
 return(0);
}





unsigned char modem_com_init(void)
{
unsigned char res_return;
unsigned char cnt_com=0;
unsigned char rep=0;

init_scen_stm_si();

while(1)
     {
     at_com_tx(cnt_com);
     at_com_rx(cnt_com);
     res_return=at_com_scen_init(&cnt_com,&rep);
       switch (res_return)
           {
           case 1: return(1);//сценарий закончен корректно
           case 2: return(2);// неответом на AT комманду
           case 3: return(3);// неответом на присоединение контекста
           }
   
    // if(at_com_scen_init(&cnt_com,&rep)>0) while(1);
     //if (at_com_scen_init(&cnt_com,&rep)==1)return(1);
     } 
}






void delay_modem_on(void)
    {
     //  set_rts_232;       
    //   modem_on;
       flag=0;
       Cnt_modem_on_off=6;

       while(1)
        {
        delay(1);
        if(flag==1)return;
        }
       
    
    }
    
    void delay_modem_off(void)
    {     
       //modem_off;
       flag=0;
       Cnt_modem_on_off=3;
       while(1)
        {
        delay(1);
        if(flag==1)return;
        }
       
    
    }
    



