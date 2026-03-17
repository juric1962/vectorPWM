//#include <iom1280.h> 
//#include "dfpin.h"
//#include "dfproc.h"
#include <string.h> 
//#include <inavr.h>
//#include "dfcnst.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "utils.h"
#include "modbus.h"
#include "map_mbus.h"

extern char Flag_Wait_OK;
extern char CountSecWaitOK;
extern int CountItemInHtmlList;

unsigned char page_bufer[1000];
char gaga[20];
void mov_s ( char  *p, int size);

extern void delay(unsigned int t);
//char read_modbus(char direction,unsigned int start_madres, char num_word, unsigned int *result);
void reset_ESP(void);

/*
__flash 
struct
{
char led_off_header[] ="Led is OFF. Turn <a href='/?led=on'>on</a>.";
char led_on_header[] ="Led is ON. Turn <a href='/?led=off'>off</a>.";

} biaka;

*/


//char *sob_51 = {"Изменение уставки Рном1,Вт\r\nPARAM="};

extern struct
{
char buf[1000];
unsigned int cnt_tx;
unsigned int cnt_rx;
unsigned int ln_buf;
unsigned int cnt_tm_out; //счетчик времени ожидания ответа
unsigned int vol_tm_out; //предел времени ожидания ответа
unsigned int cnt_rx_out; //счетчик межбайтовый промежуток
unsigned int vol_rx_out; //предел межбайтового промежутка
}At_com;

char wd_sec,work_bit,wd_sec_wait_rx;  // уровеню активности по wifi

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

 typedef struct {
    char webif_404_reply[200];
    char webif_200_header[200];
    char sob_48[200];
    char sob_49[200];
    
} KLA ;


//char __flash *point_flesh;

 //KLA  klass;
//__flash struct klass a;
//mfs->s[0]=0;
//__flash struct klass a = {"Sergey", 10, 4.5 };

 typedef struct {
          
    float f;
    char s[20];
    char d;
} BOB ;

union {
      unsigned char opa[sizeof(BOB)];
      BOB lak;
} jopa;

BOB aga;

// пример инициализации массива структур
// надо объявлять и инициализировать весь массив структур сразу
const BOB plo[3] = {
   4.5, "Sergey", 10,
  4.5, "Sergey", 10, 
  4.5, "Sergey", 10
};
  
   

const  KLA a = {
  "HTTP/1.0 404 Not Found\r\n"
    "Content-Type: text/html; charset=windows-1251\r\n"
    "Server: ATmega16\r\n"
    "\r\n"
    "<pre>Page not found\r\n\r\n"
    "<a href='/'>Home page</a></pre>\r\n",
                                  "HTTP/1.1 200 OK\r\n"
                                  "Content-Type: text/html; charset=windows-1251\r\n"
                                  "Server: ATmega1280\r\n"
                                  "\r\n"
                                  "Привет из микроконтроллера!",
    "HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=windows-1251\r\n\r\nПревед из микроконтроллера!",
    "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n\r\n<html><head><title>Wi-Fi ESP8266 DEMO</title></head><body><a href=\"/on\">on</a> <a href=\"/off\">off</a></body></html>\r\n"};



/*
struct
{
 char led_off_header[12] =="Led is OF";

}
hhhhh;
*/


const char led_off_header[] ="Led is OFF. Turn <a href='/?led=on'>on</a>.";
const char led_on_header[] ="Led is ON. Turn <a href='/?led=off'>off</a>.";



/*

__flash char  webif_404_reply[] =
    "HTTP/1.0 404 Not Found\r\n"
    "Content-Type: text/html; charset=windows-1251\r\n"
    "Server: ATmega16\r\n"
    "\r\n"
    "<pre>Page not found\r\n\r\n"
    "<a href='/'>Home page</a></pre>\r\n";

// Заголовок странички
__flash char webif_200_header[] =
                                  "HTTP/1.1 200 OK\r\n"
                                  "Content-Type: text/html; charset=windows-1251\r\n"
                                  "Server: ATmega1280\r\n"
                                  "\r\n"
                                  "Привет из микроконтроллера!";

__flash char sob_48[] ={"HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=windows-1251\r\n\r\nПревед из микроконтроллера!"};

__flash char sob_49[] ={"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n\r\n<html><head><title>Wi-Fi ESP8266 DEMO</title></head><body><a href=\"/on\">on</a> <a href=\"/off\">off</a></body></html>\r\n"}; //length: 117
      
*/



// RS232   19200,N,8,1
// RS485_1  38400,n,8,1  для контроля 
// fl_at_com.rx_rec==1 принят пакет длинной At_com.cnt_rx лежит в массиве At_com.buf[]
// fl_at_com.tm_out==1 нет ответа на запрос


extern char one_sec;
char ogni,comand_WiFi;
char *puk;

unsigned char *url, *p, *params, *name, *value, led_state,*ptr,val;
unsigned int sizik;
char ID;










// возвращает адрес следующего элемента, после копирования
unsigned char *copy_M(unsigned char *dest,const char *src)
{
 //for (ii=0;ii<sizeof(webif_200_header)-1;ii++) *dest++=*src++; 
  while(*src) *dest++=*src++; 
  return(dest);
}




 // возвращает адрес следующего элемента, после совпадения
unsigned char *find_str(int num, unsigned char *inp,unsigned char *cmp)
{  
  int ii;
  unsigned char *pi,*pc,tru;  
  
  for(ii=0;ii<num;ii++) {
    pi=inp+ii;
    pc=cmp;
    tru=0;
    while(1){
    if( (*pc==0)  && (tru==1) ) return(pi);
    if( (*pc==0)  && (tru==0) ) break;
    if(*pi!=*pc) tru=0;
    else tru=1;
    pi++;
    pc++;
  }
}
return(NULL);
}


// возвращает 0 если совпало
unsigned char *str_cmp(unsigned char *inp,unsigned char *cmp)
{ 
    unsigned char *pi,*pc,tru;
  
    pi=inp;
    pc=cmp;
    tru=0;
    while(1){
    if( (*pc==0)  && (tru==1) ) return(0);
    if( (*pc==0)  && (tru==0) ) break;
    if(*pi!=*pc) tru=0;
    else tru=1;
    pi++;
    pc++;
}


return(pi);
}


// возвращает адрес элемента если не нашел то 0
unsigned char *find_char( unsigned char *inp,unsigned char cmp)
{
  
 
  unsigned char *pi;
  
  
  pi=inp;
  
  while(*pi) {
    if (*pi==cmp) return(pi);
    pi++;
    }
return(0);
}


/*void send_kadr_to_modulo(void)
{  
  HAL_UART_Transmit_DMA(&huart3,(uint8_t*)&rs232TxBuf,rs232TxPtr);   
}*/

/*void OLD_send_kadr_to_modulo(void)
{
// выбросить в порт
// At_com.ln_buf должен быть не нулевым данные лежат в массиве
  
    At_com.vol_rx_out=50; // предел межбайтового промежутка            
    fl_at_com.rx_rec=0;               // пакет принят
    fl_at_com.tm_out=0;
    fl_at_com.tx_en=0;
    fl_at_com.rx_en=1;
    At_com.cnt_tm_out=0;
   
   
    At_com.cnt_tx=0;
    At_com.cnt_rx=0;
    UCSR0B=UCSR0B | TXEN;
    UCSR0B=UCSR0B | TXCIE;
    S5_RD;
    UDR0=UDR3=At_com.buf[0]; 
}*/


/*
void scenario(void)
{
  
  if(one_sec==0) return;
  one_sec=0;
  
  
  //if(fl_at_com.tm_out) send_kadr_to_modulo();
  send_kadr_to_modulo();
  
  switch(ogni) {
  case 0: {S1_RD;ogni++;break;}
  case 1: {S1_OFF;S2_RD;ogni++;break;}
  case 2: {S2_OFF;S4_RD;ogni++;break;}
  case 3: {S4_OFF;S3_RD;ogni++;break;}
  case 4: {S3_OFF;ogni=0;break;}
 // case 5: {S5_OFF;ogni=0;break;}
  
  default: {ogni=0;
  
  
   S1_OFF;
   S2_OFF;
   S3_OFF;
   S4_OFF;
   S5_OFF;  
   break;
 }
   
            }  // end switch
  
  

}

*/

/*void mov_s( char  *p, int size)
{
    while(size--) s_port(*p++);
}*/

void reset_ESP(void)
{
   ESP_RESET(1);
   Delay_us(10000);
   ESP_RESET(0);
   CountItemInHtmlList = 0; 
   /*
   SET_RTS0; // RESET LOW
  
  long_delay(10000);// reset wifi
  CLR_RTS0; // RESET HI
   
 __watchdog_reset();
   long_delay(1000000);//>3 sek*/
}

void debugOutput(uint8_t *src,uint16_t len)
{ 
  #ifdef WI_FI_DEBUG
  uint16_t copyPtr;
  
  return;
  for(copyPtr=0;copyPtr<len;copyPtr++)
  {
    rs485TxBuf[copyPtr]=src[copyPtr];
  }  
  RS485_DIR(1);HAL_UART_Transmit_DMA(&huart2,(uint8_t*)&rs485TxBuf,len);
  Delay_us(50000);
  #endif
}
void scenario(void)
{
  unsigned int count_ri,ii,icc;  
    
  if (oneSec) 
  {
    oneSec = 0;
    CountSecWaitOK++; 
    wd_sec++;
    wd_sec_wait_rx++;
    if(wd_sec_wait_rx > 60 ) 
    {
      wd_sec_wait_rx=0;
      if( comand_WiFi == 16) comand_WiFi = 100;
      else {reset_ESP();comand_WiFi=0;} 
      return;
    }  // контроль канала 
    
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0); // Пока только DO2 не используется
  }
  
  if (wd_sec > 10) 
  {
    wd_sec = 0;
    if (work_bit == 0) 
    {
      reset_ESP();
      comand_WiFi = 0;
      return;
    }
    else 
      work_bit = 0;
  }
  
  if ((Flag_Wait_OK == 1) && (CountSecWaitOK > 5))
  {
    reset_ESP();
    comand_WiFi=0;
    Flag_Wait_OK = 0;
    return;
  }
  
//   p=copy_P(p,&plo[0].s[0]);
//   p=copy_P(p,&plo[1].s[0]);
//   p=copy_P(p,&plo[2].s[0]);
//   p=copy_P(&jopa.opa[0],(char const __flash *)&plo[2].f);
  
//   p=copy_P((unsigned char *)&aga.f,(char const __flash *)&plo[2].f);    // из структуры flash в сруктуру memory
   
  switch(comand_WiFi)
  {  
   // контроль работы модуля 
     case 100:         
      wifiTimeOutTimer=1200; 
      puk=(char*)&wifiTxBuf[0];//At_com.buf[0];            
      sprintf(puk,"AT\r\n\0");
      puk=(char*)&wifiTxBuf[0];//&At_com.buf[0];
      while(*puk)puk++;
      wifiTxPtr=puk-(char*)&wifiTxBuf[0];//&At_com.buf[0];      
      comand_WiFi=101; // ожидание ответа           
      HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);      
      debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;   
    case 101:        
    if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){reset_ESP();comand_WiFi=0;return;}  
    if(wifiDataReady){
      wifiDataReady=0;  
      debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);
      count_ri=wifiRxPtr;//At_com.cnt_rx;
      if((wifiRxBuf[count_ri-1]==0xa)&&
         (wifiRxBuf[count_ri-2]==0xd)&&
         (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){
              waitForWiFiFlag=0;
            debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);
            wifiRxPtr=0;
            wifiDataReady=0;              
                       
             __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);   
             comand_WiFi=16;return;}
      else {/*fl_at_com.tm_out=0;*/ reset_ESP();comand_WiFi=0;return;} // нет ответа на команду повторить ее      
    }
    return;    
    
    
    ///////////////////////////////////////////////////// 22 11 2018
    case 0:         
      wifiTimeOutTimer=1200; 
      puk=(char*)&wifiTxBuf[0];//At_com.buf[0];            
      sprintf(puk,"AT\r\n\0");
      puk=(char*)&wifiTxBuf[0];//&At_com.buf[0];
      while(*puk)puk++;
      wifiTxPtr=puk-(char*)&wifiTxBuf[0];//&At_com.buf[0];      
      comand_WiFi=1; // ожидание ответа           
      HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);      
      debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;   
    case 1:        
    if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
    if(wifiDataReady){
      wifiDataReady=0;  
      debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);
      count_ri=wifiRxPtr;//At_com.cnt_rx;
      if((wifiRxBuf[count_ri-1]==0xa)&&
         (wifiRxBuf[count_ri-2]==0xd)&&
         (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){/*fl_at_com.tm_out=0;*/comand_WiFi=2;return;}
      else {/*fl_at_com.tm_out=0;*/comand_WiFi=0;return;} // нет ответа на команду повторить ее      
    }
    return;    
   ///////// next comamnd ATE0
    case 2:       
       wifiTimeOutTimer=1200;       
       puk=(char*)&wifiTxBuf[0];//&At_com.buf[0];
       sprintf(puk,"ATE0\r\n\0");
       puk=(char*)&wifiTxBuf[0];//&At_com.buf[0];
       while(*puk) puk++;
       wifiTxPtr=puk-(char*)&wifiTxBuf[0];
       comand_WiFi=3; // ожидание ответа              
       HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);
       debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;   
    case 3:        
    if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
    if(wifiDataReady){
      wifiDataReady=0;
      debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);     
      count_ri=wifiRxPtr;
      if((wifiRxBuf[count_ri-1]==0xa)&&
         (wifiRxBuf[count_ri-2]==0xd)&&
         (wifiRxBuf[count_ri-3]=='K')&&
         (wifiRxBuf[count_ri-4]=='O') ){comand_WiFi=4;return;}
      else {comand_WiFi=0;return;} // нет ответа на команду повторить ее      
    }
    return;
    ///////// next comamnd AT+CWMODE=2  AP мода
    case 4:
     wifiTimeOutTimer=1200; 
     puk=(char*)&wifiTxBuf[0];
     sprintf(puk,"AT+CWMODE=2\r\n\0");
     puk=(char*)&wifiTxBuf[0];
     while(*puk)puk++;
     wifiTxPtr=puk-(char*)&wifiTxBuf[0];
     comand_WiFi=5; // ожидание ответа     
     HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr); 
     debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;
    case 5:       
      if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
      if(wifiDataReady){
        wifiDataReady=0;
        debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);
        count_ri=wifiRxPtr;
        if((wifiRxBuf[count_ri-1]==0xa)&&
           (wifiRxBuf[count_ri-2]==0xd)&&
           (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){comand_WiFi=6;return;}
        else {comand_WiFi=0;return;} // нет ответа на команду повторить ее      
      }
    return;   
     ///////// next comamnd AT+CWJAP="ZyXEL_KEENETIC_4G_FCEA08","sMartplus"
    ////////// AT+CWSAP=\"SMART_VHF\",\"sMartplus\",5,3\r\n\0");  AP мода
    case 6:     
     wifiTimeOutTimer=8200; 
     puk=(char*)&wifiTxBuf[0];
     //sprintf(puk,"AT+CWSAP?\r\n\0");
     sprintf(puk,"AT+CWSAP=\"FC CMAPT\",\"1234567890\",10,3\r\n\0");
     puk=(char*)&wifiTxBuf[0];
     while(*puk)puk++;
     wifiTxPtr=puk-(char*)&wifiTxBuf[0];
     comand_WiFi=7; // ожидание ответа     
     HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);
     debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;   
    case 7:
      if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
      if(wifiDataReady){
        wifiDataReady=0;
        debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);       
        count_ri=wifiRxPtr;
        if((wifiRxBuf[count_ri-1]==0xa)&&
           (wifiRxBuf[count_ri-2]==0xd)&&
           (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){comand_WiFi=8;return;}
        else {comand_WiFi=0;return;} // нет ответа на команду повторить ее      
      }
    return;    
     ///////// next comamnd AT+CIPSTA="192.168.0.201"
    ///////// next comamnd AT+CIPAP="192.168.0.1"       AP мода
   case 8:
     wifiTimeOutTimer=2200; 
     puk=(char*)&wifiTxBuf[0];
     sprintf(puk,"AT+CIPAP=\"192.168.0.1\"\r\n\0");
     puk=(char*)&wifiTxBuf[0];
     while(*puk)puk++;
     wifiTxPtr=puk-(char*)&wifiTxBuf[0];
     comand_WiFi=9; // ожидание ответа     
     HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);   
     debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
   break;   
   case 9:  
      if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
      if(wifiDataReady){
        wifiDataReady=0;
        debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);        
        count_ri=wifiRxPtr;
        if((wifiRxBuf[count_ri-1]==0xa)&&
           (wifiRxBuf[count_ri-2]==0xd)&&
           (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){comand_WiFi=10;return;}
        else {comand_WiFi=0;return;} // нет ответа на команду повторить ее      
      }
    return;     
     ///////// next comamnd AT+CIPMODE=0
    case 10:
     wifiTimeOutTimer=2200; 
     puk=(char*)&wifiTxBuf[0];
     sprintf(puk,"AT+CIPMODE=0\r\n\0");
     puk=(char*)&wifiTxBuf[0];
     while(*puk)puk++;
     wifiTxPtr=puk-(char*)&wifiTxBuf[0];
     comand_WiFi=11; // ожидание ответа        
     HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);  
     debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;   
    case 11:    
      if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
      if(wifiDataReady){
        wifiDataReady=0;
        debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);        
        count_ri=wifiRxPtr;
        if((wifiRxBuf[count_ri-1]==0xa)&&
           (wifiRxBuf[count_ri-2]==0xd)&&
           (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){comand_WiFi=12;return;}
        else {wifiRxBuf[wifiRxPtr]=0;comand_WiFi=12;/*10*/;return;} // нет ответа на команду повторить ее      
      }
    return;   
     ///////// next comamnd AT+CIPMUX=1
    case 12:
     wifiTimeOutTimer=2200; 
     puk=(char*)&wifiTxBuf[0];
     sprintf(puk,"AT+CIPMUX=1\r\n\0");     
     puk=(char*)&wifiTxBuf[0];
     while(*puk)puk++;
     wifiTxPtr=puk-(char*)&wifiTxBuf[0];
     comand_WiFi=13; // ожидание ответа          
     HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);  
     debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;   
    case 13:
      if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
      if(wifiDataReady){
        wifiDataReady=0;   
        debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);        
        count_ri=wifiRxPtr;
        if((wifiRxBuf[count_ri-1]==0xa)&&
           (wifiRxBuf[count_ri-2]==0xd)&&
           (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){comand_WiFi=14;return;}
        else {comand_WiFi=0;return;} // нет ответа на команду повторить ее      
      }
    return;    
     ///////// next comamnd AT+CIPSERVER=1,80
    case 14:  
      wifiTimeOutTimer=8200; 
      puk=(char*)&wifiTxBuf[0];
      sprintf(puk,"AT+CIPSERVER=1,80\r\n\0");
      puk=(char*)&wifiTxBuf[0];
      while(*puk)puk++;
      wifiTxPtr=puk-(char*)&wifiTxBuf[0];
      comand_WiFi=115; // ожидание ответа      
      HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);
      debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);
    break;  
    ///////////////////////////////
    case 115:    
      if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
      if(wifiDataReady){
        wifiDataReady=0;  
        debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);
        count_ri=wifiRxPtr;
        if((wifiRxBuf[count_ri-1]==0xa)&&
           (wifiRxBuf[count_ri-2]==0xd)&&
           (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){comand_WiFi=116;return;}
        else {comand_WiFi=0;return;} // нет ответа на команду повторить ее      
      }
    return;    
    ///////// next comamnd AT+CIPSTO=2
    case 116:      
     wifiTimeOutTimer=2200; 
     puk=(char*)&wifiTxBuf[0];
     sprintf(puk,"AT+CIPSTO=6\r\n\0");
     puk=(char*)&wifiTxBuf[0];
     while(*puk)puk++;
     wifiTxPtr=puk-(char*)&wifiTxBuf[0];
     comand_WiFi=15; // ожидание ответа     
     HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiTxBuf,wifiTxPtr);                                  
     debugOutput((uint8_t*)&wifiTxBuf,wifiTxPtr);     
   break;
   ///////////////////////////////   
   case 15:
     if(waitForWiFiFlag&&(wifiTimeOutTimer==0)){comand_WiFi=0;return;}  
     if(wifiDataReady){
        wifiDataReady=0;      
        count_ri=wifiRxPtr;
        if((wifiRxBuf[count_ri-1]==0xa)&&
           (wifiRxBuf[count_ri-2]==0xd)&&
           (wifiRxBuf[count_ri-3]=='K')&&
           (wifiRxBuf[count_ri-4]=='O') ){
            waitForWiFiFlag=0;
            debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);
            wifiRxPtr=0;
            wifiDataReady=0;              
            comand_WiFi=16;           
             __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);             
            //sprintf(gaga,"hello\r\n\0");
            //mov_s( gaga, strlen(gaga));                          
            return;           
           }
        else {comand_WiFi=0;return;} // нет ответа на команду повторить ее      
      }
   return;         
   case 16:     
     work_bit=1;  // признак работы 
     
     if(waitForWiFiFlag&&(wifiTimeOutTimer==0))
     { 
       waitForWiFiFlag=0;
       wifiRxPtr=0;
       wifiDataReady=0;                                      
       __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
       return;
     } // выход по таймауту  
     
     if(wifiDataReady){
        wifiDataReady=0;      
        count_ri=wifiRxPtr;  
        debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);       
        wifiRxBuf[count_ri]=0; // конец строки
        if(count_ri<5) goto musor;
//**********************************************************************************
// обработка ошибок от модуля ESP ( ready ( модуль сам по себе сбросился           *
//                                   busy ( не принимает пакеты по порту)          *
//  лечение сброс модуля RST и инициализация  по новой                             *
//**********************************************************************************
      if (count_ri <20) icc=0;
      else icc=count_ri-20;
      for (ii=count_ri;ii> icc;ii--)
      {
        if((wifiRxBuf[ii]=='y')&&
           (wifiRxBuf[ii-1]=='d')&&  
           (wifiRxBuf[ii-2]=='a')&&  
           (wifiRxBuf[ii-3]=='e')&&  
           (wifiRxBuf[ii-4]=='r')){reset_ESP();comand_WiFi=0;return;}      
      }
      for (ii=count_ri;ii> icc;ii--)
      {
        if((wifiRxBuf[ii]  =='y')&&
           (wifiRxBuf[ii-1]=='s')&&  
           (wifiRxBuf[ii-2]=='u')&&  
           (wifiRxBuf[ii-3]=='b')){reset_ESP();comand_WiFi=0;return;}      
      }      
//**********************************************************************************      
      if (CreateHtmlItem((char*)wifiRxBuf,(unsigned int*)&wifiRxPtr,count_ri) == WRITE_PORT)          
      {
          HAL_UART_Transmit_DMA(&huart5,(uint8_t*)&wifiRxBuf,wifiRxPtr); 
          debugOutput((uint8_t*)&wifiRxBuf,wifiRxPtr);
          waitForWiFiFlag=0;
          wifiRxPtr=0;
          wifiDataReady=0; 
          
      }else{
        goto musor;
      }
      break;      
    // это мусор     
      {
        musor:           
        //comand_WiFi=16;
        waitForWiFiFlag=0;
        wifiRxPtr=0;
        wifiDataReady=0;                                      
        __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);     
        return;
      }
      break;
   }  
    break;
   default: {comand_WiFi=0;return;}
  }  // end switch  
} // end scenario






