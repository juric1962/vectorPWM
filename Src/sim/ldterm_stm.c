//#include <iom1280.h>
//#include <string.h>
//#include <stdio.h>
//#include <inavr.h>
//#include <stdlib.h>
#include "ozu_map.h"
#include "map_ef.h"
#include "def_conf.h"
//#include "def_link.h"
//#include "def_log.h"
//#include "dfpin.h"
#include "dfproc.h"
//#include "map_ad.h"
#include "dfcnst.h"
#include "map_mbus.h"
#include "stdint.h"


//#define VOL_PUNKT 33 //(меню от 0 до VOL_PUNKT)  //dobavka
#define VOL_PUNKT 41 //(меню от 0 до VOL_PUNKT)  //dobavka

#ifdef VERS_BASE
const char ozu_vers[]  =   {'R','M','0','1','.','0','1',':','1','9','1','1','0','7'};
#endif



extern unsigned char sel_modul;
extern uint16_t temperatura;
char temp_to_grad(uint16_t t);


extern uint16_t num_self;




void control_temperatura(void);
void fun_init_sim900(void);
void send_to_sim900(char num);

unsigned char fl_at_mom_232;
unsigned char Regim;

extern void framWriteSim800(uint16_t adres_flesh,  unsigned char *adres_ozu, uint16_t num); 
  
extern void framRead(uint16_t adres_flesh, unsigned char *adres_ozu,uint16_t num);

extern uint32_t burst_ds_r(void);
extern void set_rlt(unsigned char address, unsigned char * data);

extern void lock_it(void);



extern void s_port(unsigned char ch);
extern void    mov_lf (void);
extern void    mov_s (char size, char const* p);



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


extern unsigned char point_log_buf;
/*
const char zp_apn1[] = {'4','.','A','P','N',' ','S','I','M','1',':',' '};
const char zp_apn2[] = {'5','.','A','P','N',' ','S','I','M','2',':',' '};
const char zp_ip[] = {'6','.','I','P',' ','a','d','d','r','e','s','s',' ','L','S',':',' '};

const char zp_db1[] = {'2','2','.','D','B',' ','A','P','N',':','1','.',' '};
const char zp_db2[] = {'2','3','.','D','B',' ','A','P','N',':','2','.',' '};
const char zp_db3[] = {'2','4','.','D','B',' ','A','P','N',':','3','.',' '};
*/



const char err_t[] = {0xd,0xa,'E','R','R'};
const char ok_t[] = {0xd,0xa,'O','K'};
const char dis[] = {0xd,0xa,'D','I','S','A','B','L','E'};


const char map_vers[]  =   {'F','M','0','2','.','0','1',':','1','9','1','1','0','7'};


const char versia[] = {"sim800 inside FC СМАРТ сборка:221118"};                         //{'S','I','0','2','.','0','6',':','1','1','0','3','1','6'};

//const char zp_udp[]=        {'U','D','P','p','o','r','t','-','>'};
const char zp_udp[]=        {'7','.','U','D','P',' ','p','o','r','t',' ','L','S',':',' '};

//const char zp_num[]=        {'N','u','m','-','>'};
const char zp_num[]=        {'8','.','U','n','i','t',' ','n','u','m','b','e','r',':',' '};

const char init_sim900[]=        {'I','N','I','T','_','S','I','M','x','0','0',' ','O','K','!'};
const char command_900[]=        {'C','O','M','M','A','N','D','>'};
const char term_900[]=        {"TERMINAL SIM"};

//const char zp_nat[]=        {'S','e','s',' ','S','I','M','1','-','>'};
//const char zp_nat_r[]=      {'S','e','s',' ','S','I','M','2','-','>'};
const char zp_nat[]=        {'9','.','T','i','m','e',' ','s','e','s','s','i','o','n',' ','S','I','M','1',',','s',':',' '};
const char zp_nat_r[]=      {'1','0','.','T','i','m','e',' ','s','e','s','s','i','o','n',' ','S','I','M','2',',','s',':',' '};



//const char zp_link[]=       {'T','m',' ','c','c','h','-','>'};
const char zp_link[]=       {'1','1','.','T','i','m','e',' ','c','o','n','t','r','o','l',' ','c','h','a','n','n','e','l',',','s',':',' '};

//const char zp_tm_link[]=    {'T','m',' ','a','n','s','-','>'};
const char zp_tm_link[]=     {'1','2','.','T','i','m','e',' ','r','e','s','p','o','n','s','e',',','s',':',' '};


//const char zp_tm_no_link[]= {'T','m',' ','d','c','n','-','>'};        //!!!!!!!!!!!!!!!!dobavka
const char zp_tm_no_link[]=   {'1','3','.','T','i','m','e',' ','n','o',' ','l','i','n','k',',','m','i','n',':',' '};

//const char zp_tm_link_res[]={'T','m',' ','r','e','s','-','>'};    //!!!!!!!!!!!!!!!!dobavka
const char zp_tm_link_res[]=  {'1','4','.','T','i','m','e',' ','r','e','s','e','r','v','e',' ','S','I','M','2',',','m','i','n',':',' '};


//const char zp_tm_vz[]=      {'T','m',' ','t','a','k','e','-','>'};
const char zp_tm_vz[]=        {'1','7','.','T','i','m','e',' ','t','a','k','i','n','g',',','s',':',' '};

//const char zp_tm_cl[]=      {'T','m',' ','c','l','-','>'};
const char zp_tm_cl[]=      {'1','8','.','T','i','m','e',' ','c','o','n','t','r','o','l',' ','c','l','i','e','n','t',',','s',':',' '};

//const char zp_num_seq[]=    {'C','l',' ','m','s','q','-','>'};
const char zp_num_seq[]=     {'1','6','.','C','l','i','e','n','t',' ','n','u','m','b','e','r',':',' '};   


//const char zp_des_seq[]=    {'D','e','s',' ','m','s','g','-','>'};
const char zp_des_seq[]=     {'1','5','.','D','e','s','.',' ','m','e','s','s','a','g','e',':',' '}; 


//const char zp_type_tc1[]={'T','y','p','e',' ','T','S','1','-','>'};
//const char zp_type_tc2[]={'T','y','p','e',' ','T','S','2','-','>'};
const char zp_type_tc1[]={'1','9','.','T','y','p','e',' ','T','S','1',':',' '};
const char zp_type_tc2[]={'2','0','.','T','y','p','e',' ','T','S','2',':',' '};

//const char zp_dbg[]=        {'D','B','G','-','>'};
const char zp_dbg[]=        {'2','1','.','U','n','i','t',' ','r','e','g','i','m','e',':',' '};




//const char zp_ts1[]=        {'T','S','1','-','2','-','>'};
const char zp_ts1[]= {'2','5','.','T','S','1','-','2',':',' '};

//const char zp_temp[]=       {'t','-','>'};

//const char zp_temp[]= {'2','6','.','A','D',' ','c','o','n','v','e','r','t','e','r',':',' '}; 
const char zp_temp[]= {'2','6','.','T','e','m','p','e','r','a','t','u','r','e',':',' '};

//const char zp_vosst[]=      {'S','e','t',' ','d','e','f','-','>'};
const char zp_vosst[]=      {'2','7','.','S','e','t',' ','d','e','f','a','u','l','t',':',' '};

//const char zp_time[]=       {'T','i','m','e','-','>'};
//const char zp_date[]=       {'D','a','t','e','-','>'};
const char zp_time[]=       {'2','8','.','T','i','m','e',':',' '};
const char zp_date[]=       {'2','9','.','D','a','t','e',':',' '};

//const char zp_log_file[]=   {'L','o','g',' ','f','i','l','e','-','>'};
const char zp_log_file[]=   {'3','0','.','L','o','g',' ','f','i','l','e',':',' '};

//const char zp_keys[]=       {'K','e','y',' ','d','e','f','-','>'};
const char zp_keys[]=       {'3','1','.','K','e','y',' ','d','e','f',':',' '};


//const char zp_pinsim1[]=       {'P','I','N',' ','S','I','M','1','-','>'};
//const char zp_pinsim2[]=       {'P','I','N',' ','S','I','M','2','-','>'};

const char zp_pinsim1[]=       {'3','2','.','P','I','N',' ','S','I','M','1','-','>'};
const char zp_pinsim2[]=       {'3','3','.','P','I','N',' ','S','I','M','2','-','>'};

const char zp_traffic[]=      {'3','4','.','T','r','a','f','f','i','c',' ',' ','O','U','T',':','I','N',' ','-','>',' '};


const unsigned char keys_def[]={'s','m','a','r','t','p','l','u'};


//const unsigned char def_apn[] = {'i','n','t','e','r','n','e','t','.','m','t','s','.','r','u',0x00};

const unsigned char def_apn[] = {'i','n','t','e','r','n','e','t','.','b','e','e','l','i','n','e','.','r','u',0x00};
const unsigned char def_usname[] = {0x00};   
const unsigned char def_uspsw[] = {0x00}; 

const unsigned char def_apnr[] = {'i','n','t','e','r','n','e','t','.','b','e','e','l','i','n','e','.','r','u',0x00};
//const unsigned char def_usnamer[] = {'b','e','e','l','i','n','e',0x00};   
//const unsigned char def_uspswr[] = {'b','e','e','l','i','n','e',0x00};
const unsigned char def_usnamer[] = {0x00};   
const unsigned char def_uspswr[] = {0x00};

const unsigned char def_apn1[] = {'i','n','t','e','r','n','e','t','.','m','t','s','.','r','u',0x00};
const unsigned char def_usname1[] = {0x00};   
const unsigned char def_uspsw1[] = {0x00}; 

const unsigned char def_apn2[] = {'i','n','t','e','r','n','e','t','.','b','e','e','l','i','n','e','.','r','u',0x00};
//const unsigned char def_usname2[] = {'b','e','e','l','i','n','e',0x00};   
//const unsigned char def_uspsw2[] = {'b','e','e','l','i','n','e',0x00}; 
const unsigned char def_usname2[] = {0x00};   
const unsigned char def_uspsw2[] = {0x00}; 


const unsigned char def_apn3[] = {'i','n','t','e','r','n','e','t',0x00};
const unsigned char def_usname3[] = {0x00};   
const unsigned char def_uspsw3[] = {0x00}; 

/*
const char zp_ts_dop[]= {'3','5','.','T','S','e',':'};
const char zp_tit[]= {'3','6','.','T','I','T',':'};
const char zp_tu[]= {'3','7','.','T','U','1','-','2',':',' '};
const char zp_tii1[]= {'3','8','.','T','I','I','1',':',' '};
const char zp_sel_mod[]= {'3','9','.','M','O','D','e',':',' '};
*/

unsigned char keys[10];

extern uint16_t crc_m1(unsigned char *ka,uint16_t num,uint16_t crc);
extern void delay(uint16_t period);

unsigned char index_pa;

extern unsigned char buf_tx_232[VOL_TX_PPP];
extern uint16_t vol_tx_ppp;

extern char emei[40];
extern uint16_t modbus_mem1[SEG1];

extern unsigned char cnt_tu1, cnt_tu2;

extern unsigned char Appl_seq_buf[MAX_BUF_SEQ]; 
unsigned char point_Tail;
extern unsigned char point_Head;





unsigned char ret_version(unsigned char unit,unsigned char *ptr)
{
 unsigned char i;
//char const *su;

 
 if(unit==2)
    {
      /*
    for (i=0;i<sizeof(ozu_vers);i++){*ptr=ozu_vers[i];ptr++;}
    *ptr=0;
    return(sizeof(ozu_vers)+1);
    */
    *ptr++='I';
      *ptr++='S';
      *ptr++='I';
      *ptr++='M';
      *ptr++=':';
      
  for (i=0;i<emei[0];i++){*ptr=emei[i+1];ptr++;}  // высвечивает emei симкки
    *ptr=0;
    return(emei[0]+6);
    }
 
 
  
    
    
  //progVersionString[40]  
   
  //num=0;  
 
 if(unit==0)
    {
    for (i=0;i<sizeof(versia);i++){*ptr=versia[i];ptr++;}
     *ptr=0;  
    return(sizeof(versia)+1);
  
    }
    else
    {
    
  for (i=0;i<sizeof(map_vers);i++){*ptr=map_vers[i];ptr++;}
  *ptr=0;
    return(sizeof(map_vers)+1);
    
    
      
    
    
    }
}






void vosstan_memory_no(void)
{
unsigned char i,buf[70];

/*
   j=sizeof(map_vers);
   for(i=0;i<j;i++)buf[i]=map_vers[i];
   *(uint16_t*)&buf[j]=crc_m1(&buf[0],j,0xffff);
   WrArrayToFlesh(A_VER_MAP, &buf[0],j+2,0,0);
*/

   
 //  for(i=0;i<L_KEYS;i++) keys[i]=keys_def[i];//загрузка ключей в массив ключей  
   for(i=0;i<(L_KEYS-2);i++)keys[i]=keys_def[i];
   *(uint16_t*)&keys[L_KEYS-2]=crc_m1(&keys[0],L_KEYS-2,0xffff);
   framWriteSim800(A_KEYS, &keys[0],L_KEYS);
   
  
    
    
    

   buf[OFS_PIN_SIM1]='.';
   buf[OFS_PIN_SIM1+1]='.';
   buf[OFS_PIN_SIM1+2]='.';
   buf[OFS_PIN_SIM1+3]='.';
   *(uint16_t*)&buf[OFS_KOD_SIM1_CRC]=crc_m1(&buf[0],L_KOD_SIM1-2,0xffff);
     framWriteSim800(A_KOD_SIM1, &buf[0],L_KOD_SIM1);
     

   buf[OFS_PIN_SIM2]='.';
   buf[OFS_PIN_SIM2+1]='.';
   buf[OFS_PIN_SIM2+2]='.';
   buf[OFS_PIN_SIM2+3]='.';
   *(uint16_t*)&buf[OFS_KOD_SIM2_CRC]=crc_m1(&buf[0],L_KOD_SIM2-2,0xffff);
     framWriteSim800(A_KOD_SIM2, &buf[0],L_KOD_SIM2);
     
     
     // базу контекстов не набивать
     
    /*  
     
     // apn1
     buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
     temp=sizeof(def_apn1)-1;   
     for(i=0;i<temp;i++)  *p++=def_apn1[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usname1)-1;
     for(i=0;i<temp;i++)  *p++=def_usname1[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspsw1)-1;
     for(i=0;i<temp;i++)  *p++=def_uspsw1[i];
     *p++=0;  // коней третьего фрагмента
     WrArrayToFlesh(A_C1_GPRS, &buf[0],L_C1_GPRS_MAX,0,0);
      

     // apn2
     buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
     temp=sizeof(def_apn2)-1;   
     for(i=0;i<temp;i++)  *p++=def_apn2[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usname2)-1;
     for(i=0;i<temp;i++)  *p++=def_usname2[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspsw2)-1;
     for(i=0;i<temp;i++)  *p++=def_uspsw2[i];
     *p++=0;  // коней третьего фрагмента
     WrArrayToFlesh(A_C2_GPRS, &buf[0],L_C2_GPRS_MAX,0,0);     
     

 // apn3
     buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
     temp=sizeof(def_apn3)-1;   
     for(i=0;i<temp;i++)  *p++=def_apn3[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usname3)-1;
     for(i=0;i<temp;i++)  *p++=def_usname3[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspsw3)-1;
     for(i=0;i<temp;i++)  *p++=def_uspsw3[i];
     *p++=0;  // коней третьего фрагмента
     WrArrayToFlesh(A_C3_GPRS, &buf[0],L_C3_GPRS_MAX,0,0);       
 
    
    
     
    // трафик не обнулять

     *(uint32_t*)&buf[0]=burst_ds_r();
     *(uint32_t*)&buf[4]=0;
     *(uint32_t*)&buf[8]=0;
     *(uint16_t*)&buf[L_TRAF-2]=crc_m1(&buf[0],L_TRAF-2,0xffff);
     WrArrayToFlesh(A_TRAF, &buf[0],L_TRAF,0,0);
     
     */
     
}





void vosstan_memory(void)
{
unsigned char i,buf[70],temp,*p;

/*
   j=sizeof(map_vers);
   for(i=0;i<j;i++)buf[i]=map_vers[i];
   *(uint16_t*)&buf[j]=crc_m1(&buf[0],j,0xffff);
   WrArrayToFlesh(A_VER_MAP, &buf[0],j+2,0,0);
*/

   
 //  for(i=0;i<L_KEYS;i++) keys[i]=keys_def[i];//загрузка ключей в массив ключей  
   for(i=0;i<(L_KEYS-2);i++)keys[i]=keys_def[i];
   *(uint16_t*)&keys[L_KEYS-2]=crc_m1(&keys[0],L_KEYS-2,0xffff);
   framWriteSim800(A_KEYS, &keys[0],L_KEYS);
   
   
   
  // фиксированная структура 0-39  40-49 50-59
  // [39\=0  [49]=0  [59]=0  это конец строки
   
      buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
   
     temp=sizeof(def_apn)-1;   
     for(i=0;i<temp;i++)  *p++=def_apn[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usname)-1;
     for(i=0;i<temp;i++)  *p++=def_usname[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspsw)-1;
     for(i=0;i<temp;i++)  *p++=def_uspsw[i];
     *p++=0;  // коней третьего фрагмента
     framWriteSim800(A_C_GPRS, &buf[0],L_C_GPRS_MAX);
   
     
     
     // для второй симк пока зарезервируем пусть читает что есть
     
     /*
      buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
     
     temp=sizeof(def_apnr)-1;   
     for(i=0;i<temp;i++)  *p++=def_apnr[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usnamer)-1;
     for(i=0;i<temp;i++)  *p++=def_usnamer[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspswr)-1;
     for(i=0;i<temp;i++)  *p++=def_uspswr[i];
     *p++=0;  // коней третьего фрагмента
     WrArrayToFlesh(A_CR_GPRS, &buf[0],L_CR_GPRS_MAX,0,0);
     */
 ////////////////////////////////////////////////////////////////////////
    
     
    
     
   
     /////////////!!!
     
     ///////////////////////////////////////////////////////framRead(A_NUM,&buf[OFS_NUM],2);   08 10 2018
     
   //  *(uint16_t*)&buf[OFS_NUM]=3569;
   //  framWriteSim800(A_NUM, &buf[OFS_NUM],2);
     
     buf[OFS_IP]=DEF_IP_SRV_0;
     buf[OFS_IP+1]=DEF_IP_SRV_1;
     buf[OFS_IP+2]=DEF_IP_SRV_2;
     buf[OFS_IP+3]=DEF_IP_SRV_3;
   *(uint16_t*)&buf[OFS_PORT]=DEF_UDP_PORT;//UDP порт
   *(uint16_t*)&buf[OFS_NUM]=3569;
   //*(uint16_t*)&buf[OFS_NUM]=*(uint16_t*)&buf[0];          //свой номер не менять !!!! 
   *(uint16_t*)&buf[OFS_IP_PAR_CRC]=crc_m1(&buf[0],L_IP_PAR-2,0xffff);
     framWriteSim800(A_IP_PAR, &buf[0],L_IP_PAR);
     
     
     
   *(uint16_t*)&buf[OFS_NAT]=DEF_CNTR_NAT;
   *(uint16_t*)&buf[OFS_CCH]=DEF_CNTR_LINK;
   *(uint16_t*)&buf[OFS_TM_CH]=DEF_TM_CNTR_LINK;
   
   *(uint16_t*)&buf[OFS_TM_NO_LINK]=DEF_TM_NO_LINK;       //dobavka
   *(uint16_t*)&buf[OFS_TM_LINK_RES]=DEF_TM_LINK_RES;       //dobavka
   
   *(uint16_t*)&buf[OFS_NAT_R]=DEF_CNTR_NAT_R;
   
   *(uint16_t*)&buf[OFS_C_PAR_CRC]=crc_m1(&buf[0],L_C_PAR-2,0xffff);
     framWriteSim800(A_C_PAR, &buf[0],L_C_PAR);
     
     
      buf[OFS_DES_SEQ]=(~DEF_DES_SEQ)&0x01; 
   *(uint16_t*)&buf[OFS_NUM_CL]=DEF_NUM_DST_SEQ;
   *(uint16_t*)&buf[OFS_TM_VZ]=DEF_VOL_TM_VZAT;
   *(uint16_t*)&buf[OFS_TM_CL]=DEF_TM_CNTR_CL;
    buf[OFS_TP_TS]=DEF_TP_TS1;
    buf[OFS_TP_TS+1]=DEF_TP_TS2;
   *(uint16_t*)&buf[OFS_SEQ_PAR_CRC]=crc_m1(&buf[0],L_SEQ_PAR-2,0xffff);
     framWriteSim800(A_SEQ_PAR, &buf[0],L_SEQ_PAR);
     
     
     /*
     
     buf[OFS_GPRS_DES]=~DEF_DES_PORT_GPRS; 
   *(uint32_t*)&buf[OFS_GPRS_S]=DEF_PORT_GPRS_S;
   *(uint16_t*)&buf[OFS_PORT_GPRS_CRC]=crc_m1(&buf[0],L_PORT_GPRS-2,0xffff);
     WrArrayToFlesh(A_PORT_GPRS, &buf[0],L_PORT_GPRS,0,0);
 */
    

   buf[OFS_PIN_SIM1]='.';
   buf[OFS_PIN_SIM1+1]='.';
   buf[OFS_PIN_SIM1+2]='.';
   buf[OFS_PIN_SIM1+3]='.';
   *(uint16_t*)&buf[OFS_KOD_SIM1_CRC]=crc_m1(&buf[0],L_KOD_SIM1-2,0xffff);
     framWriteSim800(A_KOD_SIM1, &buf[0],L_KOD_SIM1);
     

   buf[OFS_PIN_SIM2]='.';
   buf[OFS_PIN_SIM2+1]='.';
   buf[OFS_PIN_SIM2+2]='.';
   buf[OFS_PIN_SIM2+3]='.';
   *(uint16_t*)&buf[OFS_KOD_SIM2_CRC]=crc_m1(&buf[0],L_KOD_SIM2-2,0xffff);
     framWriteSim800(A_KOD_SIM2, &buf[0],L_KOD_SIM2);
     
     
     // базу контекстов не набивать
     
    /*  
     
     // apn1
     buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
     temp=sizeof(def_apn1)-1;   
     for(i=0;i<temp;i++)  *p++=def_apn1[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usname1)-1;
     for(i=0;i<temp;i++)  *p++=def_usname1[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspsw1)-1;
     for(i=0;i<temp;i++)  *p++=def_uspsw1[i];
     *p++=0;  // коней третьего фрагмента
     WrArrayToFlesh(A_C1_GPRS, &buf[0],L_C1_GPRS_MAX,0,0);
      

     // apn2
     buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
     temp=sizeof(def_apn2)-1;   
     for(i=0;i<temp;i++)  *p++=def_apn2[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usname2)-1;
     for(i=0;i<temp;i++)  *p++=def_usname2[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspsw2)-1;
     for(i=0;i<temp;i++)  *p++=def_uspsw2[i];
     *p++=0;  // коней третьего фрагмента
     WrArrayToFlesh(A_C2_GPRS, &buf[0],L_C2_GPRS_MAX,0,0);     
     

 // apn3
     buf[39]=0;
      buf[49]=0;
      buf[57]=0;
      p=&buf[0];
     temp=sizeof(def_apn3)-1;   
     for(i=0;i<temp;i++)  *p++=def_apn3[i];
     *p++=0;  // коней первого фрагмента
     temp=sizeof(def_usname3)-1;
     for(i=0;i<temp;i++)  *p++=def_usname3[i];
     *p++=0;  // коней второго фрагмента
      temp=sizeof(def_uspsw3)-1;
     for(i=0;i<temp;i++)  *p++=def_uspsw3[i];
     *p++=0;  // коней третьего фрагмента
     WrArrayToFlesh(A_C3_GPRS, &buf[0],L_C3_GPRS_MAX,0,0);       
 
    
    
     
    // трафик не обнулять

     *(uint32_t*)&buf[0]=burst_ds_r();
     *(uint32_t*)&buf[4]=0;
     *(uint32_t*)&buf[8]=0;
     *(uint16_t*)&buf[L_TRAF-2]=crc_m1(&buf[0],L_TRAF-2,0xffff);
     WrArrayToFlesh(A_TRAF, &buf[0],L_TRAF,0,0);
     
     */
     
}






/*

void load_par_first(void)
{
if (e_first_on==0xff)
      {
      __watchdog_reset();
      WrArrayToFlesh(BEG_BUF_LOG,0,(L_LOG*6),0x01,0x00);
      __watchdog_reset();    
      e_first_on=0;    
      }
 if(e_debug>4)e_debug=RG_WORK; //dobavka
 Regim=e_debug;
}

*/


unsigned char check_keys(void)
{
/*unsigned char i,buf[11];


RdFromFleshToArr(A_KEYS,&buf[0],L_KEYS);

if(*(uint16_t*)&buf[L_KEYS-2]==crc_m1(&buf[0],L_KEYS-2,0xffff))
 {
  for(i=0;i<L_KEYS;i++) keys[i]=buf[i];//загрузка ключей в массив ключей  
  return(0);
 }
 
else return(1);
*/

//////////////////////////////////////////////////////////////////////////////////////framRead(A_KEYS,&keys[0],L_KEYS); 08 10 2018
//if(*(uint16_t*)&keys[L_KEYS-2]!=crc_m1(&keys[0],L_KEYS-2,0xffff))return(1);
return(1);
}






//////


