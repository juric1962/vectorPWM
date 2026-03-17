#include "ozu_map.h"
#include "def_prot.h"
#include <stdlib.h>
#include "stdint.h"

const unsigned char s1[] = {0,15,1,14,2,13,3,12,4,11,5,10,6,9,7,8};
const unsigned char s2[] = {1,9,2,10,3,11,4,12,5,13,6,14,7,15,8,0};
extern unsigned char keys[10];
extern unsigned char buf_tx_232[VOL_TX_PPP];
//##################################
  //##################################  функции кодировки
   unsigned char shl (unsigned char n)
    {
     unsigned char n1,count;
     for(count=0;count<3;count++)
        {
         n1=n&(0x80);
         n1=n1>>7;
         n=n<<1; 
         n=n|n1; 
        }
     return n; 
    } 
   
   
   
    unsigned char zamena (unsigned char i)
    {
     unsigned char i1,i2;
     i1=i&0x0f;
     i2=i&0xf0;
     i2=i2>>4;
     i1=s1[i1];
     i2=s2[i2];
     i2=i2<<4;
     i=0;
     i=i|i1;
     i=i|i2; 
     return i; 
    }
    
 
   
     void  transform_buf(unsigned char *p,uint16_t kol_byte,uint16_t s_rand)
    { 
    unsigned char N1,N2,N3,N4;  
    unsigned char C1,C2;
    union
     {
      unsigned char bytes[2]; 
      uint16_t word;
     }S;
    unsigned char N,Z; 
    unsigned char i,j;
    uint16_t count_byte;          //kol_byte - четное число
    
    C1=0xa3;
    C2=0x23;
    S.word=s_rand;
    N1=S.bytes[0]; 
    N2=S.bytes[1];
    count_byte=0;
    while(count_byte<=kol_byte-1)
       {       
         for(j=0;j<3;j++)
            for(i=0;i<8;i++)
            {
            N=N1+keys[i];
            Z=zamena(N);
            N=shl(Z);
            Z=N^N2;
            N2=N1;
            N1=Z;
            }

         for(i=0;i<8;i++)
            {
             N=N1+keys[7-i];
            Z=zamena(N);
            N=shl(Z);
            Z=N^N2;
            N2=N1;
            N1=Z;
            }
           N=N1+keys[0]; 
           Z=zamena(N);
           N=shl(Z);
           Z=N^N2;
           N2=Z;
           N3=N1;
           N4=N2;
           N1=N3+C2;
           
           N2=(N4+C1)%(0xff);
           
           *p=*p^N2; 
            p++;
           
           if((count_byte+1)<=(kol_byte-1))
           {
           *p=*p^N1;  
            p++;
           count_byte=count_byte+2; 
           } 
           else count_byte++;
            
           
           
       }//while
    
    } 
    
   
    void kodirovka(uint16_t kol_send_byte)
    {
     
   
    uint16_t kod;
    kod=rand();
    transform_buf(&buf_tx_232[TR_ID],kol_send_byte-7,kod);  
    *(uint16_t*)&buf_tx_232[TR_KOD]=kod;
    }
    
  
  //##################################
  //################################## 
