#include "fram.h"
#include "main.h"

void i2cDelay() // Задержка на 500 нс
{
  uint8_t nCount=8;
  while(nCount--)
  {
  }
}

uint8_t i2cByteWrite(uint8_t byte)
{
  uint8_t i;
  for(i=0;i<8;i++)
  { 
    if(byte&(0x80>>i))forceSDA(1);
    else forceSDA(0);    
    i2cDelay();
    forceSCL(1);
    i2cDelay();        
    forceSCL(0);
  }  
  forceSDA(1);
  i2cDelay();
  forceSCL(1);
  i2cDelay();  
  if(readSDA)
  {
    forceSCL(0);return 1;
  }
  else{
    forceSCL(0);return 0;
  }
}

uint8_t i2cByteRead(void)
{
  uint8_t i;
  uint8_t byte=0;
  for(i=0;i<8;i++)
  {  
    i2cDelay();
    forceSCL(1);
    i2cDelay();  
    if(readSDA){byte|=0x80>>i;}
    forceSCL(0);
  }  
  return byte;
}

void icReset(void)
{
  uint16_t i;
  framWriteProtect(1); 
  forceSCL(0);  
  i2cDelay();
  forceSDA(1);
  i2cDelay();
  forceSCL(1);  
  i2cDelay();  
  forceSDA(0);
  i2cDelay();  
  forceSCL(0);
  i2cDelay();  
  
  for(i=0;i<7;i++)
  { 
    forceSDA(0);    
    i2cDelay();
    forceSCL(1);
    i2cDelay();        
    forceSCL(0);
  }  
  
  i2cDelay();
  forceSDA(0);
  i2cDelay();
  forceSCL(1);
  i2cDelay();
  forceSDA(1);
  i2cDelay();  
}

uint8_t framWrite(uint16_t addr, uint8_t* src, uint16_t bytesNum)
{
  uint8_t attempt;
  uint8_t err=0;  
  uint8_t tempByte;
  uint16_t k;
   
  if(addr>=framSize){return 2;}     
  
  for(attempt=0;attempt<3;attempt++)
  {  
      framWriteProtect(0); 
      
      forceSDA(0);
      i2cDelay();
      forceSCL(0);
      i2cDelay();
      
      tempByte=0xAE;//0xA0; - Теперь все линии адреса подтянуты к лог "1" (питанию)
      if(i2cByteWrite(tempByte)){err=1;icReset();continue;}          
      tempByte=addr/256;
      if(i2cByteWrite(tempByte)){err=1;icReset();continue;}                
      tempByte=addr%256;
      if(i2cByteWrite(tempByte)){err=1;icReset();continue;}          
          
      for(k=0;k<bytesNum;k++)
      {
         if(i2cByteWrite(src[k])){err=1;icReset();continue;}  
      }    
         
      forceSDA(0);
      i2cDelay();
      forceSCL(1);
      i2cDelay();
      forceSDA(1);
      i2cDelay();
      framWriteProtect(1);  
      err=0;
      break;
  }
  if(err)diag.framWriteErr++;
  return err;
}

uint8_t framRead(uint16_t addr, uint8_t* dst, uint16_t bytesNum)
{
  uint8_t attempt;
  uint8_t err=0;    
  uint8_t tempByte;
  uint16_t k;
  
  if(addr>=framSize){return 2;}       

  for(attempt=0;attempt<3;attempt++)
  {     
    framWriteProtect(0);
    
    for(k=0;k<bytesNum;k++)dst[k]=0;
    
    forceSDA(0);
    i2cDelay();
    forceSCL(0);
    i2cDelay();
    
    tempByte=0xAE; ;//0xA0; - Теперь все линии адреса подтянуты к лог "1" (питанию)
    if(i2cByteWrite(tempByte)){err=1;icReset();continue;}             
    tempByte=addr/256;
    if(i2cByteWrite(tempByte)){err=1;icReset();continue;}         
    tempByte=addr%256;
    if(i2cByteWrite(tempByte)){err=1;icReset();continue;}     
    
    forceSDA(1);
    i2cDelay();
    forceSCL(1);
    i2cDelay();
    forceSDA(0);
    i2cDelay();
    forceSCL(0);
    i2cDelay();
      
    tempByte=0xAF;//0xA1; - Теперь все линии адреса подтянуты к лог "1" (питанию)
    if(i2cByteWrite(tempByte)){err=1;icReset();continue;}
    framWriteProtect(1);
    
    if(bytesNum>1)
    { 
      bytesNum--;
      for(k=0;k<bytesNum;k++)
      {
        dst[k]=i2cByteRead();
        forceSDA(0);
        i2cDelay();
        forceSCL(1);
        i2cDelay();  
        forceSCL(0);    
        forceSDA(1);      
      }
      dst[k]=i2cByteRead();
      forceSDA(1);
      i2cDelay();
      forceSCL(1);
      i2cDelay();  
      forceSCL(0);        
    }else{
        dst[0]=i2cByteRead();
        forceSDA(1);
        i2cDelay();
        forceSCL(1);
        i2cDelay();  
        forceSCL(0); 
    }
     
    forceSDA(0);    
    i2cDelay();
    forceSCL(1);
    i2cDelay();
    forceSDA(1);
    err=0;
    break;
  }
  if(err)diag.framReadErr++;
  return err;
}
