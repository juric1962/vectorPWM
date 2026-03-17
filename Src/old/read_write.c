//#include <iom1280.h> 
//#include "dfpin.h"
//#include "dfproc.h"
#include <string.h> 
//#include <inavr.h>
//#include "dfcnst.h"
#include <stdio.h>
#include <stdint.h>
#include "modbus.h"
#include "map_mbus.h"
#include "utils.h"

uint8_t inBuffer[MAX_LEN_BUFFER], outBuffer[MAX_LEN_BUFFER]; 
uint8_t WriteStack[MAX_WRITE_STACK][MIN_LEN_BUFFER];
char CountWriteStack; //количество пакетов для записи;
char IndexWriteStack; // текущии индекс в стеке

char read_modbus(char direction,unsigned int start_madres, char num_word, uint16_t *result)     // либо 1 слово либо 2 слова ( float  long int)
// 0- чтение в result лежат результат чтения
//  1= запись в result лежат данные для записи
//  в массиве данные лежат как есть ( младшие - старшие
// я их так и возвращаю без перевертышей
// возвращает 0 если значение не найдено 1 -если найдено , результат в result

{
 
 uint16_t byteSize;
 int i;

 outBuffer[1] = (uint8_t)(start_madres >> 8); 
 outBuffer[2] = (uint8_t)(start_madres & 0xFF);  
 outBuffer[3] = 0; 
 outBuffer[4] = (uint8_t)num_word;  
 
 switch (direction)
 {
    case READ_DATA: 
      outBuffer[0] = 3;
      if (modbusRegistersEdit(outBuffer, inBuffer, 1, &byteSize) != 0)
          return 0;
      break;
    case WRITE_DATA: 
      outBuffer[0] = 16;
      outBuffer[5] = (uint8_t)(num_word * 2);
      for (i = 0; i < num_word; i++)
      {
         outBuffer[6 + i * 2] = (uint8_t)(result[i] >> 8); 
         outBuffer[7 + i * 2] = (uint8_t)(result[i] & 0xFF); 
      };
      
      if (CountWriteStack > MAX_WRITE_STACK)
        return 1;
      for (i = 0; i < MIN_LEN_BUFFER; i++)
          WriteStack[CountWriteStack][i] = outBuffer[i];
      CountWriteStack++;
      return 1;
    default: 
      return 0;
 };
   
 switch (inBuffer[1])
 {
    case 3:
      if (inBuffer[2] == 2)
        result[0] = inBuffer[3] * 256 + inBuffer[4];
      else
        if (inBuffer[2] == 4)
        {
            result[0] = inBuffer[3] * 256 + inBuffer[4];
            result[1] = inBuffer[5] * 256 + inBuffer[6];
        }
        else
          return 0;
      break;
    case 16:
      if (inBuffer[1] != outBuffer[0])
        return 0;
      if (inBuffer[2] != outBuffer[1])
        return 0;
      if (inBuffer[3] != outBuffer[2])
        return 0;                      
      if (inBuffer[4] != outBuffer[3])
        return 0;
      if (inBuffer[5] != outBuffer[4])
        return 0;                                            
   
      break;
    default:
      return 0;
 };
 return 1;
 }
