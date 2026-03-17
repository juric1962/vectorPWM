#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>
#include "string.h"
#include "InitParam.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define _CRT_NO_SECURE_WARNINGS

#define READ_DATA 0
#define WRITE_DATA 1

#define PAGE_OK  100
#define PAGE_MAIN  101
#define PAGE_MAIN_MENU  102
#define PAGE_MENU  103
#define PAGE_MENU_1  104
#define PAGE_ERROR 105
#define PAGE_ERROR_500 106
#define PAGE_MENU_ARHIVE  107

#define MAX_VALUE 3 //

#define MAX_HTML_ITEM 50

#define STEP_CIPSEND  1
#define STEP_WAIT_OK 2
#define STEP_HTML 3
#define STEP_WAIT_SEND_OK 4
#define STEP_CLOSE 5

#define MAX_WRITE_STACK 20 //количество пакетов для записи в модбас
#define MAX_LEN_BUFFER 2048  // длина пакета
#define MIN_LEN_BUFFER 10
// Типы регистров
#define INT 0
#define UINT 1
#define DINT 2
#define UDINT 3
#define REAL 4
#define STR 5

#define WRITE_PORT 1
#define NOT_WRITE_PORT 0

// Для динамограмм
#define D_ONE_LINE 1
#define D_TWO_LINE 2
#define D_POINT 3
#define D_POINT_STAR 4

#define T_INT 1
#define T_UINT 2
#define T_DINT 3
#define T_UDINT 4
#define T_REAL 5

// Максимальная количество слов в ответе модбас для 4 команды (STR)
#define MAX_COUNT_WORD 20

// Максимальная длина html(только данные, без заголовка) пакета в байтах
#define MAX_SIZE 2048

/*
// в файле section_size.h должны быть описаны здесь закоментить
// Количество групп параметров для записи
#define MAX_ARRSECTION 36
// Количество групп параметров для чтения
#define MAX_ARRSECTION_1 20

// Количество параметров для записи
#define MAX_PARAM_ITEM 338
// Количество параметров для чтения
#define MAX_PARAM_ITEM_1 106
// Количество архивов на странице
*/

//Количество групп параметров для архивов
#define MAX_ARRSECTION_2 3

#define MAX_ARHIVE_STR 5
// Максимальное число символов в html пакете создаваемый web сервером(Пусть пока 12 кб)
#define MAX_SIZE_HTML_PACKET 12288
// Максимальное число символов в html пакете создаваемый web сервером
// Больше 2 кб порт не пропускает
#define MAX_SIZE_ONE_HTML_PACKET 2000
// Миксимальная длина названия секции
#define SECTION_SIZE 70

#define ADRES_ARHIVE_INDEX 0x3e7 //0x170
#define ADRES_ARHIVE 0x3e8 //0x6000
#define MAX_ARHIVE_STACK 200
#define NAME_POST "r_"

enum bool {false, true};

typedef union{
    float Value;
    uint16_t Word[2];
} TFloat;

typedef union {
    int32_t Value;
    uint16_t Word[2];
} TInteger;

typedef union
{
    uint32_t Value;
    uint16_t Word[2];
} TDWord;

typedef struct
{
    char Name[100];
    unsigned int Adres;
    char TypeValue;
    char BitFieldStart;
    uint16_t BitFieldCount;
    enum bool isBitField;
    unsigned char SectionIndex;
}  TParamItem;

typedef struct
{
    char Chanel;
    char ID;
    char Step;
    
}  ThtmlItem;

typedef union {
    char buf[sizeof(TParamItem)];
    TParamItem Pitem;
} TCharToParamItem;

// функция возвращает позицию первого вхождение подстроки substr в строк str начиная с позиции idx
// возвращает -1 если не найдено
int Pos(char *substr, char *str, int idx);
// возвращает true если находит подстроку в строке
enum bool CheskWordInString(char *substr, char *str);
int GetAnswerID(char *str);
// Возвращает номер канала изhtml запроса IPD+
int FindChanel(char *str);
// Удаляет элемент с индексом aIndex из массива htmlList
void DeleteHtmlItem(int aIndex);
// Удаляет все элементы массива htmlList у кторых номер канала совпадает с номером канала наеденным после CLOSED, в hml запросе
void DeleteItemOfChanel(char * str);
// Добавляет элемент в массив запросов для создания html страницы
void AddHtmlItemInList(ThtmlItem aItem);
// Обрабатывает пришедший из порта html запрос от браузера, и создает ответную страницу
char CreateHtmlItem(char *inBuf, unsigned int *aSize, unsigned int aSizeIn);
char DoSendAT(int aID, int aAnswerID, char *inBuf, unsigned int *aSize);
char CreateMenuForBrowser();
char CreateMenuForBrowser_1();
char CreateMenuForBrowser_2();

// страница с параметрами для браузера для записи
char CreatPageForBrowser(int aIndex);
// страница с параметрами для браузера для чтения
char CreatPageForBrowser_1(int aIndex);
char CreatPageForBrowserDyn(int aIndex);
char CreatPageForBrowser_2(int aIndex);
// Добавляет заголовок
void AddHead(char *aString, char a);
void CreateHead(char *aString);
void DoHTML(int aAnswerID, char *inBuf, unsigned int *aSize);
void ShiftToRight(char *aString, int aCount);
void s_port(unsigned char ch);
// Вывод на экран
void mov_s( char  *p, int size);

unsigned char *copy_P(char *dest,const char *src);
TParamItem GetParam(const char *src);
//возврвщает размер параметра в словах
char GetCountWord(void);
// 0- чтение в result лежат результат чтения
//  1= запись в result лежат данные для записи
//  в массиве данные лежат как есть ( младшие - старшие
// я их так и возвращаю без перевертышей
// возвращает 0 если значение не найдено 1 -если найдено , результат в result
char read_modbus(char direction,unsigned int start_madres, char num_word, uint16_t *result);     // либо 1 слово либо 2 слова ( float  long int)
unsigned int GetMaskInt(char aBitFieldStart, char aBitFieldCount);
unsigned long int GetMaskWord(char aBitFieldStart, char aBitFieldCount);
char WriteDataToMemory(char *aValStr);
char ReadDataFromMemory(char *aValStr);
uint16_t ReadDataFromArhive(char *aValStr, int aDirect, char *qwality);
// Функция для StrToFloat(char *aString);
float ExtractFloat(char *aString, int aIndex, int aSize);
float StrToFloat(char *aString);
char FloatToStr(float aValue, char *aString);
void reset_ESP(void);
void ClearHTMLString(void);
void CopyCIPSEND(int aData1, int aData2, char *inBuf);
// преобразует строковое неотрицательное число в цифры
// Если это не цыфра возвращает -1
int StrToInt(char *str);
int GetLastIndexArhive(void);
void convert_unix_2_date( unsigned long int time_sec);
void decodArhiv(int aCode);
float GetMaxY(void);
void DrawLine(uint16_t maxX, uint16_t maxY);
char CreatPageForBrowserWat(int aIndex);
char CreatPageForBrowserDyn(int aIndex);
#endif // UTILS_H

void decodArhiv(int aCode);
extern uint16_t ValData[MAX_COUNT_WORD];
extern const char html_br[];
extern char HTMLString[MAX_SIZE_HTML_PACKET];
extern TDWord val_DWord;