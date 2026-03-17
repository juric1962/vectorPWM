#define SEG1 37 //адреса с 0x150  до 0х0172 включительно
#define BEGIN_SEG1 0x150
#define END_SEG1 BEGIN_SEG1+SEG1-1

#define SEGWR 16 //адреса с 0x150  включительно сегмент для записи
#define BEGIN_SEGWR 0x150
#define END_SEGWR BEGIN_SEGWR+SEGWR-1


#define MB_TII1 0x150
#define MB_TII2 0x151
#define MB_TII3 0x152
#define MB_TII4 0x153
#define MB_TII5 0x154
#define MB_TII6 0x155
#define MB_TII7 0x156
#define MB_TII8 0x157

#define MB_TII15 0x15e
#define MB_TII16 0x15f

#define MB_TIT1 0x160
#define MB_TIT2 0x161

#define MB_TEMP 0x163

#define MB_TS 0x168

#define MB_TIME 0x16d  // 2 регистра

#define MB_IND_GZU 0x16f
#define MB_IND_DNS 0x170
#define MB_SOST_PSM  0x171  // код состояния автомата ПСМ
#define MB_TIMER_PSM  0x172  // двойное слово время до переключения

#define AD_TII1 MB_TII1-BEGIN_SEG1
#define AD_TII2 MB_TII2-BEGIN_SEG1
#define AD_TII3 MB_TII3-BEGIN_SEG1
#define AD_TII4 MB_TII4-BEGIN_SEG1
#define AD_TII5 MB_TII5-BEGIN_SEG1
#define AD_TII6 MB_TII6-BEGIN_SEG1
#define AD_TII7 MB_TII7-BEGIN_SEG1
#define AD_TII8 MB_TII8-BEGIN_SEG1

#define AD_TIT1 MB_TIT1-BEGIN_SEG1
#define AD_TIT2 MB_TIT2-BEGIN_SEG1

#define AD_TEMP MB_TEMP-BEGIN_SEG1

#define AD_TS MB_TS-BEGIN_SEG1
#define AD_SOST_PSM MB_SOST_PSM-BEGIN_SEG1
#define AD_TIMER_PSM MB_TIMER_PSM-BEGIN_SEG1

#define AD_TIME MB_TIME-BEGIN_SEG1

#define AD_IND_DNS MB_IND_DNS-BEGIN_SEG1
#define AD_IND_GZU MB_IND_GZU-BEGIN_SEG1



#define SEG2 4  //адреса с 0x200  до 0х0203 включительно
#define BEGIN_SEG2 0x200
#define END_SEG2 BEGIN_SEG2+SEG2-1

#define MB_TU1 0x200
#define MB_TU2 0x201
#define MB_TU1_IMP 0x202
#define MB_TU2_IMP 0x203

#define MB_COM_DISP 0x204

//#define AD_TU1     0x200-BEGIN_SEG2
//#define AD_TU2     0x201-BEGIN_SEG2
//#define AD_TU1_IMP 0x202-BEGIN_SEG2
//#define AD_TU2_IMP 0x203-BEGIN_SEG2

//конфигурация автономной работы архива ДНС

#define SEG3 34  //адреса с 0x400  до 0х0422 включительно   2 ТИТ
#define BEGIN_SEG3 0x400
#define END_SEG3 BEGIN_SEG3+SEG3-1

#define SEGWR3 34 //для записи
#define BEGIN_SEGWR3 0x400
#define END_SEGWR3 BEGIN_SEGWR3+SEGWR3-1

#define MB_TM_OTV1 0x400
#define MB_TM_OTV2 0x401
#define MB_TM_OTV3 0x402
#define MB_TM_OTV4 0x403
#define MB_TM_OTV5 0x404
#define MB_TM_OTV6 0x405
#define MB_TM_OTV7 0x406
#define MB_TM_OTV8 0x407
#define MB_TM_OTV9 0x408
#define MB_TM_OTV10 0x409
#define MB_TM_OTV11 0x40a
#define MB_TM_OTV12 0x40b
#define MB_TM_OTV13 0x40c
#define MB_TM_OTV14 0x40d
#define MB_TM_OTV15 0x40e
#define MB_TM_OTV16 0x40f
#define MB_TM_HR    0x410
#define MB_TM_MIN   0x411
#define MB_TM_SEC   0x412
#define MB_ARH_FL   0x413



#define SEG4 6  //реальное время адреса с 0x1000  до 0х1005 включительно
#define BEGIN_SEG4 0x1000
#define END_SEG4 BEGIN_SEG4+SEG4-1

#define SEGWR4 6 //для записи
#define BEGIN_SEGWR4 0x1000
#define END_SEGWR4 BEGIN_SEGWR4+SEGWR4-1







#define MB_SEC   0x1000
#define MB_MIN   0x1001
#define MB_HR    0x1002
#define MB_DAY   0x1003
#define MB_MONTH 0x1004
#define MB_YEAR  0x1005





#define SEG5 700  //архив ДНС адреса с 0x6000
#define BEGIN_SEG5 0x6000
#define END_SEG5 BEGIN_SEG5+SEG5-1


#define SEG55 330  //архив ГЗУ адреса с 0x2000
#define BEGIN_SEG55 0x2000
#define END_SEG55 BEGIN_SEG55+SEG55-1


//база ключей
#define SEG6 510 
#define BEGIN_SEG6 0x700
#define END_SEG6 BEGIN_SEG6+SEG6-1

#define SEGWR6 510 //для записи
#define BEGIN_SEGWR6 0x700
#define END_SEGWR6 BEGIN_SEGWR6+SEGWR6-1


#define SEG7 8  //идентификатор и сумма
#define BEGIN_SEG7 0x30
#define END_SEG7 BEGIN_SEG7+SEG7-1


//конф ТС
#define SEG8 8+8+8  
#define BEGIN_SEG8 0x430
#define END_SEG8 BEGIN_SEG8+SEG8-1

#define SEGWR8 8+8+8 //для записи                    // вводим алгоритм алармов для ТИТ   
#define BEGIN_SEGWR8 0x430
#define END_SEGWR8 BEGIN_SEGWR8+SEGWR8-1

//время выхода
#define SEG9 1  
#define BEGIN_SEG9 0x80
#define END_SEG9 BEGIN_SEG9+SEG9-1

#define SEGWR9 1 //для записи
#define BEGIN_SEGWR9 0x80
#define END_SEGWR9 BEGIN_SEGWR9+SEGWR9-1

#define SEGWR19 1 //для записи переключения до уса
#define BEGIN_SEGWR19 0x600
#define END_SEGWR19 BEGIN_SEGWR19+SEGWR19-1

//параметры ГЗУ
#define SEG99 20  
#define BEGIN_SEG99 0x500
#define END_SEG99 BEGIN_SEG99+SEG99-1

#define SEGWR99 1 //для записи
#define BEGIN_SEGWR99 0x500
#define END_SEGWR99 BEGIN_SEGWR99+SEGWR99-1

