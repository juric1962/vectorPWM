//#define SEG1 35 //адреса с 0x150  до 0х0172 включительно

#define MB_VERSION 0x10

#define MB_IDENT   0x30
#define MB_CRC     0x36


#define RST_EN 0x0060

#define BEGIN_KD 0x0080

#define MB_TM_VHOD   BEGIN_KD

#define AD_TM_VHOD  MB_TM_VHOD-BEGIN_KD






//#define SEG1 86 //адреса с 0x150  до 0х01c0 включительно
#define SEG1 116

#define BEGIN_SEG1 0x150
//#define END_SEG1 BEGIN_SEG1+SEG1-1

#define SEGWR 16 //адреса с 0x150  включительно сегмент дл€ записи
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
#define MB_TII9 0x158
#define MB_TII10 0x159
#define MB_TII11 0x15a
#define MB_TII12 0x15b




#define MB_TIT1 0x160
#define MB_TIT2 0x161
#define MB_TIT3 0x162
#define MB_TIT4 0x163
#define MB_TIT5 0x164
#define MB_TIT6 0x165

#define MB_EN_REACTIV 0x166
//#define MB_TIT8 0x167

#define MB_TS 0x168

#define MB_TEMP 0x169
#define MB_MAX_TOK 0x16A          // максимальный ток за последние 10 минут
#define MB_MIN_TOK 0x16B           // миниимальный ток за последние 10 минут
#define MB_TIME_LEFT_POC 0x16C           // миниимальный ток за последние 10 минут

#define MB_TIME 0x16d


#define MB_IND_HEAP 0x170


#define MB_REA_DBG 0x176 // duint реактивна€
#define MB_REB_DBG 0x178 // duint реактивна€
#define MB_REC_DBG 0x17A // duint реактивна€


#define MB_IA 0x187 //значени€ в натуре
#define MB_IB 0x188
#define MB_IC 0x189
#define MB_UA 0x18a
#define MB_UB 0x18b
#define MB_UC 0x18c
#define MB_REFABC_DBG 0x18d // float реактивна€

#define MB_IND_HEAP_DINA 0x18f
#define MB_EN 0x190
#define MB_IND_USTAVOK 0x192     // вместо var_a_ind_ustavok

#define MB_IA_DBG 0x193  // секундные замеры
#define MB_IB_DBG 0x195
#define MB_IC_DBG 0x197
#define MB_UA_DBG 0x199
#define MB_UB_DBG 0x19b
#define MB_UC_DBG 0x19d

#define MB_RENA_DBG 0x19f //в инте реактивна€
#define MB_RENB_DBG 0x1a0 //в инте реактивна€
#define MB_RENC_DBG 0x1a1 //в инте рееактивна€а
#define MB_RENABC_DBG 0x1a2 //в инте реактивна€


#define MB_EN_DBG  0x1a3   // обща€ накопленна€ энерги€ во флоате

#define MB_ENFA_DBG 0x1a5 //в float
#define MB_ENFB_DBG 0x1a7 //в float
#define MB_ENFC_DBG 0x1a9 //в float
#define MB_ENFABC_DBG 0x1ab //в float

#define MB_FULLFA_DBG 0x1ad //в float
#define MB_FULLFB_DBG 0x1af //в float
#define MB_FULLFC_DBG 0x1b1 //в float
#define MB_FULLFABC_DBG 0x1b3 //float

#define MB_COSA_DBG 0x1b5 //float
#define MB_COSB_DBG 0x1b7 //float
#define MB_COSC_DBG 0x1b9 //float
#define MB_COSABC_DBG 0x1bb //float

#define MB_EN_PLUS    0x1bd //float
#define MB_EN_MINUS   0x1bf //float

#define MB_DINA_MAX    0x1c1 //uint  // 6.2 максимальное минимальное значение динамограммы в цикле качани€
#define MB_DINA_MIN    0x1c2 //uint
#define AD_DINA_MAX  MB_DINA_MAX-BEGIN_SEG1
#define AD_DINA_MIN  MB_DINA_MIN-BEGIN_SEG1



#define AD_TII1 MB_TII1-BEGIN_SEG1
#define AD_TII2 MB_TII2-BEGIN_SEG1
#define AD_TII3 MB_TII3-BEGIN_SEG1
#define AD_TII4 MB_TII4-BEGIN_SEG1
#define AD_TII5 MB_TII5-BEGIN_SEG1
#define AD_TII6 MB_TII6-BEGIN_SEG1
#define AD_TII7 MB_TII7-BEGIN_SEG1
#define AD_TII8 MB_TII8-BEGIN_SEG1
#define AD_TII9 MB_TII9-BEGIN_SEG1
#define AD_TII10 MB_TII10-BEGIN_SEG1
#define AD_TII11 MB_TII11-BEGIN_SEG1
#define AD_TII12 MB_TII12-BEGIN_SEG1

#define AD_TIT1 MB_TIT1-BEGIN_SEG1
#define AD_TIT2 MB_TIT2-BEGIN_SEG1
#define AD_TIT3 MB_TIT3-BEGIN_SEG1
#define AD_TIT4 MB_TIT4-BEGIN_SEG1
#define AD_TIT5 MB_TIT5-BEGIN_SEG1
#define AD_TIT6 MB_TIT6-BEGIN_SEG1
#define AD_TIT7 MB_TIT7-BEGIN_SEG1
#define AD_TIT8 MB_TIT8-BEGIN_SEG1



#define AD_TEMP MB_TEMP-BEGIN_SEG1
#define AD_MAX_TOK       MB_MAX_TOK-BEGIN_SEG1
#define AD_MIN_TOK       MB_MIN_TOK-BEGIN_SEG1
#define AD_TIME_LEFT_POC MB_TIME_LEFT_POC-BEGIN_SEG1

#define AD_TS MB_TS-BEGIN_SEG1

#define AD_IND_HEAP MB_IND_HEAP-BEGIN_SEG1
#define AD_IND_HEAP_DINA MB_IND_HEAP_DINA-BEGIN_SEG1


#define AD_TIME MB_TIME-BEGIN_SEG1

#define AD_IA MB_IA-BEGIN_SEG1
#define AD_IB MB_IB-BEGIN_SEG1
#define AD_IC MB_IC-BEGIN_SEG1 
#define AD_UA MB_UA-BEGIN_SEG1
#define AD_UB MB_UB-BEGIN_SEG1
#define AD_UC MB_UC-BEGIN_SEG1


#define AD_EN MB_EN-BEGIN_SEG1
#define AD_IND_USTAVOK MB_IND_USTAVOK-BEGIN_SEG1  // указатель архива изменений уставок


#define AD_IA_DBG MB_IA_DBG-BEGIN_SEG1
#define AD_IB_DBG MB_IB_DBG-BEGIN_SEG1
#define AD_IC_DBG MB_IC_DBG-BEGIN_SEG1
#define AD_UA_DBG MB_UA_DBG-BEGIN_SEG1
#define AD_UB_DBG MB_UB_DBG-BEGIN_SEG1
#define AD_UC_DBG MB_UC_DBG-BEGIN_SEG1

#define AD_RENA_DBG MB_RENA_DBG-BEGIN_SEG1    // это старые INT
#define AD_RENB_DBG MB_RENB_DBG-BEGIN_SEG1
#define AD_RENC_DBG MB_RENC_DBG-BEGIN_SEG1
#define AD_RENABC_DBG MB_RENABC_DBG-BEGIN_SEG1  //  это все надо херить

#define AD_EN_DBG MB_EN_DBG-BEGIN_SEG1

#define AD_REA_DBG MB_REA_DBG-BEGIN_SEG1             // это новые float
#define AD_REB_DBG MB_REB_DBG-BEGIN_SEG1
#define AD_REC_DBG MB_REC_DBG-BEGIN_SEG1
#define AD_REFABC_DBG MB_REFABC_DBG-BEGIN_SEG1  // это float


#define AD_ENFA_DBG MB_ENFA_DBG-BEGIN_SEG1
#define AD_ENFB_DBG MB_ENFB_DBG-BEGIN_SEG1
#define AD_ENFC_DBG  MB_ENFC_DBG-BEGIN_SEG1
#define AD_ENFABC_DBG MB_ENFABC_DBG-BEGIN_SEG1

#define AD_FULLFA_DBG MB_FULLFA_DBG-BEGIN_SEG1
#define AD_FULLFB_DBG MB_FULLFB_DBG-BEGIN_SEG1
#define AD_FULLFC_DBG MB_FULLFC_DBG-BEGIN_SEG1
#define AD_FULLFABC_DBG MB_FULLFABC_DBG-BEGIN_SEG1

#define AD_COSA_DBG MB_COSA_DBG-BEGIN_SEG1
#define AD_COSB_DBG MB_COSB_DBG-BEGIN_SEG1
#define AD_COSC_DBG MB_COSC_DBG-BEGIN_SEG1
#define AD_COSABC_DBG MB_COSABC_DBG-BEGIN_SEG1

#define AD_EN_PLUS    MB_EN_PLUS-BEGIN_SEG1
#define AD_EN_MINUS   MB_EN_MINUS-BEGIN_SEG1
#define AD_EN_REACTIV MB_EN_REACTIV-BEGIN_SEG1  // 166-167 регистр это энерги€ реактивна€

#define SEG2 4  //адреса с 0x200  до 0х0203 включительно
#define BEGIN_SEG2 0x200
#define END_SEG2 BEGIN_SEG2+SEG2-1

#define MB_TU1 0x200
#define MB_TU2 0x201
#define MB_TU1_IMP 0x202
#define MB_TU2_IMP 0x203

//#define AD_TU1     0x200-BEGIN_SEG2
//#define AD_TU2     0x201-BEGIN_SEG2
//#define AD_TU1_IMP 0x202-BEGIN_SEG2
//#define AD_TU2_IMP 0x203-BEGIN_SEG2


#define MB_WTGR 0x300

#define MB_DINO 0x370

#define MB_ARH_TII  0x400

#define MB_CONF_TC  0x430
#define MB_CONF_TIT 0x450
#define MB_CONF_TOK 0x470
#define MB_CONF_TORM  0x480
#define MB_CONF_PW  0x490

#define MB_CONF_SRIV 0x500
#define MB_CONF_PW2  0x590


#define MB_TCHMEM    0x600
#define MB_SHABLON   0x800
#define MB_KALENDAR  0x900


#define SEG4 6  //реальное врем€ адреса с 0x1000  до 0х1005 включительно
#define BEGIN_SEG4 0x1000
#define END_SEG4 BEGIN_SEG4+SEG4-1

#define SEGWR4 6 //дл€ записи
#define BEGIN_SEGWR4 0x1000
#define END_SEGWR4 BEGIN_SEGWR4+SEGWR4-1

#define MB_SEC   0x1000
#define MB_MIN   0x1001
#define MB_HR    0x1002
#define MB_DAY   0x1003
#define MB_MONTH 0x1004
#define MB_YEAR  0x1005


#define BEGIN_CAL 0xf000

#define MB_AIGAIN   BEGIN_CAL
#define MB_BIGAIN    BEGIN_CAL+0x01
#define MB_CIGAIN   BEGIN_CAL+0x02
#define MB_AVRMSGAIN   BEGIN_CAL+0x03
#define MB_BVRMSGAIN   BEGIN_CAL+0x04
#define MB_CVRMSGAIN   BEGIN_CAL+0x05
#define MB_AIRMSOS     BEGIN_CAL+0x06
#define MB_BIRMSOS     BEGIN_CAL+0x07
#define MB_CIRMSOS     BEGIN_CAL+0x08
#define MB_AVRMSOS     BEGIN_CAL+0x09
#define MB_BVRMSOS     BEGIN_CAL+0x0a
#define MB_CVRMSOS     BEGIN_CAL+0x0b
#define MB_APHCAL      BEGIN_CAL+0x0c
#define MB_BPHCAL      BEGIN_CAL+0x0d
#define MB_CPHCAL      BEGIN_CAL+0x0e
#define MB_AWG      BEGIN_CAL+0x0f
#define MB_BWG      BEGIN_CAL+0x10
#define MB_CWG      BEGIN_CAL+0x11
#define MB_AWAG      BEGIN_CAL+0x12
#define MB_BWAG      BEGIN_CAL+0x13
#define MB_CWAG      BEGIN_CAL+0x14
#define MB_AWATTOS      BEGIN_CAL+0x15
#define MB_BWATTOS      BEGIN_CAL+0x16
#define MB_CWATTOS      BEGIN_CAL+0x17
#define MB_APCFNUM      BEGIN_CAL+0x18
#define MB_APCFDEN      BEGIN_CAL+0x19
#define MB_CAL_CRC      BEGIN_CAL+0x1a

#define AD_AIGAIN  MB_AIGAIN-BEGIN_CAL
#define AD_BIGAIN  MB_BIGAIN-BEGIN_CAL
#define AD_CIGAIN  MB_CIGAIN-BEGIN_CAL
#define AD_AVRMSGAIN  MB_AVRMSGAIN-BEGIN_CAL
#define AD_BVRMSGAIN  MB_BVRMSGAIN-BEGIN_CAL
#define AD_CVRMSGAIN  MB_CVRMSGAIN-BEGIN_CAL
#define AD_AIRMSOS   MB_AIRMSOS-BEGIN_CAL
#define AD_BIRMSOS   MB_BIRMSOS-BEGIN_CAL
#define AD_CIRMSOS   MB_CIRMSOS-BEGIN_CAL 
#define AD_AVRMSOS   MB_AVRMSOS-BEGIN_CAL
#define AD_BVRMSOS   MB_BVRMSOS-BEGIN_CAL
#define AD_CVRMSOS   MB_CVRMSOS-BEGIN_CAL 
#define AD_APHCAL    MB_APHCAL-BEGIN_CAL
#define AD_BPHCAL    MB_BPHCAL-BEGIN_CAL
#define AD_CPHCAL    MB_CPHCAL-BEGIN_CAL
#define AD_AWG      MB_AWG-BEGIN_CAL
#define AD_BWG      MB_BWG-BEGIN_CAL  
#define AD_CWG      MB_CWG-BEGIN_CAL 
#define AD_AWAG     MB_AWAG-BEGIN_CAL
#define AD_BWAG     MB_BWAG-BEGIN_CAL
#define AD_CWAG     MB_CWAG-BEGIN_CAL
#define AD_AWATTOS  MB_AWATTOS-BEGIN_CAL
#define AD_BWATTOS  MB_BWATTOS-BEGIN_CAL
#define AD_CWATTOS  MB_CWATTOS-BEGIN_CAL 
#define AD_APCFNUM  MB_APCFNUM-BEGIN_CAL
#define AD_APCFDEN  MB_APCFDEN-BEGIN_CAL 
#define AD_CAL_CRC  MB_CAL_CRC-BEGIN_CAL

 struct struct_ts
{
 
  unsigned char tek_pin :1;
  unsigned char old_pin :1;
  unsigned char fl_ch_tc :1;
//  unsigned char real_tc:1;
  unsigned char count_tc;
};

