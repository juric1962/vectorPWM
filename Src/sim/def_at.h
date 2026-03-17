
#define VOL_LIST  14
#define LN_BUF_AT 40
//комманды

#define cAT         0
#define cATE0       1
#define cATV0       2
#define cATE0V0     3
#define cATIPR      4
#define cATIFC      5
#define cATC        6 
#define cATD        7     
#define cATW        8
#define cATCREG     9
#define cATCGATT    10
#define cATCGDCONT  11
#define cATDGPRS    12
#define cATH        13
#define cATCGATT0   14
#define PLUS        15
#define PAUSA1      16
#define PAUSA2      17
#define PAUSA3      18
#define PAUSA4      19
#define cATSGAUTH   20
#define cATCGREG    21
#define PAUSA_REG   22
#define cATCPIN     23
#define cATCREG0    24
#define PAUSA5      25
#define cATCIMI     26           // код SIM карты


#define S9600        0
#define S19200       1
#define S57600       2
#define S115200      3


#define KOD_OK          0x30
#define KOD_CONNECT     0x31
#define KOD_ERR         0x34

enum t_event_modem {EVM_NONE=0,EVM_TM1,EVM_AT_OK,EVM_AT_ERR,EVM_PPP_OK,EVM_PPP_ERR,EVM_DCD_ERR,EVM_MS_LMT,EVM_RDC,EVM_CTS_ERR,EVM_DCP};
enum bool   {FALSE,TRUE};  



