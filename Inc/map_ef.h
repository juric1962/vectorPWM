// данные для SIM800 лежат в области начиная с 32000 !!!!!
// пока 431 байт


#define A_C_GPRS  32000                         //framSize 32768
#define L_C_GPRS_MAX 58//max длина области
//#define A_C_GPRS_CRC A_C_GPRS+L_C_GPRS_MAX-2


#define A_CR_GPRS  (A_C_GPRS+L_C_GPRS_MAX)
#define L_CR_GPRS_MAX 58//max длина области
//#define A_CR_GPRS_CRC A_CR_GPRS+L_CR_GPRS_MAX-2


#define A_IP_PAR  (A_CR_GPRS+L_CR_GPRS_MAX)
#define L_IP_PAR 10
//карта внутри области
      #define OFS_IP          0
      #define A_IP           ( A_IP_PAR+OFS_IP)
      #define OFS_PORT        4
      #define A_PORT          (A_IP_PAR+OFS_PORT)
      #define OFS_NUM         6
      #define A_NUM           (A_IP_PAR+OFS_NUM)
      #define OFS_IP_PAR_CRC  8   
      #define A_IP_PAR_CRC    (A_IP_PAR+OFS_IP_PAR_CRC)





#define A_C_PAR  (A_IP_PAR+L_IP_PAR)
#define L_C_PAR  14                                      //!!!!!!!!!!!!!!!!dobavka
//карта внутри области
      #define OFS_NAT          0
      #define A_NAT            (A_C_PAR+OFS_NAT)
      #define OFS_CCH          2     
      #define A_CCH            (A_C_PAR+OFS_CCH)
      #define OFS_TM_CH        4   
      #define A_TM_CH          (A_C_PAR+OFS_TM_CH)
     
      #define OFS_TM_NO_LINK   6                         //!!!!!!!!!!!!!!!!dobavka
      #define A_TM_NO_LINK     (A_C_PAR+OFS_TM_NO_LINK)
      #define OFS_TM_LINK_RES  8   
      #define A_TM_LINK_RES    (A_C_PAR+OFS_TM_LINK_RES)   //EEEEEEEEEEEEEEE dobavka

      #define OFS_NAT_R        10
      #define A_NAT_R          (A_C_PAR+OFS_NAT_R)

      #define OFS_C_PAR_CRC    12                          //!!!!!!!!!!!!!!!!dobavka
      #define A_C_PAR_CRC      (A_C_PAR+OFS_C_PAR_CRC)


/*
#define A_SEQ_PAR  A_C_PAR+L_C_PAR
#define L_SEQ_PAR  11 
//карта внутри области
      #define OFS_DES_SEQ        0
      #define A_DES_SEQ           A_SEQ_PAR+OFS_DES_SEQ
      #define OFS_NUM_CL         1
      #define A_NUM_CL           A_SEQ_PAR+OFS_NUM_CL
      #define OFS_TM_VZ          3
      #define A_TM_VZ            A_SEQ_PAR+OFS_TM_VZ
      #define OFS_TM_CL          5
      #define A_TM_CL            A_SEQ_PAR+OFS_TM_CL 
      #define OFS_TP_TS          7
      #define A_TP_TS            A_SEQ_PAR+OFS_TP_TS
      #define OFS_SEQ_PAR_CRC    9
      #define A_SEQ_PAR_CRC      A_SEQ_PAR+OFS_SEQ_PAR_CRC
*/ 

#define A_SEQ_PAR_OLD  (A_C_PAR+L_C_PAR)              // чтобы не менять раскладку памяти по остальным структурам 
#define L_SEQ_PAR_OLD  11 

  
#define A_KOD_SIM1  (A_SEQ_PAR_OLD + L_SEQ_PAR_OLD)
#define L_KOD_SIM1  6
//карта внутри области
      #define OFS_PIN_SIM1       0
      #define A_PIN_SIM1         (A_KOD_SIM1+OFS_PIN_SIM1)
      #define OFS_KOD_SIM1_CRC   4
      #define A_KOD_SIM1_CRC     (A_PIN_SIM1+OFS_KOD_SIM1_CRC)


#define A_KOD_SIM2  (A_KOD_SIM1+L_KOD_SIM1)
#define L_KOD_SIM2  6
//карта внутри области
      #define OFS_PIN_SIM2       0
      #define A_PIN_SIM2         (A_KOD_SIM2+OFS_PIN_SIM2)
      #define OFS_KOD_SIM2_CRC   4
      #define A_KOD_SIM2_CRC     (A_PIN_SIM2+OFS_KOD_SIM2_CRC)





#define BEG_BUF_LOG (A_KOD_SIM2 + L_KOD_SIM2)
#define L_LOG 40
#define L_BUF_LOG (40*6)




//трафик
#define A_TRAF  (BEG_BUF_LOG + L_BUF_LOG)
#define L_TRAF  14       // длина области без CRC
#define A_TRAF_CRC (A_TRAF+L_TRAF-2)



//Ключи шифрования

#define A_KEYS  (A_TRAF + L_TRAF)
#define L_KEYS  10       // длина области без CRC
#define A_KEYS_CRC (A_KEYS+L_KEYS-2)


// перенесем область параметров для асинхронных архивов

#define A_SEQ_PAR  (A_KEYS + L_KEYS)
#define L_SEQ_PAR  13 
//карта внутри области
      #define OFS_DES_SEQ        0
      #define A_DES_SEQ           (A_SEQ_PAR+OFS_DES_SEQ)
      #define OFS_NUM_CL         1
      #define A_NUM_CL           (A_SEQ_PAR+OFS_NUM_CL)
      #define OFS_TM_VZ          3
      #define A_TM_VZ            (A_SEQ_PAR+OFS_TM_VZ)
      #define OFS_TM_CL          5
      #define A_TM_CL            (A_SEQ_PAR+OFS_TM_CL) 
      #define OFS_TP_TS          7
      #define A_TP_TS            (A_SEQ_PAR+OFS_TP_TS)
// новые елементы индекс последнего переданного архива
      #define OFS_A_IND_DNS_KVIT 9
      #define A_IND_DNS_KVIT     (A_SEQ_PAR + OFS_A_IND_DNS_KVIT)
      #define OFS_SEQ_PAR_CRC    11
      #define A_SEQ_PAR_CRC      (A_SEQ_PAR+OFS_SEQ_PAR_CRC)












