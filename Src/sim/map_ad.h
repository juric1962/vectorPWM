
// адреса
#define  FIRST_ON      0x00          

#define  A_RW_PDP      0x01

#define  A_RW_PDP_R    0x02

#define  A_DEBUG       0x03


#define ADR_NUM_SELF           0x12 //2 байта
#define ADR_CNTR_LINK          0x14 //2 байта
#define ADR_CNTR_LINK_WAITS    0x16 //2 байта
#define ADR_CNTR_VOL_TRY               0x08 // char
#define ADR_CNTR_NAT           0x1a //2 байта
#define ADR_NUM_DST_SEQ        0x1c //2 байта

#define ADR_VOL_TM_VZAT        0x1e //2 байта

#define  ADR_IP_SRV  (0x19a)  //0x19a,0x19b,0x19c,0x19d                        

//#define  ADR_UDP_PORT_HI (0x19e)
//#define  ADR_UDP_PORT_LO (0x19f)

#define ADR_US_NAME    (0x1b0)  //15+ 1байт(0x0d)
#define ADR_PSW        (0x1c0)       //15+1байт(0x0d)
#define ADR_INIT       (0x1d0)       //47+1байт(0x0d)


