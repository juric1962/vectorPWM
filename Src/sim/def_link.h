//св€зь

//#define UDP_PORT_H  0x13
//#define UDP_PORT_L  0x8a

//#define IP_SRV_0    81
//#define IP_SRV_1    22  
//#define IP_SRV_2    207
//#define IP_SRV_3    187 


//#define IP_SRV_0    88
//#define IP_SRV_1    82  
//#define IP_SRV_2    69
//#define IP_SRV_3    228 


#define MAX_VOL_US_NAME  12
#define MAX_VOL_PSW      12
#define MAX_VOL_INIT     35


//cоединение PPP

#define REG_MODEM_INI      0x00      //режимы
#define REG_PPP_LINCING    0x01
#define REG_IP_CONNECTED   0x02
                                 
                                  //уровни PPP  
#define LAYER_HW           0      //физический уровень                   
#define LAYER_LCP          1      //
#define LAYER_PAP          2      //                                   
#define LAYER_IPCP         3      //                                     
#define LAYER_IP           4      //
                                                                        
                                     
#define DL_PPP_IN_HI     0x05      //длина входного пакета со стороны сервера
#define DL_PPP_IN_LO     0xdc                                
                                     
                                      
#define MAX_DL_LCP      40
//#define MAX_DL_PAP      40 
#define MAX_DL_PAP      60
#define MAX_DL_IPCP     35 

                                        //протоколы                 
#define PR_LCP          0      //
#define PR_PAP          1      //                                   
#define PR_IPCP         2      //                                     
#define PR_IP           3      //
 
   

//опции PPP

#define MRU             1      //
#define ACCM            2      //                                   
#define AUT_PROT        3      //                                     
#define PFC             7
#define ACFC            8 

#define IP_SELF         3      //
#define IP_PRI_DNS      0x81      // 
#define IP_SEC_DNS      0x83      // 

// оды LCP,PAP,IPCP
#define CONF_REQ        1      //
#define CONF_ACK        2      //                                   
#define CONF_NAK        3      //                                     
#define CONF_REJ        4      //  
#define TERM_REQ        5
#define TERM_ACK        6
#define CODE_REJ        7

                                   
#define VOL_LCP_MT         2      //максимальное число Max Terminate       дл€ счетчика пакетов                       
#define VOL_LCP_MC         10      //максимальное число Max Configure 
#define VOL_LCP_MF         5      //максимальное число Max Failure 

                      
#define VOL_PAP_MC         5      //максимальное число Max Configure 


#define VOL_IPCP_MT         2      //максимальное число Max Terminate       дл€ счетчика пакетов                       
#define VOL_IPCP_MC         10      //максимальное число Max Configure 




#define VOL_PAP_TM_OUT     3000      //должно быть эквивалентно 3 секундам
#define VOL_LCP_TM_OUT     3000      //должно быть эквивалентно 3 секундам
#define VOL_IPCP_TM_OUT     3000      //должно быть эквивалентно 3 секундам



//#define VOL_RX_PPP         330
//#define VOL_TX_PPP         700



//#define VOL_RX_PPP_OVER         50
//#define KOL_BYTE_RX_OUT         15   



// состо€ни€, событи€ и действи€ автомата LCP
#define BEGIN      0x00 
#define START_ST   0x01                       //состо€ни€
#define CLOSE_ST   0x02                                      
#define STOP_ST    0x03
#define CLOSING_ST 0x04
#define OSTANOV    0x05
#define SEND_REQ   0x06
#define REC_ACK    0x07
#define SEND_ACK   0x08
#define OPEN_ST    0x09
                                                  
#define UP         0x00                           //событи€
#define DOWN       0x01                                      
#define OPEN       0x02
#define CLOSE      0x03
#define TO_PL      0x04
#define TO_MI      0x05
#define RCR_PL     0x06
#define RCR_MI     0x07
#define RCA        0x08                                    
#define RCN        0x09
#define RTR        0x0a
#define RTA        0x0b 
#define RUC        0x0c
#define RXJ_PL     0x0d
#define RXJ_MI     0x0e
#define RXR        0x0f 
#define NO_EVENT   0xff 

#define TLU         0x00                           //действи€
#define TLD         0x01                                      
#define TLS         0x02
#define TLF         0x03
#define IRC         0x04
#define ZRC         0x05
#define SCR         0x06
#define SCA         0x07
#define SCN         0x08                                    
#define STR         0x09
#define STA         0x0a
#define SCJ         0x0b 
#define SER         0x0c



#define PPPINITFCS16    0xffff  /* Initial FCS value */
#define PPPGOODFCS16    0xf0b8  /* Good final FCS value */



#define LCP_ACT_VOL   15   //длина буфера действий LCP     
#define IPCP_ACT_VOL  10   //длина буфера действий LCP

# define NOPPER 0

