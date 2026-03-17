#include "stdint.h"
#define ST_ALARM 0
#define ST_VZAT  1
#define ST_VZATIE 2

#define EV_TC_ALARM 0
#define EV_TC_NORMA 1
#define EV_TM_OUT 2

extern void monitor_change_state(void);
//void monitor_rst(void);
uint16_t read_from_buf(unsigned char *p_buf,uint16_t *ptr_in,uint16_t *ptr_out,uint16_t *ptr_out_kv,uint16_t *crc,unsigned char *p_data,uint16_t max_buf);
unsigned char write_to_buf(unsigned char *p_buf,uint16_t *ptr_in,uint16_t *ptr_out,uint16_t *ptr_out_kv,uint16_t *crc,unsigned char *p_data,uint16_t l_data, uint16_t max_buf);




