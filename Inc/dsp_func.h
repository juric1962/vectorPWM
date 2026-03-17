#include "arm_math.h"

arm_status arm_fir_init_q15_r(
  arm_fir_instance_q15 * S,
  uint16_t numTaps,
  q15_t * pCoeffs,
  q15_t * pState,
  uint32_t blockSize);
  
void arm_fir_fast_q15_r(
  const arm_fir_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);  

  