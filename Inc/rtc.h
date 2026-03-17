#include "stm32f4xx_rtc.h"

extern RTC_TimeTypeDef RTC_TimeStructure;
extern RTC_DateTypeDef RTC_DateStructure; 

void RTC_Initialization(void);
void RTC_Config(RTC_DateTypeDef RTC_DateStructure, RTC_TimeTypeDef RTC_TimeStructure);
void dataTimeRenew(uint16_t msTicksCurrent);

void fromUnix(RTC_DateTypeDef *RTC_DateStructure, RTC_TimeTypeDef *RTC_TimeStructure, uint32_t secs);
uint32_t toUnix(RTC_DateTypeDef RTC_DateStructure, RTC_TimeTypeDef RTC_TimeStructure);