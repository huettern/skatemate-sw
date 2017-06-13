#ifndef __STM32F30x_CONF_H
#define __STM32F30x_CONF_H

#define USE_RTOS	0

#define USE_FULL_ASSERT

#ifdef  USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif

#endif

#include "stm32f30x_misc.h"

// #include "stm32f30x_crc.h"
// #include "stm32f30x_flash.h"
// #include "stm32f30x_iwdg.h"
// #include "stm32f30x_rtc.h"
// #include "stm32f30x_wwdg.h"
// #include "stm32f30x_dac.h"
// #include "stm32f30x_fmc.h"
// #include "stm32f30x_spi.h"
#include "stm32f30x_adc.h"
// #include "stm32f30x_dbgmcu.h"
// #include "stm32f30x_gpio.h"
// #include "stm32f30x_opamp.h"
// #include "stm32f30x_syscfg.h"
// #include "stm32f30x_can.h"
#include "stm32f30x_dma.h"
// #include "stm32f30x_hrtim.h"
// #include "stm32f30x_pwr.h"
#include "stm32f30x_tim.h"
// #include "stm32f30x_comp.h"
// #include "stm32f30x_exti.h"
// #include "stm32f30x_i2c.h"
#include "stm32f30x_rcc.h"
// #include "stm32f30x_usart.h"
