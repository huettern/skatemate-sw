/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for STMicroelectronics STM32F3-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F3_DISCOVERY
#define BOARD_NAME                  "skatemate-mc v0.2"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#define STM32_HSE_BYPASS

/*
 * MCU type as defined in the ST header.
 */
#define STM32F303xE

/*
 * IO pins assignments.
 */
#define GPIOA_CSENSE                 0U
#define GPIOA_BSENSE                 1U
#define GPIOA_ASENSE                 2U
#define GPIOA_PIN3                   3U
#define GPIOA_PIN4                   4U
#define GPIOA_PIN5                   5U
#define GPIOA_PIN6                   6U
#define GPIOA_PIN7                   7U
#define GPIOA_INH_C                  8U
#define GPIOA_INH_B                  9U
#define GPIOA_INH_A                 10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_DRV_NCS               15U

#define GPIOB_SOB                   0U
#define GPIOB_SOA                   1U
#define GPIOB_PIN2                  2U
#define GPIOB_DRV_NOCTW             3U
#define GPIOB_DRV_PWRGD             4U
#define GPIOB_PIN5                  5U
#define GPIOB_PIN6                  6U
#define GPIOB_PIN7                  7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_USART3_TX             10U
#define GPIOB_USART3_RX             11U
#define GPIOB_PIN12                 12U
#define GPIOB_INL_C                 13U
#define GPIOB_INL_B                 14U
#define GPIOB_INL_A                 15U

#define GPIOC_RFM_IRQ               0U
#define GPIOC_RFM_NCS               1U
#define GPIOC_RFM_CE                2U
#define GPIOC_SSENSE                3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_DRV_EN_GATE           6U
#define GPIOC_DRV_DCCAL             7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_SPI_SCK               10U
#define GPIOC_SPI_MISO              11U
#define GPIOC_SPI_MOSI              12U
#define GPIOC_LED_1                 13U
#define GPIOC_LED_2                 14U
#define GPIOC_LED_3                 15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U

/*
 * IO lines assignments.
 */
#define LINE_SPI3_SCK               PAL_LINE(GPIOC, GPIOC_SPI_SCK)
#define LINE_SPI3_MISO              PAL_LINE(GPIOC, GPIOC_SPI_MISO)
#define LINE_SPI3_MOSI              PAL_LINE(GPIOC, GPIOC_SPI_MOSI)

#define LINE_USB_DM                 PAL_LINE(GPIOA, GPIOA_USB_DM)
#define LINE_USB_DP                 PAL_LINE(GPIOA, GPIOA_USB_DP)

#define LINE_SWDIO                  PAL_LINE(GPIOA, GPIOA_SWDIO)
#define LINE_SWCLK                  PAL_LINE(GPIOA, GPIOA_SWCLK)

#define LINE_RFM_IRQ                PAL_LINE(GPIOC, GPIOC_RFM_IRQ)
#define LINE_RFM_CE                 PAL_LINE(GPIOC, GPIOC_RFM_CE)
#define LINE_RFM_NCS                PAL_LINE(GPIOC, GPIOC_RFM_NCS)

#define LINE_DRV_NOCTW              PAL_LINE(GPIOB, GPIOB_DRV_NOCTW)
#define LINE_DRV_PWRGD              PAL_LINE(GPIOB, GPIOB_DRV_PWRGD)
#define LINE_DRV_DCCAL              PAL_LINE(GPIOC, GPIOC_DRV_DCCAL)
#define LINE_DRV_EN_GATE            PAL_LINE(GPIOC, GPIOC_DRV_EN_GATE)
#define LINE_DRV_NCS                PAL_LINE(GPIOA, GPIOA_DRV_NCS)

#define LINE_LED1                   PAL_LINE(GPIOC, GPIOC_LED_1)
#define LINE_LED2                   PAL_LINE(GPIOC, GPIOC_LED_2)
#define LINE_LED3                   PAL_LINE(GPIOC, GPIOC_LED_3)

#define LINE_OSC_IN                 PAL_LINE(GPIOF, GPIOF_OSC_IN)
#define LINE_OSC_OUT                PAL_LINE(GPIOF, GPIOF_OSC_OUT)

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup: 
 * - All to input
 * - PushPull
 * - Speed Verry Low
 * - Floating (no pull)
 * - Output data register low
 * - No alternate function
 */
#define VAL_GPIOA_MODER            0x00000000
#define VAL_GPIOA_OTYPER           0x00000000
#define VAL_GPIOA_OSPEEDR          0x00000000
#define VAL_GPIOA_PUPDR            0x00000000
#define VAL_GPIOA_ODR              0x00000000     
#define VAL_GPIOA_AFRL             0x00000000
#define VAL_GPIOA_AFRH             0x00000000

/*
 * GPIOB setup: 
 * - All to input
 * - PushPull
 * - Speed Verry Low
 * - Floating (no pull)
 * - Output data register low
 * - No alternate function
 */
#define VAL_GPIOB_MODER            0x00000000
#define VAL_GPIOB_OTYPER           0x00000000
#define VAL_GPIOB_OSPEEDR          0x00000000
#define VAL_GPIOB_PUPDR            0x00000000
#define VAL_GPIOB_ODR              0x00000000     
#define VAL_GPIOB_AFRL             0x00000000
#define VAL_GPIOB_AFRH             0x00000000

/*
 * GPIOC setup: 
 * - All to input
 * - PushPull
 * - Speed Verry Low
 * - Floating (no pull)
 * - Output data register low
 * - No alternate function
 */
#define VAL_GPIOC_MODER            0x00000000
#define VAL_GPIOC_OTYPER           0x00000000
#define VAL_GPIOC_OSPEEDR          0x00000000
#define VAL_GPIOC_PUPDR            0x00000000
#define VAL_GPIOC_ODR              0x00000000     
#define VAL_GPIOC_AFRL             0x00000000
#define VAL_GPIOC_AFRH             0x00000000

#define VAL_GPIOD_MODER            0x00000000
#define VAL_GPIOD_OTYPER           0x00000000
#define VAL_GPIOD_OSPEEDR          0x00000000
#define VAL_GPIOD_PUPDR            0x00000000
#define VAL_GPIOD_ODR              0x00000000     
#define VAL_GPIOD_AFRL             0x00000000
#define VAL_GPIOD_AFRH             0x00000000

#define VAL_GPIOE_MODER            0x00000000
#define VAL_GPIOE_OTYPER           0x00000000
#define VAL_GPIOE_OSPEEDR          0x00000000
#define VAL_GPIOE_PUPDR            0x00000000
#define VAL_GPIOE_ODR              0x00000000     
#define VAL_GPIOE_AFRL             0x00000000
#define VAL_GPIOE_AFRH             0x00000000

#define VAL_GPIOG_MODER            0x00000000
#define VAL_GPIOG_OTYPER           0x00000000
#define VAL_GPIOG_OSPEEDR          0x00000000
#define VAL_GPIOG_PUPDR            0x00000000
#define VAL_GPIOG_ODR              0x00000000     
#define VAL_GPIOG_AFRL             0x00000000
#define VAL_GPIOG_AFRH             0x00000000

#define VAL_GPIOH_MODER            0x00000000
#define VAL_GPIOH_OTYPER           0x00000000
#define VAL_GPIOH_OSPEEDR          0x00000000
#define VAL_GPIOH_PUPDR            0x00000000
#define VAL_GPIOH_ODR              0x00000000     
#define VAL_GPIOH_AFRL             0x00000000
#define VAL_GPIOH_AFRH             0x00000000

/*
 * GPIOF setup: 
 * - All to input
 * - PushPull
 * - Speed Verry Low
 * - Floating (no pull)
 * - Output data register low
 * - No alternate function
 */
#define VAL_GPIOF_MODER            0x00000000
#define VAL_GPIOF_OTYPER           (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |     \
                                    PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT))
#define VAL_GPIOF_OSPEEDR          (PIN_OSPEED_HIGH(GPIOF_OSC_IN) |        \
                                    PIN_OSPEED_HIGH(GPIOF_OSC_OUT))
#define VAL_GPIOF_PUPDR            0x00000000
#define VAL_GPIOF_ODR              (PIN_ODR_HIGH(GPIOF_OSC_IN) |           \
                                    PIN_ODR_HIGH(GPIOF_OSC_OUT))     
#define VAL_GPIOF_AFRL             0x00000000
#define VAL_GPIOF_AFRH             0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
