
#ifndef __STM32F30x_H
#define __STM32F30x_H

#include "ch.h"

#include "stm32f3xx.h"
#include "stm32f30x_conf.h"
#include "stm32f303xe.h"


#define  CRC_CR_RESET                        ((uint32_t)0x00000001) /*!< RESET the CRC computation unit bit */
#define  CRC_CR_POLSIZE                      ((uint32_t)0x00000018) /*!< Polynomial size bits */
#define  CRC_CR_POLSIZE_0                    ((uint32_t)0x00000008) /*!< Polynomial size bit 0 */
#define  CRC_CR_POLSIZE_1                    ((uint32_t)0x00000010) /*!< Polynomial size bit 1 */

#define  FLASH_OBR_OPTERR                    ((uint32_t)0x00000001)        /*!< Option Byte Error */
#define  FLASH_OBR_RDPRT1                    ((uint32_t)0x00000002)        /*!< Read protection Level 1 */
#define  FLASH_OBR_RDPRT2                    ((uint32_t)0x00000004)        /*!< Read protection Level 2 */


#define  RCC_APB1ENR_DAC2EN                  ((uint32_t)0x04000000)        /*!< DAC 2 clock enable */

#define DAC2_BASE             (APB1PERIPH_BASE + 0x00009800)
#define DAC2                ((DAC_TypeDef *) DAC2_BASE)


/** 
  * @brief Flexible Memory Controller Bank2
  */
  
typedef struct
{
  __IO uint32_t PCR2;       /*!< NAND Flash control register 2,                       Address offset: 0x60 */
  __IO uint32_t SR2;        /*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
  __IO uint32_t PMEM2;      /*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
  __IO uint32_t PATT2;      /*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x70                                                            */
  __IO uint32_t ECCR2;      /*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
} FMC_Bank2_TypeDef;

/** 
  * @brief Flexible Memory Controller Bank3
  */
  
typedef struct
{
  __IO uint32_t PCR3;       /*!< NAND Flash control register 3,                       Address offset: 0x80 */
  __IO uint32_t SR3;        /*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
  __IO uint32_t PMEM3;      /*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
  __IO uint32_t PATT3;      /*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x90                                                            */
  __IO uint32_t ECCR3;      /*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FMC_Bank3_TypeDef;


/*!< FMC Banks registers base  address */
#define FMC_Bank1_R_BASE      (FMC_R_BASE + 0x0000)
#define FMC_Bank1E_R_BASE     (FMC_R_BASE + 0x0104)
#define FMC_Bank2_R_BASE      (FMC_R_BASE + 0x0060)
#define FMC_Bank3_R_BASE      (FMC_R_BASE + 0x0080)
#define FMC_Bank4_R_BASE      (FMC_R_BASE + 0x00A0)

#define FMC_Bank1           ((FMC_Bank1_TypeDef *) FMC_Bank1_R_BASE)
#define FMC_Bank1E          ((FMC_Bank1E_TypeDef *) FMC_Bank1E_R_BASE)
#define FMC_Bank2           ((FMC_Bank2_TypeDef *) FMC_Bank2_R_BASE)
#define FMC_Bank3           ((FMC_Bank3_TypeDef *) FMC_Bank3_R_BASE)
#define FMC_Bank4           ((FMC_Bank4_TypeDef *) FMC_Bank4_R_BASE)


#define ADC1_2_BASE           (AHB3PERIPH_BASE + 0x0300)
#define ADC1_2              ((ADC_Common_TypeDef *) ADC1_2_BASE)
#define ADC3_4_BASE           (AHB3PERIPH_BASE + 0x0700)
#define ADC3_4              ((ADC_Common_TypeDef *) ADC3_4_BASE)
#define ADC_SQR3_SQ15       ADC_SQR4_SQ15 /*!< ADC 15th conversion in regular sequence */

#define SYSCFG_CFGR1_DAC2Ch1_DMA_RMP        ((uint32_t)0x00008000) /*!< DAC2 CH1 DMA remap */

#define CAN1_BASE             (APB1PERIPH_BASE + 0x00006400)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define  RCC_APB1ENR_CAN1EN                  ((uint32_t)0x02000000)        /*!< CAN clock enable */

/* HRTIM slave definition */
typedef struct
{
  __IO uint32_t TIMxCR;     /*!< HRTIM Timerx control register,                              Address offset: 0x00  */
  __IO uint32_t TIMxISR;    /*!< HRTIM Timerx interrupt status register,                     Address offset: 0x04  */
  __IO uint32_t TIMxICR;    /*!< HRTIM Timerx interrupt clear register,                      Address offset: 0x08  */
  __IO uint32_t TIMxDIER;   /*!< HRTIM Timerx DMA/interrupt enable register,                 Address offset: 0x0C  */
  __IO uint32_t CNTxR;      /*!< HRTIM Timerx counter register,                              Address offset: 0x10  */
  __IO uint32_t PERxR;      /*!< HRTIM Timerx period register,                               Address offset: 0x14  */
  __IO uint32_t REPxR;      /*!< HRTIM Timerx repetition register,                           Address offset: 0x18  */
  __IO uint32_t CMP1xR;     /*!< HRTIM Timerx compare 1 register,                            Address offset: 0x1C  */
  __IO uint32_t CMP1CxR;    /*!< HRTIM Timerx compare 1 compound register,                   Address offset: 0x20  */
  __IO uint32_t CMP2xR;     /*!< HRTIM Timerx compare 2 register,                            Address offset: 0x24  */
  __IO uint32_t CMP3xR;     /*!< HRTIM Timerx compare 3 register,                            Address offset: 0x28  */
  __IO uint32_t CMP4xR;     /*!< HRTIM Timerx compare 4 register,                            Address offset: 0x2C  */
  __IO uint32_t CPT1xR;     /*!< HRTIM Timerx capture 1 register,                            Address offset: 0x30  */
  __IO uint32_t CPT2xR;     /*!< HRTIM Timerx capture 2 register,                            Address offset: 0x34 */
  __IO uint32_t DTxR;       /*!< HRTIM Timerx dead time register,                            Address offset: 0x38 */
  __IO uint32_t SETx1R;     /*!< HRTIM Timerx output 1 set register,                         Address offset: 0x3C */
  __IO uint32_t RSTx1R;     /*!< HRTIM Timerx output 1 reset register,                       Address offset: 0x40 */
  __IO uint32_t SETx2R;     /*!< HRTIM Timerx output 2 set register,                         Address offset: 0x44 */
  __IO uint32_t RSTx2R;     /*!< HRTIM Timerx output 2 reset register,                       Address offset: 0x48 */
  __IO uint32_t EEFxR1;     /*!< HRTIM Timerx external event filtering 1 register,           Address offset: 0x4C */
  __IO uint32_t EEFxR2;     /*!< HRTIM Timerx external event filtering 2 register,           Address offset: 0x50 */
  __IO uint32_t RSTxR;      /*!< HRTIM Timerx Reset register,                                Address offset: 0x54 */
  __IO uint32_t CHPxR;      /*!< HRTIM Timerx Chopper register,                              Address offset: 0x58 */
  __IO uint32_t CPT1xCR;    /*!< HRTIM Timerx Capture 1 register,                            Address offset: 0x5C */
  __IO uint32_t CPT2xCR;    /*!< HRTIM Timerx Capture 2 register,                            Address offset: 0x60 */
  __IO uint32_t OUTxR;      /*!< HRTIM Timerx Output register,                               Address offset: 0x64 */
  __IO uint32_t FLTxR;      /*!< HRTIM Timerx Fault register,                                Address offset: 0x68 */
  uint32_t      RESERVED0[5];/*!< Reserved,                                                                       */
}HRTIM_Timerx_TypeDef;

/* HRTIM common register definition */
typedef struct
{
  __IO uint32_t CR1;        /*!< HRTIM control register1,                                    Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< HRTIM control register2,                                    Address offset: 0x04 */
  __IO uint32_t ISR;        /*!< HRTIM interrupt status register,                            Address offset: 0x08 */
  __IO uint32_t ICR;        /*!< HRTIM interrupt clear register,                             Address offset: 0x0C */
  __IO uint32_t IER;        /*!< HRTIM interrupt enable register,                            Address offset: 0x10 */
  __IO uint32_t OENR;       /*!< HRTIM Output enable register,                               Address offset: 0x14 */
  __IO uint32_t DISR;       /*!< HRTIM Output disable register,                              Address offset: 0x18 */
  __IO uint32_t ODSR;       /*!< HRTIM Output disable status register,                       Address offset: 0x1C */
  __IO uint32_t BMCR;       /*!< HRTIM Burst mode control register,                          Address offset: 0x20 */
  __IO uint32_t BMTRGR;     /*!< HRTIM Busrt mode trigger register,                          Address offset: 0x24 */
  __IO uint32_t BMCMPR;     /*!< HRTIM Burst mode compare register,                          Address offset: 0x28 */
  __IO uint32_t BMPER;      /*!< HRTIM Burst mode period register,                           Address offset: 0x2C */
  __IO uint32_t EECR1;      /*!< HRTIM Timer external event control register1,               Address offset: 0x30 */
  __IO uint32_t EECR2;      /*!< HRTIM Timer external event control register2,               Address offset: 0x34 */
  __IO uint32_t EECR3;      /*!< HRTIM Timer external event control register3,               Address offset: 0x38 */
  __IO uint32_t ADC1R;      /*!< HRTIM ADC Trigger 1 register,                               Address offset: 0x3C */
  __IO uint32_t ADC2R;      /*!< HRTIM ADC Trigger 2 register,                               Address offset: 0x40 */
  __IO uint32_t ADC3R;      /*!< HRTIM ADC Trigger 3 register,                               Address offset: 0x44 */
  __IO uint32_t ADC4R;      /*!< HRTIM ADC Trigger 4 register,                               Address offset: 0x48 */
  __IO uint32_t DLLCR;      /*!< HRTIM DLL control register,                                 Address offset: 0x4C */
  __IO uint32_t FLTINxR1;   /*!< HRTIM Fault input register1,                                Address offset: 0x50 */
  __IO uint32_t FLTINxR2;   /*!< HRTIM Fault input register2,                                Address offset: 0x54 */
  __IO uint32_t BDMUPDR;    /*!< HRTIM Burst DMA Master Timer update register,               Address offset: 0x58 */
  __IO uint32_t BDTAUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x5C */
  __IO uint32_t BDTBUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x60 */
  __IO uint32_t BDTCUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x64 */
  __IO uint32_t BDTDUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x68 */  
  __IO uint32_t BDTEUPR;    /*!< HRTIM Burst DMA Timerx update register,                     Address offset: 0x6C */  
  __IO uint32_t BDMADR;     /*!< HRTIM Burst DMA Master Data register,                       Address offset: 0x70 */
}HRTIM_Common_TypeDef;

/** 
  * @brief High resolution Timer (HRTIM)
  */
/* HRTIM master definition */
typedef struct
{
  __IO uint32_t MCR;            /*!< HRTIM Master Timer control register,                     Address offset: 0x00 */
  __IO uint32_t MISR;           /*!< HRTIM Master Timer interrupt status register,            Address offset: 0x04 */
  __IO uint32_t MICR;           /*!< HRTIM Master Timer interrupt clear register,              Address offset: 0x08 */
  __IO uint32_t MDIER;          /*!< HRTIM Master Timer DMA/interrupt enable register         Address offset: 0x0C */
  __IO uint32_t MCNTR;          /*!< HRTIM Master Timer counter register,                     Address offset: 0x10 */
  __IO uint32_t MPER;           /*!< HRTIM Master Timer period register,                      Address offset: 0x14 */
  __IO uint32_t MREP;           /*!< HRTIM Master Timer repetition register,                  Address offset: 0x18 */
  __IO uint32_t MCMP1R;         /*!< HRTIM Master Timer compare 1 register,                   Address offset: 0x1C */
  uint32_t      RESERVED0;     /*!< Reserved,                                                                0x20 */
  __IO uint32_t MCMP2R;         /*!< HRTIM Master Timer compare 2 register,                   Address offset: 0x24 */
  __IO uint32_t MCMP3R;         /*!< HRTIM Master Timer compare 3 register,                   Address offset: 0x28 */
  __IO uint32_t MCMP4R;         /*!< HRTIM Master Timer compare 4 register,                   Address offset: 0x2C */
}HRTIM_Master_TypeDef; 
/* HRTIM  register definition */
typedef struct {
  HRTIM_Master_TypeDef HRTIM_MASTER;
  uint32_t             RESERVED0[20];
  HRTIM_Timerx_TypeDef HRTIM_TIMERx[5];
  uint32_t             RESERVED1[32];
  HRTIM_Common_TypeDef HRTIM_COMMON;
}HRTIM_TypeDef;


#define HRTIM_RSTR_EXTEVNT1   ((uint32_t)0x00000200)   /*!< External event 1 */
#define HRTIM_RSTR_EXTEVNT2   ((uint32_t)0x00000400)   /*!< External event 2 */
#define HRTIM_RSTR_EXTEVNT3   ((uint32_t)0x00000800)   /*!< External event 3 */
#define HRTIM_RSTR_EXTEVNT4   ((uint32_t)0x00001000)   /*!< External event 4 */
#define HRTIM_RSTR_EXTEVNT5   ((uint32_t)0x00002000)   /*!< External event 5 */
#define HRTIM_RSTR_EXTEVNT6   ((uint32_t)0x00004000)   /*!< External event 6 */
#define HRTIM_RSTR_EXTEVNT7   ((uint32_t)0x00008000)   /*!< External event 7 */
#define HRTIM_RSTR_EXTEVNT8   ((uint32_t)0x00010000)   /*!< External event 8 */
#define HRTIM_RSTR_EXTEVNT9   ((uint32_t)0x00020000)   /*!< External event 9 */
#define HRTIM_RSTR_EXTEVNT10  ((uint32_t)0x00040000)   /*!< External event 10 */


#define  RCC_CFGR2_PREDIV1_DIV1              ((uint32_t)0x00000000)        /*!< PREDIV1 input clock not divided */
#define  RCC_CFGR2_PREDIV1_DIV2              ((uint32_t)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
#define  RCC_CFGR2_PREDIV1_DIV3              ((uint32_t)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
#define  RCC_CFGR2_PREDIV1_DIV4              ((uint32_t)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
#define  RCC_CFGR2_PREDIV1_DIV5              ((uint32_t)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
#define  RCC_CFGR2_PREDIV1_DIV6              ((uint32_t)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
#define  RCC_CFGR2_PREDIV1_DIV7              ((uint32_t)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
#define  RCC_CFGR2_PREDIV1_DIV8              ((uint32_t)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
#define  RCC_CFGR2_PREDIV1_DIV9              ((uint32_t)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
#define  RCC_CFGR2_PREDIV1_DIV10             ((uint32_t)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
#define  RCC_CFGR2_PREDIV1_DIV11             ((uint32_t)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
#define  RCC_CFGR2_PREDIV1_DIV12             ((uint32_t)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
#define  RCC_CFGR2_PREDIV1_DIV13             ((uint32_t)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
#define  RCC_CFGR2_PREDIV1_DIV14             ((uint32_t)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
#define  RCC_CFGR2_PREDIV1_DIV15             ((uint32_t)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
#define  RCC_CFGR2_PREDIV1_DIV16             ((uint32_t)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */



#define  RCC_CFGR3_HRTIM1SW                  ((uint32_t)0x00001000)        /*!< HRTIM1SW bits */
#define  RCC_CFGR3_TIM3SW                    ((uint32_t)0x02000000)        /*!< TIM2SW bits */

#define  RCC_CFGR_MCO_PRE                    ((uint32_t)0x70000000)        /*!< MCO prescaler */
#define  RCC_CFGR_MCO_PRE_1                  ((uint32_t)0x00000000)        /*!< MCO is divided by 1 */
#define  RCC_CFGR_MCO_PRE_2                  ((uint32_t)0x10000000)        /*!< MCO is divided by 2 */
#define  RCC_CFGR_MCO_PRE_4                  ((uint32_t)0x20000000)        /*!< MCO is divided by 4 */
#define  RCC_CFGR_MCO_PRE_8                  ((uint32_t)0x30000000)        /*!< MCO is divided by 8 */
#define  RCC_CFGR_MCO_PRE_16                 ((uint32_t)0x40000000)        /*!< MCO is divided by 16 */
#define  RCC_CFGR_MCO_PRE_32                 ((uint32_t)0x50000000)        /*!< MCO is divided by 32 */
#define  RCC_CFGR_MCO_PRE_64                 ((uint32_t)0x60000000)        /*!< MCO is divided by 64 */
#define  RCC_CFGR_MCO_PRE_128                ((uint32_t)0x70000000)        /*!< MCO is divided by 128 */


/*!< PLLMUL configuration */
#define  RCC_CFGR_PLLMULL                    ((uint32_t)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL_0                  ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR_PLLMULL_1                  ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  RCC_CFGR_PLLMULL_2                  ((uint32_t)0x00100000)        /*!< Bit 2 */
#define  RCC_CFGR_PLLMULL_3                  ((uint32_t)0x00200000)        /*!< Bit 3 */


#define  RCC_CFGR_PLLMULL2                   ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
#define  RCC_CFGR_PLLMULL3                   ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
#define  RCC_CFGR_PLLMULL4                   ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
#define  RCC_CFGR_PLLMULL5                   ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
#define  RCC_CFGR_PLLMULL6                   ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
#define  RCC_CFGR_PLLMULL7                   ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
#define  RCC_CFGR_PLLMULL8                   ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
#define  RCC_CFGR_PLLMULL9                   ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
#define  RCC_CFGR_PLLMULL10                  ((uint32_t)0x00200000)        /*!< PLL input clock10 */
#define  RCC_CFGR_PLLMULL11                  ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
#define  RCC_CFGR_PLLMULL12                  ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
#define  RCC_CFGR_PLLMULL13                  ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
#define  RCC_CFGR_PLLMULL14                  ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
#define  RCC_CFGR_PLLMULL15                  ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
#define  RCC_CFGR_PLLMULL16                  ((uint32_t)0x00380000)        /*!< PLL input clock*16 */

#define  RCC_CFGR2_PREDIV1                   ((uint32_t)0x0000000F)        /*!< PREDIV1[3:0] bits */
#define  RCC_CFGR2_PREDIV1_0                 ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR2_PREDIV1_1                 ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  RCC_CFGR2_PREDIV1_2                 ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  RCC_CFGR2_PREDIV1_3                 ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  RCC_CFGR_PLLSRC_HSI_Div2            ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define  RCC_CFGR_PLLSRC_HSI_PREDIV          ((uint32_t)0x00008000)        /*!< HSI PREDIV clock selected as PLL entry clock source */
#define  RCC_CFGR_PLLSRC_PREDIV1             ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup 
   Timeout value 
   */
#if !defined  (HSE_STARTUP_TIMEOUT) 
 #define HSE_STARTUP_TIMEOUT  ((uint16_t)0x5000)   /*!< Time out for HSE start up */
#endif /* HSE_STARTUP_TIMEOUT */


#if !defined  (HSI_VALUE) 
 #define HSI_VALUE  ((uint32_t)8000000)
#endif /* HSI_VALUE */                      /*!< Value of the Internal High Speed oscillator in Hz.
                                            The real value may vary depending on the variations
                                             in voltage and temperature.  */
#if !defined  (LSI_VALUE) 
 #define LSI_VALUE  ((uint32_t)40000)    
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  ((uint32_t)32768)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */     
#if !defined  (HSE_VALUE) 
 #define HSE_VALUE            ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

     
#endif
