/**
 * @file       drv8301.h
 * @brief      TI DRV8301 driver
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       04 April 2017
 * 
 *
 * @addtogroup MC
 * @brief Motor control
 * @{
 */
#ifndef _DRV8301_H
#define _DRV8301_H

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Macros				                                                             */
/*===========================================================================*/
// Hardware settings
#define DRV_EN_GATE_PORT    GPIOC
#define DRV_EN_GATE_PIN     6
#define DRV_DCCAL_PORT      GPIOC
#define DRV_DCCAL_PIN       7
#define DRV_NFAULT_PORT     GPIOD
#define DRV_NFAULT_PIN      2
#define DRV_NOCTW_PORT      GPIOB
#define DRV_NOCTW_PIN       3
#define DRV_PWRGD_PORT      GPIOB
#define DRV_PWRGD_PIN       4

#define DRV_SPI_DEVICE      SPID3
#define DRV_SPI_SCK_PORT    GPIOC
#define DRV_SPI_SCK_PIN     10
#define DRV_SPI_MOSI_PORT   GPIOC
#define DRV_SPI_MOSI_PIN    12
#define DRV_SPI_MISO_PORT   GPIOC
#define DRV_SPI_MISO_PIN    11
#define DRV_SPI_NCS_PORT    GPIOA
#define DRV_SPI_NCS_PIN     15

#define DRV_INH_A_PORT      GPIOA
#define DRV_INH_A_PIN       10
#define DRV_INH_B_PORT      GPIOA
#define DRV_INH_B_PIN       9
#define DRV_INH_C_PORT      GPIOA
#define DRV_INH_C_PIN       8

#define DRV_INL_A_PORT      GPIOB
#define DRV_INL_A_PIN       15
#define DRV_INL_B_PORT      GPIOB
#define DRV_INL_B_PIN       14
#define DRV_INL_C_PORT      GPIOB
#define DRV_INL_C_PIN       13
  
#define DRV_SOA_PORT        GPIOB
#define DRV_SOA_PIN         1
#define DRV_SOB_PORT        GPIOB
#define DRV_SOB_PIN         0
/**
 * @brief      Enables the DC calibration
 */
#define drvDCCalEnable()  palSetPad(DRV_DCCAL_PORT, DRV_DCCAL_PIN)
/**
 * @brief      Enables the DC calibration
 */
#define drvDCCalDisable()  palClearPad(DRV_DCCAL_PORT, DRV_DCCAL_PIN)

/**
 * @brief      Enables the gate driver stage
 */
#define drvGateEnable()  palSetPad(DRV_EN_GATE_PORT, DRV_EN_GATE_PIN)

/**
 * @brief      Disables the gate driver stage
 */
#define drvGateDisable()  palClearPad(DRV_EN_GATE_PORT, DRV_EN_GATE_PIN)

/*===========================================================================*/
/* Datatypes                                                                 */
/*===========================================================================*/


/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
void drvInit(void);
void drvDumpStatus(void);



#endif
/** @} */
