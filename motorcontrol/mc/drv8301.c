/**
 * @file       drv8301.c
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
#include "drv8301.h"

#include "ch.h"
#include "hal.h"
#include "defs.h"

#include "stm32f30x_conf.h"

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/

// SPI registers
#define DRV_SPI_READ            0x8000
#define DRV_SPI_WRITE           0x0000

#define DRV_SPI_ADR(x)          ((x<<11)&0x7800)
#define DRV_SPI_SR1             DRV_SPI_ADR(0)
#define DRV_SPI_SR2             DRV_SPI_ADR(1)
#define DRV_SPI_CTRL1           DRV_SPI_ADR(2)
#define DRV_SPI_CTRL2           DRV_SPI_ADR(3)

// SR1 bits
#define SR1_FAULT           0x0400
#define SR1_GVDD_UV         0x0200
#define SR1_PVDD_UV         0x0100
#define SR1_OTSD            0x0080
#define SR1_OTW             0x0040
#define SR1_FETHA_OC        0x0020
#define SR1_FETLA_OC        0x0010
#define SR1_FETHB_OC        0x0008
#define SR1_FETLB_OC        0x0004
#define SR1_FETHC_OC        0x0002
#define SR1_FETLC_OC        0x0001

// SR2 bits
#define SR2_GVDD_OV         0x0080
#define SR2_DEVICE_ID       0x000f

// CTRL1 bits
#define CTRL1_GATE_CURRENT_1_7_A      0x0000
#define CTRL1_GATE_CURRENT_0_7_A      0x0001
#define CTRL1_GATE_CURRENT_0_25_A     0x0002
#define CTRL1_GATE_RESET_NORMAL       0x0000
#define CTRL1_GATE_RESET_LATCHED      0x0004
#define CTRL1_PWM_MODE_6_INPUT        0x0000
#define CTRL1_PWM_MODE_3_INPUT        0x0008
#define CTRL1_OCP_MODE_CURRENT_LIMIT  0x0000
#define CTRL1_OCP_MODE_LATCH_SHUTDOWN 0x0010
#define CTRL1_OCP_MODE_REPORT_ONLY    0x0020
#define CTRL1_OCP_MODE_DISABLED       0x0030
#define CTRL1_OC_ADJ_SET(x)           ((x<<6) & 0x07C0)

// CTRL2 bits
#define CTRL2_OCTW_MODE_BOTH          0x0000
#define CTRL2_OCTW_MODE_OT_ONLY       0x0001
#define CTRL2_OCTW_MODE_OC_ONLY       0x0002
#define CTRL2_GAIN_10                 0x0000
#define CTRL2_GAIN_20                 0x0004
#define CTRL2_GAIN_40                 0x0008
#define CTRL2_GAIN_80                 0x000C
#define CTRL2_DC_CAL_CH1_NORMAL       0x0000
#define CTRL2_DC_CAL_CH1_SHORT        0x0010
#define CTRL2_DC_CAL_CH2_NORMAL       0x0000
#define CTRL2_DC_CAL_CH2_SHORT        0x0020
#define CTRL2_OC_TOFF_CYCLE           0x0000
#define CTRL2_OC_TOFF_TIME            0x0040

/*
 * Maximum speed SPI configuration (9MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig hs_spicfg = {
  NULL, //Callback
  DRV_SPI_NCS_PORT,
  DRV_SPI_NCS_PIN,
  SPI_CR1_BR_0 | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPHA,// 
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 // 8bit data size
};

// /*
//  * Low speed SPI configuration (140.625kHz, CPHA=0, CPOL=0, MSb first).
//  */
// static const SPIConfig ls_spicfg = {
//   NULL, //Callback
//   DRV_SPI_NCS_PORT,
//   DRV_SPI_NCS_PIN,
//   SPI_CR1_BR_2 | SPI_CR1_BR_1,
//   SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 // 8bit data size
// };


/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
/*
 * SPI TX and RX buffers.
 */
static uint8_t mtxbuf[4];

static uint16_t mSR1Value;

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void writePacket(uint16_t data);
static uint16_t readPacket(uint16_t data);

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief       Inits the DRV and used SPI peripheral
 */
void drvInit(void)
{
  // GPIO clock enable
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  // SPI peripheral
  palSetPadMode(DRV_SPI_SCK_PORT, DRV_SPI_SCK_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_SPI_MOSI_PORT, DRV_SPI_MOSI_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_SPI_MISO_PORT, DRV_SPI_MISO_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_SPI_NCS_PORT, DRV_SPI_NCS_PIN, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPad(DRV_SPI_NCS_PORT, DRV_SPI_NCS_PIN);

  // GPIOs
  palSetPadMode(DRV_EN_GATE_PORT, DRV_EN_GATE_PIN, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);
  palClearPad(DRV_EN_GATE_PORT, DRV_EN_GATE_PIN);
  palSetPadMode(DRV_DCCAL_PORT, DRV_DCCAL_PIN, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);
  palClearPad(DRV_DCCAL_PORT, DRV_DCCAL_PIN);
  palSetPadMode(DRV_NFAULT_PORT, DRV_NFAULT_PIN, PAL_MODE_INPUT);
  palSetPadMode(DRV_NOCTW_PORT, DRV_NOCTW_PIN, PAL_MODE_INPUT);
  palSetPadMode(DRV_PWRGD_PORT, DRV_PWRGD_PIN, PAL_MODE_INPUT);

  // PWMs
  // Mapped to TIM1 Channels 1 2 3 = C B A and negative outputs respecively
  palSetPadMode(DRV_INH_A_PORT, DRV_INH_A_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_INH_B_PORT, DRV_INH_B_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_INH_C_PORT, DRV_INH_C_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_INL_A_PORT, DRV_INL_A_PIN, PAL_MODE_ALTERNATE(4) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_INL_B_PORT, DRV_INL_B_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(DRV_INL_C_PORT, DRV_INL_C_PIN, PAL_MODE_ALTERNATE(6) |
                           PAL_STM32_OSPEED_HIGHEST);

  // ADC Pins
  palSetPadMode(DRV_SOA_PORT, DRV_SOA_PIN, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(DRV_SOB_PORT, DRV_SOB_PIN, PAL_MODE_INPUT_ANALOG);


  // Write device configuration
  // Gate current: 1.7A (max)
  // 6 PWM input pins used
  // Over current mode set to limit: the corresponding FET will be shut down
  //    until the next PWM cycle
  // Over current limit set to 0.25Vds equals 156A with 1.6m RdsON
  // nOCTW pin reports both over temperature and over current
  // Current amp have Gain 10
  // DC Calibration is turned off
  drvGateEnable();
  chThdSleepMilliseconds(50); // Let the DRV power up

  // Control register 1
  writePacket( DRV_SPI_CTRL1 |
    CTRL1_GATE_CURRENT_1_7_A | CTRL1_GATE_RESET_NORMAL | 
    CTRL1_PWM_MODE_6_INPUT | CTRL1_OCP_MODE_DISABLED | CTRL1_OC_ADJ_SET(18)
    );
  // Control register 2
  writePacket( DRV_SPI_CTRL2 |
    CTRL2_OCTW_MODE_BOTH | CTRL2_GAIN_10 | 
    CTRL2_DC_CAL_CH1_NORMAL | CTRL2_DC_CAL_CH2_NORMAL | CTRL2_OC_TOFF_CYCLE
  );
}


/**
 * @brief      Dumps a status message to the DBG stream
 */
void drvDumpStatus(void)
{
  uint16_t sr2, ctl1, ctl2;
  
  // Get all registers and mask out the data bits
  mSR1Value = readPacket(DRV_SPI_SR1) & 0x07ff;
  sr2 = readPacket(DRV_SPI_SR2) & 0x07ff;
  ctl1 = readPacket(DRV_SPI_CTRL1) & 0x07ff;
  ctl2 = readPacket(DRV_SPI_CTRL2) & 0x07ff;

  DBG ("SR1      %04x\r\n\
SR2      %04x\r\n\
CTL1     %04x\r\n\
CTL2     %04x\r\n\
NFAULT   %d\r\n\
NOCTW    %d\r\n\
PWRGD    %d\r\n",
        mSR1Value, sr2, ctl1, ctl2, 
        palReadPad(DRV_NFAULT_PORT, DRV_NFAULT_PIN),
        palReadPad(DRV_NOCTW_PORT, DRV_NOCTW_PIN),
        palReadPad(DRV_PWRGD_PORT, DRV_PWRGD_PIN)
    );
}

/**
 * @brief      Get the faults of the driver
 *
 * @return     Fault register bits
 */
drvFault_t drvGetFault(void)
{
  uint16_t sr2;
  drvFault_t faults;
  // Get all registers and mask out the data bits
  sr2 = readPacket(DRV_SPI_SR2) & 0x07ff;
  faults = mSR1Value | ((sr2 & 0x0080) << 3);
  return faults;
}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Writes a packet.
 *
 * @param[in]  data  16bit packet to write
 */
static void writePacket(uint16_t data)
{
  uint8_t rx[2];
  rx[0] = 0;
  rx[1] = 0;
  mtxbuf[0] = (data>>8) & 0x00ff;
  mtxbuf[1] = data & 0x00ff;
  mtxbuf[2] = 0;
  mtxbuf[3] = 0;

  spiAcquireBus(&DRV_SPI_DEVICE);
  spiStart(&DRV_SPI_DEVICE, &hs_spicfg);

  spiSelect(&DRV_SPI_DEVICE);
  spiExchange(&DRV_SPI_DEVICE, 2, mtxbuf, rx);
  
  // After one packet, the select line must be toggled
  spiUnselect(&DRV_SPI_DEVICE);
  spiSelect(&DRV_SPI_DEVICE);

  spiExchange(&DRV_SPI_DEVICE, 2, &mtxbuf[2], rx);
  spiUnselect(&DRV_SPI_DEVICE);

  spiReleaseBus(&DRV_SPI_DEVICE);
  // After a write the DRV always returns the SR1 value
  // Data is MSB first
  mSR1Value = rx[1] | ((rx[0]<<8)&0xff00);
  mSR1Value &= 0x07ff;
}
/**
 * @brief      Read a register
 *
 * @param[in]  data  adress to read
 *
 * @return     Data returned
 */
static uint16_t readPacket(uint16_t data)
{
  uint8_t rx[2];
  rx[0] = 0;
  rx[1] = 0;
  data |= DRV_SPI_READ;
  mtxbuf[0] = ((data>>8) & 0x00ff);
  mtxbuf[1] = data & 0x00ff;
  mtxbuf[2] = 0;
  mtxbuf[3] = 0;

  spiAcquireBus(&DRV_SPI_DEVICE);
  spiStart(&DRV_SPI_DEVICE, &hs_spicfg);

  spiSelect(&DRV_SPI_DEVICE);
  spiExchange(&DRV_SPI_DEVICE, 2, mtxbuf, rx);
  
  // After one packet, the select line must be toggled
  spiUnselect(&DRV_SPI_DEVICE);
  spiSelect(&DRV_SPI_DEVICE);

  spiExchange(&DRV_SPI_DEVICE, 2, &mtxbuf[2], rx);
  spiUnselect(&DRV_SPI_DEVICE);

  spiUnselect(&DRV_SPI_DEVICE);
  spiReleaseBus(&DRV_SPI_DEVICE);
  // Data is MSB first
  return rx[1] | ((rx[0]<<8)&0xff00);
}


/** @} */
