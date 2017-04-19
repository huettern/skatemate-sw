/**
 * @file       utelemetry.c
 * @brief      Micro telemetry implementation for C
 * @details    
 * @author     Noah Huetter (noahhuetter@gmail.com)
 * @date       24 March 2017
 * 
 *
 * @addtogroup UTELEMETRY
 * @brief Micro telemetry implementation for C
 * @{
 */
#include "utelemetry.h"

#include <string.h>

/**
 * Put your includes here
 */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"

/*===========================================================================*/
/* settings                                                                */
/*===========================================================================*/
/**
 * List the measurements here
 */
static const utlmMeasurement_t utlmMeasurements[] = 
{
  {0, "Voltage", "Volt", utlm_uint8, utlm_uint8},
  {1, "Current", "Amp", utlm_uint64, utlm_float}
};
static const int mNumMeasurements = sizeof(utlmMeasurements) / sizeof(utlmMeasurement_t);

/**
 * @brief      Define the write function here to write a bytestream
 *
 * @param      dat   data
 * @param      n     number of bytes to write
 *
 * @return     none
 */
// #define DEF_WRITE(dat, n) chSequentialStreamWrite(bssusb,(const uint8_t*)dat,n)
#define DEF_WRITE(dat, n) sdWrite(&SD3, (uint8_t *) dat, n);

/**
 * Buffer size to use in RAM for creating packets
 */
#define BUFFER_SIZE 255

/*===========================================================================*/
/* private data                                                              */
/*===========================================================================*/
static bool mIsEnabled = false;
static char mBuf[BUFFER_SIZE];

/*===========================================================================*/
/* prototypes                                                                */
/*===========================================================================*/
static void sendStart(void);
static void sendStop(void);
static void sendDeclaration(uint16_t mid);

/*===========================================================================*/
/* Protocol defines                                                          */
/*===========================================================================*/
#define PACK_START_1 0xff
#define PACK_START_2 0xcc
#define PACK_STOP_1 0xcb
#define PACK_STOP_2 0xfe

#define PACK_CMD_START 0x00 
#define PACK_CMD_STOP 0x01
#define PACK_CMD_DECLARE 0x02
#define PACK_CMD_DATA 0x03

#define MSB16(x) (((x) & 0xff00) >> 8)
#define LSB16(x) ((x) & 0x00ff)

static const uint8_t mDataTypeSizes[] = 
  { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8};

/*===========================================================================*/
/* Module public functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Enable or disable the utlm module
 *
 * @param[in]  en    true to enable
 */
void utlmEnable(bool en)
{
  uint8_t ctr = 0;
  if(en && !mIsEnabled)
  {
    // send start packet
    sendStart();
    // send declarations
    for(ctr = 0; ctr < mNumMeasurements; ctr++)
    {
      sendDeclaration(ctr);
    }
  }
  else if (!en && mIsEnabled)
  {
    sendStop();
  }
}

/**
 * @brief      Send array of data
 *
 * @param[in]  mid    Measurement id
 * @param[in]  num    Number of datapoints
 * @param      xdata  pointer to the x data array
 * @param      ydata  pointer to the y data array
 */
void utlmSend(uint16_t mid, uint16_t num, void* xdata, void* ydata)
{
  uint16_t pos = 0;
  uint16_t packsize = 0;

  packsize = 9; //header+size+cmd+mid+datapoints
  packsize += num*mDataTypeSizes[utlmMeasurements[mid].xtype];
  packsize += num*mDataTypeSizes[utlmMeasurements[mid].ytype];
  packsize += 2; //stop

  if(packsize > BUFFER_SIZE) return;

  memset(mBuf,0,BUFFER_SIZE);

  mBuf[0] = PACK_START_1;
  mBuf[1] = PACK_START_2;
  mBuf[2] = LSB16(packsize);
  mBuf[3] = MSB16(packsize);
  mBuf[4] = PACK_CMD_DATA;
  mBuf[5] = LSB16(mid);
  mBuf[6] = MSB16(mid);
  mBuf[7] = LSB16(num);
  mBuf[8] = MSB16(num);
  pos = 9;

  memcpy(&mBuf[pos], xdata, num*mDataTypeSizes[utlmMeasurements[mid].xtype]);
  pos += num*mDataTypeSizes[utlmMeasurements[mid].xtype];
  memcpy(&mBuf[pos], ydata, num*mDataTypeSizes[utlmMeasurements[mid].ytype]);
  pos += num*mDataTypeSizes[utlmMeasurements[mid].ytype];

  mBuf[pos++] = PACK_STOP_1;
  mBuf[pos] = PACK_STOP_2;
  // send
  DEF_WRITE(mBuf,packsize);
}

/*===========================================================================*/
/* Module static functions.                                                  */
/*===========================================================================*/
/**
 * @brief      Sends the start packet
 */
static void sendStart(void)
{
  uint16_t packsize = 0;

  packsize = 9; //start+len+cmd+ver+stop

  if(packsize > BUFFER_SIZE) return;

  memset(mBuf,0,BUFFER_SIZE);

  mBuf[0] = PACK_START_1;
  mBuf[1] = PACK_START_2;
  mBuf[2] = LSB16(packsize);
  mBuf[3] = MSB16(packsize);
  mBuf[4] = PACK_CMD_START;
  mBuf[5] = UTLM_MAJOR_VERSION;
  mBuf[6] = UTLM_MINOR_VERSION;
  mBuf[7] = PACK_STOP_1;
  mBuf[8] = PACK_STOP_2;

  // send
  DEF_WRITE(mBuf,9);
}

/**
 * @brief      Sends the stop packet
 */
static void sendStop(void)
{
  uint16_t packsize = 0;

  packsize = 7; //start+len+cmd+ver+stop

  if(packsize > BUFFER_SIZE) return;

  memset(mBuf,0,BUFFER_SIZE);

  mBuf[0] = PACK_START_1;
  mBuf[1] = PACK_START_2;
  mBuf[2] = LSB16(packsize);
  mBuf[3] = MSB16(packsize);
  mBuf[4] = PACK_CMD_STOP;
  mBuf[5] = PACK_STOP_1;
  mBuf[6] = PACK_STOP_2;
  
  // send
  DEF_WRITE(mBuf,9);
}

/**
 * @brief      Sends the measurement declaration
 *
 * @param[in]  mid   measurement ID
 */
static void sendDeclaration(uint16_t mid)
{
  uint16_t pos = 0;
  uint16_t packsize = 0;

  packsize = 9; //header+size+cmd+mid+datatype
  packsize += strlen(utlmMeasurements[mid].name)+1;
  packsize += strlen(utlmMeasurements[mid].unit)+1;
  packsize += 2; //stop

  if(packsize > BUFFER_SIZE) return;

  memset(mBuf,0,BUFFER_SIZE);

  mBuf[0] = PACK_START_1;
  mBuf[1] = PACK_START_2;
  mBuf[2] = LSB16(packsize);
  mBuf[3] = MSB16(packsize);
  mBuf[4] = PACK_CMD_DECLARE;
  mBuf[5] = LSB16(mid);
  mBuf[6] = MSB16(mid);

  mBuf[7] = (char)utlmMeasurements[mid].xtype;
  mBuf[8] = (char)utlmMeasurements[mid].ytype;
  
  pos = 9;
  strcpy(&mBuf[pos], utlmMeasurements[mid].name);
  pos += strlen(utlmMeasurements[mid].name) + 1; // Null termination included
  strcpy(&mBuf[pos], utlmMeasurements[mid].unit);
  pos += strlen(utlmMeasurements[mid].unit) + 1; // Null termination included

  mBuf[pos++] = PACK_STOP_1;
  mBuf[pos++] = PACK_STOP_2;
  // send
  DEF_WRITE(mBuf,packsize);
}

/** @} */
