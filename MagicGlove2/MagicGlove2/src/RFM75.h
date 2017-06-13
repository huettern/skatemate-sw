/*
* RFM75.h
*
* Created: 25.04.2017 22:34:19
*  Author: Farin
*/


#ifndef RFM75_H_
#define RFM75_H_


//Ports/Pins


//Settings of the Module

//Use dynamic payload instead of fixed length payload, transmission is not faster or slower
//If this option is 0 the function Rfm75PayloadLength(pipe, length) will be available to set the payload length for the pipes (has to be set)
//Is this option 1, the global dynamic payload bit will be set and dynamic payload length will be enabled on all pipes
#define USE_DYNAMIC_PAYLOAD		1		//Possible values: 0 / 1


//Additional functions in this libary (option set to 1 will make the functions available, 0 disables the functions)
#define CRC_FUNCTIONS			1		//Function for setting the number of checksum bytes in transmission and enabling checksum
#define POWER_UP_DOWN_FUNCTION	1		//Set power up/down bit -> bring module into power down mode, very low power consumption, no rx/tx
#define TX_POWER_FUNCTION		1		//Function to set the RF output power of the module
#define ADDRNBYTE_FUNCTIONS		1		//Function to set the number of addressbytes used (3,4,5)
#define RFDATA_RATE_FUNCTION	1		//Function to set the air data rate of the module (250kBit, 1MBit, 2MBit)
#define ACK_PAYLOAD_FUNCTION	1		//Function to write bytes for transmission with the next acknowledge packet
#define SEND_NOAA_PACKET_FUNC	1		//Will enable the Rfm75SendPacketNoAA(buffer, length) function to send packet with the NO_ACK bit set
#define LOST_PACKETS_FUNCTIONS	1		//Enables two function which returns the counters from the OBSERVE_TX register
#define FIFO_STATUS_FUNCTION	1		//Enables a function to read the fifo status register


//Commands, constants and masks
#define R_REGISTER				0x00			//Add the number of the register to read to this constant
#define W_REGISTER				0x20			//Add the number of the register to write to this constant
#define WRITE_REG				0x20			//Add the number of the register to write to this constant
#define R_RX_PAYLOAD			0x61
#define W_TX_PAYLOAD			0xA0
#define FLUSH_TX				0xE1
#define FLUSH_RX				0xE2
#define REUSE_TX_PL				0xE3
#define ACTIVATE				0x50
#define R_RX_PL_WID				0x60
#define W_ACK_PAYLOAD			0xA8			//Add the number of the pipe, where the data should send with, to this constant
#define W_TX_PAYLOAD_NO_ACK		0xB0
#define RFM75_NOP				0xFF
#define STATUS					0x07

#define SWITCH_BANK_VALUE		0x53
#define ACTIVATE_FEATURE_VALUE	0x73

#define RFM75_RT_INT_MASK		0x10
#define RFM75_TX_INT_MASK		0x20
#define RFM75_RX_INT_MASK		0x40


#define REGISTER_CONFIG			0x00
#define REGISTER_EN_AA			0x01
#define REGISTER_ENRXADDR		0x02
#define REGISTER_SETUP_AW		0x03
#define REGISTER_SETUP_RETR		0x04
#define REGISTER_RF_CH			0x05
#define REGISTER_RF_SETUP		0x06
#define REGISTER_STATUS			0x07
#define REGISTER_OBSERVE_TX		0x08
#define REGISTER_CD				0x09
#define REGISTER_RX_ADDR_P0		0x0A
#define REGISTER_RX_ADDR_P1		0x0B
#define REGISTER_RX_ADDR_P2		0x0C
#define REGISTER_RX_ADDR_P3		0x0D
#define REGISTER_RX_ADDR_P4		0x0E
#define REGISTER_RX_ADDR_P5		0x0F
#define REGISTER_TX_ADDR		0x10
#define REGISTER_RX_PW_P0		0x11
#define REGISTER_RX_PW_P1		0x12
#define REGISTER_RX_PW_P2		0x13
#define REGISTER_RX_PW_P3		0x14
#define REGISTER_RX_PW_P4		0x15
#define REGISTER_RX_PW_P5		0x16
#define REGISTER_FIFO_STATUS	0x17
#define REGISTER_DYNPD			0x1C
#define REGISTER_FEATURE		0x1D


//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE 	0x40
#define FIFO_STATUS_TX_FULL 	0x20
#define FIFO_STATUS_TX_EMPTY 	0x10

#define FIFO_STATUS_RX_FULL 	0x02
#define FIFO_STATUS_RX_EMPTY 	0x01

//Function declaration
//The following 8 functions are the most used in this project.
uint8_t Rfm75Init(void);
void Rfm75SendPacket(uint8_t *, uint8_t);
void Rfm75SendPacketNoAA(uint8_t *, uint8_t);
void Rfm75WriteAckPayload(uint8_t *, uint8_t, uint8_t);
void Rfm75ReceivePacket(uint8_t *, uint8_t *, uint8_t *);
uint8_t Rfm75ReadRecPayloadLength(void);
uint8_t Rfm75ReadRecPipe(void);
void Rfm75ResetInterrupts(void);
void Rfm75Ptx(void);
void Rfm75Prx(void);
void Rfm75FlushTxBuffer(void);
void Rfm75SetCe(void);
void Rfm75ResetCe(void);
void Rfm75WriteRegister(uint8_t, uint8_t);
void Rfm75WriteCommand(uint8_t, uint8_t);
void Rfm75WriteLongRegister(uint8_t, uint8_t *, uint8_t);
uint8_t Rfm75ReadRegister(uint8_t);
void Rfm75WriteMultiple(uint8_t *, uint8_t);
void Rfm75SelectBank(uint8_t);
void Rfm75ReadBuffer(uint8_t, uint8_t *, uint8_t);



#endif /* RFM75_H_ */