/*
* RFM75.c
*
* Created: 25.04.2017 22:32:32
*  Author: Farin
*/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include "RFM75.h"
#include "SPI.h"

#define BANK0LENGTH 9
#define CEPORT			PORTE
#define CEDDR			DDRE
#define CEBIT			PORTE3

#define CSPORT			PORTE
#define CSDDR			DDRE
#define CSBIT			PORTE2


#define CE				(1<<CEBIT)
#define CS				(1<<CSBIT)


uint8_t b1_r0[]={0x40, 0x4B, 0x01, 0xE2};
uint8_t b1_r1[]={0xC0, 0x4B, 0x00, 0x00};
uint8_t b1_r2[]={0xD0, 0xFC, 0x8C, 0x02};
uint8_t b1_r3[]={0x99, 0x00, 0x39, 0x41};
uint8_t b1_r4[]={0xD9, 0x96, 0x82, 0x1B};
uint8_t b1_r5[]={0x24, 0x06, 0x7F, 0xA6};

uint8_t b1_r12[]={0x00, 0x12, 0x73, 0x05};
uint8_t b1_r13[]={0x46, 0xB4, 0x80, 0x00};
uint8_t b1_r14[]={0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF};

uint8_t b0_r11[] = {0xCC, 0xCD, 0x11, 0x02, 0x00};
//////////////////////////////////////////////////////////////////////////
// Initialization values for registers in bank 0
//////////////////////////////////////////////////////////////////////////
uint8_t Bank0_Reg[][2]={
	{0,0x02},//reflect RX_DR\TX_DS\MAX_RT,Disable CRC ,2byte,POWER UP,PTX
	{1,0x3F},//Disable auto acknowledgment data pipe5\4\3\2\1\0
	{2,0x3F},//Enable RX Addresses pipe 0
	{3,0x02},//RX/TX address field width 5byte
	{4,0xFF},//auto retransmission delay (4000us),auto retransmission count(15)
	{5,0x01},//23 channel
	{6,0x0F},//air data rate-1M,out power 5dbm,setup LNA gain.
	{7,0x07},//
	{8,0x00},//
	{9,0x00},
	{12, 0xc3},//only LSB Receive address data pipe 2, MSB bytes is equal to RX_ADDR_P1[39:8]
	{13, 0xc4},//only LSB Receive address data pipe 3, MSB bytes is equal to RX_ADDR_P1[39:8]
	{14, 0xc5},//only LSB Receive address data pipe 4, MSB bytes is equal to RX_ADDR_P1[39:8]
	{15, 0xc6},//only LSB Receive address data pipe 5, MSB bytes is equal to RX_ADDR_P1[39:8]
	{17,0x20},//Number of bytes in RX payload in data pipe0(32 byte)
	{18,0x20},//Number of bytes in RX payload in data pipe1(32 byte)
	{19,0x20},//Number of bytes in RX payload in data pipe2(32 byte)
	{20,0x20},//Number of bytes in RX payload in data pipe3(32 byte)
	{21,0x20},//Number of bytes in RX payload in data pipe4(32 byte)
	{22,0x20},//Number of bytes in RX payload in data pipe5(32 byte)
	
	{28,0x01},//Disable dynamic payload length data pipe5\4\3\2\1\0
	{29,0x07}//Enables Dynamic Payload Length,Enables Payload with ACK,Enables the W_TX_PAYLOAD_NOACK command
};

uint8_t RFM75_cmd_switch_cfg[] = { 0x50, 0x53 };
//////////////////////////////////////////////////////////////////////////
// Initializes the RFM75 module with configuration values set on top of this file
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75Init()
{
	uint8_t i;
	_delay_ms(100);
	CSDDR |= CS;
	CEDDR |= CE;
	SPI_init();
	Rfm75SelectBank(0);
	//Initialize registers
	for(i=0;i<21;i++)
	{
		Rfm75WriteRegister(Bank0_Reg[i][0],Bank0_Reg[i][1]);
		//_delay_ms(100);
		//printf("Register: 0x%X\r\n", Bank0_Reg[i][0]);
		//printf("Value: 0x%X\r\n", Rfm75ReadRegister(Bank0_Reg[i][0]));
	}
	Rfm75WriteLongRegister(0x0A, b0_r11, 5);
	Rfm75WriteLongRegister(0x10, b0_r11, 5);
	
	Rfm75WriteCommand(ACTIVATE,0x73);
	Rfm75WriteRegister(Bank0_Reg[20][0], Bank0_Reg[20][1]);
	Rfm75WriteRegister(Bank0_Reg[21][0], Bank0_Reg[21][1]);
	Rfm75SelectBank(1);
	//////////////////////////////////////////////////////////////////////////
	Rfm75WriteLongRegister(0, b1_r0, 4);
	Rfm75WriteLongRegister(1, b1_r1, 4);
	Rfm75WriteLongRegister(2, b1_r2, 4);
	Rfm75WriteLongRegister(3, b1_r3, 4);
	Rfm75WriteLongRegister(4, b1_r4, 4);
	Rfm75WriteLongRegister(5, b1_r5, 4);
	
	Rfm75WriteLongRegister(12, b1_r12, 4);
	Rfm75WriteLongRegister(13, b1_r13, 4);
	Rfm75WriteLongRegister(13, b1_r14, 11);
	//////////////////////////////////////////////////////////////////////////
	if(Rfm75ReadRegister(0x08) == 0x63)
	{
		Rfm75SelectBank(0);
		return 0xFF;
	}
	else
	{
		return 0x00;
	}
}
//////////////////////////////////////////////////////////////////////////
//Send a packet with the RFM75 module, it has to be in PTX mode
//////////////////////////////////////////////////////////////////////////
void Rfm75SendPacket(uint8_t *buffer, uint8_t length)
{
	Rfm75ResetCe();
	//Only necessary to write the payload into TX fifo and set the CE pin
	CSPORT &= ~CS;
	
	SPI_Write(W_TX_PAYLOAD);
	SPI_WriteBuffer(buffer, length);
	
	CSPORT |= CS;
	
	Rfm75SetCe();
}
//////////////////////////////////////////////////////////////////////////
//Send a packet with the RFM75 module with the no auto ack bit set,
//The module has to be in PTX mode
//////////////////////////////////////////////////////////////////////////
void Rfm75SendPacketNoAA(uint8_t *buffer, uint8_t length)
{
	Rfm75SetCe();
	//Only necessary to write the payload into TX fifo and set the CE pin
	CSPORT &= ~CS;
	
	SPI_Write(W_TX_PAYLOAD_NO_ACK);
	SPI_WriteBuffer(buffer, length);
	
	CSPORT |= CS;
	
}
//////////////////////////////////////////////////////////////////////////
//Write bytes to the TX buffer, these bytes will be send with the next Ack packet to the transmitter
//Module must be in PRX mode (primary receive) otherwise a normal packet will be send
//////////////////////////////////////////////////////////////////////////
void Rfm75WriteAckPayload(uint8_t *buffer, uint8_t length, uint8_t pipe)
{
	CSPORT &= ~CS;
	
	SPI_Write(W_ACK_PAYLOAD + pipe);
	SPI_WriteBuffer(buffer, length);
	
	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Read the received bytes in the RX fifo.
//////////////////////////////////////////////////////////////////////////
void Rfm75ReceivePacket(uint8_t *buffer, uint8_t *length, uint8_t *pipe)
{
	uint8_t count;
	
	*pipe = Rfm75ReadRecPipe();
	*length = Rfm75ReadRecPayloadLength();

	CSPORT &= ~CS;
	
	SPI_Write(R_RX_PAYLOAD);
	
	
	for(count = 0; count < *length; count++)
	{
		buffer[count] = SPI_Read(RFM75_NOP);
	}
	
	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Read the number of bytes which were received
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75ReadRecPayloadLength(void)
{
	uint8_t length;
	
	CSPORT &= ~CS;
	
	SPI_Write(R_RX_PL_WID);
	length = SPI_Read(RFM75_NOP);
	
	CSPORT |= CS;
	
	return length;
}
//////////////////////////////////////////////////////////////////////////
//Read the number of the pipe which received the data
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75ReadRecPipe(void)
{
	uint8_t pipe;
	
	pipe = Rfm75ReadRegister(0x07);
	pipe = (pipe >> 1) & 0x07;
	
	return pipe;
}
//////////////////////////////////////////////////////////////////////////
//Resets the interrupt bits in the status register
//////////////////////////////////////////////////////////////////////////
void Rfm75ResetInterrupts(void)
{
	Rfm75WriteRegister(REGISTER_STATUS, 0b01110000);
}
//////////////////////////////////////////////////////////////////////////
//Set the mode of the module to PTX (primary transmit)
//////////////////////////////////////////////////////////////////////////
void Rfm75Ptx(void)
{
	Rfm75WriteRegister(FLUSH_TX, 0);

	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_CONFIG);
	
	Rfm75ResetCe();
	
	state &= 0xFE;			//Clear bit 0
	
	Rfm75WriteRegister(W_REGISTER | REGISTER_CONFIG, state);

	Rfm75SetCe();
}
//////////////////////////////////////////////////////////////////////////
//Set the mode of the module to PRX (primary receive)
//////////////////////////////////////////////////////////////////////////
void Rfm75Prx(void)
{
	uint8_t value;

	Rfm75WriteRegister(FLUSH_RX,0);//flush Rx

	value=Rfm75ReadRegister(STATUS);	// read register STATUS's value
	Rfm75WriteRegister(W_REGISTER|STATUS,value);// clear RX_DR or TX_DS or MAX_RT interrupt flag

	Rfm75ResetCe();

	value=Rfm75ReadRegister(REGISTER_CONFIG);	// read register CONFIG's value
	
	//PRX
	value=value|0x01;//set bit 1
	Rfm75WriteRegister(W_REGISTER | REGISTER_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
	Rfm75SetCe();
}
//////////////////////////////////////////////////////////////////////////
//Flushes the TX fifo content
//////////////////////////////////////////////////////////////////////////
void Rfm75FlushTxBuffer(void)
{
	CSPORT &= ~CS;
	
	SPI_Write(FLUSH_TX);

	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Set the CE Port of the module
//////////////////////////////////////////////////////////////////////////
void Rfm75SetCe(void)
{
	CEPORT |= CE;
}
//////////////////////////////////////////////////////////////////////////
//Set the CE Port of the module
//////////////////////////////////////////////////////////////////////////
void Rfm75ResetCe(void)
{
	CEPORT &= ~CE;
}
//////////////////////////////////////////////////////////////////////////
//Writes one register
//////////////////////////////////////////////////////////////////////////
void Rfm75WriteRegister(uint8_t reg, uint8_t value)
{
	CSPORT &= ~CS;
	SPI_Write(W_REGISTER | reg);
	SPI_Write(value);
	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Writes one register
//////////////////////////////////////////////////////////////////////////
void Rfm75WriteCommand(uint8_t reg, uint8_t value)
{
	CSPORT &= ~CS;
	SPI_Write(reg);
	SPI_Write(value);
	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Writes one multibyte register
//////////////////////////////////////////////////////////////////////////
void Rfm75WriteLongRegister(uint8_t reg, uint8_t *buffer, uint8_t length)
{
	CSPORT &= ~CS;
	SPI_Write(W_REGISTER | reg);
	for(int i = 0; i < length; i++)
	{
		SPI_Write(buffer[i]);
	}
	CSPORT |= CS;
}
void Rfm75WriteMultiple(uint8_t *buffer, uint8_t length)
{
	CSPORT &= ~CS;
	for(int i = 0; i < length; i++)
	{
		SPI_Write(buffer[i]);
	}
	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Read the content of one register
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75ReadRegister(uint8_t reg)
{
	CSPORT &= ~CS;
	SPI_Write(reg);
	uint8_t data = SPI_Read(0);
	CSPORT |= CS;
	return data;
}
//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
void Rfm75SelectBank(uint8_t bank)
{
	uint8_t tmp = Rfm75ReadRegister(0x07) & 0x80;
	if (bank)
	{
		if (!tmp)
		Rfm75WriteMultiple((uint8_t *)RFM75_cmd_switch_cfg, sizeof(RFM75_cmd_switch_cfg));
	}
	else
	{
		if (tmp)
		Rfm75WriteMultiple((uint8_t *)RFM75_cmd_switch_cfg, sizeof(RFM75_cmd_switch_cfg));
	}
}
void Rfm75ReadBuffer(uint8_t reg, uint8_t *pBuf, uint8_t length)
{
	uint8_t byte_ctr;
	
	CSPORT &= ~CS;
	SPI_Read(reg);
	for(byte_ctr=0;byte_ctr<length;byte_ctr++)
	pBuf[byte_ctr] = SPI_Read(0);    // Perform SPI_RW to read UINT8 from RFM70
	CSPORT |= CS;
}