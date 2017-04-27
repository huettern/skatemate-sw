/*
* RFM75.c
*
* Created: 25.04.2017 22:32:32
*  Author: Farin
*/
/*
	This code is an adaption of the demonstration code provided by HopeRF for the RFM75 2.4GHz wireless module.
	Most of the functions aren't really used for this project and will be erased during development to save on 
	program memory on the microcontroller(ATMEGA328PB-AU).

	Important configuration details.
	Datarate is set to 250kbps(lowest possibility) and the gain to -10dbm to save on power consumption by the module.
	Further, the module doesn't wait for any ack-flags making it possible to provide a higher datarate and therefore
	a smoother user experience.

	The data sent will always consist of 16bit datawords that contain either a command or sensor data.


*/
#include <avr/io.h>
#include <util/delay.h>
//#include <avr/interrupt.h>			//Is the interrupt really necessary when we're using the CE-Flag and are actively waiting for the result to return?
#include <avr/pgmspace.h>
#include <stdint.h>
#include <RFM75.h>
#include <SPI.h>

#define BANK0LENGTH 9
//////////////////////////////////////////////////////////////////////////
// Initialization values for registers in bank 0
//////////////////////////////////////////////////////////////////////////
uint8_t Bank0_InitValues08[] =
{
	0b00001111,		//all Interrupts enabled, CRC 2 byte, Power up, PRX
	0b00000000,		//No auto ack on any pipe
	0b00000011,		//No pipe rx enabled
	0b00000011,		//Address length 5 byte
	0b00000000,		//Retransmission Delay 250탎, retransmission disabled
	0b00000000,		//Frequency 2400MHz (Channel 0)
	0b00010111,		//250Kbit air data rate, 5dBm TX power, LNA gain high
	0b01110000,		//Clear all interrupts (not disabling)
	0b00000000		//Reset lost packet counters
	//By default no dynamic payload is active, so we don't need to write here more registers
	//Dynamic Payload is inactive due to the fact that the actual data sent can always be stored in a 16bit integer. (Sensorvalue and request data.)
	/*
		Send "normal" sensor data to target.
		0b000000AA AAAAAAAA
		Send command data to target
		0b111111AA AAAAAAAA
			So far the only command needed is to ask for the battery voltage level of the target.
				0b11111100 00000001
	*/
};
#define BANK1VALUES05LENGTH	6
//////////////////////////////////////////////////////////////////////////
// Initialization Values for registers in bank 1
//////////////////////////////////////////////////////////////////////////
/*const uint8_t Bank1_InitValues05[][4] PROGMEM =
{
	{ 0x40, 0x4B, 0x01, 0xE2 },	//Register 0 to 0x404B01E2 (MSB first)
	{ 0xC0, 0x4B, 0x00, 0x00 },	//Register 1 to 0xC04B0000 (MSB first)
	{ 0xD0, 0xFC, 0x8C, 0x02 },	//Register 2 to 0xD0FC8C02 (MSB first)
	{ 0x99, 0x00, 0x39, 0x21 },	//Register 3 to 0x99003941 (MSB first)
	{ 0xD9, 0x9E, 0x86, 0x0B },	//Register 4 to 0xD99E860B (MSB first)
	{ 0xF9, 0x96, 0x8A, 0xDB }	//Register 5 to 0x24067FA6 (MSB first)
};*/
//////////////////////////////////////////////////////////////////////////
// Actual configuration values sent to the RFM75 module for registers in bank 1
//////////////////////////////////////////////////////////////////////////
uint8_t Bank1_Register0[] = { 0x40, 0x4B, 0x01, 0xE2 };
uint8_t Bank1_Register1[] = { 0xC0, 0x4B, 0x00, 0x00 };
uint8_t Bank1_Register2[] = { 0xD0, 0xFC, 0x8C, 0x02 };
uint8_t Bank1_Register3[] = { 0x99, 0x00, 0x39, 0x21 };
uint8_t Bank1_Register4[] = { 0x24, 0x06, 0x0F, 0xB6 };
uint8_t Bank1_Register5[] = { 0xF9, 0x96, 0x8A, 0xDB };

#define BANK1VALUESCDLENGTH	2
/*const uint8_t Bank1_InitValuesCD[][4] PROGMEM =
{
	{ 0x00, 0x12, 0x73, 0x05 },		//Register 0C to 0x05731200 (LSB first)
	{ 0x36, 0xB4, 0x80, 0x00 }		//Register 0D to 0x0080B436 (LSB first)
};*/
//////////////////////////////////////////////////////////////////////////
// Actual configuration values sent to the RFM75 module for registers in bank 1
//////////////////////////////////////////////////////////////////////////
uint8_t Bank1_RegisterC[] = { 0x00, 0x12, 0x73, 0x05 };
uint8_t Bank1_RegisterD[] = { 0x36, 0xB4, 0x80, 0x00 };
#define BANK1VALUESELENGTH	11
//////////////////////////////////////////////////////////////////////////
//	RFM75 Register 0E to 0xFFEF7DF208082082041041 (LSB first)
//////////////////////////////////////////////////////////////////////////
uint8_t Bank1_InitValuesE[] =
{
	0x41,		
	0x20,
	0x08,
	0x04,
	0x81,
	0x20,
	0xCF,
	0xF7,
	0xFE,
	0xFF,
	0xFF
};
//////////////////////////////////////////////////////////////////////////
// Initializes the RFM75 module with configuration values set on top of this file
//////////////////////////////////////////////////////////////////////////
void Rfm75Init()
{
	uint8_t counter;
	
	//Make the Pins output
	CEDDR |= CE;
	CSDDR |= CS;
	
	//Set initial state for the pins
	CEPORT &= ~CE;					//Switch off CE (chip enable) pin (CE pin is high active, so now it's inactive)
	CSPORT |= CS;					//Switch on CS (spi chip select) (CS pin is low active, so now it's inactive)
	
	//Module needs minimum 50ms initil delay after switching on
	_delay_ms(50);
	_delay_ms(50);
	_delay_ms(50);
	_delay_ms(50);
	//Maybe this is a little Overkill... But better be safe than sorry.

	//Write initial registers
	//Bank 0
	for(counter=0; counter<BANK0LENGTH; counter++)
	{
		Rfm75WriteRegister(counter, Bank0_InitValues08[counter]);
	}
	
	CSPORT &= ~CS;					//Activate the features
	SPI_Write(ACTIVATE);
	SPI_Write(ACTIVATE_FEATURE_VALUE);
	CSPORT |= CS;
	
	#if(USE_DYNAMIC_PAYLOAD == 1)
	Rfm75WriteRegister(REGISTER_DYNPD, 0b00111111);				//Enable dynamic payload on all pipes
	Rfm75WriteRegister(REGISTER_FEATURE, 0b00000111);			//Global enable dynamic payload, payload with ack and no auto ack packet
	#elif(USE_DYNAMIC_PAYLOAD == 0)
	Rfm75WriteRegister(REGISTER_FEATURE, 0b00000011);			//Global enable payload with ack and no auto ack packet
	#endif
	
	CSPORT &= ~CS;					//Switch to register bank 1
	SPI_Write(ACTIVATE);
	SPI_Write(SWITCH_BANK_VALUE);
	CSPORT |= CS;
	
	//Bank 1
	//for(counter=0; counter<BANK1VALUES05LENGTH; counter ++)
	//{
	//	Rfm75WriteLongRegister_P(counter, (uint8_t *)pgm_read_word(Bank1_InitValues05[counter]), 4);
	//}
	Rfm75WriteLongRegister(0, Bank1_Register0, 4);
	Rfm75WriteLongRegister(1, Bank1_Register1, 4);
	Rfm75WriteLongRegister(2, Bank1_Register2, 4);
	Rfm75WriteLongRegister(3, Bank1_Register3, 4);
	Rfm75WriteLongRegister(4, Bank1_Register4, 4);
	Rfm75WriteLongRegister(5, Bank1_Register5, 4);
	
	//for(counter=0x0C; counter<0x0C+BANK1VALUESCDLENGTH; counter ++)
	//{
	//	Rfm75WriteLongRegister_P(counter, (uint8_t *)pgm_read_word(Bank1_InitValuesCD[counter]), 4);
	//}
	
	Rfm75WriteLongRegister(0x0C, Bank1_RegisterC, 4);
	Rfm75WriteLongRegister(0x0D, Bank1_RegisterD, 4);
	Rfm75WriteLongRegister(0x0E, Bank1_InitValuesE, 11);

	CSPORT &= ~CS;					//Switch back to register bank 0
	SPI_Write(ACTIVATE);
	SPI_Write(SWITCH_BANK_VALUE);
	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Reads the statusbyte from the RFM75
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75Status(void)
{
	uint8_t value;
	
	CSPORT &= ~CS;
	
	value = SPI_Read(RFM75_NOP);
	
	CSPORT |= CS;
	
	return value;
}
//////////////////////////////////////////////////////////////////////////
//Send a packet with the RFM75 module, it has to be in PTX mode
//////////////////////////////////////////////////////////////////////////
void Rfm75SendPacket(uint8_t *buffer, uint8_t length)
{
	//Only necessary to write the payload into TX fifo and set the CE pin
	CSPORT &= ~CS;
	
	SPI_Write(W_TX_PAYLOAD);
	SPI_WriteBuffer(buffer, length);
	
	CSPORT |= CS;
	nop;
	
	Rfm75SetCe();
}
#if(SEND_NOAA_PACKET_FUNC == 1)
//////////////////////////////////////////////////////////////////////////
//Send a packet with the RFM75 module with the no auto ack bit set,
//The module has to be in PTX mode
//////////////////////////////////////////////////////////////////////////
void Rfm75SendPacketNoAA(uint8_t *buffer, uint8_t length)
{
	//Only necessary to write the payload into TX fifo and set the CE pin
	CSPORT &= ~CS;
	
	SPI_Write(W_TX_PAYLOAD_NO_ACK);
	SPI_WriteBuffer(buffer, length);
	
	CSPORT |= CS;
	nop();
	
	Rfm75SetCe();
}
#endif
#if(ACK_PAYLOAD_FUNCTION == 1)
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
#endif
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
	
	pipe = Rfm75Status();
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
//Set the transmit address of the PTX module
//////////////////////////////////////////////////////////////////////////
void Rfm75TxAddress(uint8_t *addressBytes, uint8_t length)
{
	Rfm75WriteLongRegister(REGISTER_TX_ADDR, addressBytes, length);
}
//////////////////////////////////////////////////////////////////////////
//Set the receive address of the pipe 0 (max 5 bytes)
//////////////////////////////////////////////////////////////////////////
void Rfm75RxAddress0(uint8_t *addressBytes, uint8_t length)
{
	Rfm75WriteLongRegister(REGISTER_RX_ADDR_P0, addressBytes, length);
}
//////////////////////////////////////////////////////////////////////////
//Set the receive address of the pipe 1 (max 5 bytes)
//////////////////////////////////////////////////////////////////////////
void Rfm75RxAddress1(uint8_t *addressBytes, uint8_t length)
{
	Rfm75WriteLongRegister(REGISTER_RX_ADDR_P1, addressBytes, length);
}
//////////////////////////////////////////////////////////////////////////
//Set the receive address of the pipe n (1 byte, shares upper 4 with pipe 1)
//N must be from 2 to 5
//////////////////////////////////////////////////////////////////////////
void Rfm75RxAddressN(uint8_t pipe, uint8_t lowestByte)
{
	Rfm75WriteRegister(REGISTER_RX_ADDR_P0 + pipe, lowestByte);
}
//////////////////////////////////////////////////////////////////////////
//Set the mode of the module to PTX (primary transmit)
//////////////////////////////////////////////////////////////////////////
void Rfm75Ptx(void)
{
	Rfm75ResetCe();

	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_CONFIG);
	
	state &= 0xFE;			//Clear bit 0
	
	Rfm75WriteRegister(REGISTER_CONFIG, state);
}
//////////////////////////////////////////////////////////////////////////
//Set the mode of the module to PRX (primary receive)
//////////////////////////////////////////////////////////////////////////
void Rfm75Prx(void)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_CONFIG);
	
	state |= 1;			//Set bit 0
	
	Rfm75WriteRegister(REGISTER_CONFIG, state);
	
	Rfm75SetCe();
}
//////////////////////////////////////////////////////////////////////////
//Set the channel (frequency) to send and receive on
//Frequency is 2400MHz + Channel [MHz]
//////////////////////////////////////////////////////////////////////////
void Rfm75Channel(uint8_t Channel)
{
	Rfm75WriteRegister(REGISTER_RF_CH, Channel & 0x7F);		// & 0x7F because bit 7 has to be 0
}
//////////////////////////////////////////////////////////////////////////
//Enable/disable a rx pipe
//////////////////////////////////////////////////////////////////////////
void Rfm75EnablePipe(uint8_t pipe, uint8_t enable)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_ENRXADDR);
	
	if(pipe > 5)
	pipe = 5;
	
	if(enable == 0)
	{
		state &= ~(1<<pipe);
	}
	else
	{
		state |= (1<<pipe);
	}
	
	Rfm75WriteRegister(REGISTER_ENRXADDR, state);
}
//////////////////////////////////////////////////////////////////////////
//Enable/disable auto ack on pipe
//////////////////////////////////////////////////////////////////////////
void Rfm75EnableAutoAck(uint8_t pipe, uint8_t enable)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_EN_AA);
	
	if(pipe > 5)
	pipe = 5;

	if(enable == 0)
	{
		state &= ~(1<<pipe);
	}
	else
	{
		state |= (1<<pipe);
	}
	
	Rfm75WriteRegister(REGISTER_EN_AA, state);
}
//////////////////////////////////////////////////////////////////////////
//Set the time delay for retransmission of a packet
//0 = 250탎
//1 = 500탎
//2 = 750탎
//...
//15 = 4ms
//////////////////////////////////////////////////////////////////////////
void Rfm75ReTxTime(uint8_t time)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_SETUP_RETR);
	
	if(time > 15)
	time = 15;

	state &= 0x0F;			//Set upper nibble to 0
	state |= (time<<4);	//Set upper nibble to retx time
	
	Rfm75WriteRegister(REGISTER_SETUP_RETR, state);
}
//////////////////////////////////////////////////////////////////////////
//Set the time delay for retransmission of a packet
//0 - 15 times
//////////////////////////////////////////////////////////////////////////
void Rfm75ReTxCount(uint8_t count)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_SETUP_RETR);
	
	if(count > 15)
	count = 15;

	state &= 0xF0;			//Set lower nibble to 0
	state |= count;			//Set lower nibble to retx count
	
	Rfm75WriteRegister(REGISTER_SETUP_RETR, state);
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
#if(USE_DYNAMIC_PAYLOAD == 0)
//////////////////////////////////////////////////////////////////////////
//Set static payload length for pipe
//0 - 32 byte
//////////////////////////////////////////////////////////////////////////
void Rfm75PayloadLength(uint8_t pipe, uint8_t count)
{
	if(count > 32)
	count = 32;
	
	if(pipe > 5)
	pipe = 5;

	Rfm75WriteRegister(REGISTER_RX_PW_P0 + pipe, count);
}
#endif
#if(LOST_PACKETS_FUNCTIONS == 1)
//////////////////////////////////////////////////////////////////////////
//Returns the value how often the last packet were retransmitted
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75CountArc(void)
{
	uint8_t value;
	
	CSPORT &= ~CS;
	
	SPI_Write(R_REGISTER + REGISTER_OBSERVE_TX);
	value = SPI_Read(RFM75_NOP);
	
	CSPORT |= CS;
	
	value &= 0x0F;
	
	return value;
}
//////////////////////////////////////////////////////////////////////////
//Returns the value how many packets were lost since last channel (frequency) set
//This value can maximum be 15 and will overflow at 15 to zero
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75CountLoss(void)
{
	uint8_t value;
	
	CSPORT &= ~CS;
	
	SPI_Write(R_REGISTER + REGISTER_OBSERVE_TX);
	value = SPI_Read(RFM75_NOP);
	
	CSPORT |= CS;
	
	value = (value>>4) & 0x0F;
	
	return value;
}
#endif
#if(FIFO_STATUS_FUNCTION == 1)
//////////////////////////////////////////////////////////////////////////
//Returns the value from the fifo status register
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75FifoStatus(void)
{
	uint8_t value;
	
	CSPORT &= ~CS;
	
	SPI_Write(R_REGISTER + REGISTER_FIFO_STATUS);
	value = SPI_Read(RFM75_NOP);
	
	CSPORT |= CS;
	
	return value;
}
#endif
#if(CRC_FUNCTIONS == 1)
//////////////////////////////////////////////////////////////////////////
//Enable/Disable the use of the crc (checksum) in the transmission over the air
//////////////////////////////////////////////////////////////////////////
void Rfm75EnableCrc(uint8_t enable)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_CONFIG);
	
	if(enable > 1)
	enable = 1;

	state &= 0x77;			//Set bit 7 and bit 3 to zero
	state |= (enable<<3);	//Set bit 3 to state of enable
	
	Rfm75WriteRegister(REGISTER_CONFIG, state);
}
//////////////////////////////////////////////////////////////////////////
//Set number of bytes used for crc (checksum) for transmission over the air
//Count must be the value 1 or 2 [bytes] if the values is over 2 it will be set to 2
//If value is 0 it will result in an value for 2 bytes crc
//////////////////////////////////////////////////////////////////////////
void Rfm75CrcBytes(uint8_t count)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_CONFIG);
	
	count --;
	if(count > 2)
	count = 1;

	state &= 0x7B;			//Set bit 7 and bit 2 to zero
	state |= (count<<2);	//Set bit 2 to state of count
	
	Rfm75WriteRegister(REGISTER_CONFIG, state);
}
#endif


#if(POWER_UP_DOWN_FUNCTION == 1)
//////////////////////////////////////////////////////////////////////////
//Set the power up/down bit
//If power_up bit is 0 the module is a power down mode with a very low power consumption, but no RF rx/tx function is active
//////////////////////////////////////////////////////////////////////////
void Rfm75PowerMode(uint8_t powerMode)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_CONFIG);
	
	if(powerMode > 1)
	powerMode = 1;

	state &= 0xFD;				//Set bit 1 to zero
	state |= (1<<powerMode);	//Set bit 1 to state of powerMode
	
	Rfm75WriteRegister(REGISTER_CONFIG, state);
}
#endif
#if(TX_POWER_FUNCTION == 1)
//////////////////////////////////////////////////////////////////////////
//Set the transmission RF power of the module
//Values for power:
//0: -10dBm
//1: -5dBm
//2: -0dBm
//3: +5dBm
//Values over 3 will result in 3
//////////////////////////////////////////////////////////////////////////
void Rfm75OutputPower(uint8_t power)
{
	uint8_t state;
	
	state = Rfm75ReadRegister(REGISTER_RF_SETUP);
	
	if(power > 3)
	power = 3;

	state &= 0x29;			//Set bit 7,6,4,2,1 to zero
	state |= (power<<1);	//Set bit 2 to state of count
	
	Rfm75WriteRegister(REGISTER_RF_SETUP, state);
}
#endif
#if(ADDRNBYTE_FUNCTIONS == 1)
//////////////////////////////////////////////////////////////////////////
//Set the number of addressbytes to use
//Values: 3,4,5
//Values under 3 will result in 3, values over 5 will result in 5
//////////////////////////////////////////////////////////////////////////
void Rfm75AddrBytes(uint8_t count)
{
	if(count < 3)
	count = 3;
	if(count > 5)
	count = 5;

	count -= 2;		//0->Illegal; 1->3byte; 2->4byte; 3->5byte
	Rfm75WriteRegister(REGISTER_SETUP_AW, count);
}
#endif
#if(RFDATA_RATE_FUNCTION == 1)
//////////////////////////////////////////////////////////////////////////
//Set the air data rate of the module (250kBit, 1MBit, 2Mbit)
//Values 0: 250kBit, 1: 1MBit, 2: 2MBit
//////////////////////////////////////////////////////////////////////////
void Rfm75AirDataRate(uint8_t datarate)
{
	uint8_t state;

	//Translate the easy values for the user into the valuesfor the register
	if(datarate == 0)
	datarate = 0b00100000;
	else if(datarate == 1)
	datarate = 0b00000000;
	else if(datarate == 2)
	datarate = 0b00001000;
	
	
	state = Rfm75ReadRegister(REGISTER_RF_SETUP);

	state &= 0x07;			//Set bit 7,6,5,4,3 to zero
	state |= datarate;		//Set data rate bits
	
	Rfm75WriteRegister(REGISTER_RF_SETUP, state);
	
}
#endif
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
//Writes one multibyte register
//////////////////////////////////////////////////////////////////////////
void Rfm75WriteLongRegister(uint8_t reg, uint8_t *buffer, uint8_t length)
{
	CSPORT &= ~CS;
	SPI_Write(W_REGISTER | reg);
	SPI_WriteBuffer(buffer, length);
	CSPORT |= CS;
}
//////////////////////////////////////////////////////////////////////////
//Read the content of one register
//////////////////////////////////////////////////////////////////////////
uint8_t Rfm75ReadRegister(uint8_t reg)
{
	uint8_t state;
	CSPORT &= ~CS;
	SPI_Write(R_REGISTER | reg);
	state = SPI_Read(RFM75_NOP);
	CSPORT |= CS;
	
	return state;
}