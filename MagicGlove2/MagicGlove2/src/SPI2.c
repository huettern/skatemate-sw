/*
* SPI.c
*
* Created: 25.04.2017 22:41:59
*  Author: Farin
*/
#include <avr/io.h>
#include <stdint.h>
#include "SPI.h"
//////////////////////////////////////////////////////////////////////////
// Routine to initialize the SPI-Interface to the RFM75-Module
//////////////////////////////////////////////////////////////////////////
void SPI_init(void)
{	
	//TODO: Include chip select not pin of RFM75; So far it's configured as input!!!! Change before use!!
	DDRB  |= ((1<<DDB5)|(1<<DDB3)|(1<<DDB2)); //spi pins on port b MOSI SCK,SS outputs
	PORTB |= (1<<DDB2);
	SPCR0  = ((1<<SPE)|(1<<MSTR)|(1<<SPR0));  // SPI enable, Master, f/16
}
//////////////////////////////////////////////////////////////////////////
// Routine to send 1 char over SPI
//////////////////////////////////////////////////////////////////////////
char SPI_Transceive(char cData)
{
	SPDR0 = cData;
	while(!(SPSR0 & (1<<SPIF)));
	return SPDR0;
}
//////////////////////////////////////////////////////////////////////////
// reads data from the spi-interface receive-buffer
//////////////////////////////////////////////////////////////////////////
uint8_t SPI_Read(uint8_t data)
{
	//Write data into SPI data register
	SPDR0 = data;
	//Wait until transfer is completed
	while(!(SPSR0 & (1<<SPIF)));
	//Return received data from the data register
	return SPDR0;
}
//////////////////////////////////////////////////////////////////////////
// Writes data to the spi-interface
//////////////////////////////////////////////////////////////////////////
void SPI_Write(uint8_t data)
{
	//Write data into SPI data register
	SPDR0 = data;
	//Wait until transfer is completed
	while(!(SPSR0 & (1<<SPIF)));
}
//////////////////////////////////////////////////////////////////////////
// Writes the data to the spi-interface send-buffer
//////////////////////////////////////////////////////////////////////////
void SPI_WriteBuffer(uint8_t *data, uint8_t length)
{	
	uint8_t i;
	for(i=0; i<length; i++)
	{
		SPI_Write(data[i]);
	}
}

