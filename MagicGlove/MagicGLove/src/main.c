/**
 * main.c
 *
 * \brief Empty user application template
 *
 */


#define F_CPU 8000000UL
#include <asf.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void SPI_init(void);
char SPI_Transceive(char cData);

char serial_data;
char data_save;
int transmit_completed= 0;
char temp = 0;
//////////////////////////////////////////////////////////////////////////
// Board-Initialization, Starting Interfaces, Setting up control loop
//////////////////////////////////////////////////////////////////////////
int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	board_init();
	SPI_init();
	SPI_Transceive(0b00000000);	//Read Register CONFIG
	sei();						//Enable global Interrupts!
	int i = 1;
	int j = 0;
	DDRD = 0xFF;
	DDRB &= ~(1 << PB1);      // PB1 as Input
	PORTB &= ~(1 << PB1);     // PB1 as High-Z Input(Tri-State)
	while(1)
	{
		if((PINB & (1 << PB1))) 
		{
			PORTD = 0xFF;
		}
		_delay_ms(50);
		PORTD = i;
		if(j == 0) {
			i = i*2;
		}
		else {
			i = i/2;
		}
		if(i == 0x80) {
			j = 1;
		}
		if(i == 1) {
			j = 0;
		}
	}
	while(1)
	{
		//READ data from analog input(FLEX SESOR)

		//ADDITIONAL: Clear Data

		//WRITE data to SPI-Interface

		//Wait for Acknoledge?!

		//Delay?!
		
	}
}
//////////////////////////////////////////////////////////////////////////
// Routine to initialize the SPI-Interface to the RFM75-Module
//////////////////////////////////////////////////////////////////////////
void SPI_init(void)
{
	//TODO: Include chip select not pin of RFM75; So far it's configured as input!!!! Change before use!!
	DDRB = ((1<<DDB5)|(1<<DDB4)|(1<<DDB3)); //spi pins on port b MOSI SCK,SS outputs
	SPCR = ((1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<CPOL)|(1<<CPHA));  // SPI enable, Master, f/16
}
//////////////////////////////////////////////////////////////////////////
// Routine to send 1 char over SPI
//////////////////////////////////////////////////////////////////////////
char SPI_Transceive(char cData)
{
	SPDR = cData;
	while(!(SPSR & (1<<SPIF)))
	;
	return SPDR;
}
//////////////////////////////////////////////////////////////////////////
// Dedicated Interrupt routine for the SPI Status register
//////////////////////////////////////////////////////////////////////////
ISR(SPI_STC_vect)
{
	switch( SPSR ) /* read and clear spi status register */
	{
		case 0x80:
			serial_data=SPDR; /* read receive data */
			transmit_completed=1;/* set software flag */
		break;
		case 0x10:
			/* put here for mode fault tasking */
		break;
		case 0x40:
			/* put here for overrun tasking */
		break;
	}
}
//////////////////////////////////////////////////////////////////////////
// Dedicated Interrupt routine for the Tactile Switch 
//////////////////////////////////////////////////////////////////////////
/*ISR()
{
	//Send Battery status request 

}*/