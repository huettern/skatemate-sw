/**
* main.c
*
* \brief Empty user application template
*
*/
#define ADCCHANNEL 0

#include <asf.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <SPI.h>
#include <RFM75.h>


void ADC_INIT(void);
uint16_t ADC_Read(void);
void ShowBatteryStatus(char cStatus);

char serial_data;
char data_save;
int transmit_completed= 0;
char temp = 0;
//////////////////////////////////////////////////////////////////////////
// Board-Initialization, Starting Interfaces, Setting up control loop
//////////////////////////////////////////////////////////////////////////
int main(void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	board_init();
	Rfm75Init();
	uint8_t temp = Rfm75Status();
	DDRD = 0xFF;
	PORTD = temp;
	/*Initialize SPI-Interface to RFM75 */
	/*SPI_init();
	sei();						//Enable global Interrupts!

	int i = 1;
	int j = 0;
	DDRD = 0xFF;
	// Initialize Tactile Switch on PB1
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
	}*/
	/*
		The main measurement will take place every 50 ms for now. If this proves to be too much/less it will be changed accordingly. 
		The chip is probably capable of something around a 10 ms rate at highest in regards that the RFM75 module is the bottleneck.
		Steps to measure and send data:
		- Request measurement from ADC-Channel 0 and actively wait for its data to return.
		- (ADDITIONAL): Maybe slowing down steep angles etc. could be done here to prevent the user from falling of due to higher accelerations.
		- Send data to the board without acknoledgement.(Therefore a 10ms value would be smart).
		- Wait for 50 ms to restart the process.
	*/
	while(1)
	{
		//READ data from analog input(FLEX SESOR)
		char cSensorData = ADC_Read();

		//ADDITIONAL: Clear Data

		//WRITE data to SPI-Interface
		SPI_Transceive(cSensorData);
		//Delay?!
	}
}
//////////////////////////////////////////////////////////////////////////
// Initializes the AD-Converter
//////////////////////////////////////////////////////////////////////////
void ADC_INIT(void)
{
	/* Initialize ADC */
	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADCSRA |= (1<<ADEN);
	//Dummy Readout as mentioned in datasheet
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {
	}
	(void) ADCW;
}
//////////////////////////////////////////////////////////////////////////
// Sets the 8 LED's according to the battery status
//////////////////////////////////////////////////////////////////////////
void ShowBatteryStatus(char cStatus)
{
	PORTD = cStatus;
}
//////////////////////////////////////////////////////////////////////////
// Reads the current ADC-Value
//////////////////////////////////////////////////////////////////////////
uint16_t ADC_Read(void)
{
	ADMUX = (ADMUX & ~(0x1F)) | (ADCCHANNEL & 0x1F);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {
	}
	return ADCW;
}
//////////////////////////////////////////////////////////////////////////
// Dedicated Interrupt routine for the Tactile Switch
//////////////////////////////////////////////////////////////////////////
/*ISR()
{
//Send Battery status request

}*/
//////////////////////////////////////////////////////////////////////////
// Dedicated Interrupt routine for the SPI Status register
//////////////////////////////////////////////////////////////////////////
/*ISR(SPI0_STC_vect)
{
	switch( SPSR0 ) //read and clear spi status register 
	{
		case 0x80:
		serial_data=SPDR0; // read receive data 
		transmit_completed=1;// set software flag 
		break;
		case 0x10:
		// put here for mode fault tasking
		break;
		case 0x40:
		// put here for overrun tasking
		break;
	}
}*/