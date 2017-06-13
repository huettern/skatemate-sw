/**
* main.c
*
* \brief Empty user application template
*
*/
#include <stdlib.h>
#include <string.h>

#define ADCCHANNEL 0

//avr headers
#include <avr/io.h>
#include <util/delay.h>

#include "spi.h"
#include "rfm7x.h"

char buf[32];
void ADC_INIT(void);
uint16_t ADC_Read(uint8_t);
int main(void)
{
	DDRD = 0xFF;
	//PORTB.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
	//clk_init();
	ADC_INIT();
	rfm_io_init();
	spi_init();

	//PORTB.OUTSET = PIN0_bm;
	while(!rfm7x_is_present()); // wait for end of rfm POR // it takes something about 16 ms
	//PORTB.OUTCLR = PIN0_bm;

	rfm7x_init();
	
	_delay_ms(2); // >1,5ms startup delay
	rfm7x_toggle_reg4(); // couldn't reproduce any "PLL is not locked" condition, but better to do it after all (probably have to be executed after every power up)
	_delay_ms(0.5); // probably not necessary, but it is said to be used

	rfm7x_mode_transmit();
	uint8_t lastvalue;
	while (1)
	{/*
		strcpy(buf,"counter: ");
		ltoa(counter, (char*)&buf[9], 10);*/
		uint16_t temp2 = ADC_Read(0);
		buf[0] = temp2*0.25;
		//buf[1] = temp&0x0F;
		PORTD = (double)temp2*0.25;
		lastvalue = temp2;
		while (rfm7x_tx_fifo_full())
		{
			uint8_t tmp = rfm7x_reg_read(RFM7x_REG_STATUS);
			if (tmp & RFM7x_STATUS_IRQ_MAX_RT)
			{
				//PORTB.OUTSET = PIN2_bm;

				rfm7x_mode_transmit(); // have the same affect as the following 2 lines

				//rfm7x_reg_write(RFM7x_CMD_WRITE_REG | RFM7x_REG_STATUS, 0x70); // nRF24l01+ and SI24R1 can live only with this line
				//rfm7x_reg_write(RFM7x_CMD_FLUSH_TX, 0); // but bk242x is "protected" from overrunning MAX_RT counter - it have to be executed to unlock any further transmissions in AA mode

				//_delay_ms(1);
				//PORTB.OUTCLR = PIN2_bm;
			}
		}

		rfm7x_transmit((uint8_t *)buf, 1);

		_delay_ms(10);

		//PORTB.OUTTGL = PIN1_bm;
	}
}
/*
#define ADCCHANNEL 0

#include <avr/interrupt.h>
#include <util/delay.h>
#include "SPI.h"
#include "RFM75.h"

void ADC_INIT(void);
uint16_t ADC_Read(void);
void ShowBatteryStatus(char cStatus);

//////////////////////////////////////////////////////////////////////////
// Board-Initialization, Starting Interfaces, Setting up control loop
//////////////////////////////////////////////////////////////////////////
int main(void)
{
	DDRE = 0x0B;
	DDRD = 0xFF;
	if(Rfm75Init() == 0xFF)
	{
		PORTD = 0xFF;
	}
	uint8_t data[32] = { 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A, 0xFF, 0x0A };
	Rfm75Ptx();
	Rfm75SendPacket(data, 32);
	
	//DDRB &= ~(1 << PORTB1);      // PB1 as Input
	//PORTB &= ~(1 << PORTB1);     // PB1 as High-Z Input(Tri-State)
	DDRD = 0xFF;
	CSDDR |= CS;
	 Insert system clock initialization code here (sysclk_init()). 
	SPI_init();
	CSPORT &= ~CS;
	SPI_Write(0x07);
	uint8_t data = SPI_Read(0);
	CSPORT |= CS;
	_delay_ms(250);
	//Switch register bank
	CSPORT &= ~CS;
	SPI_Write(0x50);
	SPI_Write(0x53);
	CSPORT |= CS;
	//Read chip id
	_delay_ms(250);
	CSPORT &= ~CS;
	SPI_Write(0x08);
	data = SPI_Read(0);
	CSPORT |= CS;
	if(data == 0x63)
	{
		PORTD = 0xFF;
	}*/
	/*Rfm75Init();
	//Rfm75SetCe();
	while(1)
	{
	uint8_t data = 0xFF;
	CSPORT &= ~CS;
	SPI_Write(ACTIVATE);
	CSPORT |= CS;
	CSPORT &= ~CS;
	SPI_Write(SWITCH_BANK_VALUE);
	CSPORT |= CS;
	CSPORT &= ~CS;
	Rfm75ReadRegister(0x08);
	CSPORT |= CS;
	for(int i = 0; i < 5; i++)
	{
	
		CSPORT &= ~CS;
		data = SPI_Read(0xFF);
		PORTD = data;
		CSPORT |= CS;
}
CSPORT |= CS;
	_delay_ms(1000);
	}
	while(1)
	{
		

	}






	// Initialize Tactile Switch on PB1
	int i = 1;
	int j = 0;
	uint8_t tempdata[1] = { 0xFF } ;
	while(1)
	{
		tempdata[0] = 0xFF;
		Rfm75SendPacket(tempdata, 1);
		PORTD = 0xFF;
		_delay_ms(1000);
		tempdata[0] = 0x00;
		Rfm75SendPacket(tempdata, 1);
		PORTD = 0x00;
		_delay_ms(1000);
	}
	while(1)
	{
		if((PINB & (1 << PORTB1)))
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
	ADC_INIT();
	while(1)
	{
		int temp = ADC_Read();
		PORTD = temp;
		_delay_ms(50);	
	}
	while(1)
	{	
		if((PINB & (1 << PORTB1)))
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
	
	The main measurement will take place every 50 ms for now. If this proves to be too much/less it will be changed accordingly.
	The chip is probably capable of something around a 10 ms rate at highest in regards that the RFM75 module is the bottleneck.
	Steps to measure and send data:
	- Request measurement from ADC-Channel 0 and actively wait for its data to return.
	- (ADDITIONAL): Maybe slowing down steep angles etc. could be done here to prevent the user from falling of due to higher accelerations.
	- Send data to the board without acknoledgement.(Therefore a 10ms value would be smart).
	- Wait for 50 ms to restart the process.
	
	while(1)
	{
		//READ data from analog input(FLEX SESOR)
		char cSensorData = ADC_Read();

		//ADDITIONAL: Clear Data

		//WRITE data to SPI-Interface
		SPI_Transceive(cSensorData);
		//Delay?!
	}
}*/
//////////////////////////////////////////////////////////////////////////
// Initializes the AD-Converter
//////////////////////////////////////////////////////////////////////////
void ADC_INIT(void)
{
	//Initialize ADC 
	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADPS2);
	ADCSRA |= (1<<ADEN);
	//Dummy Readout as mentioned in datasheet
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {
	}
	(void) ADCW;
}/*
//////////////////////////////////////////////////////////////////////////
// Sets the 8 LED's according to the battery status
//////////////////////////////////////////////////////////////////////////
void ShowBatteryStatus(char cStatus)
{
	PORTD = cStatus;
}*/
//////////////////////////////////////////////////////////////////////////
// Reads the current ADC-Value
//////////////////////////////////////////////////////////////////////////
uint16_t ADC_Read(uint8_t channel)
{
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {
	}
	return ADCW;
}
/*
//////////////////////////////////////////////////////////////////////////
// Dedicated Interrupt routine for the Tactile Switch
//////////////////////////////////////////////////////////////////////////
ISR()
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