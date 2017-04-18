/*
 * Batterymanagement.c
 *
 * Created: 06.04.2017 13:36:31
 * Author : Mike
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>

enum Routine {one,two,tree};

void init_timer(){
	int compare = 128;						// 16MHz/256/compare
	TCCR0B |= (1<<WGM02);
	TCCR0B = (1<<CS02)|(1<<CS00);			// start Timer0 with Prescaler = 1024 (16MHz / 1024 / 256 = 61Hz ~16.4ms)
	OCR0A = compare;						// Match register
	TIMSK0=(1<<OCIE0A)|(1<<TOIE0);			// enable/allow Timer1 Compare A Match Interrupt Enable, overflow interrupt enable
	sei();									// allow interrupts
	
	// Timer 1 für Display refresh (250ms)
	int compare1 = 15625;			// Für 250ms: 16MHz/256
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<CS12);			// start Timer1 with Prescaler = 256 (16MHz / 256 / 65536 = ~0.95Hz 1.048S)
	OCR1A = compare1;				// Match register
	TIMSK1|=(1<<OCIE1A);			// enable/allow Timer1 Compare A Match Interrupt Enable
}

void init_IO(){
	/* 
		B0,B1,B2 Dout -> LED
		
	*/
	DDRB =0b11000010;	// Pin 4 als Eingabe (0)	,1 als Ausgabe (1)
	PORTB=0b00010000;	// Pin 4 Mit Pullup(1)		,1 augeschaltet(0)
	
	DDRC=0xff;			//alle als Ausgang
	PORTC=0x00;			//Alle aus
}

ISR(TIMER1_COMPA_vect){			//Display Refresh
	
}
ISR(TIMER0_COMPA_vect){
	
}
ISR(TIMER0_OVF_vect){
	
}
int statemachine(){
	static enum Routine state = one;	//init state Start
	
	switch(state){
		case one:
		break;
		case two:
		break;
		case tree:
		break;
	}
	return state;
}

int main(void)
{
	init_IO();
	init_timer();
	while (1)
	{
		statemachine();
		}
		
	}
}

