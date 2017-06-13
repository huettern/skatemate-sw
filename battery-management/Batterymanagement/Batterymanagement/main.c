/*
 * Projekt 4
 * Projectname: Battery Management
 * creator:     Mike Mössner
 * date:        07.05.2017
*/
//#include <SoftwareSerial.h>  
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define MAX_CURRENT			  100	// ??????????? max current for PWM
#define MIN_CURRENT			  0		// min Current for PWM
#define CELL_VOLTAGE_MAX      880   //(1024/5V)*4,3V  =880,64 =~880
#define CELL_VOLTAGE_SOLL     850   //(1024/5V)*4,15V =849,92 =~850
#define CELL_VOLATGE_MIN      615   //(1024/5V)*3,00V =614,40 =~615
#define LIMIT_LOW_CURRENT     50    // ??? switches from CV => OFF
#define BALANCER_HYSTERESE    10    // Unterschied zwischen Min/Max Spannung 0,05V*(1024/5V)= 10,24
#define PWM                   OCR0A
#define BAUDRATE			  9600UL

enum Routine{Battery, Charge_CC, Charge_CV, Balance, Shutdown}state;
volatile unsigned int second=0;     //Timeout counter
int cell[6];                        //cellVoltage
int cell_Max      =0;
int cell_Min      =0;
int current       =0;
int inputVoltage  =0;


void init_timer(void){      
  //Timer 1    
  TCCR1A =  0x00;           // Clear Register (some bits were Preset...)
  TCCR1B =  (1<<WGM12);     // Clear Timer on Compare match (CTC) mode (mode 4)
  TCCR1B|=  (1<<CS12);      // start Timer1 with Prescaler = 256 (16MHz^-1 * 256 * 65536 = max ~2,17s)          
  OCR1A  =  31250;          // 1 sec tics (Match Register A) (8MHz/256)
  TIMSK1|=  (1<<OCIE1A);    // enable/allow Timer1 Compare A Match Interrupt Enable
  sei();                    // enable Interrupts
}

ISR(TIMER1_COMPA_vect){
    second++;   
}

void init_PWM(void){
    //Timer 0
    TCCR0A |= (1 << COM0A1);                // Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode)
    TCCR0A |= (1 << WGM01) | (1 << WGM00);  // fast PWM Mode
    TCCR0B |= (1 << CS00);                  // set prescaler to 1 and starts PWM (8MHz*(1*256bit)^-1 = 31.250 kHz
    OCR0A = 0;                              // PWM OFF
  }

void init_IO(){
  /*
 *  Port B:
 *  PB0 LED2
 *  PB1 LED1
 *  PB2 LED3
 *  PB3 MOSI
 *  PB4 MISO
 *  PB5 SCK
 *  PB6 NC
 *  PB7 Selbsthaltung
 */
    DDRB  = 0b11100001;
    PORTB = (1<<7);    // Selbsthaltung ein
    
   /*
 *  Port C:
 *  PC0 ADC cell 1
 *  PC1 ADC cell 2
 *  PC2 ADC cell 3
 *  PC3 ADC cell 4
 *  PC4 ADC cell 5
 *  PC5 ADC cell 6
 *  PC6 Reset pin 
 *  PC7 NC
 */  
    DDRC  = 0x00;      // PORTC 0(PCB Pin A0) > Input
    PORTC = 0x00;      // no Pullups
    
  /*
 *  Port D:
 *  PD0 Balance cell 1
 *  PD1 Balance cell 2
 *  PD2 Balance cell 3
 *  PD3 Balance cell 4
 *  PD4 Balance cell 5
 *  PD5 Balance cell 6
 *  PD6 PWM output
 *  PD7 Engine Control output
 */  
    DDRD  = 0xFF;
    PORTD = 0x00;
  }

void init_ADC(){
    ADMUX   = (1<<REFS0);                             // URef= VCC
    ADCSRB  = 0x00;                                   // clear register
    ADCSRA  = (1<<ADPS2)|(1<<ADPS1);                  // Prescaler 64:  8GHz/64 = 125kHz (50-200kHz)    
    ADCSRA |= (1<<ADEN);                              // Enable ADC
  }

int ADC_Read(int ch)
{
  ADMUX &= 0b11111000;      // clear MUX bit
  ADMUX |= ch;              // select new channel
  
  ADCSRA|=(1<<ADSC);        //trigger Conversion
  while(ADCSRA&(1<<ADSC))   //wait for 1st Conversion finished
  {                        
  }// drop first conversation after channel switch 
  
  ADCSRA|=(1<<ADSC);        //trigger Conversion
  while(ADCSRA&(1<<ADSC))   //wait for 2st Conversion finished
  {
  }
  return ADC;
}


void ReadCell(void){
        // Read
        cell[0]=ADC_Read(0);      
        cell[1]=(int)(ADC_Read(1)*2-ADC_Read(0));     
        cell[2]=(int)(ADC_Read(2)*3.04-ADC_Read(1)*2.03);
        cell[3]=(int)(ADC_Read(3)*4.02-ADC_Read(2)*3.04);
        cell[4]=(int)(ADC_Read(4)*4.96-ADC_Read(3)*4.02);
        cell[5]=(int)(ADC_Read(5)*6.15-ADC_Read(4)*4.96);
        
        current= ADC_Read(6); 
        inputVoltage= ADC_Read(7);
                
        //find Min/Max
        cell_Max=cell[0];
        cell_Min=cell[0];
        for(int i=0; i<=5; i++){ 
          if(cell[i]>cell_Max){cell_Max=cell[i];} 
          if(cell[i]<cell_Min){cell_Min=cell[i];} 
          }

        // check for Errors/ undervoltage protection
        if(cell_Min<=CELL_VOLATGE_MIN){
          state=Shutdown;
		}
		
		// check if the cord got pulled
		if(current<= 10){	// no current measured
		  PORTB&=~(0b111);	// Turn off LED1 - LED3
		  state=Battery;
		
        } 
  }
  
void Statemachine(){
  int old_Max = cell_Max;
  static int kp;	//Regelung P Anteil
  static int ki;	//Regelung I Anteil
  
  switch(state){
    
    case Battery:
      PORTD&=~(0b111111);     // C1-C6 off  (Balancer Transistoren)
      PWM   = 0x00;           // PWM_OFF
	  
      if(inputVoltage>=300){
        PORTD&=~(1<<7);         // Shutoff EngineControl (Noah)
        state=Charge_CC;
        }
      else{
        PORTD|=(1<<7);         // Power on EngineControl (Noah)
        }
        
      // Check if Board is in use
       if(second>=60){
          if(cell_Max+2 < old_Max){   // used more than 0.01V/min of the battery
          second=0;                   //reset Timer
          }
        }
       if(second>=600){          // Timeout after 10 min no battery usage
          state=Shutdown;  
       }
      break;  
      
    case Charge_CC:
        PORTB |= (1<<1);  //Turn on LED 1
        
        if(cell_Max>=CELL_VOLTAGE_MAX){
          state=Balance;
          }
		
		// PI-Regler
        if(PWM<=MAX_CURRENT){
		  /* dt=1.0/fs
		  *  error= soll-ist
		  *  itern += error*dt*ki
		  *  if(item>MAX) {iterm=MAX}
		  *  PWM=kp*error+iterm;
		  */
		  
          PWM++;  // increment current (PWM)
          }
		  
		  
        if(cell_Min>=CELL_VOLTAGE_SOLL){
          state=Charge_CV;
          }
        break;

    case Charge_CV:
        PORTB |= (1<<0);  //Turn on LED 2
        
        if(cell_Max>CELL_VOLTAGE_SOLL && PWM>=MIN_CURRENT){
          PWM--; // decrement current (PWM)
        }
        else if(cell_Max<=CELL_VOLTAGE_SOLL && PWM<=MAX_CURRENT){
          PWM++;  // increment current (PWM)
          }
        if(cell_Max>=CELL_VOLTAGE_MAX){
          state=Balance;
          }
        if(current<= LIMIT_LOW_CURRENT){
		  PWM=0x00;
		  PORTD|=(1<<3);	//Turn on LED3
          state=Shutdown;
          }
        break;

    case Balance:
        PWM=0x00;
        for(int i=0; i<=5; i++){
          if(cell[i]>cell_Min){
            //zelle i entladen
            PORTD |= (1<<i); //Ci on (Balancer Transistoren)
          }
          else{
            // zellentransistor schliessen 
            PORTD&=~(1<<i);     //Ci off(Balancer Transistoren)             
          }
        }
        
          if(cell_Max<=cell_Min + BALANCER_HYSTERESE){
            PORTD&=~(0b111111);     //C1-C6 off  (Balancer Transistoren)     
            state=Charge_CC;
            }
        break;
        
    case Shutdown:
		  PWM=0x00;			// PWM off
          while(1){
          PORTB&=~(1<<7);   // Selbsthaltung off
          }
          break;
  }
}

void init()
{
  init_IO();
  init_timer();
  init_ADC();
  state=Battery;      //  Start State            
}

 int uart_putc(const uint8_t c)
 {
	 // Warten, bis UDR bereit ist für einen neuen Wert
	 while (!(UCSR0A & (1 << UDRE0)));
	 // UDR Schreiben startet die Übertragung
	 UDR0 = c;
	 return 1;
 }

 int uart_putLine(char *c)
 {
	 for(int i = 0; i < strlen(c); i++)
	 {
		 uart_putc(c[i]);
	 }
	 uart_putc('\r');
	 uart_putc('\n');
	 return 1;
 }

 void USART_Init()
 {
	 uint16_t ubrr = (uint16_t) ((uint32_t) F_CPU/(16*BAUDRATE) - 1);
	 /*Set baud rate to 9600 */
	 UBRR0H = (uint8_t) (ubrr>>8);
	 UBRR0L = (uint8_t) (ubrr);
	 /*Enable receiver and transmitter */
	 UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	 UCSR0A = (1<<UDRE0);
	 /* Set frame format: 8data, 1stop bit */
	 //UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	 UCSR0C = (1 << UMSEL01) | (1 << UCSZ01) | (1 << UCSZ00);    // Set frame: 8data, 1 stp
 }


 int main(){
	 init();
	 USART_Init();
	 while(1){
		 ReadCell();
		 //Statemachine();
		 PORTB |=(1<<0)| (1<<1);
		 
		 char buffer[5];
		 while (!(UCSR0A & (1 << UDRE0)));
		 /* Serial.print("State : ");
		 Serial.println(statetext[state]);
		 
		 Serial.print("MAX   : ");
		 Serial.print((double)cell_Max/204,8);
		 Serial.print(" V (");
		 Serial.print(cell_Max);
		 Serial.println(")");
		 
		 Serial.print("MIN   : ");
		 Serial.print((double)cell_Min/204,8);
		 Serial.print(" V (");
		 Serial.print(cell_Min);
		 Serial.println(")");
		 */
		 static char *statetext[]={"BAT", "CCC", "CCV", "BAL", "SHT"};
		 uart_putLine("=================================");
		 uart_putLine("Zeit");
		 uart_putLine(itoa(second,buffer,10));
		 uart_putLine("PWM:");
		 uart_putLine(itoa(((PWM*100)/256),buffer,10));
		 uart_putLine("Min: ");
		 uart_putLine(cell_Min);
		 uart_putLine("MAX: ");
		 uart_putLine(cell_Max);
		 uart_putLine("State: ");
		 uart_putLine(statetext[state]);
		 
		 uart_putLine("\nZelle 1:");
		 uart_putLine(itoa(cell[0],buffer, 10));
		 uart_putLine("Zelle 2:");
		 uart_putLine(itoa(cell[1],buffer, 10));
		 uart_putLine("Zelle 3:");
		 uart_putLine(itoa(cell[2],buffer, 10));
		 uart_putLine("Zelle 4:");
		 uart_putLine(itoa(cell[3],buffer, 10));
		 uart_putLine("Zelle 5:");
		 uart_putLine(itoa(cell[4],buffer, 10));
		 uart_putLine("Zelle 6:");
		 uart_putLine(itoa(cell[5],buffer, 10));
		 uart_putLine("\n\n");
		 _delay_ms(500);
		 PORTB&=~(0b11);	// Turn off LED1 and LED2
		 _delay_ms(500);
		 
	 }
 }
