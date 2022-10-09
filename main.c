/*
 * GccApplication3.c
 *
 * Created: 10/22/2021 12:36:23 PM
 * Author : Nishant
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

char str[25];
unsigned long rising_edge = 0;
unsigned long falling_edge = 0;
unsigned long period =0;
unsigned int flag = 0;
unsigned int mode = 0, mode_flag = 0;
int dutyCycle = 50, update;

void Initializeadc(){
	
	cli();

	//clear for ADC
	//clear power reduction for ADC
	PRR &= ~(1<<PRADC);
	//select Vref = AVcc
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	//set the ADC clock div by 128
	//16M/128=125KHz
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	//select channel 0
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);
	//set to auto trigger
	ADCSRA |= (1<<ADATE);
	//Set to free running
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);
	//disable digital input
	DIDR0 |= (1<<ADC0D);
	//enable ADC interrupt
	ADCSRA |= (1<<ADEN);
	//enable ADC intterupt
	ADCSRA |= (1<<ADIE);
	//start conversion
	ADCSRA |= (1<<ADSC);

	sei();
}

void Initialize()
{
	cli();
	
	DDRD |= (1<<DDD5); //set D5 output

	TCCR0A = 0b01000011; 
	TCCR0B = 0b00001100; //PRESCALE 256
	OCR0A = 60;
	
	TIFR0 |= (OCF0A); //clearing interrupt flag
	TCNT0 = 0b00000000;

	DDRB |= (1<<DDB3); //Setting D6 as output
	TCCR2B |= (1<<CS20);

	// MODE - CTC
	TCCR2A &= ~(1 << WGM20);
	TCCR2A |= (1 << WGM21);
	TCCR2B &= ~(1 << WGM22);

	TCCR2A |= (1<<COM2A0);
	
	TCCR0A |= (1 << COM0A0);
	TCCR0A |= (1 << COM0B1);

	OCR2A = 159; //(1.6*10^7/(2*N*F desired)) - 1
	TIFR2 |= (OCF2A); //clearing interrupt flag
	TIMSK2 |= (1<<TOIE2);
	TCNT2 = 0b00000000;

	DDRB &= ~(1<<DDB0);

	//PRESCALE 8
	TCCR1B |= (1<<ICES1);
	TCCR1B &= ~(1<<CS12);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS10);

	//NORMAL MODE
	TCCR1A &= ~(1 << WGM10);
	TCCR1A &= ~(1 << WGM11);
	TCCR1B &= ~(1 << WGM12);
	TCCR1B &= ~(1 << WGM13);
	sei();
	TIMSK1 |= (1<<ICIE1);
	Initializeadc();
	
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT23);
}

ISR(PCINT2_vect)
{
	UART_putstring("Entering PIN Change Interrupt");
	if(mode_flag == 0)
	{
		mode = !mode;
		mode_flag = 1;
	}
	else
	mode_flag = 0;
	
}

void buzzer_val(unsigned long temp1){
	if (mode == 1)
	{
		if (temp1 >= 2 && temp1 <12 )
		{
			OCR0A = 28;
		}
		else if (temp1 >= 12 && temp1 <22 )
		{
			OCR0A = 30;
		}
		else if (temp1 >= 22 && temp1 <32 )
		{
			OCR0A = 34;
		}
		else if (temp1 >= 32 && temp1 <42 )
		{
			OCR0A = 38;
		}
		else if (temp1 >= 42 && temp1 <52 )
		{
			OCR0A = 42;
		}
		else if (temp1 >= 52 && temp1 <62 )
		{
			OCR0A = 44;
		}
		else if (temp1 >= 62 && temp1 <72 )
		{
			OCR0A = 51;
		}
		else if (temp1 >= 72 && temp1 <90 )
		{
			OCR0A = 58;
		}
		else
		{
			OCR0A = 0;
		}
	}
	else if(mode == 0)
		OCR0A = (temp1*0.5)+28;
}

void printval(unsigned long temp)
{
	unsigned long time;
	unsigned long dist;
	time = (float)temp * 1/2;
	dist = time * 0.032 / 2;
	_delay_ms(100);
	buzzer_val(dist);
}

ISR(TIMER2_OVF_vect){
	
	if(flag)
	{
		TCCR2B &= ~(1<<CS20);
		flag = 0;
	}
}

ISR(TIMER1_CAPT_vect){
	if (TCCR1B & (1<<ICES1)){
		rising_edge = ICR1;
	}
	else{
		falling_edge = ICR1;
	}
	if (rising_edge != 0 && falling_edge != 0) {
		period = falling_edge - rising_edge;
		rising_edge = 0;
		falling_edge = 0;
		printval(period);
		flag = 1;
		TCCR2B |= (1<<CS20);
	}
	TCCR1B ^= (1 << ICES1);
	TIFR1 |= (1 << ICF1);
}

int main(void)
{
	UART_init(BAUD_PRESCALER);
	Initialize();
	
	while(1)
	{
		while(!(ADCSRA & (1<<ADIF)));
		
		if(ADC < 102)
		dutyCycle = 5;
		else if(ADC < 205)
		dutyCycle = 10;
		else if(ADC < 307)
		dutyCycle = 15;
		else if(ADC < 410)
		dutyCycle = 20;
		else if(ADC < 512)
		dutyCycle = 25;
		else if(ADC < 614)
		dutyCycle = 30;
		else if(ADC < 717)
		dutyCycle = 35;
		else if(ADC < 819)
		dutyCycle = 40;
		else if(ADC < 922)
		dutyCycle = 45;
		else if(ADC < 1024)
		dutyCycle = 50;
		
		OCR0B = (OCR0A * dutyCycle) / 100.0;
		
		UART_putstring("Printing ADC: ");
		sprintf(str,"%u\n",ADC);
		UART_putstring(str);
		UART_putstring("Printing OCR0A: ");
		sprintf(str,"%u\n",OCR0A);
		UART_putstring(str);
		UART_putstring("Printing OCR0B: ");
		sprintf(str,"%u\n",OCR0B);
		UART_putstring(str);
		
		ADCSRA &= ~(1<<ADIF);
	}
}