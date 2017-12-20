/*
 * BipedRobotMotorControlSoftware.cpp
 *
 * Created: 11/16/2016 8:52:36 PM
 * Author : Tamas
 */ 
#define F_CPU 16000000
#include <avr/io.h>
#include <stdlib.h>
//#include <avr/interrupt.h>
#include <stdio.h>
//#include "PID.h"
//#include "math.h"
//#include "wire.h"
#include <util/delay.h>
#include "USART.h"
#include <avr/interrupt.h>
/********************************************************************************
Macros and Defines
********************************************************************************/

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//-----TWI Communication protocol-----
#define TWI_BUFFER_SIZE 13
/********************************************************************************
Function Prototypes
********************************************************************************/
void setMotor(const float &value,const int &motor);
void receiveCommand(int howMany);
void slavesRespond();
//-------Global variables-----------------

int main(void)
{

	//-------PWM setup-----------------
	//Timer0
	PORTD &=~(1<<PIND6);
	PORTD &=~(1<<PIND5);
	DDRD |= (1 << DDD5);										// PB1 kimenetként való konfigurálása
	DDRD |= (1 << DDD6);
	OCR0A = 0xA0;
	OCR0B = 5;
	TCCR0A |= (1<<COM0A1);										// Nem invertáló mód beállítása
	TCCR0A |= (1<<COM0B1);
	TCCR0A |= (1 << WGM02) | (1 << WGM00);						// 10 bites fázis korrigált PWM beállítása
	
	TIMSK0 |= ( 1 << TOIE0);					//Overflow interrupt enable
	//Timer1
	PORTB &=~(1<<PINB2);
	PORTB &=~(1<<PINB1);
	DDRB |= (1 << DDB1);										// PB1 kimenetként való konfigurálása
	DDRB |= (1 << DDB2);
	OCR1A = 2;
	OCR1B = 5;
	TCNT1=0xFF;
	TCCR1A |= (1 << COM1A1);									// Nem invertáló mód beállítása
	TCCR1A |= (1 << COM1B1);
	TCCR1A |= (1 << WGM10);										// 8 bites gyors PWM beállítása
	//TCCR1B |= (1 << WGM13);
	TCCR0B |= (1 << CS11) | (1 << CS10);						// elõosztó beállítása 8 ra, és pwm indítása
	TCCR1B |= (1 << CS11) | (1 << CS10);										// elõosztó beállítása 8 ra, és pwm indítása
	//TIMSK1 |= (1 << TOIE1);										// Overflow interrupt enable
	
	
	//-------ADC setup-----------------
	DDRD |= (1 << DDD2);
	PORTD &=~(1<<PIND2);

	ADCSRA |= (1<<ADATE);
	ADCSRB |= (1<<ADTS2);					//Timer/Counter0 Overflow trigger
	ADCSRA |= (1<<ADEN);					// ADEN: Set to turn on ADC , by default it is turned off
	ADCSRA |= (1<<ADPS2)| (1<<ADPS0);			//set to make division factor 128
	ADCSRA |= (1<<ADIE);					//ADIE: Interrupt enable
	sei();
//unsigned char Mcustatus=MCUSR;
MCUSR=0x00;

//-------USART Initializing---------------
//usart_init ( MYUBRR ); // fire up the usart
//usart_putchar('D');
//sprintf(out_str,"MCU status: %x\n", Mcustatus & 0xff);
//usart_pstr(out_str);
    while (1) 
    {
		//setMotor(20,2);
		_delay_ms(10);
		//usart_putchar('K');
    }
}

ISR(ADC_vect){
	PORTD  |=(1<<PIND2);
	_delay_ms(1) ;
	PORTD &=~(1<<PIND2);
	//usart_pstr("ADC /n");
}