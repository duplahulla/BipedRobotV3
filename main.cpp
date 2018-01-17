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
#include "twiSlave.h"
#include <avr/interrupt.h>
/********************************************************************************
Macros and Defines
********************************************************************************/

#define BAUD 115200
#define MYUBRR F_CPU/16/BAUD-1
#define ADC_A_Isense 0x00
#define ADC_A_Psense 0x02
#define ADC_B_Isense 0x01
#define ADC_B_Psense 0x03

#define I2C_A_Psense_address 0x00
#define I2C_B_Psense_address 0x02
#define I2C_A_Isense_address 0x04
#define I2C_B_Isense_address 0x06

//-----TWI Communication protocol-----
#define SLAVE_ADRS	0x40
/********************************************************************************
Function Prototypes
********************************************************************************/
void setMotor(const float &value,const int &motor);
void receiveCommand(int howMany);
void slavesRespond();
//-------Global variables-----------------
uint8_t measurementBuffer[8];
int main(void){

	//-------PWM setup-----------------
	//Timer0
	PORTD &=~(1<<PIND6);
	PORTD &=~(1<<PIND5);
	DDRD |= (1 << DDD5);										// PD5-6 as output
	DDRD |= (1 << DDD6);
	OCR0A = 0xFF;
	OCR0B = 0xFF-50;
	TCCR0A |= (1<<COM0A1);										// Set non inverting mode
	TCCR0A |= (1<<COM0B1);
	TCCR0A |= (1 << WGM02) | (1 << WGM00);						// 10 bit phase corriged PWM mode
	
	TIMSK0 |= ( 1 << TOIE0);					//Overflow interrupt enable
	//Timer1
	PORTB &=~(1<<PINB2);
	PORTB &=~(1<<PINB1);
	DDRB |= (1 << DDB1);										// PB1-2 as output
	DDRB |= (1 << DDB2);
	OCR1A = 0;													// set Duty cycle to 0
	OCR1B = 0;
	TCNT1=0xFF;													// set phase shift relative to Timer 0
	TCCR1A |= (1 << COM1A1);									// Non inverting mode
	TCCR1A |= (1 << COM1B1);
	TCCR1A |= (1 << WGM10);										// 8 bit phase corriged PWM mode
    TIMSK1 |= (1 << TOIE1);										// Overflow interrupt enable
	TCCR0B |= (1 << CS11); //| (1 << CS10);						// Set prescaler to 8 and start PWM on timer0
	TCCR1B |= (1 << CS11); //| (1 << CS10);						// Set prescaler to 8 and start PWM on timer1

	//-------IIC Initializing------------------
	sei(); 				// Enable interrupts.
	I2C_init( SLAVE_ADRS );		// Initialize TWI hardware for Slave operation.
	
	//-------ADC setup-----------------
	ADMUX=0x00;
	ADCSRA |= (1<<ADATE);					//Enable auto triggering
	ADCSRB |= (1<<ADTS2);					//Timer/Counter0 Overflow trigger
	ADCSRA |= (1<<ADEN);					// ADEN: Set to turn on ADC , by default it is turned off
	ADCSRA |= (1<<ADPS2)| (1<<ADPS1);			//set to make division factor 128
	ADCSRA |= (1<<ADIE);					//ADIE: Interrupt enable
	//sei();
unsigned char Mcustatus=MCUSR;
MCUSR=0x00;
	//-------USART Initializing---------------
	//usart_init ( MYUBRR ); // fire up the usart
	//usart_putchar('D');
	//sprintf(out_str,"MCU status: %x\n", Mcustatus & 0xff);
	//usart_pstr(out_str);
	DDRC &=~(1<<PINC4);
	DDRC &=~(1<<PINC5);
	PORTC &=~(1<<PINC4);
	PORTC &=~(1<<PINC4);
	DDRD |= (1 << DDD1);
    while (1) 
    {
		_delay_us(1);
		PORTD &=~(1<<PIND1);
		_delay_us(1);
		PORTD |=(1<<PIND1);
		//usart_putchar('K');
    }
}

ISR(ADC_vect){
	switch(ADMUX){
	
		 case ADC_A_Isense :
			 I2C_writeRegister(I2C_A_Isense_address,ADCL,ADCH);
			 ADMUX=ADC_A_Psense;
			 ADCSRA &= ~(1<<ADATE);					//Disable auto trigger
			 ADCSRA |= (1<<ADSC);
		 break;
		 case ADC_A_Psense :
			 I2C_writeRegister(I2C_A_Psense_address,ADCL,ADCH);
			 ADMUX=ADC_B_Isense;
			 ADCSRA |= (1<<ADATE);					//Enable auto trigger
			 ADCSRB &= ~(1<<ADTS1);
			 ADCSRB |= (1<<ADTS2);					//Timer/Counter0 Overflow trigger
		 break;
		 case ADC_B_Isense :
			 I2C_writeRegister(I2C_B_Isense_address,ADCL,ADCH);
			 ADMUX=ADC_B_Psense;
			 ADCSRA &= ~(1<<ADATE);					//Disable auto trigger
			 ADCSRA |= (1<<ADSC);
		 break;
		 case ADC_B_Psense :
			 I2C_writeRegister(I2C_B_Psense_address,ADCL,ADCH);
			 ADMUX=ADC_A_Isense;
			 ADCSRA |= (1<<ADATE);					//Enable auto trigger
			 ADCSRB |= (1<<ADTS1);
			 ADCSRB |= (1<<ADTS2);					//Timer/Counter1 Overflow trigger
		 break;
	}
}
ISR(TIMER0_OVF_vect)
{
	PORTD  |=(1<<PIND2);
}
ISR(TIMER1_OVF_vect)
{
	PORTD  |=(1<<PIND2);
}