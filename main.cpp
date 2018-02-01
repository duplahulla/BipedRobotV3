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
#include <avr/eeprom.h>
/********************************************************************************
Macros and Defines
********************************************************************************/

#define BAUD 115200
#define MYUBRR F_CPU/16/BAUD-1
//A and B channel current sense ADC channel switched up because of design error
#define ADC_A_Isense 0x01
#define ADC_A_Psense 0x02
#define ADC_B_Isense 0x00
#define ADC_B_Psense 0x03

#define I2C_Tx_A_Psense_address 0x01
#define I2C_Tx_B_Psense_address 0x03
#define I2C_Tx_A_Isense_address 0x05
#define I2C_Tx_B_Isense_address 0x07

//-----TWI Communication protocol-----
#define SLAVE_ADRS	0x40
/********************************************************************************
Function Prototypes
********************************************************************************/
void StopMotorA();
void StopMotorB();

typedef union
{
	short i16;
	unsigned char u8[2];
} I16_U8;

//-------Global variables-----------------
volatile bool ADCFlag= false;			//true if ADC data needs to be processed		
volatile uint8_t ADCDataH=0;
volatile uint8_t ADCDataL=0;
volatile uint8_t ADMUXState=0;
const uint8_t* SlaveEEAddressPointer=(uint8_t *)(0x00);

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
	OCR1A = 0xFF;
	OCR1B = 0xFF-50;
	TCNT1=0xFF;													// set phase shift relative to Timer 0
	TCCR1A |= (1 << COM1A1);									// Non inverting mode
	TCCR1A |= (1 << COM1B1);
	TCCR1A |= (1 << WGM10);										// 8 bit phase corriged PWM mode
    TIMSK1 |= (1 << TOIE1);										// Overflow interrupt enable
	TCCR0B |= (1 << CS11);//|(1 << CS10);						// Set prescaler to 8 and start PWM on timer0
	TCCR1B |= (1 << CS11);//|(1 << CS10);						// Set prescaler to 8 and start PWM on timer1

	//-------IIC Initializing------------------
	
	uint8_t SlaveAddress=0x08;
	eeprom_busy_wait();
	SlaveAddress=eeprom_read_byte(SlaveEEAddressPointer);
	sei(); 				// Enable interrupts.
	I2C_init( SlaveAddress );		// Initialize TWI hardware for Slave operation.
	
	//-------ADC setup-----------------
	ADMUX=0x00;
	ADCSRA |= (1<<ADATE);					//Enable auto triggering
	ADCSRB |= (1<<ADTS2);					//Timer/Counter0 Overflow trigger
	ADCSRA |= (1<<ADEN);					// ADEN: Set to turn on ADC , by default it is turned off
	ADCSRA |= (1<<ADPS2)| (1<<ADPS1);//| (1<<ADPS0);		//set to make division factor 128
	ADCSRA |= (1<<ADIE);					//ADIE: Interrupt enable
	//sei();
unsigned char Mcustatus=MCUSR;
MCUSR=0x00;
	//-------USART Initializing---------------
	//usart_init ( MYUBRR ); // fire up the usart
	//usart_putchar('D');
	//sprintf(out_str,"MCU status: %x\n", Mcustatus & 0xff);
	//usart_pstr(out_str);

	DDRD |= (1 << DDD1);
	//StopMotorA();
	StopMotorB();
    while (1) 
    {
		if(ADCFlag){
			ADCFlag=false;
switch(ADMUXState){
	case ADC_A_Isense :
	I2C_writeTxBuffer16Bit(I2C_Tx_A_Isense_address,ADCH, ADCL);
	ADCSRA &= ~(1<<ADATE);					//Disable auto trigger
	ADMUX=ADC_A_Psense;
	ADCSRA |= (1<<ADSC);					//Start ADC right now
	break;
	case ADC_A_Psense :
	I2C_writeTxBuffer16Bit(I2C_Tx_A_Psense_address,ADCH, ADCL);
	ADMUX=ADC_B_Isense;
	ADCSRA |= (1<<ADATE);					//Enable auto trigger
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB |= (1<<ADTS2);					//Timer/Counter0 Overflow trigger
	break;
	case ADC_B_Isense :
	I2C_writeTxBuffer16Bit(I2C_Tx_B_Isense_address,ADCH, ADCL);
	ADMUX=ADC_B_Psense;
	ADCSRA &= ~(1<<ADATE);					//Disable auto trigger
	ADCSRA |= (1<<ADSC);					//Start ADC right now
	break;
	case ADC_B_Psense :
	I2C_writeTxBuffer16Bit(I2C_Tx_B_Psense_address,ADCH, ADCL);
	ADMUX=ADC_A_Isense;
	ADCSRA |= (1<<ADATE);					//Enable auto trigger
	ADCSRB |= (1<<ADTS1);
	ADCSRB |= (1<<ADTS2);					//Timer/Counter1 Overflow trigger
	break;
}
			//switch(ADMUXState){
				//case ADC_A_Isense :
				//I2C_writeTxBuffer16Bit(I2C_Tx_A_Isense_address,1, ADCDataL);
				//break;
				//case ADC_A_Psense :
				//I2C_writeTxBuffer16Bit(I2C_Tx_A_Psense_address,2, ADCDataL);
				//break;
				//case ADC_B_Isense :
				//I2C_writeTxBuffer16Bit(I2C_Tx_B_Isense_address,3, ADCDataL);
				//break;
				//case ADC_B_Psense :
				//I2C_writeTxBuffer16Bit(I2C_Tx_B_Psense_address,4, ADCDataL);
				//break;
			//}
			//Handle the ADC state machine
			
			//handling internal process
			
			
		}
		if(I2C_gotMessage()){
			int16_t value=I2C_readRxBuffer16Bit(0x01);
			if(value>0 && value<255){
				OCR1B = 0xFF-value;
			}
			I16_U8 converter;
			converter.i16=value;
			//I2C_writeTxBuffer16Bit(I2C_Tx_A_Psense_address,converter.u8[1], converter.u8[0]);
		}
_delay_us(1);
PORTD &=~(1<<PIND1);
		
		_delay_us(1);
		PORTD |=(1<<PIND1);
		//usart_putchar('K');
    }
}

ISR(ADC_vect){
	sei();
	ADCFlag=true;
	ADCDataH=ADCH;
	ADCDataL=ADCL;
	ADMUXState=ADMUX;
}
ISR(TIMER0_OVF_vect)
{
}
ISR(TIMER1_OVF_vect)
{
}
void StopMotorA(){
		OCR1A = 0x00;
		OCR1B = 0x00;
}
void StopMotorB(){
		OCR0A = 0x00;
		OCR0B = 0x00;
}