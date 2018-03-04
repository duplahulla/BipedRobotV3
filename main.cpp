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
#include "PID.h"
/********************************************************************************
Macros and Defines
********************************************************************************/

#define BAUD 115200
#define MYUBRR F_CPU/16/BAUD-1
//A and B channel current sense ADC channel switched up because of design error
#define ADC_A_Isense 0x00
#define ADC_A_Psense 0x02
#define ADC_B_Isense 0x01
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
	uint16_t u16;
	unsigned char u8[2];
} U16_U8;

//-------Global variables-----------------
volatile bool ADCFlag= false;			//true if ADC data needs to be processed		
const uint8_t* SlaveEEAddressPointer=(uint8_t *)(0x00);
PID MotorACurrent;
PID MotorBCurrent;
bool MotorAForward=true;
bool MotorBForward=true;
int16_t MotorACurrentADC=0;
int16_t value=0;
volatile uint8_t ADCMuxContainer;
volatile U16_U8 ADCContainer;

int main(void){
	//-------PID setup-----------------
	uint16_t samples[3]={0,0,0};
	ADCContainer.u16=0;
	MotorACurrent.SetTunings(6,0.02,0.01);
	MotorACurrent.SetOutputLimits(0,255);
	MotorACurrent.Initialize();
	MotorACurrent.Setpoint=0;
	MotorBCurrent.SetTunings(0.16,0.2,0);
	MotorBCurrent.SetOutputLimits(0,255);
	MotorBCurrent.Initialize();
	MotorBCurrent.Setpoint=0;
	//-------PWM setup-----------------
	//Timer0
	PORTD &=~(1<<PIND6);
	PORTD &=~(1<<PIND5);
	DDRD |= (1 << DDD5);										// PD5-6 as output
	DDRD |= (1 << DDD6);
	OCR0A = 0x00;
	OCR0B = 0x00;
	TCCR0A |= (1<<COM0A1);										// Set inverting mode
	TCCR0A |= (1<<COM0B1);
	TCCR0A |= (1<<COM0A0);										
	TCCR0A |= (1<<COM0B0);
	TCCR0A |= (1 << WGM02) | (1 << WGM00);						// 10 bit phase corriged PWM mode
	
	TIMSK0 |= ( 1 << TOIE0);					//Overflow interrupt enable
	//Timer1
	PORTB &=~(1<<PINB2);
	PORTB &=~(1<<PINB1);
	DDRB |= (1 << DDB1);										// PB1-2 as output
	DDRB |= (1 << DDB2);
	OCR1A = 0x00;
	OCR1B = 0x00;
	TCNT1=0xFF;													// set phase shift relative to Timer 0
	TCCR1A |= (1 << COM1A1);									//  inverting mode
	//TCCR1A |= (1 << COM1A0);
	TCCR1A |= (1 << COM1B1);
	//TCCR1A |= (1 << COM1B0);									//inverting mode
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
	StopMotorA();
	StopMotorB();
    while (1) 
    {
		if(ADCFlag){
			ADCFlag=false;
			switch(ADMUX){
				case ADC_A_Isense :
				
				I2C_writeTxBuffer16Bit(I2C_Tx_A_Isense_address,ADCContainer.u8[1], ADCContainer.u8[0]);
				ADCSRA &= ~(1<<ADATE);					//Disable auto trigger
				ADMUX=ADC_A_Psense;
				ADCSRA |= (1<<ADSC);					//Start ADC right now

				break;
				case ADC_A_Psense :
				MotorACurrent.Input=ADCContainer.u16;
				I2C_writeTxBuffer16Bit(I2C_Tx_A_Psense_address,ADCContainer.u8[1], ADCContainer.u8[0]);
				ADMUX=ADC_B_Isense;
				ADCSRA |= (1<<ADATE);					//Enable auto trigger
				ADCSRB &= ~(1<<ADTS1);
				ADCSRB |= (1<<ADTS2);					//Timer/Counter0 Overflow trigger
								MotorACurrent.Compute();
								if(MotorACurrent.Output>=0){
									//OCR1A = 0x00;
									//OCR1B = abs(MotorACurrent.Output);
									}else{
									//OCR1A = abs(MotorACurrent.Output);
									//OCR1B = 0x00;
								}
				break;
				case ADC_B_Isense :
				//samples[0]=samples[1];
				//samples[1]=samples[2];
				//samples[2]=ADCContainer.u16;
				//ADCContainer.u16=(85 * samples[2] + 256 * samples[1] + 128 * samples[0]) >> 8;
				MotorBCurrent.Input=ADCContainer.u16;
				I2C_writeTxBuffer16Bit(I2C_Tx_B_Isense_address,ADCContainer.u8[1], ADCContainer.u8[0]);
				ADMUX=ADC_B_Psense;
				ADCSRA &= ~(1<<ADATE);					//Disable auto trigger
				ADCSRA |= (1<<ADSC);					//Start ADC right now
				MotorBCurrent.Compute();
				if(MotorBForward==true){  //TODO in current mode needs to be flipped!!!
					OCR0A = 0x00;
					OCR0B = MotorBCurrent.Output;
					
					}else{
					OCR0A = MotorBCurrent.Output;
					OCR0B = 0x00;
				}
				break;
				case ADC_B_Psense :
				I2C_writeTxBuffer16Bit(I2C_Tx_B_Psense_address,ADCContainer.u8[1], ADCContainer.u8[0]);
				ADMUX=ADC_A_Isense;
				ADCSRA |= (1<<ADATE);					//Enable auto trigger
				ADCSRB |= (1<<ADTS1);
				ADCSRB |= (1<<ADTS2);					//Timer/Counter1 Overflow trigger
				break;
			}
			
		}
		
		if(I2C_gotMessage()){
			value=I2C_readRxBuffer16Bit(0x01);
			MotorACurrent.Setpoint=abs(value);
			if (value==0){
				StopMotorA();
			}else if(value>0){
				MotorAForward=true;
				OCR1A = abs(value);
				OCR1B = 0x00;
				
				
			}else if(value<0){
				MotorAForward=false;
				OCR1A = 0x00;
				OCR1B = abs(value);
			}
			 value=I2C_readRxBuffer16Bit(0x03);
			MotorBCurrent.Setpoint=abs(value);
			if (value==0){
				StopMotorB();
				}else if(value>0){
				MotorBForward=true;
				//OCR0A = abs(value);
				//OCR0B = 0x00;
				}else if(value<0){
					//OCR0A = 0x00;
					//OCR0B = abs(value);
				
				MotorBForward=false;
			}
			U16_U8 converter;
			converter.u16=value;
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
	ADCContainer.u16=ADC;
	ADCFlag=true;
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