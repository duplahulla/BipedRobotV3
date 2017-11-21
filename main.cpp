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

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//-----TWI Communication protocol-----
#define TWI_BUFFER_SIZE 13
#define SLAVE_ADRS	0x40
/********************************************************************************
Function Prototypes
********************************************************************************/
void setMotor(const float &value,const int &motor);
void receiveCommand(int howMany);
void slavesRespond();
	//-------Global variables-----------------

int main(void)
{
	//unsigned char Mcustatus=MCUSR;
	MCUSR=0x00;
	//-------USART Initializing---------------
	usart_init ( MYUBRR ); // fire up the usart
	usart_putchar('D');
	//sprintf(out_str,"MCU status: %x\n", Mcustatus & 0xff);
	//usart_pstr(out_str);
	
	//-------IIC Initializing------------------
	
	twiSlaveInit( SLAVE_ADRS );		// Initialize TWI hardware for Slave operation.	
	sei();							// Enable interrupts.
	twiSlaveEnable();				// Enable the TWI interface to receive data.
	uint8_t *var=twiGetRxBuffer();
    while (1) 
    {
		//setMotor(20,2);
	
		for(int i=0;i<TWI_RX_BUFFER_SIZE;i++){
			var[i]=+1;
		}
		twiSetTxBuffer(var);
		//usart_putchar('K');
    }
}

