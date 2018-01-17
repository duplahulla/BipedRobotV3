#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#include "twiSlave.h"
#include "USART.h"
volatile uint8_t buffer_address;
volatile uint8_t txbuffer[0xFF];
volatile uint8_t rxbuffer[0xFF];



// TWI Slave Receiver status codes
#define	TWI_SRX_ADR_ACK				0x60  // Own SLA+W has been received ACK has been returned
#define	TWI_SRX_ADR_ACK_M_ARB_LOST	0x68  // Own SLA+W has been received; ACK has been returned
#define	TWI_SRX_GEN_ACK				0x70  // General call address has been received; ACK has been returned
#define	TWI_SRX_GEN_ACK_M_ARB_LOST	0x78  // General call address has been received; ACK has been returned
#define	TWI_SRX_ADR_DATA_ACK		0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define	TWI_SRX_ADR_DATA_NACK		0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define	TWI_SRX_GEN_DATA_ACK		0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define	TWI_SRX_GEN_DATA_NACK		0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define	TWI_SRX_STOP_RESTART		0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Slave Transmitter status codes
#define	TWI_STX_ADR_ACK				0xA8  // Own SLA+R has been received; ACK has been returned
#define	TWI_STX_ADR_ACK_M_ARB_LOST	0xB0  // Own SLA+R has been received; ACK has been returned
#define	TWI_STX_DATA_ACK			0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define	TWI_STX_DATA_NACK			0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define	TWI_STX_DATA_ACK_LAST_BYTE	0xC8  // Last byte in TWDR has been transmitted (TWEA = 0); ACK has been received

// TWI Miscellaneous status codes
#define	TWI_NO_STATE				0xF8  // No relevant state information available; TWINT = 0
#define	TWI_BUS_ERROR				0x00  // Bus error due to an illegal START or STOP condition

void I2C_init(uint8_t address){
	// load address into TWI address register
	TWAR = (address << 1);
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
	txbuffer[0]=0;
	txbuffer[1]=1;
	txbuffer[2]=2;
	txbuffer[3]=3;
	txbuffer[4]=4;
	txbuffer[5]=5;
	txbuffer[6]=6;
	txbuffer[7]=7;
	txbuffer[8]=8;
	txbuffer[9]=9;
	txbuffer[10]=10;
}

void I2C_stop(void){
	// clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}
void I2C_writeRegister(uint8_t address,uint8_t dataL, uint8_t dataH){
	//if(address<(0xFF-1)){
	txbuffer[11]=dataL;
	txbuffer[11+1]=dataH;
	//}
}


ISR( TWI_vect )
{
		
		switch(TW_STATUS)
		{
			case TW_SR_SLA_ACK:
			buffer_address = 0;
			TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			break;
			case TW_SR_DATA_ACK:
			// received data from master, call the receive callback
			rxbuffer[buffer_address]=TWDR;
			buffer_address++;
			TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			break;
			case TW_ST_SLA_ACK:
			// master is requesting data, call the request callback
			buffer_address = 0;
			TWDR=txbuffer[buffer_address];
			buffer_address++;
			TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			break;
			case TW_ST_DATA_ACK:
			// master is requesting data, call the request callback
			TWDR=txbuffer[buffer_address];
			buffer_address++;
			TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			break;
			case TW_BUS_ERROR:
			// some sort of erroneous state, prepare TWI to be readdressed
			TWCR = 0;
			TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			break;
			default:
			TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			break;
		}
	//
	//
	//cli();
	////debug
	//uint8_t data;
	//uint8_t state=TWSR;
	//switch( state )
	//{
		//
		//case TWI_SRX_ADR_ACK:				// 0x60 Own SLA+W has been received ACK has been returned. Expect to receive data.
		////		case TWI_SRX_ADR_ACK_M_ARB_LOST:	// 0x68 Own SLA+W has been received; ACK has been returned. RESET interface.
		//buffer_address = 0xFF;
		//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be DATA.
		//break;
//
		//case TWI_SRX_ADR_DATA_ACK:			// 0x80 Previously addressed with own SLA+W; Data received; ACK'd
		//case TWI_SRX_GEN_DATA_ACK:			// 0x90 Previously addressed with general call; Data received; ACK'd
		//// Put data into RX buffer
		//// save the received byte inside data
		//data = TWDR;
		//
		//// check wether an address has already been transmitted or not
		//if(buffer_address == 0xFF){
			//
		//buffer_address = data;
		//}
		//else{ // if a databyte has already been received
		//
		//// store the data at the current address
		//rxbuffer[buffer_address] = data;
		//
		//// increment the buffer address
		//buffer_address++;
		//}
		//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be more DATA.
		//break;
		//
		//case TWI_SRX_GEN_ACK:				// 0x70 General call address has been received; ACK has been returned
		////		case TWI_SRX_GEN_ACK_M_ARB_LOST:	// 0x78 General call address has been received; ACK has been returned
		//// TODO: Set General Address flag
		//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be DATA.
		//break;
//
		//case TWI_STX_ADR_ACK:				// 0xA8 Own SLA+R has been received; ACK has been returned. Load DATA.
		////		case TWI_STX_ADR_ACK_M_ARB_LOST:	// 0xB0 Own SLA+R has been received; ACK has been returned
		//case TWI_STX_DATA_ACK:				// 0xB8 Data byte in TWDR has been transmitted; ACK has been received. Load DATA.
		//// copy data from TWDR to the temporary memory
		//data = TWDR;
		//
		//// if no buffer read address has been sent yet
		//if( buffer_address == 0xFF ){
			//buffer_address = data;
		//}
		//
		//// copy the specified buffer address into the TWDR register for transmission
		//TWDR =buffer_address;
		//// increment buffer read address
		//buffer_address++;
		//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event.
		//break;
//
		//case TWI_STX_DATA_NACK:				// 0xC0 Data byte in TWDR has been transmitted; NOT ACK has been received. End of Sending.
		//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be new message.
		//break;
//
		//case TWI_SRX_STOP_RESTART:			// 0xA0 A STOP condition or repeated START condition has been received while still addressed as Slave
		//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event.
		//break;
//
		//case TWI_SRX_ADR_DATA_NACK:			// 0x88 Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
		//case TWI_SRX_GEN_DATA_NACK:			// 0x98 Previously addressed with general call; data has been received; NOT ACK has been returned
		//case TWI_STX_DATA_ACK_LAST_BYTE:	// 0xC8 Last byte in TWDR has been transmitted (TWEA = 0); ACK has been received
		//case TWI_NO_STATE:					// 0xF8 No relevant state information available; TWINT = 0
		//case TWI_BUS_ERROR:					// 0x00 Bus error due to an illegal START or STOP condition
		//TWCR =   (1<<TWSTO)|(1<<TWINT);   // Recover from TWI_BUS_ERROR
		//// TODO: Set an ERROR flag to tell main to restart interface.
		//break;
//
		//default:							// OOPS
		//TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be more DATA.
		//break;
	//}
	//sei();
	////usart_putchar(state);
}