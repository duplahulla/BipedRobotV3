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

typedef union
{
	unsigned short u16;
	unsigned char u8[2];
} U16_U8;

void I2C_init(uint8_t address){
	txbuffer[1]=12;
	txbuffer[2]=12;
	txbuffer[3]=12;
	txbuffer[4]=12;
	txbuffer[5]=12;
	txbuffer[6]=12;
	txbuffer[7]=12;
	txbuffer[0]=12;
	// load address into TWI address register
	TWAR = (address << 1);
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
	
}

void I2C_stop(void){
	// clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}
void I2C_writeTxBuffer16Bit(uint8_t address, uint8_t dataH, uint8_t dataL){
	txbuffer[address]=dataH;
	txbuffer[address+1]=dataL;
}
uint16_t I2C_readRxBuffer16Bit(uint8_t address){
	U16_U8 buf;
	buf.u8[0]=rxbuffer[address];
	buf.u8[1]=rxbuffer[address+1];
	return buf.u16;
}
uint8_t I2C_readRxBuffer8Bit(uint8_t address){
	return rxbuffer[address];
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

}