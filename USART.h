/*
 * USART.h
 *
 * Created: 11/23/2016 8:12:40 PM
 *  Author: Tamas
 */ 


#ifndef USART_H_
#define USART_H_
void usart_init(uint16_t ubrr);
char usart_getchar( void );
void usart_putchar( char data );
void usart_pstr(char *s);
unsigned char usart_kbhit(void);




#endif /* USART_H_ */