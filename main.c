/*
 * main.c
 *
 *  Created on: Nov 4, 2016
 *      Author: mati
 */


#include <avr/io.h>
#include <avr/interrupt.h>

#define FOSC 14745600
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define CLED (1<<PB1)
#define LEFT_SWITCH (1<<PB0)
#define RIGHT_SWITCH (1<<PC5)
#define SPACE_SWITCH (1<<PB2)
#define SOUND (1<<PD2)

typedef struct Switch
{
	uint8_t pin;
	volatile uint8_t* pinState;
	char sendOnRelease, sendOnPress;
	uint8_t lock_keyDownEvent, lock_keyUpEvent;
}
Switch;

void USART_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter + data to read interrupt */
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
		;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void processInput(Switch* sw)
{
	// when key DOWN
	if(!(*sw->pinState & sw->pin))
	{
		if(!sw->lock_keyDownEvent)
		{
			sw->lock_keyDownEvent = 1;
			USART_Transmit(sw->sendOnPress);
		}
		// unlock KEY UP EVENT
		if(sw->lock_keyUpEvent)
			++sw->lock_keyUpEvent;
	}
	// when key UP
	else
	{
		if(!sw->lock_keyUpEvent)
		{
			sw->lock_keyUpEvent = 1;
			USART_Transmit(sw->sendOnRelease);
		}
		// unlock KEY DOWN  EVENT
		if(sw->lock_keyDownEvent)
			++sw->lock_keyDownEvent;
	}
}

void init_8msTimer1()
{
	TCCR1B |= (1<<WGM12) | (1<<CS11);
	OCR1A = 1842;
	TIMSK1 |= (1<<OCIE1A);
}

volatile uint16_t progTConnection, progTSound;

int main()
{
	init_8msTimer1();
	USART_Init(MYUBRR);

	DDRB |= CLED;
	DDRD |= SOUND;

	DDRB &= ~(LEFT_SWITCH|SPACE_SWITCH);
	PORTB |= LEFT_SWITCH|SPACE_SWITCH;
	DDRC &= ~RIGHT_SWITCH;
	PORTC |= RIGHT_SWITCH;

	Switch Aswitch;
	Aswitch.pin = LEFT_SWITCH;
	Aswitch.pinState = &PINB;
	Aswitch.sendOnPress = 'A';
	Aswitch.sendOnRelease = 'a';
	Aswitch.lock_keyDownEvent = 0;
	Aswitch.lock_keyUpEvent = 1;

	Switch Dswitch;
	Dswitch.pin = RIGHT_SWITCH;
	Dswitch.pinState = &PINC;
	Dswitch.sendOnPress = 'D';
	Dswitch.sendOnRelease = 'd';
	Dswitch.lock_keyDownEvent = 0;
	Dswitch.lock_keyUpEvent = 1;

	Switch Sswitch;
	Sswitch.pin = SPACE_SWITCH;
	Sswitch.pinState = &PINB;
	Sswitch.sendOnPress = 'S';
	Sswitch.sendOnRelease = 's';
	Sswitch.lock_keyDownEvent = 0;
	Sswitch.lock_keyUpEvent = 1;

	sei();

	for(;;)
	{
		processInput(&Aswitch);
		processInput(&Dswitch);
		processInput(&Sswitch);

		if(!progTConnection)
		{
			PORTB &= ~CLED;
		}
		if(!progTSound)
		{
			PORTD &= ~SOUND;
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	if(progTConnection)
		--progTConnection;
	if(progTSound)
		--progTSound;
}

ISR(USART_RX_vect)
{
	char dummy = UDR0;
	if(dummy == 'c')
	{
		progTConnection = 2000;
		PORTB |= CLED;
	}
	else if(dummy == 's')
	{
		progTSound = 200;
		PORTD |= SOUND;
	}
}
