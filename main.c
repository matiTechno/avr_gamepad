#include <avr/io.h>
#include <avr/interrupt.h>

#define FOSC 14745600
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define CONNECTION_LED (1<<PB1)
#define EFFECT (1<<PD2)
// A - left switch
// D - right switch
// S - middle switch
#define A_SWITCH (1<<PB0)
#define D_SWITCH (1<<PC5)
#define S_SWITCH (1<<PB2)

typedef struct
{
	uint8_t pin;
	volatile uint8_t* pinState;
	char sendOnRelease, sendOnPress;
	uint8_t lock_keyDownEvent, lock_keyUpEvent;
}
Switch;

void usartInit(unsigned int ubrr)
{
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)(ubrr);
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00); // 8 data + 1 stop
}

void usartTransmit(uint8_t data)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void processInput(Switch* sw)
{
	// key DOWN
	if(!(*sw->pinState & sw->pin))
	{
		if(!sw->lock_keyDownEvent)
		{
			sw->lock_keyDownEvent = 1;
			usartTransmit(sw->sendOnPress);
		}
		// unlock key UP EVENT
		if(sw->lock_keyUpEvent)
			++sw->lock_keyUpEvent;
	}
	// key UP
	else
	{
		if(!sw->lock_keyUpEvent)
		{
			sw->lock_keyUpEvent = 1;
			usartTransmit(sw->sendOnRelease);
		}
		// unlock key DOWN EVENT
		if(sw->lock_keyDownEvent)
			++sw->lock_keyDownEvent;
	}
}

void init_timer1_10ms_tick()
{
	TCCR1B |= (1<<WGM12) | (1<<CS11);
	OCR1A = 18431;
	TIMSK1 |= (1<<OCIE1A);
}

volatile uint8_t connectionT, effectT;

int main()
{
	init_timer1_10ms_tick();
	usartInit(MYUBRR);
	sei();

	DDRB |= CONNECTION_LED;
	DDRD |= EFFECT;
	PORTB |= A_SWITCH | S_SWITCH;
	PORTC |= D_SWITCH;

	Switch Aswitch;
	Aswitch.pin = A_SWITCH;
	Aswitch.pinState = &PINB;
	Aswitch.sendOnPress = 'A';
	Aswitch.sendOnRelease = 'a';

	Switch Dswitch;
	Dswitch.pin = D_SWITCH;
	Dswitch.pinState = &PINC;
	Dswitch.sendOnPress = 'D';
	Dswitch.sendOnRelease = 'd';

	Switch Sswitch;
	Sswitch.pin = S_SWITCH;
	Sswitch.pinState = &PINB;
	Sswitch.sendOnPress = 'S';
	Sswitch.sendOnRelease = 's';

	for(;;)
	{
		processInput(&Aswitch);
		processInput(&Dswitch);
		processInput(&Sswitch);
	}
}

ISR(TIMER1_COMPA_vect, ISR_NOBLOCK)
{
	if(connectionT)
		if(!--connectionT)
			PORTB &= ~CONNECTION_LED;

	if(effectT)
		if(!--effectT)
			PORTD &= ~EFFECT;
}

ISR(USART_RX_vect)
{
	char dummy = UDR0;
	if(dummy == 'C')
	{
		PORTB |= CONNECTION_LED;
		connectionT = 10;
	}
	else if(dummy == 'S')
	{

		PORTD |= EFFECT;
		effectT = 10;
	}
}
