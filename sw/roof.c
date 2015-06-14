#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint16_t period = 100;

uint8_t get_adc()
{
	uint8_t i, s;
	uint8_t val = 0;

	DDRB |= _BV(PB2);
	for (i = 0; i < 255; i++) {
		s = (ACSR >> ACO) & 1;
		val += s;
		PORTB = (PORTB & ~_BV(PB2)) | (s << PB2);
	}

	PORTB &= ~_BV(PB2);
	DDRB &= ~_BV(PB2);

	return val;
}

#define MIN_OFFSET 150

/* every zero-crossing */
ISR (INT0_vect)
{
	if (period < 64) {
		PORTD |= _BV(PD3);
		TCNT1 = 0;
		OCR1A = 0xFFFF;
	} else {
		PORTD &= ~_BV(PD3);
		TCNT1 = 0;
		OCR1A = period + MIN_OFFSET;

	}
}

/* output compare match */
ISR (TIMER1_COMPA_vect)
{
	PORTD |= _BV(PD3);
	_delay_us(5);
	PORTD &= ~_BV(PD3);

	OCR1A = 1250 + period + MIN_OFFSET;
}

int main()
{
	uint8_t adc;

	/* comparator ADC */
	ACSR = 0;
	DIDR = _BV(PB0) | _BV(PB1);

	/* PD3 is output, PD5 is LED */
	DDRD = _BV(PD3) | _BV(PD5);
	PORTD = 0;

	/* zero-crossing interrupt */
	MCUCR = _BV(ISC00) | ~_BV(ISC01);     /* falling edge */
	GIMSK = _BV(INT0);		     /* INT0 */

	/* Timer1 - overflow every 50Hz half-period,
	   output compare triggers optotriac */

	/* clk/64 = 125kHz => count to 1250 for 50Hz half-period */

	/* mode 0 WGM = 0000

	   OCR1 = 100 < power_level < 1000

	 */

	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	OCR1A = period;
	TCNT1 = 0;

	TIMSK |= _BV(OCIE1A);

	sei();

	for (;;) {
		adc = get_adc ();
		_delay_ms (50.0);

		period  = adc;
		period *= 4;

	}

	return 0;
}
