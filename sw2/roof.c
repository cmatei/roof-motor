#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>

/* hardware pins

   - direction relay: PD3
   - optotriac: PD4

   - zero crossing: PD2/INT0

   - limit1, limit2: PB4, PB5
   - manual dir: PB0 (orange)
   - auto cmd: PB1 (brown)
   - manual move: PB2 (blue)
   - manual/auto: PB3 (green)

*/

#define PB_INPUT_MASK (_BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3) | _BV(PB4) | _BV(PB5))

#define PB_MODE_AUTO  _BV(PB3)
#define PB_MANUAL_RUN _BV(PB2)
#define PB_MANUAL_DIR _BV(PB0)
#define PB_AUTO_CMD   _BV(PB1)

#define PB_LIMIT1     _BV(PB4)
#define PB_LIMIT2     _BV(PB5)

#define PB_NUM_INPUTS 6

#define MODE_AUTO()  (!!(debounced & PB_MODE_AUTO))
#define AUTO_CMD()   (!!(debounced & PB_AUTO_CMD))
#define MANUAL_RUN() (!!(debounced & PB_MANUAL_RUN))
#define MANUAL_DIR() (!!(debounced & PB_MANUAL_DIR))

#define LIMIT1()     (!!(debounced & PB_LIMIT1))
#define LIMIT2()     (!!(debounced & PB_LIMIT2))


#define PD_RELAY _BV(PD3)
#define PD_OPTO  _BV(PD4)


#define DEBOUNCE_CHECK_MSEC 10
#define DEBOUNCE_PRESS_MSEC 20
#define DEBOUNCE_RELEASE_MSEC 100

#define COUNT_RELEASE (DEBOUNCE_RELEASE_MSEC / DEBOUNCE_CHECK_MSEC)
#define COUNT_PRESS   (DEBOUNCE_PRESS_MSEC / DEBOUNCE_CHECK_MSEC)

static volatile uint8_t debounce_counts[PB_NUM_INPUTS] = {
	COUNT_RELEASE, COUNT_RELEASE, COUNT_RELEASE,
	COUNT_RELEASE, COUNT_RELEASE, COUNT_RELEASE,
};

static volatile uint8_t debounced;	     /* debounced inputs state */



/* eeprom addresses */
uint8_t EEMEM nv_invert_direction; 	     /* default direction in auto mode */
static uint8_t invert_direction;

/* prevent the wrapping of the idle state counter */
#define MAX_IDLE_COUNT 40000
static volatile uint16_t idle_count;



#define BASE_DELAY               10 /* 1 tick is 10ms */
#define MODE_CHANGE_DELAY       100 /* 1 sec mandatory idle when switching auto/manual */
#define DIRECTION_CHANGE_DELAY   50 /* 500 ms for relay commutation */
#define RELAY_RELEASE_DELAY    1000 /* release relay after 10 sec idle */

#define MINPAUSE 4
#define MAXPAUSE 250
#define T1_PULSE_WIDTH 15


volatile int pause = MAXPAUSE;

volatile uint8_t running = 0;

static uint8_t mode_was_changed;
static uint8_t direction_was_changed;
static uint8_t relay_is_idle;

static uint8_t relay_on;

static uint8_t mode_auto;
static uint8_t direction;

static inline uint16_t get_idle_count()
{
	uint16_t ret;

	cli();
	ret = idle_count;
	sei();

	return ret;
}

static inline void reset_idle_count()
{
	cli();
	idle_count = 0;
	sei();
}

static void debounce_inputs()
{
	uint8_t raw = ~(PINB & PB_INPUT_MASK); // 1 is pressed, i.e. pin grounded
	uint8_t bit, i;

	/* PB 0..5 */
	for (bit = 1, i = 0; i < PB_NUM_INPUTS; i++, bit <<= 1) {

		if ((raw & bit) == (debounced & bit)) {
			if (debounced & bit)
				debounce_counts[i] = COUNT_RELEASE;
			else
				debounce_counts[i] = COUNT_PRESS;
		} else {
			if (--debounce_counts[i] == 0) {
				debounced &= ~bit;
				debounced |= (raw & bit);

				if (debounced & bit)
					debounce_counts[i] = COUNT_RELEASE;
				else
					debounce_counts[i] = COUNT_PRESS;
			}
		}
	}

#if 0
	/* algo is like this ... */
	if (raw == debounced) {
		if (debounced)
			count = COUNT_RELEASE;
		else
			count = COUNT_PRESS;
	} else {
		if (--count == 0) {
			debounced = raw;

			if (debounced)
				count = COUNT_RELEASE;
			else
				count = COUNT_PRESS
		}
	}
#endif
}

/* every zero-crossing */
ISR (INT0_vect)
{
	debounce_inputs();

	/* count the time spent being idle, up to max idle count */
	if (!running) {
		if (idle_count < MAX_IDLE_COUNT)
			idle_count++;
	} else {
		idle_count = 0;
	}


#if 1
	if (pause > MINPAUSE) {
		pause--;
		OCR1A = pause;
	}

#endif

	TCCR1B = 0x04;
	TCNT1 = 0;
}

ISR (TIMER1_COMPA_vect)
{
	if (running)
		PORTD |= PD_OPTO;

	TCNT1 = 65536 - T1_PULSE_WIDTH;
}

ISR (TIMER1_OVF_vect)
{
	PORTD &= ~PD_OPTO;
	TCCR1B = 0x00;
}

static void set_relay(uint8_t on)
{
	if (on) {
		PORTD |= PD_RELAY;
		relay_on = 1;
	} else {
		PORTD &= ~PD_RELAY;
		relay_on = 0;
	}
}

static void pulse_relay()
{
	set_relay(1);
	_delay_ms(500);

	set_relay(0);
	_delay_ms(500);
}

/* if the manual move button is pressed at boot time,
   invert polarity of auto input and save the option in config */
static void do_startup_config()
{
	uint8_t i;

	/* make sure it's boolean even if eprom not init (0xff) */
	invert_direction = !!eeprom_read_byte(&nv_invert_direction);

	for (i = 0; i < 20; i++) {
		if (MANUAL_RUN()) {
			invert_direction = !invert_direction;

			eeprom_write_byte(&nv_invert_direction, invert_direction);

			pulse_relay();
			pulse_relay();

			for (;;);
		}

		_delay_ms(100);
	}
}

static uint8_t observe_mode_change_delay(uint16_t idle)
{
	if (mode_was_changed) {
		if (idle < MODE_CHANGE_DELAY)
			return 1;

		mode_was_changed = 0;
	}

	return 0;
}

static uint8_t observe_direction_change_delay(uint16_t idle)
{
	if (direction_was_changed) {
		if (idle < DIRECTION_CHANGE_DELAY)
			return 1;

		direction_was_changed = 0;
	}

	return 0;
}

static uint8_t get_commanded_direction()
{
	uint8_t cmd;

	cmd = mode_auto ? AUTO_CMD() : MANUAL_DIR();

	if (invert_direction)
		return !cmd;

	return cmd;
}

static void hcf()
{
	pulse_relay();
	pulse_relay();

	cli();
	for(;;);
}

int main()
{
	uint16_t idle;
	uint8_t cmd;

	DDRD = PD_RELAY | PD_OPTO;
	PORTD = 0;

	DDRB = 0;

        /* zero-crossing interrupt */
        //MCUCR = _BV(ISC00) | _BV(ISC01);     /* rising edge */
        MCUCR = _BV(ISC01);		     /* falling edge */
        GIMSK = _BV(INT0);                   /* INT0 */

	TCCR1A = 0x00;
	TCCR1B = 0x00;
	TIMSK = _BV(TOIE1) | _BV(OCIE1A);

	OCR1 = MAXPAUSE;

	sei();

	/* option to revert auto mode polarity */
	do_startup_config();

	reset_idle_count();

	mode_auto = MODE_AUTO();
	direction = get_commanded_direction();

	for (;;) {

		idle = get_idle_count();

		/* check that we're not in mandatory idle state */
		if (observe_mode_change_delay(idle) || observe_direction_change_delay(idle)) {
			_delay_ms(BASE_DELAY);
			continue;
		}

		/* power off relay after some idle time */
		if (idle >= RELAY_RELEASE_DELAY) {
			if (!relay_is_idle) {

				reset_idle_count();

				set_relay(0);

				direction_was_changed = 1;
				relay_is_idle = 1;

				continue;
			}
		}

		/* manual/auto switch */
		if (mode_auto != MODE_AUTO()) {
			reset_idle_count();

			mode_auto = MODE_AUTO();
			mode_was_changed = 1;

			running = 0;

			continue;
		}


		/* direction change, possibly while running */
		cmd = get_commanded_direction();
		if (direction != cmd) {
			reset_idle_count();

			direction = cmd;
			direction_was_changed = 1;

			running = 0;

			continue;
		}

		/* run? */
		if (mode_auto || MANUAL_RUN()) {

			/* limits */
			if ((direction && !LIMIT1()) || (!direction && !LIMIT2())) {
				if (running)
					reset_idle_count();
				running = 0;
				continue;
			}

			/* ensure proper direction */
			if (!running) {
				if (relay_is_idle || (direction != relay_on)) {
					reset_idle_count();

					set_relay(direction);

					relay_is_idle = 0;
					direction_was_changed = 1;

					continue;
				}

				pause = MAXPAUSE;
				OCR1 = MAXPAUSE;

				running = 1;
			}
		} else {
			running = 0;
		}
	}
}
