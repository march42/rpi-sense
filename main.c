/*
 * main.c - Atmel main loop for LED matrix driving
 *
 * Copyright (c) 2015 Raspberry Pi Foundation
 *
 * Author: Serge Schneider <serge@raspberrypi.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *	* Neither the name of Raspberry Pi nor the
 *	  names of its contributors may be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <stdio.h>
#include <stdlib.h>

#define EE_WP			(1 << PB0)
#define KEYS_INT		(1 << PB6)
#define FRAME_INT		(1 << PB7)

#define LED_SDO			(1 << PC0)
#define LED_CLKR		(1 << PC1)
#define LED_LE			(1 << PC2)
#define LED_SDI			(1 << PC3)
#define LED_OE_N		(1 << PC7)

#define set(port,x) port |= (x)
#define clr(port,x) port &= ~(x)

extern void draw_loop();
extern void clear_gain(void);
extern void delay(uint8_t ticks);
extern void write_data(uint32_t data, char type);
extern void check_keys(void);

/*	seems to be unused, maybe was intend to be I2C register addresses
enum REG_ADDR {
	REG_ID = 192,
	REG_CFG_LOW,
	REG_CFG_HIGH,
};
*/

/*	bit count values for LED2472G shift register
**	LED_LE must rise after transmitting n-th bit
**	TODO: use le_key for write_data and read_data
*/
typedef enum {
	DAT_LATCH = 22,
	CONF_WRITE = 20,
	CONF_READ  = 18,
	GAIN_WRITE = 16,
	GAIN_READ = 14,
	DET_OPEN = 13,
	DET_SHORT = 12,
	DET_OPEN_SHORT = 11,
	THERM_READ = 10,
} le_key;

volatile uint8_t pixels[] = {
	// line 1
	0x1F, 0x1F, 0x1F, 0x1F, 0x14, 0x03, 0x00, 0x00,	// red 1-8
	0x00, 0x00, 0x03, 0x12, 0x1F, 0x1F, 0x1F, 0x1F,	// green 1-8
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,	// blue 1-8
	// line 2
	0x1F, 0x1F, 0x1F, 0x12, 0x03, 0x00, 0x00, 0x00,	// red 9-16
	0x00, 0x04, 0x14, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,	// green 9-16
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x1D,	// blue 9-16
	// line 3
	0x1F, 0x1F, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00,	// red 17-24
	0x05, 0x15, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0B,	// green 17-24
	0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x1F, 0x1F,	// blue 17-24
	// line 4
	0x1F, 0x0F, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,	// red 25-32
	0x17, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0A, 0x00,	// green 25-32
	0x00, 0x00, 0x00, 0x00, 0x0A, 0x1F, 0x1F, 0x1F,	// blue 25-32
	// line 5
	0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,	// red 33-40
	0x1F, 0x1F, 0x1F, 0x1F, 0x1D, 0x08, 0x00, 0x00,	// green 33-40
	0x00, 0x00, 0x01, 0x0B, 0x1F, 0x1F, 0x1F, 0x1F,	// blue 33-40
	// line 6
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x14,	// red 41-48
	0x1F, 0x1F, 0x1F, 0x1B, 0x07, 0x00, 0x00, 0x00,	// green 41-48
	0x00, 0x01, 0x0C, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,	// blue 41-48
	// line 7
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x15, 0x1F,	// red 49-56
	0x1F, 0x1F, 0x19, 0x06, 0x00, 0x00, 0x00, 0x00,	// green 49-56
	0x02, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x12,	// blue 49-56
	// line 8
	0x00, 0x00, 0x00, 0x00, 0x05, 0x17, 0x1F, 0x1F,	// red 57-64
	0x1F, 0x17, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,	// green 57-64
	0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x02,	// blue 57-64
};

#ifndef NDEBUG
volatile uint8_t registers[] = {
	0x00, 0x01, 0x02,					// PORTA
	0x03, 0x04, 0x05,					// PORTB
	0x06, 0x07, 0x08,					// PORTC
	0x09, 0x0A, 0x0B,					// PORTD
	0x0C,								// MCUSR
	0x0D,								// MCUCR
	0x0E,								// PORTCR
	0x0F,								// PRR
};
#endif

volatile uint8_t keys		= 0xF2;		// joystick input status
volatile uint8_t watchdog	= 0xF6;		// watchdog interrupt counter
volatile uint8_t i2c_reg	= 0xFF;		// I2C register address
volatile char i2c_busy		= 1;		// I2C transactions in progress
volatile char redrawleds	= 1;		// redraw LEDs after pixels has been updated

ISR(BADISR_vect, ISR_NAKED)
{
	/*	maybe here must the interrupt flags be cleared
	*/
	reti();						//	do nothing, just return
}
ISR(INT0_vect, ISR_ALIASOF(BADISR_vect));
ISR(INT1_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT0_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT1_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT2_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT3_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_CAPT_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_COMPA_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_COMPB_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_OVF_vect, ISR_ALIASOF(BADISR_vect));
ISR(SPI_STC_vect, ISR_ALIASOF(BADISR_vect));
ISR(ADC_vect, ISR_ALIASOF(BADISR_vect));
ISR(EE_READY_vect, ISR_ALIASOF(BADISR_vect));
ISR(ANALOG_COMP_vect, ISR_ALIASOF(BADISR_vect));

ISR(TIMER0_OVF_vect)
{
	redrawleds	= 1;			// set flag to redraw LEDs
	TIFR0		= 0;			// clear interrupt flag register
	reti();						// return from interrupt and enable global interrupts
}
ISR(TIMER0_COMPA_vect, ISR_ALIASOF(TIMER0_OVF_vect));
ISR(TIMER0_COMPB_vect, ISR_ALIASOF(TIMER0_OVF_vect));

ISR(WDT_vect)
{
	//	watchdog ISR is only used for wakeup
	++watchdog;					// count
	redrawleds	= 1;			// set flag to redraw LEDs
	MCUSR	&= ~(1 << WDRF);	// clear watchdog reset flag
	/*	WDIF is cleared by hardware when executing the corresponding interrupt handling vector.
	WDTCSR	&= ~(1 << WDIF);	// clear interrupt flag
	*/
	reti();						// return from interrupt and enable global interrupts
}

/*	defined in rpi-sense.S	ISR(TWI_vect, ISR_ALIASOF(BADISR_vect));*/

__fuse_t __fuse __attribute__((section (".fuse"))) =
{
	// CKSEL2+CKSEL3 dont exist according to data sheet
	// internal 8MHz oscillator, no DIV8 prescaling
	.low = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),	// FUSE_CKOUT, FUSE_SUT1, FUSE_CKSEL1, FUSE_CKDIV8
	// ICSP enabled, BODLEVEL=2.9V
	.high = (FUSE_SPIEN & FUSE_BODLEVEL2 & FUSE_BODLEVEL0),	// FUSE_RSTDISBL, FUSE_DWEN, FUSE_WDTON, FUSE_EESAVE, FUSE_BODLEVEL1
	.extended = (0xFF),	// FUSE_SELFPRGEN
};

#define watchdog_prescalerbits(wdp)	(((wdp >> 3 &1) << WDP3) | ((wdp >> 2 &1) << WDP2) | ((wdp >> 1 &1) << WDP1) | ((wdp &1) << WDP0))
#define clock_prescalerbits(clkps)	(((clkps >> 3 &1) << CLKPS3) | ((clkps >> 2 &1) << CLKPS2) | ((clkps >> 1 &1) << CLKPS1) | ((clkps &1) << CLKPS0))
#define timer0_prescalerbits(cs0)	(((cs0 >> 2 &1) << CS02) | ((cs0 >> 1 &1) << CS01) | ((cs0 &1) << CS00))
/*	I/O port fundamentals
**	PINx	I/O address
**	DDRx	data direction 0=input, 1=output
**	PORTx	I/O address
**		while DDRxn==0 (input pin)	- writing PORTxn to 1 activates pull-up resistor
**		while DDRxn==1 (output pin)	- writing PORTxn to 1 drives active-high, 0 drives active-low
**		writing PINxn to 1 toggles value of PORTxn	- DDRxn irrelevant
**	NOTE from data sheet:
**		When switching between tri-state ({DDxn, PORTxn} = 0b00) and output high ({DDxn, PORTxn}
**		= 0b11), an intermediate state with either pull-up enabled {DDxn, PORTxn} = 0b01) or output
**		low ({DDxn, PORTxn} = 0b10) must occur. Normally, the pull-up enabled state is fully acceptable,
**		as a high-impedance environment will not notice the difference between a strong high driver
**		and a pull-up. If this is not the case, the PUD bit in the MCUCR Register can be set to disable all
**		pull-ups in all ports.
**		Switching between input with pull-up and output low generates the same problem. The user
**		must use either the tri-state ({DDxn, PORTxn} = 0b00) or the output high state ({DDxn, PORTxn}
**		= 0b11) as an intermediate step.
**		When reading back a software assigned pin value, a nop instruction must be inserted.
*/
int main(void)
{
	// define pull-up values
	//MCUCR	|= (1 << PUD);					// disable pull-up resistors
	PORTA	= 0;
	PORTB	= EE_WP;						// enable write protect pull-up resistor
	PORTC	= 0;
	PORTD	= 0;
	// define output pins
	DDRA	= 0;
	DDRB	= EE_WP | FRAME_INT | KEYS_INT;
	DDRC	= LED_SDI | LED_CLKR | LED_LE | LED_OE_N;
	DDRD	= 0xFF;

	/*	configure clocks
	*/
	CLKPR	= (1 << CLKPCE);				// enable CLKPS change
	CLKPR	= clock_prescalerbits(0);		// no clock pre-scaler divisor (8MHz)

	/*	configure TIMER0
	**	use TIMER0 for regularly LED updates wont work, if using SLEEP
	**	TIMER0 wont run in power down mode, but in idle
	**	CS0:	0=disabled, 1=clkIO, 2=clkIO/8, 3=clkIO/64, 4=clkIO/256, 5=clkIO/1024, 6=T0 pin falling, 7=T0 pin rising
	*/
	TCNT0	= 0;							// clear timer0 counter register
	OCR0A	= 0;							// clear timer0 output compare register
	OCR0B	= 0;							// clear timer0 output compare register
	TCCR0A	= (0 << CTC0) | timer0_prescalerbits(5);	// TWI clock select clkT0=clkIO/1024 =8kHz
	TIFR0	= 0;							// clear interrupt flag register
	TIMSK0	= (0 << OCIE0B) | (0 << OCIE0A) | (1 << TOIE0);	// interrupt enable flags
	//PRR		&= ~(1 << PRTIM0);				// disable PRTIM0 flag bit

	// configure TWI serial I2C interface
	TWBR	= 0xff;							// TWI bit rate
	TWAR	= 0x46 << 1;					// TWI slave address
	TWCR	= (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWIE);
	//PRR		&= ~(1 << PRTWI);				// disable PRTWI flag bit
	i2c_busy	= 0;						// clear I2C operation in progress flag

	/*	watchdog has on chip 128kHz oscillator
	**	prescaler bits 0=2k, 1=4k, 2=8k, 3=16k, 4=32k, 5=64k, 6=128k, 7=256k, 8=512k, 9=1024k
	**	timeout 0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms, 6=1s, 7=2s, 8=4s, 9=8s
	*/
	cli();									// global interrupt disable
	wdt_reset();							// reset watchdog timer
	MCUSR	&= ~(1 << WDRF);				// clear WDRF flag
	WDTCSR	= (1 << WDCE) | (1 << WDE);		// enable watchdog change
	WDTCSR	= (1 << WDIE) | (0 << WDE) | watchdog_prescalerbits(3);	// enable interrupt, disable reset

	/*	configure SLEEP mode
	**	sleep modes - 0b10=power-down, 0b01=ADC noise reduction, 0b00=IDLE
	*/
#ifdef USE_SLEEP
	SMCR	= (1 << SM1) | (0 << SM0) | (0 << SE);	// enable SLEEP to power-down mode
#else
	SMCR	= (0 << SM1) | (0 << SM0) | (0 << SE);	// enable SLEEP to IDLE mode
#endif	// USE_SLEEP

	//	disable TIMER1,SPI,ADC clock for power reduction
	PRR		= (0 << PRTWI) | (0 << PRTIM0) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC);

	for(;;)
	{
		wdt_reset();		// reset watchdog timer
		sei();				// Set Enable global Interrupts

		while(0 != i2c_busy)
		{
			//	just wait for I2C operation to finish
			delay(200);		// (clkT0=8kHz, 200/8000=1/40s =25ms
		}

		//clear_gain();
		if(0 == redrawleds)
		{
			check_keys();	// skip LED update and only check joystick
		}
		else
		{
			draw_loop();	// copy pixels to LED shift register
			// draw_loop always finishes with check_keys
		}

#ifndef NDEBUG
		// copy registers to I2C data buffer
		int pos = 0;
		registers[pos++]	= PINA;
		registers[pos++]	= DDRA;
		registers[pos++]	= PORTA;
		registers[pos++]	= PINB;
		registers[pos++]	= DDRB;
		registers[pos++]	= PORTB;
		registers[pos++]	= PINC;
		registers[pos++]	= DDRC;
		registers[pos++]	= PORTC;
		registers[pos++]	= PIND;
		registers[pos++]	= DDRD;
		registers[pos++]	= PORTD;
		registers[pos++]	= MCUSR;
		registers[pos++]	= MCUCR;
		registers[pos++]	= WDTCSR;
		registers[pos++]	= SMCR;
#endif	// NDEBUG

		if(0 == i2c_busy && 0 == redrawleds)
		{
			// only if I2C operation ended and redraw unnecessary
			sei();			// enable interrupts before SLEEP
			SMCR	|= (1 << SE);	// enable SLEEP mode
			sleep_cpu();	// sleep and wait for interrupt
			SMCR	&= ~(1 << SE);	// clear SLEEP enable flag
			_NOP();
			_NOP();			// just to be sure a little delay after wake up from SLEEP
		}
	}

	return(0);	// hopefully we never reach this
}
