/*
 * main.c - Atmel main loop for LED matrix driving
 *
 * (C) Copyright 2017 Marc Hefter
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

#include "rpi-sense.h"

ISR(BADISR_vect, ISR_NAKED)
{
	/*	maybe here must the interrupt flags be cleared
	*/
	reti();										//	do nothing, just return
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
	//	without attribute ISR_NAKED registers get pushed, popped and RETI gets added
	++counttmr;									// count (interrupt frequency 1/32768ns)
}
ISR(TIMER0_COMPA_vect, ISR_ALIASOF(BADISR_vect));	// TIMER0 COMParator A unused
ISR(TIMER0_COMPB_vect, ISR_ALIASOF(BADISR_vect));	// TIMER0 COMParator B unused

ISR(WDT_vect)
{
	//	without attribute ISR_NAKED registers get pushed, popped and RETI gets added
	++watchdog;									// count
	//	watchdog ISR is only used for wakeup from SLEEP
}

__fuse_t __fuse __attribute__((section (".fuse"))) =
{
	// CKSEL2+CKSEL3 don't exist according to data sheet
	// internal 8MHz oscillator, no DIV8 prescaler
	.low = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),	// FUSE_CKOUT, FUSE_SUT1, FUSE_CKSEL1, FUSE_CKDIV8
	// ICSP enabled, BODLEVEL=2.9V
	.high = (FUSE_SPIEN & FUSE_BODLEVEL2 & FUSE_BODLEVEL0),	// FUSE_RSTDISBL, FUSE_DWEN, FUSE_WDTON, FUSE_EESAVE, FUSE_BODLEVEL1
	.extended = (0xFF),	// FUSE_SELFPRGEN
};

/*	routines for I2C register handling
*/
#if	defined(USE_REGWRITE) && !defined(TWI_DATA_RAMPY)
void write_registers(void)
{
	switch(i2caddr)
	{
	// watchdog control
	case REG_WDTCSR:
		WDTCSR	= _BV(WDCE) | _BV(WDE);	// enable watchdog change
		WDTCSR	= reg_ui8(0,REG_WDTCSR);
		break;
	// sleep mode control
	case REG_SMCR:
		SMCR	= reg_ui8(0,REG_SMCR);
		break;
	// copy debugging registers (0xEx)
	case REG_PINA:
		PINA	= reg_ui8(0,REG_PINA);
		break;
	case REG_DDRA:
		DDRA	= reg_ui8(0,REG_DDRA);
		break;
	case REG_PORTA:
		PORTA	= reg_ui8(0,REG_PORTA);
		break;
	case REG_PINB:
		PINB	= reg_ui8(0,REG_PINB);
		break;
	case REG_DDRB:
		DDRB	= reg_ui8(0,REG_DDRB);
		break;
	case REG_PORTB:
		PORTB	= reg_ui8(0,REG_PORTB);
		break;
	case REG_PINC:
		PINC	= reg_ui8(0,REG_PINC);
		break;
	case REG_DDRC:
		DDRC	= reg_ui8(0,REG_DDRC);
		break;
	case REG_PORTC:
		PORTC	= reg_ui8(0,REG_PORTC);
		break;
	case REG_PIND:
		PIND	= reg_ui8(0,REG_PIND);
		break;
	case REG_DDRD:
		DDRD	= reg_ui8(0,REG_DDRD);
		break;
	case REG_PORTD:
		PORTD	= reg_ui8(0,REG_PORTD);
		break;
	case REG_MCUSR:
		MCUSR	= reg_ui8(0,REG_MCUSR);
		break;
	case REG_MCUCR:
		MCUCR	= reg_ui8(0,REG_MCUCR);
		break;
	case REG_PORTCR:
		PORTCR	= reg_ui8(0,REG_PORTCR);
		break;
	case REG_PRR:
		PRR		= reg_ui8(0,REG_PRR);
		break;
	}
}
#endif	// defined(USE_REGWRITE) && !defined(TWI_DATA_RAMPY)

void read_registers(void)
{
	//	copy register data to buffer
	reg_ui8(0,REG_EE_WP)	= ((PORTB & _BV(EE_WP)) ?0 :1);	// copy EEPROM_WP disabled to I2C data buffer
	reg_ui8(0,REG_WDTCSR)	= WDTCSR;	// copy register to I2C data buffer
	reg_ui8(0,REG_SMCR)		= SMCR;		// copy register to I2C data buffer

#	if !defined(NDEBUG) || defined(USE_REGWRITE)
		// copy debugging registers (0xEx) to I2C data buffer
		reg_ui8(0,REG_PINA)		= PINA;		// copy register to I2C data buffer
		reg_ui8(0,REG_DDRA)		= DDRA;		// copy register to I2C data buffer
		reg_ui8(0,REG_PORTA)	= PORTA;	// copy register to I2C data buffer
		reg_ui8(0,REG_PINB)		= PINB;		// copy register to I2C data buffer
		reg_ui8(0,REG_DDRB)		= DDRB;		// copy register to I2C data buffer
		reg_ui8(0,REG_PORTB)	= PORTB;	// copy register to I2C data buffer
		reg_ui8(0,REG_PINC)		= PINC;		// copy register to I2C data buffer
		reg_ui8(0,REG_DDRC)		= DDRC;		// copy register to I2C data buffer
		reg_ui8(0,REG_PORTC)	= PORTC;	// copy register to I2C data buffer
		reg_ui8(0,REG_PIND)		= PIND;		// copy register to I2C data buffer
		reg_ui8(0,REG_DDRD)		= DDRD;		// copy register to I2C data buffer
		reg_ui8(0,REG_PORTD)	= PORTD;	// copy register to I2C data buffer
		reg_ui8(0,REG_MCUSR)	= MCUSR;	// copy register to I2C data buffer
		reg_ui8(0,REG_MCUCR)	= MCUCR;	// copy register to I2C data buffer
		reg_ui8(0,REG_PORTCR)	= PORTCR;	// copy register to I2C data buffer
		reg_ui8(0,REG_PRR)		= PRR;		// copy register to I2C data buffer
#	endif	// !defined(NDEBUG) || defined(USE_REGWRITE)
}

#if defined(USE_LEDWRITE)
void write_led(uint8_t _looping)
{
	switch(_looping &0x3)
	{
	case 0:
		if(1 == reg_ui8(0, REG_LED_CONF +3))		// check write indicator flag
		{
			//	unused byte3 (bit31-24) used as write indicator flag
			write_data(LED_CFG,CONF_WRITE);
			//reg_ui8(0, REG_LED_CONF +3)		= 0;	// clear write indicator flag
		}
		LED_CFG		= read_data(CONF_READ);
		break;
	case 1:
		LED_ERROR	= read_data(DET_OPEN_SHORT);
		break;
	case 2:
		if(1 == reg_ui8(0, REG_LED_GAIN +3))		// check write indicator flag
		{
			//	unused byte3 (bit31-24) used as write indicator flag
			write_data(LED_GAIN,GAIN_WRITE);
			//reg_ui8(0, REG_LED_GAIN +3)		= 0;	// clear write indicator flag
		}
		LED_GAIN	= read_data(GAIN_READ);
		break;
	case 3:
		LED_THERMAL	= read_data(THERM_READ);
		break;
	}
}
#endif // defined(USE_LEDWRITE)

#if	defined(USE_LEDREAD) || defined(USE_LEDWRITE)
void read_led(uint8_t _looping)
{
	if(0xFF == _looping || 0 == (_looping &0x3))
		LED_CFG		= read_data(CONF_READ);
	if(0xFF == _looping || 1 == (_looping &0x3))
		LED_ERROR	= read_data(DET_OPEN_SHORT);
	if(0xFF == _looping || 2 == (_looping &0x3))
		LED_GAIN	= read_data(GAIN_READ);
	if(0xFF == _looping || 3 == (_looping &0x3))
		LED_THERMAL	= read_data(THERM_READ);
}
#endif // USE_LEDREAD || USE_LEDWRITE

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
	uint8_t _looping		= 0;				// local counter variable for main working loop

	/*	variable initialization and preparation
	*/
#	if defined(TWI_DATA_RAMPY)
		i2creg.ptr	= (uint8_t*)&registers;		// set address of registers
		DEC_i2caddr;							// decrement address - will be incremented before access
#	else // defined(TWI_DATA_RAMPY)
		i2cpage		= 0;						// set register page to 0
		i2caddr		= 0xFF;						// set address of registers to -1 - will be incremented before access
#	endif // defined(TWI_DATA_RAMPY)
	CLR_i2cflags;								// clear all flags

	/*	configure TWI serial I2C interface
	**	configure for interrupt driven SLAVE
	**	at 100kHz bus clock one data byte received every 90ys (720 CPU cycles)
	**	at 400kHz bus clock one data byte received every 22.5ys (180 CPU cycles)
	*/
	TWBR		= 0xff;							// TWI bit rate
	TWAR		= FW_I2CSLA << 1;				// TWI slave address
	TWCR		= _BV(TWEA) | _BV(TWEN) | _BV(TWINT) | _BV(TWIE);
	//PRR			&= ~_BV(PRTWI);					// disable PRTWI flag bit

	/*	startup speeding to prevent problem on loading kernel modules
	**	first of all start TWI system
	*/
	sei();										// Set Enable global Interrupts

	// define pull-up values
	//MCUCR		|= (1 << PUD);						// disable pull-up resistors
	PORTA		= 0;
	PORTB		= _BV(EE_WP);					// enable write protect pull-up resistor
	PORTC		= 0;
	PORTD		= 0;
	// define output pins
	DDRA		= 0;
	DDRB		= _BV(EE_WP) | _BV(FRAME_INT) | _BV(KEYS_INT);
	DDRC		= _BV(LED_SDI) | _BV(LED_CLKR) | _BV(LED_LE) | _BV(LED_OE_N);
	DDRD		= 0xFF;

	/*	configure clocks
	**	CLKPR pre-scaler is set to 0b0011 if CKDIV8 fuse is programmed, to 0b0000 otherwise
	**	CLKPS 0000==1, 0001==2, 0010==4, 0011==8, 0100==16, 0101==32, 0110==64, 0111==128, 1000==256
	**	clock cycle is 125ns at 8MHz
	*/
	CLKPR		= _BV(CLKPCE);					// enable CLKPS change
	CLKPR		= clock_prescalerbits(0);		// no clock pre-scaler divisor (8MHz)

	/*	watchdog has on chip 128kHz oscillator
	**	prescaler bits 0=2k, 1=4k, 2=8k, 3=16k, 4=32k, 5=64k, 6=128k, 7=256k, 8=512k, 9=1024k
	**	timeout 0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms, 6=1s, 7=2s, 8=4s, 9=8s
	*/
	watchdog	= 0;							// clear watchdog counter
	wdt_reset();								// reset watchdog timer
	MCUSR		&= ~_BV(WDRF);					// clear WDRF flag
	wdt_reg		= _BV(WDE) | watchdog_prescalerbits(6);	// enable watchdog reset
#if !defined(NDEBUG)
	wdt_reg		|= _BV(WDIE);					// enable interrupt and reset if NOT in release
#endif // defined(NDEBUG)
	WDTCSR		= _BV(WDCE) | _BV(WDE);			// enable watchdog change
	WDTCSR		= wdt_reg;						// copy watchdog configuration to WDTCSR

	/*	configure TIMER0
	**	TIMER0 wont run in power down mode, but in idle
	**	TIMER0 gets fed from system clock through pre-scaler - 8MHz == 1/125ns
	**	CS0:	0=disabled, 1=clkIO, 2=clkIO/8, 3=clkIO/64, 4=clkIO/256, 5=clkIO/1024, 6=T0 pin falling, 7=T0 pin rising
	**	TOV0 interrupt with prescaler==5 (clkT0=7.8125kHz, 128ys) overflow at 30Hz every 32768ys
	**	TOV0 interrupt with prescaler==4 (clkT0=31.25kHz, 32ys) overflow at 122Hz every 8192ys
	**	TOV0 interrupt with prescaler==3 (clkT0=125kHz, 8ys) overflow at 488Hz every 2048ys
	*/
	counttmr	= 0;							// clear TIMER0 overflow interrupt counter
	TCNT0		= 0;							// clear timer0 counter register
	OCR0A		= 0;							// clear timer0 output compare register
	OCR0B		= 0;							// clear timer0 output compare register
	TCCR0A		= timer0_prescalerbits(5);		// TWI clock select clkT0=clkIO/1024 =7812.5kHz, tT0=128ys
	TIFR0		= 0;							// clear interrupt flag register
#if !defined(NDEBUG)
	TIMSK0		= _BV(TOIE0);					// interrupt enable flags
#else // !defined(NDEBUG)
	TIMSK0		= 0;							// disable TIMER0 interrupts
#endif // !defined(NDEBUG)

	/*	configure SLEEP mode
	**	sleep modes - 0b10=power-down, 0b01=ADC noise reduction, 0b00=IDLE
	*/
	SMCR		= (0 << SM1) | (0 << SM0) | (1 << SE);	// enable SLEEP to IDLE mode

	//	disable TIMER1, SPI and ADC for power reduction
	PRR			= _BV(PRTIM1) | _BV(PRSPI) | _BV(PRADC);
	/*
	**	#define __AVR_HAVE_PRR	((1<<PRADC)|(1<<PRSPI)|(1<<PRTIM1)|(1<<PRTIM0)|(1<<PRTWI))
	**	#define __AVR_HAVE_PRR_PRADC
	**	#define __AVR_HAVE_PRR_PRSPI
	**	#define __AVR_HAVE_PRR_PRTIM1
	**	#define __AVR_HAVE_PRR_PRTIM0
	**	#define __AVR_HAVE_PRR_PRTWI
	*/

	// write LED2472G configuration and default gain from registers
	write_data(reg_ui32(0,REG_LED_CONF), CONF_WRITE);			//	configure LED2472G shift register
	write_data(reg_ui32(0,REG_LED_GAIN), GAIN_WRITE);			//	clear LED2472G shift register gain

	_looping	= 0;
	while(TRUE)
	{
#if !defined(NDEBUG) || defined(USE_REGWRITE)
		/*	to keep watchdog in interrupt+reset mode, interrupt needs to be reenabled after every occureance
		**	BUT in release firmware we WANT the watchdog to reset the device by default
		*/
		WDTCSR		= wdt_reg;					// enable watchdog timer interrupt - will be cleared automatically if interrupt+reset mode
#endif // !defined(NDEBUG) || defined(USE_REGWRITE)
		wdt_reset();							// reset watchdog timer
		sei();									// Set Enable global Interrupts

		draw_loop();							// copy pixels to LED shift register

		if(0 != runflags.validreg)
		{
			//	received new DATA in valid register
			runflags.validreg	= 0;			// clear new data flag

			if(0 < i2cpage)
			{
				// ignore everything not on first page
			}
#if defined(USE_REGWRITE)
			else
			{
				write_registers();
			}
#endif // defined(USE_REGWRITE)

		}
		else if(4 == (_looping & 0x0F))			// every 16th run
		{
			read_registers();
		}

#if defined(USE_LEDREAD) || defined(USE_LEDWRITE)
		else if(4 > (_looping >> 6))			// every 64,128,172,256th run
		{
#	if defined(USE_LEDWRITE)
			write_led(_looping >> 6);
#	else // defined(USE_LEDWRITE)
			read_led(_looping >> 6);
#	endif // defined(USE_LEDWRITE)
		}
#endif // defined(USE_LEDREAD) || defined(USE_LEDWRITE)

#if !defined(NDEBUG)
		reg_ui8(0,0xCF)		= _looping;			// copy looping counter for debugging
#endif // !defined(NDEBUG)

#ifdef USE_SLEEP
		if(0 == runflags.inprogress)
		{
			DDRD	= 0xFF;
			PORTD	= 0x00;
			clr(PORTC, LED_OE_N);
			// only if I2C operation ended and redraw unnecessary
			sei();								// enable interrupts before SLEEP
			SMCR	|= _BV(SE);					// enable SLEEP mode
			TIMSK0	&= ~_BV(TOIE0);				// clear TOI0 interrupt enable flags
			sleep_cpu();						// sleep and wait for interrupt
			_NOP();
			_NOP();								// just to be sure a little delay after wake up from SLEEP
			SMCR	&= ~_BV(SE);				// clear SLEEP enable flag
			TIMSK0	|= _BV(TOIE0);				// set TOI0 interrupt enable flags
		}
#endif

		_looping++;								// count
	}

	return(0);									// hopefully we never reach this
}
