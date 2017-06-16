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
	/*	interrupt flags will be cleared by hardware on RETI
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
/*	defined in rpi-sense.S	ISR(TWI_vect, ISR_ALIASOF(BADISR_vect));	*/

ISR(TIMER0_OVF_vect, ISR_NAKED)
{
	//	without attribute ISR_NAKED registers get pushed, popped and RETI gets added
	asm volatile(
		"PUSH r0" "\n\t"
		"LDS r0, (0x100 + %0)" "\n\t"
		"INC r0" "\n\t"
		"STS (0x100 + %0), r0" "\n\t"
		"POP r0" "\n\t"
		 : :"M"(REG_TOV0C));
	//++counttmr;									// count (interrupt frequency 1/32768ns)
	reti();										//	return and clear interrupt flag
}
ISR(TIMER0_COMPA_vect, ISR_ALIASOF(TIMER0_OVF_vect));	// TIMER0 COMParator A used on 4MHz CPU clock

ISR(TIMER0_COMPB_vect, ISR_NAKED)				// TIMER0 COMParator B used to wakeup from sleep
{
	reti();										//	return and clear interrupt flag
}

ISR(WDT_vect, ISR_NAKED)
{
	//	without attribute ISR_NAKED registers get pushed, popped and RETI gets added
	//	watchdog ISR is only used for wakeup from SLEEP
	asm volatile(
		"PUSH r0" "\n\t"
		"LDS r0, (0x100 + %0)" "\n\t"
		"INC r0" "\n\t"
		"STS (0x100 + %0), r0" "\n\t"
		"POP r0" "\n\t"
		: :"M"(REG_WDC));
	//++watchdog;									// count
	reti();										//	return and clear interrupt flag
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
	uint8_t _waitT0			= 0;				// local wait variable for main loop

	/*	variable initialization and preparation
	*/
	i2creg.ptr		= (uint8_t*)&registers;		// i2cpage is always +1 (memory start of registers)
	CLR_i2cflags;								// clear all flags

	/*	configure clocks
	**	CLKPR pre-scaler is set to 0b0011 if CKDIV8 fuse is programmed, to 0b0000 otherwise
	**	CLKPS 0000==1, 0001==2, 0010==4, 0011==8, 0100==16, 0101==32, 0110==64, 0111==128, 1000==256
	**	clock cycle is 125ns at 8MHz, 250ns at 4MHz
	*/
	CLKPR		= _BV(CLKPCE);					// enable CLKPS change
	CLKPR		= DEFAULT_CLKPR;				// clock pre-scaler 8MHz or 4MHz

	/*	configure TWI serial I2C interface
	**	configure for interrupt driven SLAVE
	**	at 100kHz bus clock one data byte received every 90ys (720 CPU cycles)
	**	at 400kHz bus clock one data byte received every 22.5ys (180 CPU cycles)
	**	normal mode (TWHSR &_BV(TWHS) ==0) - TWI clock is fed from I/O clock clkIO
	**	high speed mode (TWHSR==_BV(TWHS)) - TWI clock is fed from system clock clkCPU, clkIO must be EXACTLY clkCPU/2
	**	TWSR, twi_prescalerbits(0-3) selects pre-scaler 1, 4, 16, 64
	**	fSCL = clkIO/clkCPU / (16 + (2 * TWBR * TWPS))
	**	in master mode TWBR must be >=10 so maximum TWI speed is 222kHz
	*/
	TWSR		= DEFAULT_TWSR;					// set pre-scaler
	TWHSR		= DEFAULT_TWHSR;				// TWHS bit flag defined in headers as TWIHS
	TWBR		= DEFAULT_TWBR;					// TWI bit rate
	TWAR		= FW_I2CSLA << 1;				// TWI slave address
	TWAMR		= (I2C_MAXPAGE << 1);			// TWI slave address mask
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

	/*	watchdog has on chip 128kHz oscillator
	**	prescaler bits 0=2k, 1=4k, 2=8k, 3=16k, 4=32k, 5=64k, 6=128k, 7=256k, 8=512k, 9=1024k
	**	timeout 0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms, 6=1s, 7=2s, 8=4s, 9=8s
	*/
	watchdog	= 0;							// clear watchdog counter
	wdt_reset();								// reset watchdog timer
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
	**	for I2C_HIGHSPEED and clkIO=4MHz PS==5 sets clkT0=3.90625kHz, 256ys and overflow every 65536ys
	*/
	counttmr	= 0;							// clear TIMER0 overflow interrupt counter
	TCCR0A		= timer0_prescalerbits(5);		// TIMER0 clock select clkT0=clkIO/1024 =7812.5kHz, tT0=128ys
	TIFR0		= 0;							// clear interrupt flag register
#if defined(NDEBUG)
	TIMSK0		= 0;							// disable TIMER0 interrupts
#else // defined(NDEBUG)
#	if (I2C_HIGHSPEED)
		OCR0A	= 0x7F;							// timer0 output compare register hits after value while overflow hits before value
		TIMSK0	= _BV(TOIE0) | _BV(OCIE0A);		// enable comparator at half time to achieve 32768ys interrupt
#	else // (I2C_HIGHSPEED)
		TIMSK0	= _BV(TOIE0);					// interrupt enable flags
#	endif // (I2C_HIGHSPEED)
#endif // defined(NDEBUG)

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
		//	_waitT0 is used to synchronize main loop to 100Hz
		_waitT0			= TCNT0;				// current T0 counter on loop start

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

#if defined(USE_REGWRITE)
#	if (1 < I2C_PAGES)
			if(1 < i2cpage)						// first page on address 0x0100
			{
				// ignore everything not on first page
			}
			else
#	endif // (1 < I2C_PAGES)
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

#ifdef USE_SLEEP
		if(0 == runflags.inprogress)
		{
			DDRD		= 0xFF;
			PORTD		= 0x00;
			clr(PORTC, LED_OE_N);
			// only if I2C operation ended and redraw unnecessary
			sei();								// enable interrupts before SLEEP
			SMCR		|= _BV(SE);				// enable SLEEP mode
			OCR0B		= _waitT0;				// set next wake up (to get 100Hz loop)
#			if (I2C_HIGHSPEED)
				OCR0B	+= 39;					// 39x256ys = ~10ms ==100Hz
#			else // (I2C_HIGHSPEED)
				OCR0B	+= 78;					// 78x128ys = ~10ms ==100Hz
#			endif // (I2C_HIGHSPEED)
			TIMSK0		|= _BV(OCIE0B);			// enable comparator at next 100Hz interval
			sleep_cpu();						// sleep and wait for interrupt
			_NOP();								// just to be sure a little delay after wake up from SLEEP
			SMCR		&= ~_BV(SE);			// clear SLEEP enable flag
			TIMSK0		&= ~_BV(OCIE0B);		// disable comparator
		}
#else // USE_SLEEP
#	if (I2C_HIGHSPEED)
		while(39 > (uint8_t)(TCNT0 - _waitT0))	// 39x256ys = ~10ms ==100Hz
#	else // (I2C_HIGHSPEED)
		while(78 > (uint8_t)(TCNT0 - _waitT0))	// 78x128ys = ~10ms ==100Hz
#	endif // (I2C_HIGHSPEED)
		{
			_NOP();								// wait
		}
#endif // USE_SLEEP

		_looping++;								// count
	}

}
