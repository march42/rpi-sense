/*
** variables.c - global variables
**
** (C) Copyright 2017 Marc Hefter
**
** Author: Marc Hefter
** All rights reserved.
**
** based on rpi-sense.S and main.c by Serge Schneider
** Copyright (c) 2015 Raspberry Pi Foundation
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**	* Redistributions of source code must retain the above copyright
**	  notice, this list of conditions and the following disclaimer.
**	* Redistributions in binary form must reproduce the above copyright
**	  notice, this list of conditions and the following disclaimer in the
**	  documentation and/or other materials provided with the distribution.
**	* Neither the name of Raspberry Pi nor the
**	  names of its contributors may be used to endorse or promote products
**	  derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//	_VARIABLES_C_ used to indicate declaration of global variables
#define _VARIABLES_C_
#include "rpi-sense.h"

/*__attribute__((naked))*/	uint8_t	ld_RAMPY(void)
{
	register uint8_t DATA asm("r24");
	asm("LD %0, Y" :"=r"(DATA) :);
	return(DATA);	// dummy return to satisfy compiler - warning: control reaches end of non-void function
}

/*__attribute__((naked))*/	void	st_RAMPY(uint8_t DATA)
{
	asm("ST Y, %0" ::"r"(DATA));
}

/*	routines for I2C register handling
*/
#if	defined(USE_REGWRITE) && !defined(TWI_DATA_RAMPY)
void write_registers(void)
{
	/*
	register uint8_t DATA asm("r24");
	asm("LD %0, Y" :"=r"(DATA) :);
	*/
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
