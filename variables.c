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
	asm("DEC YL" "\n\t"
		"LD %0, Y" "\n\t"
		"INC YL" :"=r"(DATA) :);
	return(DATA);	// DATA is on register r24 used for return
}

/*__attribute__((naked))*/	void	st_RAMPY(uint8_t DATA)
{
	asm("DEC YL" "\n\t"
		"ST Y, %0" "\n\t"
		"INC YL" ::"r"(DATA));
}

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
