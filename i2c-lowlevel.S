/*
 * i2c-lowlevel.S - assembly routines for I2C/TWI bus handling
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

.section .text

#if !defined(TWI_VECTOR_M) && !defined(TWI_VECTOR_S)
.func	TWI_vect
/*	used registers
**	PARAMH	TW_STATUS (TWSR & TW_STATUS_MASK)
**	PARAML	TWCR
**	PARAMH	TWDR
**	PARAML	masked i2caddr
**	PARAMH	I2C register DATA
**
**	YH:YL pointer is always used for I2C registers
*/
TWI_vect:
	;		save registers for ISR
	;PUSH	ZEROREG								; save register __zero_reg__
	PUSH	TEMPREG								; save register __temp_reg__
	IN		TEMPREG, _SFR_IO_ADDR(SREG)			; save register SREG
	PUSH	TEMPREG								; save register SREG
	PUSH	PARAMH								; save register
	PUSH	PARAML								; save register
	;		prepare and get TW_STATUS
	SETFLAG_INPROGRESS							; set TWI/I2C in progress flag
	LDS		PARAMH, TWSR						; load status register
	ANDI	PARAMH, TW_STATUS_MASK				; mask out pre scaler bits
	BRBS	SREG_Z, TWI_vect_RESET				; TW_BUS_ERROR==0
	;		check TW_STATUS
	CPI		PARAMH, TW_ST_DATA_ACK				; slave transmitter data ACKnowledged
	BREQ	TWI_vect_STDATA
	CPI		PARAMH, TW_ST_SLA_ACK				; slave transmitter SLA+R received
	BREQ	TWI_vect_STSLAR
	CPI		PARAMH, TW_SR_DATA_ACK				; slave receiver DATA ACKnowledged
	BREQ	TWI_vect_SRADDR
	CPI		PARAMH, TW_SR_SLA_ACK				; slave receiver SLA+W received
	BREQ	TWI_vect_SRSLAW
	CPI		PARAMH, TW_SR_STOP					; slave receiver stop condition
	BREQ	TWI_vect_STOP
	CPI		PARAMH, TW_ST_LAST_DATA				; slave transmitter last byte acknowledged
	BREQ	TWI_vect_STOP
	CPI		PARAMH, TW_SR_DATA_NACK				; slave receiver data received NACK returned
	BREQ	TWI_vect_STOP
	CPI		PARAMH, TW_ST_DATA_NACK				; slave transmitter data sent NACK received
	BREQ	TWI_vect_STOP

	// I2C spec: "If two or more masters try to put information onto the bus, the first to produce a ‘one’ when the other produces a ‘zero’ will lose the arbitration"
	CPI		PARAMH, TW_SR_ARB_LOST_SLA_ACK		; slave receiver arbitration lost in SLA+W
	BREQ	TWI_vect_RESET
	CPI		PARAMH, TW_ST_ARB_LOST_SLA_ACK		; slave transmitter arbitration lost in SLA+R
	BREQ	TWI_vect_RESET

	RJMP	TWI_vect_NACK						; jump to NACK

TWI_vect_RESET:
	LDS		PARAML,	TWCR						; load TWCR register
	SBR		PARAML,	_BV(TWSTO)					; set TWSTO flag
	STS		TWCR, PARAML						; This will not generate a STOP condition, but the TWI returns to unaddressed and releases lines
TWI_vect_RESETWAIT:
	LDS		PARAML,	TWCR						; load TWCR register
	SBRC	PARAML,	TWSTO						; skip if bit cleared - TWI has been reinitialized
	RJMP	TWI_vect_RESETWAIT					; while TWSTO set
TWI_vect_STOP:									; STOP condition
	CLR_i2cflags								; clear i2cflags status register
	RJMP	TWI_vect_ACK						; jump to acknowledge

TWI_vect_SRSLAW:								; slave receiver SLA+W received
	SETFLAG_MODEADDR							; DATA is new address
#	if (1 < I2C_PAGES)
	LDS		i2cpage, TWDR						; this is our slave address
	LSR		i2cpage								; shift out R/W bit
	SUBI	i2cpage, (FW_I2CSLA -1)				; decrement slave address to I2C registers page
#	endif // (1 < I2C_PAGES)
	RJMP	TWI_vect_ACK						; jump to ACKnowledge

TWI_vect_SRADDR:								; Slave Receiver ADDRess received
	SSFLAG_MODEADDR								; DATA is not new address
	RJMP	TWI_vect_SRDATA						; jump to DATA receive
	;		DATA==address
	LDS		i2caddr, TWDR						; load address from TWI buffer
	CLRFLAG_MODEADDR							; clear receive address flag
	RJMP	TWI_vect_ACK						; jump to ACKnowledge

TWI_vect_STSLAR:								; Slave Transmitter SLA+R acknowledged
#	if (1 < I2C_PAGES)
	LDS		i2cpage, TWDR						; this is our slave address
	LSR		i2cpage								; shift out R/W bit
	SUBI	i2cpage, (FW_I2CSLA -1)				; decrement slave address to match I2C registers page
#	endif // (1 < I2C_PAGES)
	;RJMP	TWI_vect_ACK						; jump to ACKnowledge

TWI_vect_STDATA:								; Slave Transmitter data acknowledged
	CPI		i2caddr, REG_WAI					; Who-Am-I register address
	BREQ	TWI_vect_STWAI
#	if (1 < I2C_PAGES)
		CPI		i2cpage, hi8(registers)			; everything not on first page is valid
		BRNE	TWI_vect_STREG					; register DATA
#	endif // (1 < I2C_PAGES)
	CPI		i2caddr, REG_KEYS					; joystick status register address
	BREQ	TWI_vect_STKEYS
#	if defined(I2C_VALIDATE_ADDRESS)
		CPI		i2caddr, (8*8*3)				; pixels buffer size
		BRLO	TWI_vect_STREG					; send register DATA
		MOV		PARAML, i2caddr					; load register address
		ANDI	PARAML, 0xF8					; mask out lower 3 bit
		CPI		PARAML, REG_WAI					; administrative registers
		BREQ	TWI_vect_STREG					; send register DATA
		ANDI	PARAML, 0xF0					; mask out lower nibble
#		if !defined(NDEBUG) || defined(USE_REGWRITE)
			CPI		PARAML, REG_PINA			; port registers
			BREQ	TWI_vect_STREG				; send register DATA
#		endif // !defined(NDEBUG) || defined(USE_REGWRITE)
#		if defined(USE_LEDREAD) || defined(USE_LEDWRITE)
			CPI		PARAML, REG_LED_CONF		; LED_CONF registers
			BREQ	TWI_vect_STREG				; transmit register DATA
#		endif // defined(USE_LEDREAD) || defined(USE_LEDWRITE)
		RJMP	TWI_vect_STINV					; jump to INVALID data
#	endif // defined(I2C_VALIDATE_ADDRESS)
TWI_vect_STREG:									; Slave Transmitter send register data
	;		load data from registers buffer
	LD		PARAMH, Y							; load DATA from registers buffer
	STS		TWDR, PARAMH						; store DATA to TWI buffer
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	STS		TWCR, PARAML						; store flags to control register and clear interrupt flag
	INC		YL									; increment i2caddr without i2cpage rollover
	RJMP	TWI_vect_RETI						; jump to return

TWI_vect_STINV:									; Slave Transmitter send INVALID data
	STS		TWDR, r1							; copy __zero_reg__ to TWI transmit buffer
	RJMP	TWI_vect_NACK						; Not ACKnowledge

TWI_vect_STWAI:									; Slave Transmitter Who-Am-I
#	if (1 < I2C_PAGES)
		LDI		PARAMH, (FW_I2CID -1)			; load firmware ID
		ADD		PARAMH, i2cpage					; add current i2cpage
#	else // (1 < I2C_PAGES)
		LDI		PARAMH, FW_I2CID				; load firmware ID
#	endif // (1 < I2C_PAGES)
	STS		TWDR, PARAMH						; store data to buffer
	RJMP	TWI_vect_ACK						; acknowledge

TWI_vect_STKEYS:								; joystick status register address
	CLR_KEYS_INT								; lower KEYS_INT interrupt flag to Raspberry
	RJMP	TWI_vect_STREG						; jump to transmit register data

TWI_vect_NACK:
	LDI		PARAML, _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	RJMP	TWI_vect_TWSR						; store PARAML to TWSR and return
TWI_vect_ACK:
	;		prepare TWI flags
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
TWI_vect_TWSR:
	STS		TWCR, PARAML						; store flags to status register and clear interrupt flag
TWI_vect_RETI:
	;		restore registers after ISR
	POP		PARAML								; restore register
	POP		PARAMH								; restore register
	POP		TEMPREG								; restore register SREG
	OUT		_SFR_IO_ADDR(SREG), TEMPREG			; restore register SREG
	POP		TEMPREG								; restore register __temp_reg__
	;POP		ZEROREG								; restore register __zero_reg__
	RETI										; return and enable interrupts

TWI_vect_SRDATA:								; Slave Receiver DATA received
	;		DATA
	LDS		PARAMH, TWDR						; load data from TWI buffer
	;		check address
	CPI		i2caddr, REG_WAI					; Who-Am-I register
	BREQ	TWI_vect_SRWAI
#	if (1 < I2C_PAGES)
		CPI		i2cpage, hi8(registers)			; i2cpage -0 - everything not on first page is valid
		BRNE	TWI_vect_SRREG					; NOT ZERO - page>0 register DATA received
#	endif // (1 < I2C_PAGES)
	CPI		i2caddr, REG_EE_WP					; EEPROM write protection
	BREQ	TWI_vect_SREEWP
#	if defined(I2C_VALIDATE_ADDRESS)
		CPI		i2caddr, (8*8*3)				; pixels buffer size
		BRLO	TWI_vect_SRREG					; register DATA received
		CPI		i2caddr, REG_KEYS				; joystick status register address
		BREQ	TWI_vect_NACK					; invalid register address
		CPI		i2caddr, REG_VER				; firmware version
		BREQ	TWI_vect_NACK					; invalid register address
		MOV		PARAML, i2caddr					; load register address
#		if defined(USE_REGWRITE)
			ANDI	PARAML, 0xFC				; mask out lower 2 bit
			CPI		PARAML, REG_WDTCSR			; WDTCSR..TOV0C CPU register
			BREQ	TWI_vect_SRREG				; register DATA received
#		endif // defined(USE_REGWRITE)
#		if defined(USE_LEDWRITE)
			ANDI	PARAML, 0xF8				; mask out lower 3 bit
			CPI		PARAML, REG_LED_CONF		; LED_CONF registers
			BREQ	TWI_vect_SRREG				; register DATA received
#		endif // defined(USE_LEDWRITE)
#		if defined(USE_REGWRITE)
			ANDI	PARAML, 0xF0				; mask out lower nibble
			CPI		PARAML, REG_PINA			; port registers
			BREQ	TWI_vect_SRREG				; register DATA received
#		endif // defined(USE_REGWRITE)
		RJMP	TWI_vect_NACK					; invalid register address
#	endif // defined(I2C_VALIDATE_ADDRESS)
TWI_vect_SRREG:
	;		prepare TWI flags and store to TWCR
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	STS		TWCR, PARAML						; store flags to control register and clear interrupt flag
	;		write DATA to buffer
	ST		Y, PARAMH							; store DATA to registers buffer
	INC		YL									; increment i2caddr without i2cpage rollover
	SETFLAG_VALIDREG							; new register data to handle
	RJMP	TWI_vect_RETI						; jump to return

TWI_vect_SRWAI:
	CPI		PARAMH, 0x42						; software RESET marker
	BRNE	TWI_vect_NACK						; jump to NACK
	;		prepare TWI flags and store to TWCR
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	STS		TWCR, PARAML						; store flags to control register and clear interrupt flag
	RJMP	SYSTEM_RESET						; execute RESET

TWI_vect_SREEWP:
	CPI		PARAMH, 0x01						; EEPROM write protection disable marker
	BRNE	TWI_vect_SREEWP0
	CLR_EE_WP									; disable EEPROM write protection
	RJMP	TWI_vect_ACK						; jump to ACKnowledge
TWI_vect_SREEWP0:
	SET_EE_WP									; enable EEPROM write protection
	RJMP	TWI_vect_ACK						; jump to ACKnowledge
.endfunc
#endif	// !defined(TWI_VECTOR_M) && !defined(TWI_VECTOR_S)

#if defined(USE_REGWRITE)
.func	write_registers
;		no parameters
;		no return value
write_registers:								; write new register value to given register
	SFS_i2cflag(I2C_validreg)					; skip if validreg flag is set
	RET											; return if validreg flag cleared
	CLR_i2cflag(I2C_validreg)					; clear validreg flag
	MOVW	XL, YL								; copy pointer value
	DEC		XL									; decrement pointer to last address
	LD		TEMPREG, X							; load register buffer DATA
	CPI		XL, REG_WDTCSR
	BREQ	write_registers_WDTCSR
	CPI		XL, REG_SMCR
	BREQ	write_registers_SMCR
	CPI		XL, REG_PINA
	BREQ	write_registers_PINA
	CPI		XL, REG_DDRA
	BREQ	write_registers_DDRA
	CPI		XL, REG_PORTA
	BREQ	write_registers_PORTA
	CPI		XL, REG_PINB
	BREQ	write_registers_PINB
	CPI		XL, REG_DDRB
	BREQ	write_registers_DDRB
	CPI		XL, REG_PORTB
	BREQ	write_registers_PORTB
	CPI		XL, REG_PINC
	BREQ	write_registers_PINC
	CPI		XL, REG_DDRC
	BREQ	write_registers_DDRC
	CPI		XL, REG_PORTC
	BREQ	write_registers_PORTC
	CPI		XL, REG_PIND
	BREQ	write_registers_PIND
	CPI		XL, REG_DDRD
	BREQ	write_registers_DDRD
	CPI		XL, REG_PORTD
	BREQ	write_registers_PORTD
	CPI		XL, REG_MCUSR
	BREQ	write_registers_MCUSR
	CPI		XL, REG_MCUCR
	BREQ	write_registers_MCUCR
	CPI		XL, REG_PORTCR
	BREQ	write_registers_PORTCR
	CPI		XL, REG_PRR
	BREQ	write_registers_PRR
	RET											; return if register not in list
write_registers_WDTCSR:
	LDI		PARAML, _BV(WDCE) | _BV(WDE)		; enable watchdog change
	STS		WDTCSR, PARAML						; store DATA to register
	STS		WDTCSR, TEMPREG						; store DATA to register
	RET											; return
write_registers_SMCR:
	OUT		_SFR_IO_ADDR(SMCR), TEMPREG			; store DATA to register
	RET											; return
write_registers_PINA:
	OUT		_SFR_IO_ADDR(PINA), TEMPREG			; store DATA to register
	RET											; return
write_registers_DDRA:
	OUT		_SFR_IO_ADDR(DDRA), TEMPREG			; store DATA to register
	RET											; return
write_registers_PORTA:
	OUT		_SFR_IO_ADDR(PORTA), TEMPREG		; store DATA to register
	RET											; return
write_registers_PINB:
	OUT		_SFR_IO_ADDR(PINB), TEMPREG			; store DATA to register
	RET											; return
write_registers_DDRB:
	OUT		_SFR_IO_ADDR(DDRB), TEMPREG			; store DATA to register
	RET											; return
write_registers_PORTB:
	OUT		_SFR_IO_ADDR(PORTB), TEMPREG		; store DATA to register
	RET											; return
write_registers_PINC:
	OUT		_SFR_IO_ADDR(PINC), TEMPREG			; store DATA to register
	RET											; return
write_registers_DDRC:
	OUT		_SFR_IO_ADDR(DDRC), TEMPREG			; store DATA to register
	RET											; return
write_registers_PORTC:
	OUT		_SFR_IO_ADDR(PORTC), TEMPREG		; store DATA to register
	RET											; return
write_registers_PIND:
	OUT		_SFR_IO_ADDR(PIND), TEMPREG			; store DATA to register
	RET											; return
write_registers_DDRD:
	OUT		_SFR_IO_ADDR(DDRD), TEMPREG			; store DATA to register
	RET											; return
write_registers_PORTD:
	OUT		_SFR_IO_ADDR(PORTD), TEMPREG		; store DATA to register
	RET											; return
write_registers_MCUSR:
	OUT		_SFR_IO_ADDR(MCUSR), TEMPREG		; store DATA to register
	RET											; return
write_registers_MCUCR:
	OUT		_SFR_IO_ADDR(MCUCR), TEMPREG		; store DATA to register
	RET											; return
write_registers_PORTCR:
	OUT		_SFR_IO_ADDR(PORTCR), TEMPREG		; store DATA to register
	RET											; return
write_registers_PRR:
	STS		PRR, TEMPREG						; store DATA to register
	RET											; return
.endfunc
#endif // defined(USE_REGWRITE)
