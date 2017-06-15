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
	;PUSH	r1									; save register __zero_reg__
	PUSH	r0									; save register __temp_reg__
	IN		r0, _SFR_IO_ADDR(SREG)				; save register SREG
	PUSH	r0									; save register SREG
	PUSH	PARAMH								; save register
	PUSH	PARAML								; save register
	;		prepare and get TW_STATUS
	SETFLAG_INPROGRESS							; set TWI/I2C in progress flag
	LDS		PARAMH, TWSR						; load status register
	ANDI	PARAMH, TW_STATUS_MASK				; mask out pre scaler bits
	BRBS	SREG_Z, TWI_vect_RESET				; TW_BUS_ERROR
	;		check TW_STATUS
	CPI		PARAMH, TW_ST_DATA_ACK				; slave transmitter data ACKnowledged
	BREQ	TWI_vect_STDATA
	CPI		PARAMH, TW_ST_SLA_ACK				; slave transmitter SLA+R received
	BREQ	TWI_vect_STDATA
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
	;CPI		PARAMH, TW_BUS_ERROR				; I2C bus error
	;BREQ	TWI_vect_RESET
	// I2C spec: "If two or more masters try to put information onto the bus, the first to produce a ‘one’ when the other produces a ‘zero’ will lose the arbitration"
	CPI		PARAMH, TW_SR_ARB_LOST_SLA_ACK		; slave receiver arbitration lost in SLA+W
	BREQ	TWI_vect_RESET
	CPI		PARAMH, TW_ST_ARB_LOST_SLA_ACK		; slave transmitter arbitration lost in SLA+R
	BREQ	TWI_vect_RESET

#	if defined(OPTIMIZE_USENACKFLAG)
		//SETFLAG_NACK								; set NACK flag
#	endif // defined(OPTIMIZE_USENACKFLAG)
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
	RJMP	TWI_vect_ACK						; jump to ACKnowledge

TWI_vect_SRADDR:								; Slave Receiver ADDRess received
	SSFLAG_MODEADDR								; DATA is not new address
	RJMP	TWI_vect_SRDATA						; jump to DATA receive
	;		DATA==address
	LDS		i2caddr, TWDR						; load address from TWI buffer
	DEC		i2caddr								; decrement address, always incremented before access
	CLRFLAG_MODEADDR							; clear receive address flag
	RJMP	TWI_vect_ACK						; jump to ACKnowledge

TWI_vect_STDATA:								; Slave Transmitter data acknowledged
	INC		i2caddr								; increment address
	MOV		PARAML, i2caddr						; load i2caddr to register
	CPI		PARAML, REG_WAI						; Who-Am-I register address
	BREQ	TWI_vect_STWAI
#	if (1 < I2C_PAGES)
		CP		i2cpage, ZEROREG				; everything not on first page is valid
		BRNE	TWI_vect_STREG					; register DATA
#	endif // (1 < I2C_PAGES)
	CPI		PARAML, REG_KEYS					; joystick status register address
	BREQ	TWI_vect_STKEYS
#	if defined(I2C_VALIDATE_ADDRESS)
		CPI		PARAML, (8*8*3)					; pixels buffer size
		BRLO	TWI_vect_STREG					; send register DATA
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
#	if defined(TWI_DATA_RAMPY)
	LD		PARAMH, Y							; load DATA from registers buffer
	STS		TWDR, PARAMH						; store DATA to TWI buffer
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	STS		TWCR, PARAML						; store flags to control register and clear interrupt flag
#	else // defined(TWI_DATA_RAMPY)
	PUSH	XH									; save RAM pointer register
	PUSH	XL									; save RAM pointer register
	LDI		XL, lo8(registers)					; load buffer address
	LDI		XH, hi8(registers)					; load buffer address
	ADD		XL, i2caddr							; add I2C register address to RAMPX
	ADC		XH, i2cpage							; add I2C register page to RAMPX
	LD		PARAMH, X							; load DATA from buffer
	;		load DATA to TWDR, prepare TWI flags and store to TWCR
	STS		TWDR, PARAMH						; store DATA to TWI buffer
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	STS		TWCR, PARAML						; store flags to control register and clear interrupt flag
	POP		XL									; restore RAM pointer register
	POP		XH									; restore RAM pointer register
#	endif // defined(TWI_DATA_RAMPY)
	RJMP	TWI_vect_RETI						; jump to return

TWI_vect_STINV:									; Slave Transmitter send INVALID data
	LDI		PARAMH, 0x00						; invalid
	STS		TWDR, PARAMH						; store DATA to TWI buffer
	RJMP	TWI_vect_NACK						; Not ACKnowledge

TWI_vect_STWAI:									; Slave Transmitter Who-Am-I
	LDI		PARAMH, FW_I2CID					; load firmware ID
#	if (1 < I2C_PAGES)
		ADD		PARAMH, i2cpage					; add current i2cpage
#	endif // (1 < I2C_PAGES)
	STS		TWDR, PARAMH						; store data to buffer
	RJMP	TWI_vect_ACK						; acknowledge

TWI_vect_STKEYS:								; joystick status register address
	CLR_KEYS_INT								; lower KEYS_INT interrupt flag to Raspberry
	RJMP	TWI_vect_STREG						; jump to transmit register data

TWI_vect_NACK:
#if defined(OPTIMIZE_USENACKFLAG)
	SETFLAG_NACK								; set NACK flag
#else // defined(OPTIMIZE_USENACKFLAG)
	CLRFLAG_NACK								; clear NACK flag
	LDI		PARAML, _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	RJMP	TWI_vect_TWSR						; store PARAML to TWSR and return
#endif // defined(OPTIMIZE_USENACKFLAG)

TWI_vect_ACK:
	;		prepare TWI flags
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
#	if defined(OPTIMIZE_USENACKFLAG)
		SCFLAG_NACK								; skip if NACK flag cleared
		ANDI	PARAML, ~_BV(TWEA)				; clear automatic enable flag
		CLRFLAG_NACK							; clear NACK flag
#	endif // defined(OPTIMIZE_USENACKFLAG)
TWI_vect_TWSR:
	STS		TWCR, PARAML						; store flags to status register and clear interrupt flag
TWI_vect_RETI:
	;		restore registers after ISR
	POP		PARAML								; restore register
	POP		PARAMH								; restore register
	POP		r0									; restore register SREG
	OUT		_SFR_IO_ADDR(SREG), r0				; restore register SREG
	POP		r0									; restore register __temp_reg__
	;POP		r1									; restore register __zero_reg__
	RETI										; return and enable interrupts

TWI_vect_SRDATA:								; Slave Receiver DATA received
	;		DATA
	LDS		PARAMH, TWDR						; load data from TWI buffer
	INC		i2caddr								; increment register address
	MOV		PARAML, i2caddr						; load register address
	;		check address
	CPI		PARAML, REG_WAI						; Who-Am-I register
	BREQ	TWI_vect_SRWAI
#	if (1 < I2C_PAGES)
		CP		i2cpage, ZEROREG				; i2cpage -0 - everything not on first page is valid
		BRNE	TWI_vect_SRREG					; NOT ZERO - page>0 register DATA received
#	endif // (1 < I2C_PAGES)
	CPI		PARAML, REG_EE_WP					; EEPROM write protection
	BREQ	TWI_vect_SREEWP
#	if defined(I2C_VALIDATE_ADDRESS)
		CPI		PARAML, (8*8*3)					; pixels buffer size
		BRLO	TWI_vect_SRREG					; register DATA received
		CPI		PARAML, REG_KEYS				; joystick status register address
		BREQ	TWI_vect_NACK					; invalid register address
		CPI		PARAML, REG_VER					; firmware version
		BREQ	TWI_vect_NACK					; invalid register address
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
#	if defined(TWI_DATA_RAMPY)
	ST		Y, PARAMH							; store DATA to registers buffer
#	else // defined(TWI_DATA_RAMPY)
	PUSH	XH									; save RAM pointer register
	PUSH	XL									; save RAM pointer register
	LDI		XL, lo8(registers)					; load buffer address
	LDI		XH, hi8(registers)					; load buffer address
	ADD		XL, i2caddr							; add register address to buffer address
	ADC		XH, i2cpage							; add I2C page and CARRY flag to high byte of address
	ST		X, PARAMH							; store DATA to buffer
	POP		XL									; restore RAM pointer register
	POP		XH									; restore RAM pointer register
#	endif // defined(TWI_DATA_RAMPY)
	SETFLAG_VALIDREG							; new register data to handle
	RJMP	TWI_vect_RETI						; jump to return

TWI_vect_SRWAI:
	CPI		PARAMH, 0x42						; software RESET marker
#	if (1 < I2C_PAGES)
		BRNE	TWI_vect_SRPAGE					; check for I2C page change
#	else // (1 < I2C_PAGES)
		BRNE	TWI_vect_NACK					; jump to NACK
#	endif // (1 < I2C_PAGES)
	;		prepare TWI flags and store to TWCR
	LDI		PARAML, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWINT)
	STS		TWCR, PARAML						; store flags to control register and clear interrupt flag
	RJMP	SYSTEM_RESET						; execute RESET
TWI_vect_SRPAGE:
#	if (1 < I2C_PAGES)
		MOV		PARAML, PARAMH					; copy DATA
		ANDI	PARAML, ~(I2C_MAXPAGE)			; mask out page number
		CPI		PARAML, 0xA0					; change page marker
		BRNE	TWI_vect_NACK					; jump to NACK
		ANDI	PARAMH, I2C_MAXPAGE				; mask maximum register pages
		MOV		i2cpage, PARAMH					; load page address
		RJMP	TWI_vect_ACK					; jump to ACKnowledge
#	else // (1 < I2C_PAGES)
		RJMP	TWI_vect_NACK					; jump to NACK
#	endif // (1 < I2C_PAGES)

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

#if defined(USE_REGWRITE) && defined(TWI_DATA_RAMPY)
.func	write_registers
;		no parameters
;		no return value
write_registers:								; write new register value to given register
	SFS_i2cflag(I2C_validreg)					; skip if validreg flag is set
	RET											; return if validreg flag cleared
	CLR_i2cflag(I2C_validreg)					; clear validreg flag
	PUSH	ZH									; save register
	PUSH	ZL									; save register
	MOVW	ZL, YL								; copy RAMPY to RAMPZ
	LD		TEMPREG, Z							; load register buffer DATA
	POP		ZL									; restore register
	POP		ZH									; restore register
	CPI		i2caddr, REG_WDTCSR
	BREQ	write_registers_WDTCSR
	CPI		i2caddr, REG_SMCR
	BREQ	write_registers_SMCR
	CPI		i2caddr, REG_PINA
	BREQ	write_registers_PINA
	CPI		i2caddr, REG_DDRA
	BREQ	write_registers_DDRA
	CPI		i2caddr, REG_PORTA
	BREQ	write_registers_PORTA
	CPI		i2caddr, REG_PINB
	BREQ	write_registers_PINB
	CPI		i2caddr, REG_DDRB
	BREQ	write_registers_DDRB
	CPI		i2caddr, REG_PORTB
	BREQ	write_registers_PORTB
	CPI		i2caddr, REG_PINC
	BREQ	write_registers_PINC
	CPI		i2caddr, REG_DDRC
	BREQ	write_registers_DDRC
	CPI		i2caddr, REG_PORTC
	BREQ	write_registers_PORTC
	CPI		i2caddr, REG_PIND
	BREQ	write_registers_PIND
	CPI		i2caddr, REG_DDRD
	BREQ	write_registers_DDRD
	CPI		i2caddr, REG_PORTD
	BREQ	write_registers_PORTD
	CPI		i2caddr, REG_MCUSR
	BREQ	write_registers_MCUSR
	CPI		i2caddr, REG_MCUCR
	BREQ	write_registers_MCUCR
	CPI		i2caddr, REG_PORTCR
	BREQ	write_registers_PORTCR
	CPI		i2caddr, REG_PRR
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
#endif // defined(USE_REGWRITE) && defined(TWI_DATA_RAMPY)
