/*
 * rpi-sense.S - Atmel assembly routines for LED framebuffer scan-out
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

PARAM	= 24
LINE	= 24
PWM		= 25

.macro CLOCK_OUT
	SBI	_SFR_IO_ADDR(PORTC), LED_CLKR
	CBI	_SFR_IO_ADDR(PORTC), LED_CLKR
.endm

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
	LD		TEMPREG, Y							; load register buffer DATA
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
	STS		SMCR, TEMPREG						; store DATA to register
	RET											; return
write_registers_PINA:
	STS		PINA, TEMPREG						; store DATA to register
	RET											; return
write_registers_DDRA:
	STS		DDRA, TEMPREG						; store DATA to register
	RET											; return
write_registers_PORTA:
	STS		PORTA, TEMPREG						; store DATA to register
	RET											; return
write_registers_PINB:
	STS		PINB, TEMPREG						; store DATA to register
	RET											; return
write_registers_DDRB:
	STS		DDRB, TEMPREG						; store DATA to register
	RET											; return
write_registers_PORTB:
	STS		PORTB, TEMPREG						; store DATA to register
	RET											; return
write_registers_PINC:
	STS		PINC, TEMPREG						; store DATA to register
	RET											; return
write_registers_DDRC:
	STS		DDRC, TEMPREG						; store DATA to register
	RET											; return
write_registers_PORTC:
	STS		PORTC, TEMPREG						; store DATA to register
	RET											; return
write_registers_PIND:
	STS		PIND, TEMPREG						; store DATA to register
	RET											; return
write_registers_DDRD:
	STS		DDRD, TEMPREG						; store DATA to register
	RET											; return
write_registers_PORTD:
	STS		PORTD, TEMPREG						; store DATA to register
	RET											; return
write_registers_MCUSR:
	STS		MCUSR, TEMPREG						; store DATA to register
	RET											; return
write_registers_MCUCR:
	STS		MCUCR, TEMPREG						; store DATA to register
	RET											; return
write_registers_PORTCR:
	STS		PORTCR, TEMPREG						; store DATA to register
	RET											; return
write_registers_PRR:
	STS		PRR, TEMPREG						; store DATA to register
	RET											; return
.endfunc
#endif // defined(USE_REGWRITE) && defined(TWI_DATA_RAMPY)

.func	SYSTEM_RESET							; execute SYSTEM_RESET procedure
;	no parameters
SYSTEM_RESET:
	CLI											; stop global interrupts
	CLR		ZEROREG								; __zero_reg__ =0

	LDI		PARAML, 0xFF						; PARAML =0xFF
	STS		PRR, PARAML							; disable I/O systems for power reduction
	STS		EIFR, PARAML						; clear interrupt flags
	STS		EIMSK, ZEROREG						; disable interrupts
	STS		PCIFR, PARAML						; clear interrupt flags
	STS		PCICR, ZEROREG						; disable interrupts
	/*		no need to clear PCMSKx if PCICR==0
	STS		PCMSK0, ZEROREG						; disable interrupts
	STS		PCMSK1, ZEROREG						; disable interrupts
	STS		PCMSK2, ZEROREG						; disable interrupts
	STS		PCMSK3, ZEROREG						; disable interrupts
	*/
	STS		TIFR0, PARAML						; clear interrupt flags
	STS		TIMSK0, ZEROREG						; disable TIMER0 interrupts
	STS		TIFR1, PARAML						; clear interrupt flags
	STS		TIMSK1, ZEROREG						; disable TIMER1 interrupts

	LDS		PARAML, SPSR						; read SPSR, SPDR to clear interrupt flag
	LDS		PARAML, SPDR						; read SPSR, SPDR to clear interrupt flag
	STS		SPCR, ZEROREG						; disable SPI system and interrupt

	LDI		PARAML, _BV(TWINT) | _BV(TWSTO)		; clear TWI interrupt flag and issue STOP condition
	STS		TWCR, PARAML						; clear interrupt flag
	STS		TWCR, ZEROREG						; disable TWI system and interrupt

	STS		ADCSRB, ZEROREG						; disable analog multiplexer
	LDI		PARAML, _BV(ACI)					; clear analog comparator interrupt flag
	STS		ACSR, PARAML
	LDI		PARAML, _BV(ACD)					; set analog comparator disable flag
	STS		ACSR, PARAML

	LDI		PARAML, _BV(ADIF)					; clear ADC interrupt flag
	STS		ADCSRA, PARAML
	STS		ADCSRA, ZEROREG						; disable ADC

	STS		EECR, ZEROREG						; disable EEPROM Programming

	LDI		PARAML, _BV(SM1) | _BV(SE)			; enable SLEEP mode to power down
	STS		SMCR, PARAML						; write SMCR
	LDI		PARAML, _BV(WDIF)					; clear interrupt flag
	STS		WDTCSR, PARAML						; clear interrupt flag
	LDI		PARAML, _BV(WDCE) | _BV(WDE) | watchdog_prescalerbits(6)	; enable change WDTCSR flags
	STS		WDTCSR, PARAML						; first write enables change
	STS		WDTCSR, PARAML						; second write enables watchdog reset, pre-scaler==6 (t==1s)

	SEI											; enable global interrupts
SYSTEM_RESET_SLEEP:
	SLEEP										; send CPU to SLEEP, till watchdog reset occures
	RJMP	SYSTEM_RESET_SLEEP					; loop until watchdog resets system
	; point of no return - code execution should never reach beyond
	RJMP	SYSTEM_RESET_SLEEP					; loop until watchdog resets system
.endfunc

.func	draw_loop
draw_loop:
	;		save registers
	PUSH	XH								; r27, XH pointer
	PUSH	XL								; r26, XL pointer
	PUSH	PWM								; r25, PWM
	PUSH	LINE							; r24, LINE, PARAM
	PUSH	r21								; bits to transmit
	PUSH	r20								; pixels register
	;		prepare
	CBI		_SFR_IO_ADDR(PORTC), FRAME_INT	; clear FRAME_INT interrupt flag
	LDI		XL, lo8(pixels)					; load memory address of pixels buffer to Y register
	LDI		XH, hi8(pixels)
	LDI		LINE, 0xFE						; LINE=0xFE bit 0 cleared

draw_loop_line:									; 8 run loop send line to LED2472G
	OUT		_SFR_IO_ADDR(PORTD), LINE		; set COLUMN_ENABLEx low
	LDI		PWM, 0							; PWM=0

draw_loop_col:								; set bits to transmit counter
	LDI		r21, 24							; number of bits 8R+8G+8B to transmit
draw_loop_bit:								; 24 run loop send bits to LED2472G
	LD		r20, X+							; load pixels register
	CP		PWM, r20
	BRSH	draw_loop_SDI0					; PWM >= r20
	SBI		_SFR_IO_ADDR(PORTC), LED_SDI	; set SDI active-high
	RJMP	draw_loop_SDI1					; PWM < r20
draw_loop_SDI0:
	CBI		_SFR_IO_ADDR(PORTC), LED_SDI	; set SDI active-low
draw_loop_SDI1:
	CPI		r21, 1
	BRNE	draw_loop_LE0					; r21 != 1
	SBI		_SFR_IO_ADDR(PORTC), LED_LE		; raise LED_LE with bit1 transmission
draw_loop_LE0:
	CLOCK_OUT								; SCK clock pulse
	DEC		r21								; r21--
	BRNE	draw_loop_bit					; r21!=0 next register, bit

	; 24bits transmitted
	CBI		_SFR_IO_ADDR(PORTC), LED_LE		; lower LED_LE, after transmit 24bit cycle finished
	INC		PWM								; PWM++
	SBRC	PWM, 6							; skip if PWM < 64
	RJMP	draw_loop_nextline				; line loop ended
	SBIW	YL, 24							; YH:YL -= 24 back to address before 24 bit loop runs
	//SUBI	YL, 24							; YL -= 24
	//SBCI	YH, 0							; YH -= CARRY
	RJMP	draw_loop_col					; next LINE

draw_loop_nextline:							; line loop ended
	SEC		; set CARRY flag and clear if bit7==0 to be sure
	ROL		LINE							; shift LINE with CARRY bit - C=7,7=6,6=5,5=4,4=3,3=2,2=1,1=0,0=C
	BRCS	draw_loop_line					; branch if CARRY=1 - next line output

	SBI		_SFR_IO_ADDR(PORTB), FRAME_INT	; raise FRAME_INT interrupt to Pi
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; /Scan

	; restore registers
	POP		r20
	POP		r21
	POP		LINE
	POP		PWM
	POP		XL								; r26, XL pointer
	POP		XH								; r27, XH pointer
	; continue with joystick reading
	RJMP	check_keys
.endfunc

.func	check_keys
check_keys:
	PUSH	PWM								; save register
	PUSH	PARAM							; save register
	SBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; Scan
	LDI		PARAM, 0xFF
	OUT		_SFR_IO_ADDR(DDRD), PARAM		; switch PORTD to output
	OUT		_SFR_IO_ADDR(PORTD), PARAM		; switch PORTD high
	LDI		PARAM, 0x07
	OUT		_SFR_IO_ADDR(DDRD), PARAM		; switch bit 3-7 to input for joystick
	LDS		PWM, keys
	IN		PARAM, _SFR_IO_ADDR(PIND)		; read joystick status
	COM		PARAM							; one-complement - change bits to active-high, joystick switches to GND
	LSR		PARAM							; shift bits to the right
	LSR		PARAM							; shift bits to the right
	LSR		PARAM							; shift bits to the right
	STS		keys, PARAM						; keys = (PORTD>>3) 000,C,CTR,A,B,D
	CPSE	PARAM, PWM						; if keys changed (compare, skip if equal)
	SBI		_SFR_IO_ADDR(PORTB), KEYS_INT	; raise KEYS_INT to Pi
	LDI		PARAM, 0xFF
	OUT		_SFR_IO_ADDR(DDRD), PARAM		; change PORTD to output
	COM		PARAM
	OUT		_SFR_IO_ADDR(PORTD), PARAM		; switch PORTD low (activate MOSFET switches)
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; /Scan
	POP		PARAM							; restore register
	POP		PWM								; restore register
	RET		; return control back to main()
.endfunc

.func delay
;	(r25),r24 uint8_t r24/PARAM = delay for clkT0 ticks
;	F_CPU==8MHz, T0 prescaler==5, clkT0=128ys
delay:
	PUSH	r19
	PUSH	r20
	IN		r20, _SFR_IO_ADDR(TCNT0)			; start = TCNT0;
lloop:
	IN		r19, _SFR_IO_ADDR(TCNT0)			; do {
	SUB		r19, r20							; diff = TCNT0 - start;
	CP		r19, PARAM							; } while (diff < ticks);
	BRLO	lloop
	POP		r20
	POP		r19
	RET
 .endfunc

.func write_data
;	Function arguments are allocated left to right.
;	They are assigned from r25 to r8, respectively.
;	All arguments take up a even number of registers
;	(r25),r24,r23,r22 uint32_t DATA
;	(r21),r20 uint8_t TYPE
write_data:
	; save registers - Registers r18-r27 and r30-r31 can be used in a function without saving
	;PUSH	r25								; unused DATA 24-31
	;PUSH	r24								; DATA 16-23
	;PUSH	r23								; DATA 8-15
	;PUSH	r22								; DATA 0-7
	;PUSH	r21								; unused - copy of TYPE for loop bit counting
	;PUSH	r20								; TYPE
	;PUSH	r0								; __temp_reg__
	LDI		r21, 24							; number of bits to transmit
write_data1:								; do {
	SBRS	r24, 7							; 	if (data&1)
	RJMP	write_data2
	SBI		_SFR_IO_ADDR(PORTC), LED_SDI	; 		set LED_SDI;
	RJMP	write_data3
write_data2:								; 	else
	CBI		_SFR_IO_ADDR(PORTC), LED_SDI	;		clr LED_SDI;
write_data3:
	//	r25 rotation unneeded, because value is unused
	CLC										; clear CARRY bit
	SBRC	r24, 7							; skip if bit23 cleared
	SEC										; set CARRY bit if LED_SDO is active high
	ROL		r22								; DATA<<=1; CARRY=bit7 , bit  6-0 , CARRY=bit23
	ROL		r23								; DATA<<=1; CARRY=bit15, bit 14-8 , CARRY=bit7
	ROL		r24								; DATA<<=1; CARRY=bit23, bit 22-16, CARRY=bit15
	CP		r20, r21						; 	if (type == i)
	BRLO	write_data4
	SBI		_SFR_IO_ADDR(PORTC), LED_LE		; 		set LDE_LE;
write_data4:
	CLOCK_OUT								; clock pulse out to LED2472G
	DEC		r21								; 	i--;
	BRNE	write_data1						; } while (i!=0);
	CBI		_SFR_IO_ADDR(PORTC), LED_LE		; 		clr LDE_LE
	CBI		_SFR_IO_ADDR(PORTC), LED_SDI	;		clr LED_SDI;
	; restore registers
	;POP		r0								; restore __temp_reg__
	;POP		r20
	;POP		r21								; unused
	;POP		r22
	;POP		r23
	;POP		r24
	;POP		r25								; unused
	RET
.endfunc

.func read_data
;	8-bit values are returned in r24.
;	16-bit values are returned in r25:r24.
;	32-bit values are returned in r25:r24:r23:r22.
;	64-bit values are returned in r25:r24:r23:r22:r21:r20:r19:r18.
;	(r25),r24 uint8_t TYPE
;	(r25),r24,r23,r22 uint32_t RETURN
read_data:
	; save registers - Registers r18-r27 and r30-r31 can be used in a function without saving
	/*	no need to save, r25::r22 will be return
	PUSH	r25
	PUSH	r24
	PUSH	r23
	PUSH	r22
	*/
	PUSH	r21								; save register for write_data call
	PUSH	r20								; save register for write_data call, later used as bit loop counter
	MOVW	r20, r24						; TYPE parameter for write_data, later used as bit loop counter
	LDI		r22, 0xFF						; 0xFF to activate all outputs
	LDI		r23, 0xFF						; 0xFF to activate all outputs
	LDI		r24, 0xFF						; 0xFF to activate all outputs
	;		skip output activation if not error detection
	CPI		r24, 11							; 11==DET_OPEN, 12=DET_SHORT, 13=DET_OPEN_SHORT
	BRLO	read_data4						; <11 no additional time needed
	CPI		r24, 14							; 14==THERM_READ
	BRSH	read_data4						; >=14 no additional time needed
	;		for error detection all outputs must be set
	PUSH	r20								; save TYPE parameter for later use
	LDI		r20, DAT_LATCH					; switch all outputs on before error detection
	RCALL	write_data						; write_data doesn't change r20 register
	LDI		r20, GAIN_WRITE					; highest gain for error detection
	RCALL	write_data						; write_data doesn't change r20 register
	POP		r20								; restore TYPE parameter
read_data4:
	; first the LE sequence must be sent
	RCALL	write_data						; write_data doesn't change r20 register
	CPI		r20, 11							; 11==DET_OPEN, 12=DET_SHORT, 13=DET_OPEN_SHORT
	BRLO	read_data2						; <11 no additional time needed
	CPI		r20, 14							; 14==THERM_READ
	BRSH	read_data2						; >=14 no additional time needed
	LDI		r25, 0xFF						; 0xFF to switch PORTD to output and active-high
	; now give time to aquire data - if 11 <= r20 < 14
	; error detection r20==11,12,13 needs at least 1ys and all LED columns are enabled
	OUT		_SFR_IO_ADDR(DDRD), r25			; switch PORTD to output
	OUT		_SFR_IO_ADDR(PORTD), r25		; switch PORTD active-high
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; When the ROE, GOE and BOE signals become low, error detection starts.
	; LED_OE_N low needs delay of 1ys = 8 CPU cycles a 125ns
	LSR		_SFR_IO_ADDR(PORTD)				; COL_EN1 to active low
	ROR		_SFR_IO_ADDR(PORTD)				; COL_EN2
	ROR		_SFR_IO_ADDR(PORTD)				; COL_EN3
	ROR		_SFR_IO_ADDR(PORTD)				; COL_EN4
	ROR		_SFR_IO_ADDR(PORTD)				; COL_EN5
	ROR		_SFR_IO_ADDR(PORTD)				; COL_EN6
	ROR		_SFR_IO_ADDR(PORTD)				; COL_EN7
	ROR		_SFR_IO_ADDR(PORTD)				; COL_EN8
	SBI		_SFR_IO_ADDR(PORTC), LED_CLKR	; raise LED_CLKR to make result available
	OUT		_SFR_IO_ADDR(PORTD), r25		; switch PORTD active-high
	SBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; LED_OE_N kept low for at least 1 ys
	; after first CLK pulse data will be made available
read_data2:
	; read data from LED2472G
	LDI		r20, 24							; number of bits to receive
	CBI		_SFR_IO_ADDR(PORTC), LED_SDI	; clr LED_SDI;
read_data3:									; do {
	SBI		_SFR_IO_ADDR(PORTC), LED_CLKR	; raise LED_CLKR
	CLC										; clear CARRY bit
	SBIC	_SFR_IO_ADDR(PINC), LED_SDO		; skip if LED_SDO cleared
	SEC										; set CARRY bit if LED_SDO is active high
	ROL		r22								; DATA<<=1; CARRY=bit8 , bit  7-1 , CARRY=LED_SDO
	ROL		r23								; DATA<<=1; CARRY=bit16, bit 15-9 , CARRY=bit8
	ROL		r24								; DATA<<=1; CARRY=bit24, bit 23-17, CARRY=bit16
	CBI		_SFR_IO_ADDR(PORTC), LED_CLKR	; lower LED_CLKR
	SBIC	_SFR_IO_ADDR(PINC), LED_SDO		; skip if LED_SDO cleared
	ORI		r22, _BV(0)						; set bit0 if LED_SDO is set
	//	r25 rotation unneeded, because value is unused
	DEC		r20								; 	i--;
	BRNE	read_data3						; } while (i!=0);
	CLR		r25								; clear return bit24-31
	POP		r20								; restore registers
	POP		r21								; restore register
	/*	r25:r24:r23:r22 is return value, save/restore would be a big-bad-bug
	POP		r22
	POP		r23
	POP		r24
	POP		r25
	*/
	RET
.endfunc
