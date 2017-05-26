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

#include <avr/io.h>
#include <util/twi.h>

LINE	= 24
PARAM	= 24
PWM		= 25
I2C_OFF	= 26	; XL

; PORTA
.equiv	LSM_INT_M	, PA0
.equiv	LSM_INT1_AG	, PA1
.equiv	LSM_INT2_AG	, PA2
.equiv	DEBUG_TP1	, PA4

; PORTB
.equiv	EE_WP		, PB0
.equiv	DEBUG_TP2	, PB1
.equiv	ICSP_CS		, PB2
.equiv	ICSP_MOSI	, PB3
.equiv	ICSP_MISO	, PB4
.equiv	ICSP_CLK	, PB5
.equiv	KEYS_INT	, PB6
.equiv	FRAME_INT	, PB7

; PORTC
.equiv	LED_SDO		, PC0
.equiv	LED_CLKR	, PC1
.equiv	LED_LE		, PC2
.equiv	LED_SDI		, PC3
.equiv	SDA			, PC4
.equiv	SCL			, PC5
;.equiv	ICSP_RST	, PC6
.equiv	LED_OE_N	, PC7

; PORTD
.equiv	COL_EN8		, PD0
.equiv	COL_EN7		, PD1
.equiv	COL_EN6		, PD2
.equiv	COL_EN5		, PD3
.equiv	COL_EN4		, PD4
.equiv	COL_EN3		, PD5
.equiv	COL_EN2		, PD6
.equiv	COL_EN1		, PD7

.macro CLOCK_OUT
	SBI	_SFR_IO_ADDR(PORTC), LED_CLKR
	CBI	_SFR_IO_ADDR(PORTC), LED_CLKR
.endm

.global draw_loop
.global delay
.global read_data
.global write_data
.global TWI_vect
.global	check_keys
.global	SYSTEM_RESET

.extern	registers
.extern i2c_reg
.extern i2c_busy
.extern redrawleds

.equiv	I2C_PIXELS	, 0x00
.equiv	pixels		, (registers + I2C_PIXELS)
.equiv	I2C_WAI		, 0xf0
.equiv	I2C_ID		, (registers + I2C_WAI)
.equiv	I2C_VER		, 0xf1
.equiv	VERSION		, (registers + I2C_VER)
.equiv	I2C_KEYS	, 0xf2
.equiv	keys		, (registers + I2C_KEYS)
.equiv	I2C_EE_WP	, 0xf3
.equiv	eewp		, (registers + I2C_EE_WP)
.equiv	I2C_WDTCSR	, 0xf4
.equiv	wdt_reg		, (registers + I2C_WDTCSR)
.equiv	I2C_SMCR	, 0xf5
.equiv	sm_reg		, (registers + I2C_SMCR)
.equiv	I2C_WDC		, 0xf6
.equiv	watchdog	, (registers + I2C_WDC)
.equiv	I2C_TOV0C	, 0xf7
.equiv	counttmr	, (registers + I2C_TOV0C)

.section .text

.func	SYSTEM_RESET							; execute SYSTEM_RESET procedure
;	no parameters
SYSTEM_RESET:
	PUSH	r25									; save register
	PUSH	r24									; save register
	PUSH	r1									; save register
	PUSH	r0									; save register
	CLI											; stop global interrupts

	CLR		r0									; r0=0
	CLR		r1
	COM		r1									; r1=0xFF

	STS		PRR, r0								; disable I/O systems for power reduction

	STS		EIFR, r1							; clear interrupt flags
	STS		EIMSK, r0							; disable interrupts
	STS		PCIFR, r1							; clear interrupt flags
	STS		PCICR, r0							; disable interrupts
	/*		no need to clear PCMSKx if PCICR==0
	STS		PCMSK0, r0							; disable interrupts
	STS		PCMSK1, r0							; disable interrupts
	STS		PCMSK2, r0							; disable interrupts
	STS		PCMSK3, r0							; disable interrupts
	*/
	STS		TIFR0, r1							; clear interrupt flags
	STS		TIMSK0, r0							; disable TIMER0 interrupts
	STS		TIFR1, r1							; clear interrupt flags
	STS		TIMSK1, r0							; disable TIMER1 interrupts
	LDS		r24, SPSR							; read SPSR, SPDR to clear interrupt flag
	LDS		r24, SPDR							; read SPSR, SPDR to clear interrupt flag
	STS		SPCR, r0							; disable SPI system and interrupt
	LDI		r24, (1 << TWINT) | (1 << TWSTO)	; clear TWI interrupt flag and issue STOP condition
	STS		TWCR, r24							; clear interrupt flag
	STS		TWCR, r0							; disable TWI system and interrupt
	STS		ADCSRB, r0							; disable analog multiplexer
	LDI		r24, (1 << ACI)						; clear analog comparator interrupt flag
	STS		ACSR, r24
	LDI		r24, (1 << ACD)						; set analog comparator disable flag
	STS		ACSR, r24
	LDI		r24, (1 << ADIF)					; clear ADC interrupt flag
	STS		ADCSRA, r24
	STS		ADCSRA, r0							; disable ADC
	STS		EECR, r0							; disable EEPROM Programming

	LDI		r24, (1 << SM1) | (1 << SE)			; enable SLEEP mode to power down
	STS		SMCR, r24							; write SMCR
	LDI		r25, (1 << WDIF)					; clear interrupt flag
	STS		WDTCSR, r25							; clear interrupt flag
	LDI		r24, (1 << WDCE) | (1 << WDE)		; enable change WDTCSR flags
	STS		WDTCSR, r24							; first write enables change
	STS		WDTCSR, r24							; second write enables watchdog reset, pre-scaler==0 (t==16ms)
	SEI											; enable global interrupts
	SLEEP										; send CPU to SLEEP, till watchdog reset occures

	RJMP	.-2									; loop until watchdog resets system
	; point of no return - code execution should never reach beyond
	POP		r0									; restore register
	POP		r1									; restore register
	POP		r24									; restore register
	POP		r25									; restore register
	RET											; return
.endfunc

;TODO: Add reads and NACK bad addresses.
;PARAM (r24) - TWSR
;PWM (r25) - SREG
;DI2C/XL (r26) - Address
.func	TWI_vect
TWI_vect:	// ISR, global interrupts disabled on entry
	WDR		; reset watchdog timer
	; save registers
	PUSH	PARAM
	PUSH	PWM
	PUSH	YL
	PUSH	YH
	PUSH	I2C_OFF
	IN		PWM, _SFR_IO_ADDR(SREG)
	; set I2C busy flag
	LDS		PARAM, i2c_busy						; load i2c_busy flags
	ORI		PARAM, 1							; i2c_busy |= 1
	STS		i2c_busy, PARAM						; store i2c_busy flags
	LDS		I2C_OFF, i2c_reg					; I2C_OFF holds register address
	; check TWSR status register
	LDS		PARAM, TWSR							; load status register
	ANDI	PARAM, TW_STATUS_MASK				; mask out pre-scaler bits
	CPI		PARAM, TW_ST_DATA_ACK				; data transmitted, ACK received
	BREQ	ltsend								; send DATA to master
	CPI		PARAM, TW_ST_SLA_ACK				; SLA+R received, ACK returned
	BREQ	ltslar								; send current DATA to master
	CPI		PARAM, TW_SR_DATA_ACK				; data received, ACK returned
	BREQ	lreceive							; read DATA from master
	CPI		PARAM, TW_SR_SLA_ACK				; SLA+W received, ACK returned
	BREQ	lrsla								; next DATA is register address
	CPI 	PARAM, TW_SR_STOP					; stop or repeated start condition received while selected
	BREQ	TWI_stop
	CPI		PARAM, TW_ST_DATA_NACK				; data transmitted, NACK received
	BREQ	ltdnack
	CPI 	PARAM, TW_ST_LAST_DATA				; last data byte transmitted, ACK received
	BREQ	TWI_stop
	CPI 	PARAM, TW_SR_DATA_NACK				; data received, NACK returned
	BREQ	TWI_stop

	CPI		PARAM, TW_BUS_ERROR					; illegal start or stop condition
	BREQ	TWI_reset
	CPI		PARAM, TW_NO_INFO					; no state information available
	BREQ	TWI_reset
	// I2C specs: "If two or more masters try to put information onto the bus, the first to produce a ‘one’ when the other produces a ‘zero’ will lose the arbitration"
	;CPI 	PARAM, TW_SR_ARB_LOST_GCALL_ACK			; arbitration lost in SLA+RW, general call received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_ACK					; general call received, ACK returned
	CPI 	PARAM, TW_ST_ARB_LOST_SLA_ACK		; arbitration lost in SLA+RW, SLA+R received, ACK returned
	BREQ	TWI_reset
	CPI 	PARAM, TW_SR_ARB_LOST_SLA_ACK		; arbitration lost in SLA+RW, SLA+W received, ACK returned
	BREQ	TWI_reset

	;CPI 	PARAM, TW_SR_GCALL_DATA_ACK				; general call data received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_DATA_NACK			; general call data received, NACK returned

ldone_NACK:
	; without TWEA set the next transaction will be NACKed
	LDI		PARAM,  (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
ldone:
	STS		TWCR, PARAM							; load PARAM to TWCR
ldone_RET:
	; restore registers
	OUT		_SFR_IO_ADDR(SREG), PWM
	POP		I2C_OFF
	POP		YH
	POP		YL
	POP		PWM
	POP		PARAM
	RETI	; return from ISR and global interrupt enable

lreceive:	; slave receiver DATA received
	LDS		PARAM, i2c_busy						; load i2c_busy flags
	SBRS	PARAM, 1							; skip if DATA is address - i2c_busy.1==1
	RJMP	TWI_write							; i2c_busy.address==0 receive data from master
	RJMP	lraddress							; check and store address received

lrsla:		; SLA+W received, ACK sent
	/*		SLA+W resets i2c_busy to defined value
	LDS		PARAM, i2c_busy						; load i2c_busy flags
	ORI		PARAM, 0b00000010					; set next DATA is address flag
	*/
	LDI		PARAM, 0b00000011					; i2c_busy flags in-progress and next DATA is address
	STS		i2c_busy, PARAM						; store i2c_busy flags
	LDI		PARAM, (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone

ltslar:
	RJMP	TWI_read_chkaddr

ltsend:		; DATA sent
	RJMP	TWI_read							; transmit data to master

ltdnack:	; NACK received
	CLR		PARAM
	STS		i2c_busy, PARAM						; clear i2c_busy flags
	;		just ignore NACK and wait for next transaction - most probably STOP condition
	LDI		PARAM, (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone

TWI_reset:
	LDS		PARAM,	TWCR						; load TWCR register
	SBR		PARAM,	TWSTO						; set TWSTO flag
	STS		TWCR,	PARAM						; This will not generate a STOP condition, but the TWI returns to unaddressed and releases lines
	LDS		PARAM,	TWCR						; load TWCR register
	SBRC	PARAM,	TWSTO						; skip if bit cleared - TWI has been reinitialized
	RJMP	TWI_reset							; while TWSTO set
	CLR		PWM									; clear register PWM=0
	STS		i2c_busy, PWM						; store i2c_busy flags
TWI_stop:
	LDI		PARAM, (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	LDS		PWM, i2c_busy						; load i2c_busy flags
	SBRS	PWM, 2								; skip if invalid address flag
	ORI		PARAM, (1 << TWEA)					; enable acknowledge if not invalid address
	ANDI	PWM, 0xFE							; clear i2c_busy.in-progress flag
	STS		i2c_busy, PWM						; store i2c_busy flags
	RJMP	ldone

lraddress:	; slave receiver DATA==address
	LDI		I2C_OFF, 1							; set i2c_busy flag
	STS		i2c_busy, I2C_OFF					; store i2c_busy=1 flag, following data is real DATA
	LDS		I2C_OFF, TWDR						; load DATA - register address
	STS		i2c_reg, I2C_OFF					; i2c_reg = DATA, register address for coming read transactions
	; now address could be check with available addresses and TWEA disabled if unknown address

#if defined(USE_LEDREAD) || defined(USE_LEDWRITE)
	CPI		I2C_OFF, (8*8*3)+(4*4)				; address < (8*8*3) pixels + (4*4) LED registers
#else
	CPI		I2C_OFF, (8*8*3)					; address<(8*8*3) pixels buffer
#endif
	BRLO	lrsregOK							; address valid

#if	!defined(NDEBUG) || defined(USE_REGWRITE)
	CPI		I2C_OFF, 0xE0						; address < 0xE0 (&& >=192) debug registers
#else

	CPI		I2C_OFF, I2C_WAI					; address < 0xF0 (&& >=192) Who-Am-I
#endif
	BRLO	lrsregNOK							; address invalid

	CPI		I2C_OFF, I2C_TOV0C +1				; address<=0xF7 TOV0 counter
	BRLO	lrsregOK							; address valid

	; other addresses are invalid
lrsregNOK:
	; for real NACK TWEA=0 must be set to disable ACK
	LDI		PARAM, (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone							; return and continue at ldone
lrsregOK:
	LDI		PARAM, (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone							; return and continue at ldone

TWI_NACK:
	LDI		PARAM,  (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	.+2								; skip next instruction
TWI_ACK:
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	STS		TWCR, PARAM						; load PARAM to TWCR and start TWI
TWI_ADDR:
	; increment i2c_reg
	LDS		I2C_OFF, i2c_reg				; load current register address
	INC		I2C_OFF							; increment
	STS		i2c_reg, I2C_OFF				; store register address for next transaction
	RJMP	ldone_RET
.endfunc

.func	TWI_read
TWI_read:	; TWI slave transmitter
	; read data @I2C_OFF and write to TWDR, return new TWCR value in PARAM
	; always increment I2C_OFF for next transaction

	;		check if invalid address flag is set
	LDS		PWM, i2c_busy						; load i2c_busy flags
	SBRS	PWM, 2								; skip if invalid address flag
	RJMP	TWI_read_chkaddr					; continue with checking address
	STS		TWDR, PWM							; send data to master
	RJMP	ldone_NACK

TWI_read_chkaddr:
#if defined(USE_LEDREAD) || defined(USE_LEDWRITE)
	CPI		I2C_OFF, (8*8*3)+(4*4)			; address < (8*8*3) pixels + (4*4) LED registers
#else
	CPI		I2C_OFF, (8*8*3)				; address<(8*8*3) pixels buffer
#endif
	BRLO    ltsreg

	CPI     I2C_OFF, I2C_KEYS				; if address==I2C_KEYS
	BREQ    ltskeys
	CPI     I2C_OFF, I2C_EE_WP				; if address==I2C_EE_WP
	BREQ    ltsee

#	if defined(USE_REGWRITE) || (!defined(NDEBUG))
	CPI		I2C_OFF, 0xE0					; registers 0xE0-0xEF
	BRLO	ltsnack							; <0xE0
#	else
	CPI     I2C_OFF, I2C_WAI				; if address < 0xF0
	BRLO	ltsnack							; address invalid
#	endif

	CPI     I2C_OFF, I2C_TOV0C +1			; if address<=I2C_TOV0C
	BRLO    ltsreg							; address <= I2C_TOV0C

ltsnack:
	LDS		PARAM, i2c_busy					; load i2c_busy flags
	ORI		PARAM, (1 << 2)					; set invalid address flag
	STS		i2c_busy, PARAM					; store i2c_busy flags
	CLR		PARAM							; PARAM=0
	STS		TWDR, PARAM						; store data to send
	RJMP	TWI_NACK

ltskeys:	// joystick
	CBI     _SFR_IO_ADDR(PORTB), KEYS_INT	; clear KEYS_INT flag on read
	RJMP    ltsreg
ltsee:		// EEPROM write protection
	CLR		PARAM
	SBIS    _SFR_IO_ADDR(PORTB), EE_WP		; skip if EE_WP output set
	LDI     PARAM, 1						; set 1 if output active-low - write protection disabled
	STS		eewp, PARAM						; store value to buffers
	STS     TWDR, PARAM						; PARAM=(PORTB,EE_WP)
	RJMP	TWI_ACK
ltsreg:		; send from register buffer
	LDI     YL, lo8(registers)				; load low byte buffer addres
	LDI     YH, hi8(registers)				; load high byte buffer address
	ADD     YL, I2C_OFF						; add offset to buffer address
	CLR     I2C_OFF							; clear temporary variable
	ADC     YH, I2C_OFF						; add 0 and CARRY flag to high byte of address
	LD      PARAM, Y						; load from register buffer address
	STS     TWDR, PARAM						; store to data register
	RJMP	TWI_ACK
.endfunc

.func	TWI_write
TWI_write:	; TWI slave receiver
	; write data from TWDR to @I2C_OFF, return new TWCR value in PARAM
	; always increment I2C_OFF for next transaction
	LDS		PARAM, TWDR						; Data

#if	defined(USE_LEDWRITE)
	CPI		I2C_OFF, (8*8*3)+(4*4)			; if address<0xD0
#	else
	CPI		I2C_OFF, (8*8*3)				; if address<192
#	endif
	BRLO	lrsregisters					; I2C_OFF < 192 (+16 USE_LEDWRITE)

	CPI		I2C_OFF, I2C_EE_WP				; if address==0xF3
	BREQ	lree							; receive disable EE_WP
	CPI		I2C_OFF, I2C_WAI				; if address==0xF0
	BREQ	lrwai							; receive reset flag

#	if defined(USE_REGWRITE)
	CPI		I2C_OFF, I2C_WDTCSR				; if address==0xF4
	BREQ	lrwdtreg						; receive wdtcsr
	CPI		I2C_OFF, I2C_SMCR				; if address==0xF5
	BREQ	lrsmreg							; receive smcr
	CPI		I2C_OFF, 0xE0					; if address==0xE0
	BRLO	lrsnack							; I2C_OFF < 0xE0 - invalid
	CPI		I2C_OFF, 0xF0					; if address==0xF0
	BRLO	lrsregisters					; I2C_OFF < 0xF0 && >= 0xE0 - valid
#	endif

lrsnack:
	RJMP	ldone_NACK						; jump to NACK and RETI
lree:
	SBI		_SFR_IO_ADDR(PORTB), EE_WP
	CPSE	PARAM, 0
	CBI		_SFR_IO_ADDR(PORTB), EE_WP
	/*		new value not stored to buffer - its better to read it back from PORT with next main loop
	STS		eewp, PARAM						; store new value to registers buffer
	*/
	RJMP	TWI_ACK							; jump to ACK, i2c_reg++ and RETI

lrwdtreg:
	LDI		PWM, (1 << WDCE) | (1 << WDE)	; enable change WDTCSR flags
	STS		WDTCSR, PWM						; first write enables change
	STS		WDTCSR, PARAM					; write new value to register
	RJMP	TWI_ACK							; jump to ACK, i2c_reg++ and RETI

lrsmreg:
	STS		SMCR, PARAM						; write new value to register
	RJMP	TWI_ACK							; jump to ACK, i2c_reg++ and RETI

lrwai:		; receive reset flag
	/*		check received byte for MARKER and reset system if equal
	**		RESET_MARKER == 0x42
	*/
	CPI		PARAM, 0x42
	BRNE	lrsnack							; not equal, so send NACK
	RJMP	SYSTEM_RESET					; execute SYSTEM_RESET procedure

lrsregisters:
	; load Y register with buffer address
	LDI     YL, lo8(registers)
	LDI     YH, hi8(registers)
	ADD     YL, I2C_OFF						; add offset to Y register
	; clear I2C_OFF and add CARRY to YH
	CLR     I2C_OFF
	ADC     YH, I2C_OFF
	ST		Y, PARAM						; store PARAM to buffer
	;		load flags and start TWI transactions
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	STS		TWCR, PARAM					; load PARAM to TWCR and start TWI

#	if defined(USE_REGWRITE)
	/*		TODO: buffer must be written to register
	**		I2C_OFF	register address
	**		PARAM	new value
	*/
	;		check register address and write corresponding port
	CPI		I2C_OFF, 0xE0					; PINA
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PINA, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xE1					; DDRA
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		DDRA, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xE2					; PORTA
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PORTA, PARAM					; store value to register
	RJMP	lrsreg_done						; return

	CPI		I2C_OFF, 0xE3					; PINB
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PINB, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xE4					; DDRB
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		DDRB, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xE5					; PORTB
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PORTB, PARAM					; store value to register
	RJMP	lrsreg_done						; return

	CPI		I2C_OFF, 0xE6					; PINC
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PINC, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xE7					; DDRC
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		DDRC, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xE8					; PORTC
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PORTC, PARAM					; store value to register
	RJMP	lrsreg_done						; return

	CPI		I2C_OFF, 0xE9					; PIND
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PIND, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xEA					; DDRD
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		DDRD, PARAM						; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xEB					; PORTD
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PORTD, PARAM					; store value to register
	RJMP	lrsreg_done						; return

	CPI		I2C_OFF, 0xEC					; MCUSR
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		MCUSR, PARAM					; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xED					; MCUCR
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		MCUCR, PARAM					; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xEE					; PORTCR
	BRNE	.+6								; skip (4+2 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PORTCR, PARAM					; store value to register
	RJMP	lrsreg_done						; return
	CPI		I2C_OFF, 0xEF					; PRR
	BRNE	.+4								; skip (4 byte instruction) if not equal (Z==0, Rd-Rr!=0)
	STS		PRR, PARAM						; store value to register
#	endif
lrsreg_done:
	;		increment i2c_reg
	INC		I2C_OFF							; increment
	STS		i2c_reg, I2C_OFF				; store register address for next transaction
	RJMP	ldone_RET						; jump to RETI
.endfunc

.func	draw_loop
draw_loop:
	;		save registers
	PUSH	YH								; r29, YH pointer
	PUSH	YL								; r28, YL pointer
	PUSH	PWM								; r25, PWM
	PUSH	LINE							; r24, LINE, PARAM
	PUSH	r21								; bits to transmit
	PUSH	r20								; pixels register
	;		prepare
	CLR		LINE
	STS		redrawleds, LINE				; clear flag redrawleds=0
	CBI		_SFR_IO_ADDR(PORTC), FRAME_INT	; clear FRAME_INT interrupt flag
	LDI		YL, lo8(pixels)					; load memory address of pixels buffer to Y register
	LDI		YH, hi8(pixels)
	LDI		LINE, 0xFE						; LINE=0xFE bit 0 cleared

draw_loop_line:									; 8 run loop send line to LED2472G
	OUT		_SFR_IO_ADDR(PORTD), LINE		; set COLUMN_ENABLEx low
	LDI		PWM, 0							; PWM=0

draw_loop_col:								; set bits to transmit counter
	LDI		r21, 24							; number of bits 8R+8G+8B to transmit
draw_loop_bit:								; 24 run loop send bits to LED2472G
	LD		r20, Y+							; load pixels register
	CP		PWM, r20
	BRSH	draw_loop_5DI0					; PWM >= r20
	SBI		_SFR_IO_ADDR(PORTC), LED_SDI	; set SDI active-high
	RJMP	draw_loop_SDI1					; PWM < r20
draw_loop_5DI0:
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
	SUBI	YL, 24							; YL -= 24, back to address before 24 bit loop runs
	SBCI	YH, 0							; YH -= CARRY
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
	POP		YL								; r28, YL pointer
	POP		YH								; r29, YH pointer
	; continue with joystick reading
	RJMP	check_keys
.endfunc

.func	check_keys
check_keys:
	PUSH	PWM								; save register
	PUSH	PARAM							; save register
	SBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; Scan
	LDI		PARAM, 0xFF
	OUT		_SFR_IO_ADDR(PORTD), PARAM		; switch PORTD high
	LDI		PARAM, 0x07
	OUT		_SFR_IO_ADDR(DDRD), PARAM		; switch bit 3-7 to input for joystick
	LDS		PWM, keys
	IN		PARAM, _SFR_IO_ADDR(PIND)		; read joystick status
	COM		PARAM							; change bits to active-high - joystick switches to GND
	LSR		PARAM
	LSR		PARAM
	LSR		PARAM
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

;	(r25),r24 uint8_t r24/PARAM = delay for clkT0 ticks
.func delay
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
	; save registers
	;PUSH	r25								; unused
	PUSH	r24
	PUSH	r23
	PUSH	r22
	;PUSH	r21								; unused
	PUSH	r20
	PUSH	r18
	LDI		r18, 24							; number of bits to transmit
write_data1:								; do {
	SBRS	r22, 0							; 	if (data&1)
	RJMP	write_data2
	SBI		_SFR_IO_ADDR(PORTC), LED_SDI	; 		set LED_SDI;
	RJMP	write_data3
write_data2:								; 	else
	CBI		_SFR_IO_ADDR(PORTC), LED_SDI	;		clr LED_SDI;
write_data3:
	/*	r25 rotation unneeded, because value is unused
	LSR		r25								; DATA>>=1;           0, bit 31-25, CARRY=bit24
	*/
	ROR		r24								; DATA>>=1; CARRY=bit24, bit 23-17, CARRY=bit16
	ROR		r23								; DATA>>=1; CARRY=bit16, bit 15-9 , CARRY=bit8
	ROR		r22								; DATA>>=1; CARRY=bit8 , bit  7-1 , CARRY=bit0
	CP		r24, r18						; 	if (type == i)
	BRNE	write_data4
	SBI		_SFR_IO_ADDR(PORTC), LED_LE		; 		set LDE_LE;
write_data4:
	CLOCK_OUT
	DEC		r18								; 	i--;
	BRNE	write_data1						; } while (i!=0);
	CBI		_SFR_IO_ADDR(PORTC), LED_LE		; 		clr LDE_LE
	; restore registers
	POP		r18
	POP		r20
	;POP		r21								; unused
	POP		r22
	POP		r23
	POP		r24
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
	; save registers
	/*	no need to save, r25::r22 will be return
	PUSH	r25
	PUSH	r24
	PUSH	r23
	PUSH	r22
	*/
	PUSH	r21								; save register for write_data call
	PUSH	r20								; save register for write_data call
	; first the LE sequence must be sent
	MOVW	r20, r24						; TYPE parameter for write_data
	CLR		r25
	CLR		r24
	CLR		r23
	CLR		r22								; DATA=0 for write_data
	RCALL	write_data
	; now give time to aquire data
	MOV		r20, r24						; copy register
	CPI		r24, 11							; 11==DET_OPEN, 12=DET_SHORT, 13=DET_OPEN_SHORT
	BRLO	read_data2						; <11 no additional time needed
	CPI		r24, 14							; 14==THERM_READ
	BRSH	read_data2						; >=14 no additional time needed
	; error detection needs at least 1ys
	PUSH	PARAM							; save PARAM/r24 register
	LDI		PARAM, 0xFF						; PARAM=0xFF
	OUT		_SFR_IO_ADDR(DDRD), PARAM		; switch PORTD to output
	COM		PARAM							; PARAM=0x00
	OUT		_SFR_IO_ADDR(PORTD), PARAM		; switch PORTD active-low
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; When the ROE, GOE and BOE signals become low, error detection starts.
	LDI		PARAM, 8						; 8 *128ns = 1024ys
	RCALL	delay							; delay(8) - 1024ys delay
	POP		PARAM							; restore PARAM/r24 register
	; after first CLK pulse data will be made available
read_data2:
	POP		r20								; restore register
	POP		r21								; restore register
	; read data from LED2472G
	PUSH	r18								; save register
	LDI		r18, 24							; number of bits to receive
	CBI		_SFR_IO_ADDR(PORTC), LED_SDI	; clr LED_SDI;
read_data3:									; do {
	SBI		_SFR_IO_ADDR(PORTC), LED_CLKR	; raise LED_CLKR
	SBIC	_SFR_IO_ADDR(PORTC), LED_OE_N	; skip if LED_OE_N cleared - first clock
	RJMP	read_data4						; skip delay
	CBI		_SFR_IO_ADDR(PORTC), LED_CLKR	; lower LED_CLKR before delay
	PUSH	PARAM							; save PARAM/r24 register
	LDI		PARAM, 1						; 1 *128ns
	RCALL	delay							; delay(1) - 128ys delay
	POP		PARAM							; restore PARAM/r24 register
	SBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; kept low for at least 1 ?s
read_data4:
	CLC		; clear CARRY
	SBIC	_SFR_IO_ADDR(PORTC), LED_SDO	; skip if LED_SDO cleared
	SEC		; set CARRY if LED_SDO is set
	CBI		_SFR_IO_ADDR(PORTC), LED_CLKR	; lower LED_CLKR
	ROL		r22								; DATA<<=1; CARRY=bit8 , bit  7-1 , CARRY=LED_SDO
	ROL		r23								; DATA<<=1; CARRY=bit16, bit 15-9 , CARRY=bit8
	ROL		r24								; DATA<<=1; CARRY=bit24, bit 23-17, CARRY=bit16
	/*	r25 rotation unneeded, because value is unused
	**	but without rotation value need to be cleared
	ROL		r25								; DATA<<=1; CARRY=bit32, bit 31-25, CARRY=bit24
	*/
	DEC		r18								; 	i--;
	BRNE	read_data3						; } while (i!=0);
	CLR		r25								; unused DATA.31-24=0
	/*	r25:r24:r23:r22 is return value, save/restore would be a big-bad-bug
	POP		r22
	POP		r23
	POP		r24
	POP		r25
	*/
	POP		r18								; restore registers
	RET
.endfunc
