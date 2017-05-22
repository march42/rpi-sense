/*
 * rpi-sense.S - Atmel assembly routines for LED framebuffer scan-out
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

#include <avr/io.h>
#include <util/twi.h>

LINE	= 24
PARAM	= 24
PWM		= 25
I2C_OFF	= 26	; XL

.equiv I2C_WAI	, 0xf0
.equiv I2C_VER	, 0xf1
.equiv I2C_KEYS	, 0xf2
.equiv I2C_EE_WP, 0xf3
.equiv I2C_WDC	, 0xf6
.equiv I2C_ID	, 's'
#ifndef	NDEBUG
.equiv VERSION	, 0xA1	// debug version
#else
.equiv VERSION	, 0x01
#endif

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

.macro COL_OUT reg
	CP	PWM, \reg
	BRSH	lclear\reg
	SBI	_SFR_IO_ADDR(PORTC), LED_SDI
	RJMP	ldone\reg
lclear\reg:
	CBI	_SFR_IO_ADDR(PORTC), LED_SDI
ldone\reg:
	CLOCK_OUT
.endm

.global draw_loop
.global delay
.global read_data
.global write_data
.global TWI_vect
.global	check_keys

.extern pixels
#ifndef NDEBUG
.extern	registers
#endif
.extern keys
.extern	watchdog
.extern i2c_reg
.extern i2c_busy
.extern redrawleds

.section .text

;TODO: Add reads and NACK bad addresses.
;PARAM (r24) - TWSR
;PWM (r25) - SREG
;DI2C/XL (r26) - Address
.func	TWI_vect
TWI_vect:	// ISR, global interrupts disabled on entry
	CLI		; clear interrupt enable flag
	WDR		; reset watchdog timer
	; save registers
	PUSH	PARAM
	PUSH	PWM
	PUSH	YL
	PUSH	YH
	PUSH	I2C_OFF
	IN		PWM, _SFR_IO_ADDR(SREG)
	; set I2C busy flag
	LDI		PARAM, 1
	STS		i2c_busy, PARAM				; i2c_busy=1
	LDS		I2C_OFF, i2c_reg			; I2C_OFF holds previous register address
	; check TWSR status register
	LDS		PARAM, TWSR					; load status register
	ANDI	PARAM, TW_STATUS_MASK		; mask out 
	;TW_BUS_ERROR	; illegal start or stop condition
	;TW_NO_INFO	; no state information available
	// slave transmitter
	CPI		PARAM, TW_ST_SLA_ACK		; SLA+R received, ACK returned
	BREQ	ltsend
	CPI		PARAM, TW_ST_DATA_ACK		; data transmitted, ACK received
	BREQ	ltsend
	CPI		PARAM, TW_ST_DATA_NACK		; data transmitted, NACK received
	BREQ	ltdnack
	CPI 	PARAM, TW_ST_LAST_DATA		; last data byte transmitted, ACK received
	BREQ	TWI_stop
	;CPI 	PARAM, TW_ST_ARB_LOST_SLA_ACK	; arbitration lost in SLA+RW, SLA+R received, ACK returned
	// slave receiver
	CPI		PARAM, TW_SR_DATA_ACK		; data received, ACK returned
	BREQ	lreceive
	CPI 	PARAM, TW_SR_DATA_NACK		; data received, NACK returned
	BREQ	TWI_stop
	CPI		PARAM, TW_SR_SLA_ACK		; SLA+W received, ACK returned
	BREQ	lrack
	CPI 	PARAM, TW_SR_STOP			; stop or repeated start condition received while selected
	BREQ	TWI_stop
	;CPI 	PARAM, TW_SR_ARB_LOST_GCALL_ACK	; arbitration lost in SLA+RW, general call received, ACK returned
	;CPI 	PARAM, TW_SR_ARB_LOST_SLA_ACK	; arbitration lost in SLA+RW, SLA+W received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_ACK		; general call received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_DATA_ACK	; general call data received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_DATA_NACK	; general call data received, NACK returned
	; without TWEA set the next transaction will be NACKed
	LDI		PARAM,  (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone
ltsend:		RJMP	TWI_read			; transmit data to master
lreceive:	RJMP	TWI_write			; receive data from master
ltack:	; SLA+R, ACK sent
lrack:	; SLA+W, ACK sent
	LDI		PARAM, 0xff					; Address = 0xff
	STS		i2c_reg, PARAM				; 0xFF is marker for register address receive
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone
ltdnack:	; NACK received
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone
TWI_NACK:	; send NACK on next receive
	LDI		PARAM,  (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone
TWI_reset:
	LDS		PARAM,	TWCR				; load TWCR register
	SBR		PARAM,	TWSTO				; set TWSTO flag
	STS		TWCR,	PARAM				; This will not generate a STOP condition, but the TWI returns to unaddressed and releases lines
	LDS		PARAM,	TWCR				; load TWCR register
	SBRC	PARAM,	TWSTO				; skip if bit cleared - TWI has been reinitialized
	RJMP	TWI_reset					; while TWSTO set
	; without TWEA set the next transaction will be NACKed
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone
TWI_stop:
	CLR		PARAM
	STS		i2c_busy, PARAM				; i2c_busy=0
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone
ldone:
	STS		TWCR, PARAM					; load PARAM to TWCR
	WDR		; reset watchdog timer
	SEI		; set interrupt enable flag
	; restore registers
	OUT		_SFR_IO_ADDR(SREG), PWM
	POP		I2C_OFF
	POP		YH
	POP		YL
	POP		PWM
	POP		PARAM
	RETI	; return from ISR and global interrupt enable
.endfunc

.func	TWI_read
TWI_read:
	; read data @I2C_OFF and write to TWDR, return new TWCR value in PARAM
	CPI     I2C_OFF, 192				; if address<192
	BRLO    ltspixel

#ifndef	NDEBUG
	CPI		I2C_OFF, 0xE0				; registers 0xE0-0xEB
	BRLO	ltsnack						; <0xE0
	CPI		I2C_OFF, 0xF0
	BRLO	ltsregisters				; 0xE0 <= I2C_OFF <= 0xEF
#endif

	CPI     I2C_OFF, I2C_WAI			; if address==I2C_WAI
	BREQ    ltswai
	CPI     I2C_OFF, I2C_VER			; if address==I2C_VER
	BREQ    ltsver
	CPI     I2C_OFF, I2C_KEYS			; if address==I2C_KEYS
	BREQ    ltskeys
	CPI     I2C_OFF, I2C_EE_WP			; if address==I2C_EE_WP
	BREQ    ltsee
	CPI     I2C_OFF, I2C_WDC			; if address==I2C_WDC
	BREQ    ltswdc
ltsnack:
	LDI		PARAM, 0xff
	STS		TWDR, PARAM
	; for real NACK TWEA=0 must be set to disable ACK
	RJMP	ltdone
ltswai:		// WHO-AM-I
	LDI     PARAM, I2C_ID
	STS     TWDR, PARAM
	RJMP    ltdone
ltsver:		// VERSION
	LDI     PARAM, VERSION
	STS     TWDR, PARAM
	RJMP    ltdone
ltskeys:	// joystick
	CBI     _SFR_IO_ADDR(PORTB), KEYS_INT	; clear KEYS_INT flag on read
	LDS		PARAM, keys
	STS     TWDR, PARAM
	RJMP    ltdone
ltsee:		// EEPROM write protection
	LDI     PARAM, 0
	SBIS    _SFR_IO_ADDR(PORTB), EE_WP		; skip if EE_WP output set
	LDI     PARAM, 1
	STS     TWDR, PARAM						; PARAM=(PORTB,EE_WP)
	RJMP    ltdone
ltswdc:		// watchdog interrupt counter
	LDS		PARAM, watchdog
	STS     TWDR, PARAM
	RJMP    ltdone
ltspixel:
	LDI     YL, lo8(pixels)
	LDI     YH, hi8(pixels)
	ADD     YL, I2C_OFF
	INC     I2C_OFF
	STS     i2c_reg, I2C_OFF
	CLR     I2C_OFF
	ADC     YH, I2C_OFF
	LD      I2C_OFF, Y
	STS     TWDR, I2C_OFF
	RJMP    ltdone

#ifndef	NDEBUG
ltsregisters:
	; load Y register with buffer address
	LDI     YL, lo8(registers)
	LDI     YH, hi8(registers)
	; calculate buffer offset
	SUBI	I2C_OFF, 0xE0			; I2C_OFF -= 0xE0 (I2C register address)
	ADD     YL, I2C_OFF				; add offset to Y register
	; add CARRY to YH
	BRCC	.+2						; skip if CARRY cleared
	INC		YH
	; load register from buffer
	LD		I2C_OFF, Y				; load address to register
	STS		TWDR, I2C_OFF			; store register to TWDR
	; increment register address
	SUBI	I2C_OFF, 0x1F			; add I2C register address +1 (+0xE1 == -0x1F)
	STS		i2c_reg, I2C_OFF		; ++i2c_reg for multi byte block transfers
	; TODO: bit 7 is always set, if finishing took to long
	RJMP	ltdone
#endif

ltdone:
	LDI	PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	WDR		; reset watchdog timer
	RJMP	ldone
.endfunc

.func	TWI_write
TWI_write:
	; write data from TWDR to @I2C_OFF, return new TWCR value in PARAM
	LDS		PARAM, TWDR				; Data
	CPI		I2C_OFF, 192			; if address<192
	BRLO	lrspixel
	CPI		I2C_OFF, 0xff			; if address==0xff
	BREQ	lrsreg
	CPI		I2C_OFF, I2C_EE_WP		; if address==0xF3
	BREQ	lree

#ifndef	NDEBUG
	CPI		I2C_OFF, 0xE0			; registers 0xE0-0xEB
	BRLO	lrsnack					; <0xE0
	CPI		I2C_OFF, 0xF0
	BRLO	lrsregisters			; 0xE0 <= I2C_OFF <= 0xEF
#endif

lrsnack:
	; NACK here
	; for real NACK TWEA=0 must be set to disable ACK
	RJMP	lrdone					; else...
lree:
	SBI		_SFR_IO_ADDR(PORTB), EE_WP
	CPSE	PARAM, 0
	CBI		_SFR_IO_ADDR(PORTB), EE_WP
	RJMP	lrdone
lrsreg:		; write address
	STS		i2c_reg, PARAM			; address = DATA
	; now address could be check with available addresses and TWEA disabled if unknown address
	CPI		PARAM, 192				; address<192 pixels buffer
	BRLO	lrsregOK				; address valid
	;CPI		PARAM, I2C_WAI		; address==0xF0 Who-Am-I
	;BREQ	lrsregOK				; address valid
	;CPI		PARAM, I2C_VER		; address==0xF1 firmware version
	;BREQ	lrsregOK				; address valid
	;CPI		PARAM, I2C_KEYS		; address==0xF2 keys status
	;BREQ	lrsregOK				; address valid
	CPI		PARAM, I2C_EE_WP		; address==0xF3 EEPROM write protection
	BREQ	lrsregOK				; address valid
lrsregNOK:
	; for real NACK TWEA=0 must be set to disable ACK
	LDI	PARAM,  (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone					; return and continue at ldone
lrsregOK:
	RJMP	lrdone

#ifndef	NDEBUG
lrsregisters:
	; load Y register with buffer address
	LDI     YL, lo8(registers)
	LDI     YH, hi8(registers)
	; increment register address
	INC		I2C_OFF
	STS		i2c_reg, I2C_OFF		; ++i2c_reg for multi byte block transfers
	SUBI	I2C_OFF, 0xE1			; I2C_OFF -= 0xE0 (I2C register address) + 1 (previous increment)
	ADD     YL, I2C_OFF				; add offset to Y register
	; clear I2C_OFF and add CARRY to YH
	CLR     I2C_OFF
	ADC     YH, I2C_OFF
	; store PARAM to buffer
	ST		Y, PARAM
	; TODO: buffer must be written to register
	RJMP	lrdone
#endif

lrspixel:
	LDI	YL, lo8(pixels)
	LDI	YH, hi8(pixels)
	ADD	YL, I2C_OFF
	INC	I2C_OFF
	STS	i2c_reg, I2C_OFF
	CLR	I2C_OFF
	ADC	YH, I2C_OFF
	ST	Y, PARAM
	LDI		LINE, 1
	STS		redrawleds, LINE	; set flag redrawleds=1
	RJMP	lrdone
lrdone:
	LDI	PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	WDR		; reset watchdog timer
	RJMP	ldone				; return and continue at ldone
.endfunc

.func draw_loop
draw_loop:
/*	wait for I2C to finish in main
	WDR		; reset watchdog timer
while_i2c_busy:
	LDS		_SFR_IO_ADDR(GPIOR0), i2c_busy
	CPI		_SFR_IO_ADDR(GPIOR0), 0
	BRNE	while_i2c_busy
*/
/*	unsure, if useful or necessary, so disabled for now
	LDS		_SFR_IO_ADDR(GPIOR0), TWCR
	ANDI	_SFR_IO_ADDR(GPIOR0), (1 << TWINT)
	BRNE 	while_i2c_busy		; TWINT set if i2c operation finished
*/
/*
	LDI		LINE, 0x00						; LINE=0x00
	OUT		_SFR_IO_ADDR(PORTD), LINE		; switch PORTD active-low
	COM		LINE							; LINE=0
	OUT		_SFR_IO_ADDR(DDRD), LINE		; switch PORTD to output
*/
	WDR		; reset watchdog timer
	;	check if update is needed
	LDS		LINE, redrawleds				; redrawleds needed, after pixels update
	CPI		LINE, 0
	BRNE	.+2								; (FIX HACK) relocation truncated to fit: R_AVR_7_PCREL against `no symbol'
	RJMP	check_keys						; skip update and continue to joystick
	CLR		LINE
	STS		redrawleds, LINE				; clear flag redrawleds=0
Lframe_loop:
	CBI		_SFR_IO_ADDR(PORTC), FRAME_INT	; clear FRAME_INT interrupt flag
	LDI		YL, lo8(pixels)					; load memory address of pixels buffer to Y register
	LDI		YH, hi8(pixels)
	LDI		LINE, 0xFE						; LINE=0xFE bit 0 cleared
Lline_loop:
	//	bit 0 left unset
	OUT		_SFR_IO_ADDR(PORTD), LINE		; set COLUMN_ENABLEx
	LDI		PWM, 0							; PWM=0
	LD		r0, Y+
	LD		r1, Y+
	LD		r2, Y+
	LD		r3, Y+
	LD		r4, Y+
	LD		r5, Y+
	LD		r6, Y+
	LD		r7, Y+
	LD		r8, Y+
	LD		r9, Y+
	LD		r10, Y+
	LD		r11, Y+
	LD		r12, Y+
	LD		r13, Y+
	LD		r14, Y+
	LD		r15, Y+
	LD		r16, Y+
	LD		r17, Y+
	LD		r18, Y+
	LD		r19, Y+
	LD		r20, Y+
	LD		r21, Y+
	LD		r22, Y+
	LD		r23, Y+
Lpwm_loop:
	COL_OUT	r0
	COL_OUT	r1
	COL_OUT	r2
	COL_OUT	r3
	COL_OUT	r4
	COL_OUT	r5
	COL_OUT	r6
	COL_OUT	r7
	COL_OUT	r8
	COL_OUT	r9
	COL_OUT	r10
	COL_OUT	r11
	COL_OUT	r12
	COL_OUT	r13
	COL_OUT	r14
	COL_OUT	r15
	COL_OUT	r16
	COL_OUT	r17
	COL_OUT	r18
	COL_OUT	r19
	COL_OUT	r20
	COL_OUT	r21
	SBI		_SFR_IO_ADDR(PORTC), LED_LE		; LE @1-2 used to mark data latch - load bits to data register
	COL_OUT	r22
	COL_OUT	r23
	CBI		_SFR_IO_ADDR(PORTC), LED_LE		; LE low mark data end
	INC		PWM								; ++PWM
	SBRS	PWM, 6							; skip if bit set PWM bit 6
	RJMP	Lpwm_loop						; next 
	ROL		LINE							; shift LINE with CARRY bit - C=7,7=6,6=5,5=4,4=3,3=2,2=1,1=0,0=C
	BRCC	lframeend						; branch if CARRY=0 - after 8 loops
	INC		LINE							; ++LINE
	RJMP	Lline_loop						; next line output
lframeend:
	SBI		_SFR_IO_ADDR(PORTB), FRAME_INT	; raise FRAME_INT interrupt to Pi

	; continue with joystick reading
	RJMP	check_keys
.endfunc

.func	check_keys
check_keys:
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
	CPSE	PARAM, PWM						; if keys changed
	SBI		_SFR_IO_ADDR(PORTB), KEYS_INT	; raise KEYS_INT to Pi
	LDI		PARAM, 0xFF
	OUT		_SFR_IO_ADDR(DDRD), PARAM		; change PORTD to output
	COM		PARAM
	OUT		_SFR_IO_ADDR(PORTD), PARAM		; switch PORTD low (activate MOSFET switches)
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; /Scan
	RET		; return control back to main()
.endfunc

; r24/PARAM = delay for clkT0 ticks
.func delay
delay:
	PUSH	r19
	PUSH	r20
	IN	r20, _SFR_IO_ADDR(TCNT0)			; start = TCNT0;
lloop:
	IN	r19, _SFR_IO_ADDR(TCNT0)			; do {
	SUB	r19, r20							; diff = TCNT0 - start;
	CP	r19, PARAM							; } while (diff < ticks);
	BRLO	lloop
	POP	r20
	POP	r19
	RET
 .endfunc

.func write_data
; r20-23 DATA
; r24 TYPE
write_data:
	PUSH	r18
	;PUSH	r20
	PUSH	r21
	PUSH	r22
	PUSH	r23
	PUSH	r24
	LDI	r18, 24	;
1:						; do {
	SBRS	r23, 0				; 	if (data&1)
	RJMP	2f
	SBI	_SFR_IO_ADDR(PORTC), LED_SDI	; 		set LED_SDI;
	RJMP	3f
2:						; 	else
	CBI	_SFR_IO_ADDR(PORTC), LED_SDI	;		clr LED_SDI;
3:
	ASR	r23				; 	data <<= 1;
	ROR	r22
	ROR	r21
	;ROR	r20								; Don't need?
	CP	r24, r18			; 	if (type == i)
	BRNE	4f
	SBI	_SFR_IO_ADDR(PORTC), LED_LE	; 		set LDE_LE;
4:
	CLOCK_OUT
	DEC	r18				; 	i--;
	BRNE	1b				; } while (i!=0);
	CBI	_SFR_IO_ADDR(PORTC), LED_LE	; 		clr LDE_LE
	POP	r24
	POP	r23
	POP	r22
	POP	r21
	;POP	r20
	POP	r18
	RET
.endfunc

.func read_data
; r22-25 RETURN
; r24 TYPE
read_data:
	PUSH	r18
	PUSH	r19
	;PUSH	r20
	PUSH	r21
	PUSH	r22
	PUSH	r23
	PUSH	r24
	LDI	r18, 24	;
1:						; do {
	IN	r19, _SFR_IO_ADDR(PORTC)	; ret |= PINC&1;
	ANDI	r19, LED_SDI


	SBRS	r23, 0				; 	if (data&1)
	RJMP	2f
	SBI	_SFR_IO_ADDR(PORTC), LED_SDI	; 		set LED_SDI;
	RJMP	3f
2:						; 	else
	CBI	_SFR_IO_ADDR(PORTC), LED_SDI	;		clr LED_SDI;
3:
	ASR	r23				; 	data <<= 1;
	ROR	r22
	ROR	r21
	;ROR	r20								; Don't need?


	CP	r24, r18			; 	if (type == i)
	BRNE	4f
	SBI	_SFR_IO_ADDR(PORTC), LED_LE	; 		set LDE_LE;
4:
	CLOCK_OUT
	DEC	r18				; 	i--;
	BRNE	1b				; } while (i!=0);
	CBI	_SFR_IO_ADDR(PORTC), LED_LE	; 		clr LDE_LE
	POP	r24
	POP	r23
	POP	r22
	POP	r21
	;POP	r20
	POP	r19
	POP	r18
	RET
.endfunc
