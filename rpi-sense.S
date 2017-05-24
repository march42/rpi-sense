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
.equiv	I2C_WDC		, 0xf6
.equiv	watchdog	, (registers + I2C_WDC)

.section .text

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
	LDS		PARAM, i2c_busy				; load i2c_busy flags
	ORI		PARAM, 1					; i2c_busy |= 1
	STS		i2c_busy, PARAM				; store i2c_busy flags
	LDS		I2C_OFF, i2c_reg			; I2C_OFF holds previous register address
	; check TWSR status register
	LDS		PARAM, TWSR					; load status register
	ANDI	PARAM, TW_STATUS_MASK		; mask out pre-scaler bits
	CPI		PARAM, TW_SR_DATA_ACK		; data received, ACK returned
	BREQ	lreceive					; read DATA from master
	CPI		PARAM, TW_ST_DATA_ACK		; data transmitted, ACK received
	BREQ	ltsend						; send DATA to master
	CPI 	PARAM, TW_SR_STOP			; stop or repeated start condition received while selected
	BREQ	TWI_stop
	CPI 	PARAM, TW_ST_LAST_DATA		; last data byte transmitted, ACK received
	BREQ	TWI_stop
	CPI		PARAM, TW_SR_SLA_ACK		; SLA+W received, ACK returned
	BREQ	lrsla						; next DATA is register address
	CPI		PARAM, TW_ST_SLA_ACK		; SLA+R received, ACK returned
	BREQ	ltsend						; send current DATA to master
	CPI 	PARAM, TW_SR_DATA_NACK		; data received, NACK returned
	BREQ	TWI_stop
	CPI		PARAM, TW_ST_DATA_NACK		; data transmitted, NACK received
	BREQ	ltdnack

	;TW_BUS_ERROR	; illegal start or stop condition
	;TW_NO_INFO	; no state information available
	// slave transmitter
	;CPI 	PARAM, TW_ST_ARB_LOST_SLA_ACK	; arbitration lost in SLA+RW, SLA+R received, ACK returned
	// slave receiver
	// I2C spec: "If two or more masters try to put information onto the bus, the first to produce a ‘one’ when the other produces a ‘zero’ will lose the arbitration"
	;CPI 	PARAM, TW_SR_ARB_LOST_GCALL_ACK	; arbitration lost in SLA+RW, general call received, ACK returned
	;CPI 	PARAM, TW_SR_ARB_LOST_SLA_ACK	; arbitration lost in SLA+RW, SLA+W received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_ACK		; general call received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_DATA_ACK	; general call data received, ACK returned
	;CPI 	PARAM, TW_SR_GCALL_DATA_NACK	; general call data received, NACK returned
	; without TWEA set the next transaction will be NACKed
	LDI		PARAM,  (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
ldone:
	STS		TWCR, PARAM					; load PARAM to TWCR
	WDR		; reset watchdog timer
	; restore registers
	OUT		_SFR_IO_ADDR(SREG), PWM
	POP		I2C_OFF
	POP		YH
	POP		YL
	POP		PWM
	POP		PARAM
	RETI	; return from ISR and global interrupt enable

lrsla:		; SLA+W received, ACK sent
	LDS		PARAM, i2c_busy				; load i2c_busy flags
	ORI		PARAM, 0b00000010			; set next DATA is address flag
	STS		i2c_busy, PARAM				; store i2c_busy flags
	CLR		I2C_OFF						; reset I2C_OFF to first register I2C_OFF=0x00
	LDI		PARAM, (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone

ltsend:		; DATA sent
	RJMP	TWI_read					; transmit data to master

ltdnack:	; NACK received
	;		just ignore NACK and wait for next transaction - most probably STOP condition
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone

TWI_NACK:	; send NACK on next receive
	LDI		PARAM, (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
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

lreceive:	; slave receive DATA received
	LDS		PARAM, i2c_busy				; load i2c_busy flags
	SBRS	PARAM, 1					; skip if DATA is address - i2c_busy.1==1
	RJMP	TWI_write					; i2c_busy.address==0 receive data from master
lraddress:	; slave receive DATA==address
	LDI		I2C_OFF, 1					; set i2c_busy flag
	STS		i2c_busy, I2C_OFF			; store i2c_busy=1 flag, following data is real DATA
	LDS		I2C_OFF, TWDR				; load DATA - register address
	STS		i2c_reg, I2C_OFF			; i2c_reg = DATA, register address for coming read transactions
	; now address could be check with available addresses and TWEA disabled if unknown address
#ifdef	USE_LEDREAD
	CPI		I2C_OFF, (8*8*3)+(4*4)		; address < (8*8*3) pixels + (4*4) LED registers
#else
	CPI		I2C_OFF, (8*8*3)			; address<(8*8*3) pixels buffer
#endif
	BRLO	lrsregOK					; address valid
#ifndef NDEBUG
	CPI		I2C_OFF, 0xE0				; address < 0xE0 (&& >=192) debug registers
#else
	CPI		I2C_OFF, I2C_WAI			; address < 0xF0 (&& >=192) Who-Am-I
#endif
	CPI		I2C_OFF, I2C_WDC +1			; address<=0xF6 watchdog counter
	BRLO	lrsregOK					; address invalid
	; other addresses are invalid
lrsregNOK:
	; for real NACK TWEA=0 must be set to disable ACK
	LDI		PARAM, (0<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone						; return and continue at ldone
lrsregOK:
	LDI		PARAM, (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	RJMP	ldone						; return and continue at ldone
.endfunc

.func	TWI_read
TWI_read:	; TWI slave receiver
	; read data @I2C_OFF and write to TWDR, return new TWCR value in PARAM
	; always increment I2C_OFF for next transaction
#ifdef	USE_LEDREAD
	CPI		I2C_OFF, (8*8*3)+(4*4)		; address < (8*8*3) pixels + (4*4) LED registers
#else
	CPI		I2C_OFF, (8*8*3)			; address<(8*8*3) pixels buffer
#endif
	BRLO    ltsreg
#ifndef	NDEBUG
	CPI		I2C_OFF, 0xE0				; registers 0xE0-0xEB
	BRLO	ltsnack						; <0xE0
	CPI		I2C_OFF, 0xF0
	BRLO	ltsreg						; 0xE0 <= I2C_OFF <= 0xEF
#endif
	CPI     I2C_OFF, I2C_KEYS			; if address==I2C_KEYS
	BRLO	ltsreg						; address < I2C_KEYS
	BREQ    ltskeys
	CPI     I2C_OFF, I2C_EE_WP			; if address==I2C_EE_WP
	BREQ    ltsee
	CPI     I2C_OFF, I2C_WDC +1			; if address<=I2C_WDC
	BRLO    ltsreg						; address <= I2C_WDC

ltsnack:
#ifndef NDEBUG
	COM		I2C_OFF
	STS		TWDR, I2C_OFF					; simply send back complement of address
	COM		I2C_OFF
#else
	LDI		TWDR, 0xFF						; all bits high like pulled-up
#endif
	INC     I2C_OFF							; increment offset
	STS     i2c_reg, I2C_OFF				; store offset for next transaction
	;		for real NACK TWEA=0 must be set to disable ACK
	RJMP	ltdone

ltskeys:	// joystick
	CBI     _SFR_IO_ADDR(PORTB), KEYS_INT	; clear KEYS_INT flag on read
	RJMP    ltsreg
ltsee:		// EEPROM write protection
	INC		I2C_OFF							; increment register offset address
	STS		i2c_reg, I2C_OFF				; store offset for next transaction
	CLR		I2C_OFF							; clear
	SBIS    _SFR_IO_ADDR(PORTB), EE_WP		; skip if EE_WP output set
	LDI     I2C_OFF, 1						; set 1 if output active-low - write protection disabled
	STS		eewp, I2C_OFF					; store value to buffers
	STS     TWDR, I2C_OFF					; PARAM=(PORTB,EE_WP)
	RJMP    ltdone
ltsreg:		; send from register buffer
	LDI     YL, lo8(registers)				; load low byte buffer addres
	LDI     YH, hi8(registers)				; load high byte buffer address
	ADD     YL, I2C_OFF						; add offset to buffer address
	INC		I2C_OFF							; increment register offset address
	STS		i2c_reg, I2C_OFF				; store offset for next transaction
	CLR     I2C_OFF							; clear temporary variable
	ADC     YH, I2C_OFF						; add 0 and CARRY flag to high byte of address
	LD      I2C_OFF, Y						; load from register buffer address
	STS     TWDR, I2C_OFF					; store to data register
	RJMP    ltdone

ltdone:
	LDI		PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	WDR		; reset watchdog timer
	RJMP	ldone
.endfunc

.func	TWI_write
TWI_write:	; TWI slave transmitter
	; write data from TWDR to @I2C_OFF, return new TWCR value in PARAM
	; always increment I2C_OFF for next transaction
	LDS		PARAM, TWDR				; Data
	CPI		I2C_OFF, 192			; if address<192
	BRLO	lrsregisters			; I2C_OFF < 192
	CPI		I2C_OFF, I2C_EE_WP		; if address==0xF3
	BREQ	lree

lrsnack:
	; NACK here
	; for real NACK TWEA=0 must be set to disable ACK
	RJMP	lrdone					; else...
lree:
	SBI		_SFR_IO_ADDR(PORTB), EE_WP
	CPSE	PARAM, 0
	CBI		_SFR_IO_ADDR(PORTB), EE_WP
	;STS		eewp, PARAM				; store new value to registers buffer
	INC     I2C_OFF					; increment offset
	STS     i2c_reg, I2C_OFF		; store offset for next transaction
	RJMP	lrdone

lrsregisters:
	; load Y register with buffer address
	LDI     YL, lo8(registers)
	LDI     YH, hi8(registers)
	ADD     YL, I2C_OFF				; add offset to Y register
	; increment register address
	INC		I2C_OFF
	STS		i2c_reg, I2C_OFF		; ++i2c_reg for multi byte block transfers
	; clear I2C_OFF and add CARRY to YH
	CLR     I2C_OFF
	ADC     YH, I2C_OFF
	ST		Y, PARAM				; store PARAM to buffer
	; TODO: buffer must be written to register
	RJMP	lrdone

lrdone:
	LDI	PARAM,  (1<<TWEA) | (1<<TWEN) | (1<<TWIE) | (1<<TWINT)
	WDR		; reset watchdog timer
	RJMP	ldone				; return and continue at ldone
.endfunc

.func draw_loop
draw_loop:
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
	INC		PWM								; PWM++
	SBRS	PWM, 6							; skip if bit set PWM bit 6 (64 runs)
	RJMP	Lpwm_loop						; next
	SEC		; set CARRY flag and clear if bit7==0 to be sure
	SBRS	LINE, 7							; skip if bit7==1
	CLC										; clear CARRY if bit7==0
	ROL		LINE							; shift LINE with CARRY bit - C=7,7=6,6=5,5=4,4=3,3=2,2=1,1=0,0=C
	BRCC	lframeend						; branch if CARRY=0 - after 8 loops
	INC		LINE							; LINE++
	RJMP	Lline_loop						; next line output
lframeend:
	SBI		_SFR_IO_ADDR(PORTB), FRAME_INT	; raise FRAME_INT interrupt to Pi
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; /Scan

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
	CPSE	PARAM, PWM						; if keys changed (compare, skip if equal)
	SBI		_SFR_IO_ADDR(PORTB), KEYS_INT	; raise KEYS_INT to Pi
	LDI		PARAM, 0xFF
	OUT		_SFR_IO_ADDR(DDRD), PARAM		; change PORTD to output
	COM		PARAM
	OUT		_SFR_IO_ADDR(PORTD), PARAM		; switch PORTD low (activate MOSFET switches)
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; /Scan
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
	BRLO	read_data1						; error detection needs at least 1ys
	RJMP	read_data2						; >=14 no additional time needed
read_data1:
	; error detection needs at least 1ys
	CBI		_SFR_IO_ADDR(PORTC), LED_OE_N	; When the ROE, GOE and BOE signals become low, error detection starts.
	PUSH	PARAM							; save PARAM/r24 register
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
