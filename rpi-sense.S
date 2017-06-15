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

	//SEI											; enable global interrupts
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
#if (I2C_HIGHSPEED)
	LSR		PARAM								; half clock means half ticks for same delay
#endif // (I2C_HIGHSPEED)
delay_loop:
	IN		r19, _SFR_IO_ADDR(TCNT0)			; do {
	SUB		r19, r20							; diff = TCNT0 - start;
	CP		r19, PARAM							; } while (diff < ticks);
	BRLO	delay_loop
	POP		r20
	POP		r19
	RET
.endfunc
