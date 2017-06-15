/*
** led2472g-SPI.S - code routines for low level handling of LED2472G driver
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

#include "rpi-sense.h"

.section .text

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
	SBI	_SFR_IO_ADDR(PORTC), LED_CLKR		; clock pulse out to LED2472G
	CBI	_SFR_IO_ADDR(PORTC), LED_CLKR
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
