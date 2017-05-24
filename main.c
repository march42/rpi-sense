/*
 * main.c - Atmel main loop for LED matrix driving
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
 
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <stdio.h>
#include <stdlib.h>

#define EE_WP			(1 << PB0)
#define KEYS_INT		(1 << PB6)
#define FRAME_INT		(1 << PB7)

#define LED_SDO			(1 << PC0)
#define LED_CLKR		(1 << PC1)
#define LED_LE			(1 << PC2)
#define LED_SDI			(1 << PC3)
#define LED_OE_N		(1 << PC7)

#define set(port,x) port |= (x)
#define clr(port,x) port &= ~(x)

extern void draw_loop(void);
extern void check_keys(void);
extern void delay(uint8_t ticks);

/*	bit count values for LED2472G shift register
**	LED_LE must rise after transmitting n-th bit
**	TODO: use le_key for write_data and read_data
*/
typedef enum {
	DAT_LATCH = 1,
	CONF_WRITE = 3,
	CONF_READ  = 5,
	GAIN_WRITE = 7,
	GAIN_READ = 9,
	DET_OPEN = 11,
	DET_SHORT = 12,
	DET_OPEN_SHORT = 13,
	THERM_READ = 14,
} le_key;
extern void write_data(uint32_t data, le_key type);
extern uint32_t read_data(le_key type);

/*	I2C register addresses
*/
enum REG_ADDR {
	// LED pixels buffers
	REG_PIXELS=0, REG_PIXELS_MAX=((8*8*3)-1),
	/*	unused - left from initial version
	REG_ID = 192,
	REG_CFG_LOW,
	REG_CFG_HIGH,
	*/
	// data from LED2472G shift register (high byte at high address, evenly aligned)
	REG_LED_CFG=0xC0,							// configuration registers (24+8 bit)
	REG_LED_GAIN=0xC4,							// gain (24+8 bit)
	REG_LED_ERROR=0xC8,							// error open/short detection (24+8 bit)
	REG_LED_THERMAL=0xCC,						// thermal alert (24+8 bit)
#ifndef NDEBUG
	// debugging registers
	REG_PINA=0xE0, REG_DDRA, REG_PORTA,
	REG_PINB, REG_DDRB, REG_PORTB,
	REG_PINC, REG_DDRC, REG_PORTC,
	REG_PIND, REG_DDRD, REG_PORTD,
	REG_MCUSR, REG_MCUCR,
	REG_PORTCR, REG_PRR,
#endif
	// administrative registers
	REG_WAI=0xF0, REG_VER=0xF1,
	REG_KEYS=0xF2,
	REG_EE_WP=0xF3,
	REG_WDTCSR=0xF4,
	REG_SMCR=0xf5,
	REG_WDC=0xF6,
};

#define I2C_ID	's'
#ifndef	NDEBUG
#	define VERSION	0xA1	/* debug version */
#else
#	define VERSION	0x01	/* release version */
#endif

typedef union
{
	uint8_t		ui8 [0x100];
	uint16_t	ui16[0x100 >>1];
	uint32_t	ui32[0x100 >>2];
}	I2C_PAGE_BUFFER;

volatile	I2C_PAGE_BUFFER	registers = { .ui8 = {
//	offset 0x00 - line 1
0x1F, 0x1F, 0x1F, 0x1F, 0x14, 0x03, 0x00, 0x00,	// red 1-8
0x00, 0x00, 0x03, 0x12, 0x1F, 0x1F, 0x1F, 0x1F,	// green 1-8
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,	// blue 1-8
//	offset 0x18 - line 2
0x1F, 0x1F, 0x1F, 0x12, 0x03, 0x00, 0x00, 0x00,	// red 9-16
0x00, 0x04, 0x14, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,	// green 9-16
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x1D,	// blue 9-16
//	offset 0x30 - line 3
0x1F, 0x1F, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00,	// red 17-24
0x05, 0x15, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0B,	// green 17-24
0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x1F, 0x1F,	// blue 17-24
//	offset 0x48 - line 4
0x1F, 0x0F, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,	// red 25-32
0x17, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0A, 0x00,	// green 25-32
0x00, 0x00, 0x00, 0x00, 0x0A, 0x1F, 0x1F, 0x1F,	// blue 25-32
//	offset 0x60 - line 5
0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,	// red 33-40
0x1F, 0x1F, 0x1F, 0x1F, 0x1D, 0x08, 0x00, 0x00,	// green 33-40
0x00, 0x00, 0x01, 0x0B, 0x1F, 0x1F, 0x1F, 0x1F,	// blue 33-40
//	offset 0x78 - line 6
0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x14,	// red 41-48
0x1F, 0x1F, 0x1F, 0x1B, 0x07, 0x00, 0x00, 0x00,	// green 41-48
0x00, 0x01, 0x0C, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,	// blue 41-48
//	offset 0x90 - line 7
0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x15, 0x1F,	// red 49-56
0x1F, 0x1F, 0x19, 0x06, 0x00, 0x00, 0x00, 0x00,	// green 49-56
0x02, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x12,	// blue 49-56
//	offset 0xA8 - line 8
0x00, 0x00, 0x00, 0x00, 0x05, 0x17, 0x1F, 0x1F,	// red 57-64
0x1F, 0x17, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,	// green 57-64
0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x02,	// blue 57-64
//	offset 0xC0
0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
//	offset 0xD0
0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
//	offset 0xE0
0xE0, 0xE1, 0xE2,					// PINA,DDRA,PORTA
0xE3, 0xE4, 0xE5,					// PINB,DDRB,PORTB
0xE6, 0xE7, 0xE8,					// PINC,DDRC,PORTC
0xE9, 0xEA, 0xEB,					// PIND,DDRD,PORTD
0xEC,								// MCUSR
0xED,								// MCUCR
0xEE,								// PORTCR
0xEF,								// PRR
//	offset 0xF0
I2C_ID, VERSION,					// Who-Am-I and firmware version
0xF2,								// keys
0xF3,								// EEPROM write protect disabled
0xF4,								// watchdog timer control register wdt_reg
0xF5,								// SMCR Sleep Mode Control Register
0xF6,								// I2C_WDC watchdog interrupt counter
0xF7,
0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
}};

//	some defines, to ease access
#define	reg_ui8(pos)	(registers.ui8[pos])
#define	reg_ui16(pos)	(registers.ui16[pos >>1])
#define	reg_ui32(pos)	(registers.ui32[pos >>2])
#define pixels			(reg_ui8(REG_PIXELS))
#define keys			(reg_ui8(REG_KEYS))
#define watchdog		(reg_ui8(REG_WDC))
#define wdt_reg			(reg_ui8(REG_WDTCSR))
#define sm_reg			(reg_ui8(REG_SMCR))
#ifdef	USE_LEDREAD
#	define LED_CFG		(reg_ui32(REG_LED_CFG))
#	define LED_GAIN		(reg_ui32(REG_LED_GAIN))
#	define LED_ERROR	(reg_ui32(REG_LED_ERROR))
#	define LED_THERMAL	(reg_ui32(REG_LED_THERMAL))
#endif

volatile uint8_t i2c_reg	= 0xFF;		// I2C register address
volatile char i2c_busy		= 1;		// I2C transactions in progress - bit0 =active, bit1 =DATA is address
#define I2C_BUSY_INPROGRESS	(1 << 0)	// transaction in progress - sometimes between START and STOP
#define I2C_BUSY_ADDRESS	(1 << 1)	// next DATA is register address
volatile char redrawleds	= 1;		// redraw LEDs after pixels has been updated

ISR(BADISR_vect, ISR_NAKED)
{
	/*	maybe here must the interrupt flags be cleared
	*/
	reti();						//	do nothing, just return
}
ISR(INT0_vect, ISR_ALIASOF(BADISR_vect));
ISR(INT1_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT0_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT1_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT2_vect, ISR_ALIASOF(BADISR_vect));
ISR(PCINT3_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_CAPT_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_COMPA_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_COMPB_vect, ISR_ALIASOF(BADISR_vect));
ISR(TIMER1_OVF_vect, ISR_ALIASOF(BADISR_vect));
ISR(SPI_STC_vect, ISR_ALIASOF(BADISR_vect));
ISR(ADC_vect, ISR_ALIASOF(BADISR_vect));
ISR(EE_READY_vect, ISR_ALIASOF(BADISR_vect));
ISR(ANALOG_COMP_vect, ISR_ALIASOF(BADISR_vect));

ISR(TIMER0_OVF_vect)
{
	//	without attribute ISR_NAKED registers get pushed, popped and RETI gets added
	redrawleds	= 1;					// set flag to redraw LEDs
}
ISR(TIMER0_COMPA_vect, ISR_ALIASOF(TIMER0_OVF_vect));
ISR(TIMER0_COMPB_vect, ISR_ALIASOF(TIMER0_OVF_vect));

ISR(WDT_vect)
{
	//	without attribute ISR_NAKED registers get pushed, popped and RETI gets added
	//	watchdog ISR is only used for wakeup
	++watchdog;					// count
	redrawleds	= 1;			// set flag to redraw LEDs
	MCUSR	&= ~(1 << WDRF);	// clear watchdog reset flag
}

/*	defined in rpi-sense.S
ISR(TWI_vect)
{
	twi_busy	|= 1;							// set i2c_busy flag
	wdt_reset();								// reset watchdog timer

	switch(TW_STATUS)
	{

	//	MASTER
	case TW_START:								// START condition transmitted
	case TW_REP_START:							// REPEATED START condition transmitted
		break;

	// MASTER RECEIVER
	case TW_MR_ARB_LOST:						// arbitration lost in SLA+R or NACK
		GOTO	lbl_TWI_RESET;					// stop/reset the TWI system
	case TW_MR_SLA_ACK:							// SLA+R transmitted, ACK received
	case TW_MR_DATA_ACK:						// DATA received, ACK returned
		break;
	case TW_MR_SLA_NACK:						// SLA+R transmitted, NACK received
	case TW_MR_DATA_NACK:						// DATA received, NACK returned
		break;

	// MASTER TRANSMITTER
	case TW_MT_ARB_LOST:						// arbitration lost in SLA+W or DATA
		GOTO	lbl_TWI_RESET;					// stop/reset the TWI system
	case TW_MT_SLA_ACK:							// SLA+W transmitted, ACK received
	case TW_MT_DATA_ACK:						// DATA transmitted, ACK received
		break;
	case TW_MT_SLA_NACK:						// SLA+W transmitted, NACK received
	case TW_MT_DATA_NACK:						// DATA transmitted, NACK received
		break;

	// SLAVE

	// SLAVE RECEIVER
	case TW_SR_ARB_LOST_SLA_ACK:				// arbitration lost in SLA+RW, SLA+W received, ACK returned
	case TW_SR_ARB_LOST_GCALL_ACK:				// arbitration lost in SLA+RW, general call received, ACK returned
		GOTO	lbl_TWI_RESET;					// stop/reset the TWI system
	case TW_SR_SLA_ACK:							// SLA+W received, ACK returned
		i2c_busy	|= I2C_BUSY_ADDRESS;		// next DATA received is register address
	case TW_SR_DATA_ACK:						// data received, ACK returned
	case TW_SR_GCALL_ACK:						// general call received, ACK returned
	case TW_SR_GCALL_DATA_ACK:					// general call data received, ACK returned
		break;
	case TW_SR_DATA_NACK:						// data received, NACK returned
	case TW_SR_GCALL_DATA_NACK:					// general call data received, NACK returned
	case TW_SR_STOP:							// stop or repeated start condition received while selected
		break;

	// SLAVE TRANSMITTER
	case TW_ST_ARB_LOST_SLA_ACK:				// arbitration lost in SLA+RW, SLA+R received, ACK returned
		GOTO	lbl_TWI_RESET;					// stop/reset the TWI system
	case TW_ST_SLA_ACK:							// SLA+R received, ACK returned
		i2c_busy	|= I2C_BUSY_ADDRESS;		// next DATA received is register address
	case TW_ST_DATA_ACK:						// data transmitted, ACK received
		break;
	case TW_ST_DATA_NACK:						// data transmitted, NACK received
	case TW_ST_LAST_DATA:						// last data byte transmitted, ACK received
		break;

	// ERROR CONDITIONS
	case TW_BUS_ERROR:							// illegal START or STOP condition
	case TW_NO_INFO:							// no state information available
		GOTO	lbl_TWI_RESET;					// stop/reset the TWI system

	default:
lbl_TWI_RESET:
		TWCR	|= (1 << TWSTO);				// MASTER generate STOP, SLAVE release to recover from error
	}

	TWCR	|= (1 << TWINT);					// clear TWINT flag and start TWI operation
}
*/

__fuse_t __fuse __attribute__((section (".fuse"))) =
{
	// CKSEL2+CKSEL3 dont exist according to data sheet
	// internal 8MHz oscillator, no DIV8 prescaling
	.low = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),	// FUSE_CKOUT, FUSE_SUT1, FUSE_CKSEL1, FUSE_CKDIV8
	// ICSP enabled, BODLEVEL=2.9V
	.high = (FUSE_SPIEN & FUSE_BODLEVEL2 & FUSE_BODLEVEL0),	// FUSE_RSTDISBL, FUSE_DWEN, FUSE_WDTON, FUSE_EESAVE, FUSE_BODLEVEL1
	.extended = (0xFF),	// FUSE_SELFPRGEN
};

#define watchdog_prescalerbits(wdp)	(((wdp >> 3 &1) << WDP3) | ((wdp >> 2 &1) << WDP2) | ((wdp >> 1 &1) << WDP1) | ((wdp &1) << WDP0))
#define clock_prescalerbits(clkps)	(((clkps >> 3 &1) << CLKPS3) | ((clkps >> 2 &1) << CLKPS2) | ((clkps >> 1 &1) << CLKPS1) | ((clkps &1) << CLKPS0))
#define timer0_prescalerbits(cs0)	(((cs0 >> 2 &1) << CS02) | ((cs0 >> 1 &1) << CS01) | ((cs0 &1) << CS00))
/*	I/O port fundamentals
**	PINx	I/O address
**	DDRx	data direction 0=input, 1=output
**	PORTx	I/O address
**		while DDRxn==0 (input pin)	- writing PORTxn to 1 activates pull-up resistor
**		while DDRxn==1 (output pin)	- writing PORTxn to 1 drives active-high, 0 drives active-low
**		writing PINxn to 1 toggles value of PORTxn	- DDRxn irrelevant
**	NOTE from data sheet:
**		When switching between tri-state ({DDxn, PORTxn} = 0b00) and output high ({DDxn, PORTxn}
**		= 0b11), an intermediate state with either pull-up enabled {DDxn, PORTxn} = 0b01) or output
**		low ({DDxn, PORTxn} = 0b10) must occur. Normally, the pull-up enabled state is fully acceptable,
**		as a high-impedance environment will not notice the difference between a strong high driver
**		and a pull-up. If this is not the case, the PUD bit in the MCUCR Register can be set to disable all
**		pull-ups in all ports.
**		Switching between input with pull-up and output low generates the same problem. The user
**		must use either the tri-state ({DDxn, PORTxn} = 0b00) or the output high state ({DDxn, PORTxn}
**		= 0b11) as an intermediate step.
**		When reading back a software assigned pin value, a nop instruction must be inserted.
*/
int main(void)
{
	uint8_t _looping = 0;					// local counter variable for main working loop

	// define pull-up values
	//MCUCR	|= (1 << PUD);					// disable pull-up resistors
	PORTA	= 0;
	PORTB	= EE_WP;						// enable write protect pull-up resistor
	PORTC	= 0;
	PORTD	= 0;
	// define output pins
	DDRA	= 0;
	DDRB	= EE_WP | FRAME_INT | KEYS_INT;
	DDRC	= LED_SDI | LED_CLKR | LED_LE | LED_OE_N;
	DDRD	= 0xFF;

	/*	configure clocks
	*/
	CLKPR	= (1 << CLKPCE);				// enable CLKPS change
	CLKPR	= clock_prescalerbits(0);		// no clock pre-scaler divisor (8MHz)

	/*	configure TIMER0
	**	use TIMER0 for regularly LED updates wont work, if using SLEEP
	**	TIMER0 wont run in power down mode, but in idle
	**	CS0:	0=disabled, 1=clkIO, 2=clkIO/8, 3=clkIO/64, 4=clkIO/256, 5=clkIO/1024, 6=T0 pin falling, 7=T0 pin rising
	**	TOV0 interrupt with prescaler==5 every 32768ns ca. 30Hz
	*/
	TCNT0	= 0;							// clear timer0 counter register
	OCR0A	= 0;							// clear timer0 output compare register
	OCR0B	= 0;							// clear timer0 output compare register
	TCCR0A	= (0 << CTC0) | timer0_prescalerbits(5);	// TWI clock select clkT0=clkIO/1024 =8kHz, tT0=128ns
	TIFR0	= 0;							// clear interrupt flag register
	TIMSK0	= (0 << OCIE0B) | (0 << OCIE0A) | (0 << TOIE0);	// interrupt enable flags
	//PRR		&= ~(1 << PRTIM0);				// disable PRTIM0 flag bit

	// configure TWI serial I2C interface
	TWBR	= 0xff;							// TWI bit rate
	TWAR	= 0x46 << 1;					// TWI slave address
	TWCR	= (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWIE);
	//PRR		&= ~(1 << PRTWI);				// disable PRTWI flag bit
	i2c_busy	= 0;						// clear I2C operation in progress flag

	/*	watchdog has on chip 128kHz oscillator
	**	prescaler bits 0=2k, 1=4k, 2=8k, 3=16k, 4=32k, 5=64k, 6=128k, 7=256k, 8=512k, 9=1024k
	**	timeout 0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms, 6=1s, 7=2s, 8=4s, 9=8s
	*/
	cli();									// global interrupt disable
	wdt_reset();							// reset watchdog timer
	MCUSR	&= ~(1 << WDRF);				// clear WDRF flag
	WDTCSR	= (1 << WDCE) | (1 << WDE);		// enable watchdog change
	WDTCSR	= (1 << WDIE) | (0 << WDE) | watchdog_prescalerbits(3);	// enable interrupt, disable reset

	/*	configure SLEEP mode
	**	sleep modes - 0b10=power-down, 0b01=ADC noise reduction, 0b00=IDLE
	*/
	SMCR	= (0 << SM1) | (0 << SM0) | (0 << SE);	// enable SLEEP to IDLE mode

	//	disable TIMER1,SPI,ADC clock for power reduction
	PRR		= (0 << PRTWI) | (0 << PRTIM0) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC);

#	ifdef	USE_LEDREAD
	LED_CFG		= (1 << 6);					//	enable LED drivers auto-off
	write_data(LED_CFG, CONF_WRITE);		//	configure LED2472G shift register
	LED_GAIN	= 0;
	write_data(LED_GAIN, GAIN_WRITE);		//	clear LED2472G shift register gain
#	else
	write_data(0x00000040, CONF_WRITE);		//	configure LED2472G shift register
	write_data(0x00000000, GAIN_WRITE);		//	clear LED2472G shift register gain
#	endif

	for(_looping = 0; ; _looping++)
	{
		wdt_reset();						// reset watchdog timer
		sei();								// Set Enable global Interrupts

#ifdef UNUSED_CODE
		//while(0 == (TWCR & (1 << TWINT)))	// TWINT flag not set
		while(0 != i2c_busy)
		{
			//	just wait for I2C operation to finish
			delay(200);						// (clkT0=8kHz, 200/8000=1/40s =25ms
		}
#endif

		draw_loop();						// copy pixels to LED shift register

		registers.ui8[REG_EE_WP]	= ((PORTB & EE_WP) ?0 :1);	// copy EEPROM_WP disabled to I2C data buffer
		registers.ui8[REG_WDTCSR]	= WDTCSR;	// copy register to I2C data buffer
		registers.ui8[REG_SMCR]		= SMCR;		// copy register to I2C data buffer

#		ifndef NDEBUG
		// copy registers to I2C data buffer
		registers.ui8[REG_PINA]		= PINA;
		registers.ui8[REG_DDRA]		= DDRA;
		registers.ui8[REG_PORTA]	= PORTA;
		registers.ui8[REG_PINB]		= PINB;
		registers.ui8[REG_DDRB]		= DDRB;
		registers.ui8[REG_PORTB]	= PORTB;
		registers.ui8[REG_PINC]		= PINC;
		registers.ui8[REG_DDRC]		= DDRC;
		registers.ui8[REG_PORTC]	= PORTC;
		registers.ui8[REG_PIND]		= PIND;
		registers.ui8[REG_DDRD]		= DDRD;
		registers.ui8[REG_PORTD]	= PORTD;
		registers.ui8[REG_MCUSR]	= MCUSR;
		registers.ui8[REG_MCUCR]	= MCUCR;
		registers.ui8[REG_PORTCR]	= PORTCR;
		registers.ui8[REG_PRR]		= PRR;
#		endif	// NDEBUG

#		ifdef	USE_LEDREAD
		if(0 == (_looping & 0x7F))
		{
			//	only every 128 loop runs
			LED_CFG		= read_data(CONF_READ);
			LED_GAIN	= read_data(GAIN_READ);
			LED_ERROR	= read_data(DET_OPEN_SHORT);
			LED_THERMAL	= read_data(THERM_READ);
		}
#		endif

#ifdef USE_SLEEP
		if(0 == i2c_busy && 0 == redrawleds)
		{
			// only if I2C operation ended and redraw unnecessary
			sei();							// enable interrupts before SLEEP
			SMCR	|= (1 << SE);			// enable SLEEP mode
			sleep_cpu();					// sleep and wait for interrupt
			SMCR	&= ~(1 << SE);			// clear SLEEP enable flag
			_NOP();
			_NOP();							// just to be sure a little delay after wake up from SLEEP
		}
#endif
	}

	return(0);								// hopefully we never reach this
}
