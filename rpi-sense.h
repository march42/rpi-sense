/*
** rpi-sense.h - macros, constants, variables and function prototypes
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

#ifndef	_RPI_SENSE_H_
#	define	_RPI_SENSE_H_	__FILE__

#	define	F_CPU	(8*1000*1000ull)

#	define	FW_I2CSLA			0x46
#	define	FW_I2CID			's'
#	define	VERSION				0x03
#	ifndef	NDEBUG
#		define FW_VERSION		(0xA0 | VERSION)			/* debug version */
#	else
#		define FW_VERSION		VERSION						/* release version */
#	endif

#	include <avr/io.h>
#	include <avr/sfr_defs.h>
#	include <util/twi.h>

#	if defined(__ASSEMBLER__)
#		define	ENUMKEYVALUE(key,value)		.equiv key,value

#	else /* !defined(__ASSEMBLER__) */
#		define	ENUMKEYVALUE(key,value)		key=value,

#		include <avr/cpufunc.h>
#		include <avr/interrupt.h>
#		include <avr/sleep.h>
#		include <avr/wdt.h>
#		include	<util/delay.h>

#		ifndef	TRUE
#			define	FALSE	(0==1)
#			define	TRUE	(~FALSE)
#		endif
#	endif

	/*	define used PORT bits
	*/
#	define	LSM_INT_M		PA0
#	define	LSM_INT1_AG		PA1
#	define	LSM_INT2_AG		PA2
#	define	DEBUG_TP1		PA4
#	define	EE_WP			PB0
#	define	DEBUG_TP2		PB1
#	define	ICSP_CS			PB2
#	define	ICSP_MOSI		PB3
#	define	ICSP_MISO		PB4
#	define	ICSP_CLK		PB5
#	define	KEYS_INT		PB6
#	define	FRAME_INT		PB7
#	define	LED_SDO			PC0
#	define	LED_CLKR		PC1
#	define	LED_LE			PC2
#	define	LED_SDI			PC3
#	define	SDA				PC4
#	define	SCL				PC5
#	define	ICSP_RST		PC6
#	define	LED_OE_N		PC7
#	define	COL_EN8			PD0
#	define	COL_EN7			PD1
#	define	COL_EN6			PD2
#	define	COL_EN5			PD3
#	define	COL_EN4			PD4
#	define	COL_EN3			PD5
#	define	COL_EN2			PD6
#	define	COL_EN1			PD7

#	define	I2C_inprogress	0					// I2C transaction in progress
#	define	I2C_modeADDR	1					// slave receiver register address
#	define	I2C_validreg	2					// received valid register data, to be processed inside main()
#	define	I2C_NACK		3					// disable automatic ACKnowledge

/*	bit count values for LED2472G shift register
**	LED_LE must rise after transmitting n-th bit
**	TODO: use le_key for write_data and read_data
*/

	/*	I2C register addresses
	**	register buffer page definition
	**	external variable prototype
	*/
#	if !defined(I2C_PAGES)
#		define	I2C_PAGES	(1)
#	elif (1 < I2C_PAGES) && ((I2C_PAGES * 256) > ((RAMEND - RAMSTART) + 1))
#		error "Your CPU has insufficient SRAM for I2C_PAGES=x register pages"
#	endif
#	if (4 < I2C_PAGES)
#		define	I2C_MAXPAGE	(7)
#	elif (2 < I2C_PAGES)
#		define	I2C_MAXPAGE	(3)
#	elif (1 < I2C_PAGES)
#		define	I2C_MAXPAGE	(1)
#	else // if (0 < I2C_PAGES)
#		define	I2C_MAXPAGE	(0)
#	endif // (I2C_PAGES>4 ?7 :(I2C_PAGES>2 ?3 :(I2C_PAGES -1)))

#	define	ZEROREG			r1
#	define	TEMPREG			r0

#	if defined(__ASSEMBLER__)
		//	.S inline assembler files
#		if defined(TWI_DATA_RAMPY)
			i2caddr			= 28
			i2cpage			= 29
#		else // defined(TWI_DATA_RAMPY)
			i2caddr			= 14
			i2cpage			= 15
#		endif // defined(TWI_DATA_RAMPY)
		i2cflags			= 16

		PARAML				= 24
		PARAMH				= 25

		ENUMKEYVALUE(DAT_LATCH,1)
		ENUMKEYVALUE(CONF_WRITE,3)
		ENUMKEYVALUE(CONF_READ,5)
		ENUMKEYVALUE(GAIN_WRITE,7)
		ENUMKEYVALUE(GAIN_READ,9)
		ENUMKEYVALUE(DET_OPEN,11)
		ENUMKEYVALUE(DET_SHORT,12)
		ENUMKEYVALUE(DET_OPEN_SHORT,13)
		ENUMKEYVALUE(THERM_READ,14)

		ENUMKEYVALUE(REG_PIXELS,0)
		ENUMKEYVALUE(REG_PIXELS_MAX,((8*8*3)-1))
		// data from LED2472G shift register (high byte at high address, evenly aligned)
		ENUMKEYVALUE(REG_LED_CONF,0xC0)		// configuration registers (24+8 bit)
		ENUMKEYVALUE(REG_LED_GAIN,0xC4)		// gain (24+8 bit)
		ENUMKEYVALUE(REG_LED_ERROR,0xC8)	// error open/short detection (24+8 bit)
		ENUMKEYVALUE(REG_LED_THERMAL,0xCC)	// thermal alert (24+8 bit)
		// debugging registers
		ENUMKEYVALUE(REG_PINA,0xE0)
		ENUMKEYVALUE(REG_DDRA,0xE1)
		ENUMKEYVALUE(REG_PORTA,0xE2)
		ENUMKEYVALUE(REG_PINB,0xE3)
		ENUMKEYVALUE(REG_DDRB,0xE4)
		ENUMKEYVALUE(REG_PORTB,0xE5)
		ENUMKEYVALUE(REG_PINC,0xE6)
		ENUMKEYVALUE(REG_DDRC,0xE7)
		ENUMKEYVALUE(REG_PORTC,0xE8)
		ENUMKEYVALUE(REG_PIND,0xE9)
		ENUMKEYVALUE(REG_DDRD,0xEA)
		ENUMKEYVALUE(REG_PORTD,0xEB)
		ENUMKEYVALUE(REG_MCUSR,0xEC)
		ENUMKEYVALUE(REG_MCUCR,0xED)
		ENUMKEYVALUE(REG_PORTCR,0xEE)
		ENUMKEYVALUE(REG_PRR,0xEF)
		// administrative registers
		ENUMKEYVALUE(REG_WAI,0xF0)
		ENUMKEYVALUE(REG_VER,0xF1)
		ENUMKEYVALUE(REG_KEYS,0xF2)
		ENUMKEYVALUE(REG_EE_WP,0xF3)
		ENUMKEYVALUE(REG_WDTCSR,0xF4)
		ENUMKEYVALUE(REG_SMCR,0xF5)
		ENUMKEYVALUE(REG_WDC,0xF6)
		ENUMKEYVALUE(REG_TOV0C,0xF7)

		.extern	registers
		/*	following are defined as registers not as variable
		.extern i2cpage
		.extern i2caddr
		.extern i2cflags
		*/

		.global	draw_loop
		.global delay
		.global read_data
		.global write_data
		.global TWI_vect
		.global write_registers
		.global	check_keys
		.global	SYSTEM_RESET

		.equiv	pixels		, (registers + REG_PIXELS)
		.equiv	keys		, (registers + REG_KEYS)
		.equiv	eewp		, (registers + REG_EE_WP)
		.equiv	wdt_reg		, (registers + REG_WDTCSR)
		.equiv	sm_reg		, (registers + REG_SMCR)
		.equiv	watchdog	, (registers + REG_WDC)
		.equiv	counttmr	, (registers + REG_TOV0C)
#		if	defined(USE_LEDREAD) || defined(USE_LEDWRITE)
			.equiv	LED_CFG		, (registers + REG_LED_CONF)
			.equiv	LED_GAIN	, (registers + REG_LED_GAIN)
			.equiv	LED_ERROR	, (registers + REG_LED_ERROR)
			.equiv	LED_THERMAL	, (registers + REG_LED_THERMAL)
#		endif

#		define	DEC_i2caddr				DEC i2caddr
#		define	INC_i2caddr				INC i2caddr
#		define	CLR_i2cflags			CLR i2cflags
#		define	SET_i2cflag(bit)		SBR i2cflags, (1 << bit)
#		define	SFS_i2cflag(bit)		SBRS i2cflags, bit
#		define	CLR_i2cflag(bit)		CBR i2cflags, (1 << bit)
#		define	SFC_i2cflag(bit)		SBRC i2cflags, bit
#		define	SET_OUTPUT(port,pin)	SBI _SFR_IO_ADDR(port), pin
#		define	CLR_OUTPUT(port,pin)	CBI _SFR_IO_ADDR(port), pin

#	else	/* !defined(__ASSEMBLER__) */
		//	.c/.cpp C/C++ source files

#		if defined(TWI_DATA_RAMPY)
			register union i2creg_enum
			{
				uint8_t		*ptr;
				uint8_t		ui8 [2];
			}	i2creg	asm("r28");
			uint8_t	ld_RAMPY(void);
			void	st_RAMPY(uint8_t DATA);
#			define	i2caddr	(i2creg.ui8[0])
#			define	i2cpage	(i2creg.ui8[1])
#		else // defined(TWI_DATA_RAMPY)
			register uint8_t	i2caddr		asm("r14");
			register uint8_t	i2cpage		asm("r15");
#		endif // defined(TWI_DATA_RAMPY)
#		define	DEC_i2caddr				asm volatile("DEC %0":"=r" (i2caddr):)
#		define	INC_i2caddr				asm volatile("INC %0":"=r" (i2caddr):)

		struct runtimeflag_bits {
			unsigned char inprogress:1;			// I2C transaction in progress
			unsigned char modeADDR:1;			// slave receiver register address
			unsigned char validreg:1;			// received valid register data, to be processed inside main()
			unsigned char NACK:1;				// disable automatic ACKnowledge
			unsigned char bit4:1;
			unsigned char bit5:1;
			unsigned char bit6:1;
			unsigned char bit7:1;
		};
		register struct runtimeflag_bits runflags asm("r16");	// runtime flags
#		define	CLR_i2cflags			asm volatile("CLR %0":"=r" (runflags):)
#		define	SET_i2cflag(bit)		asm volatile("SBR %0,(1 << %1)":"=r"(runflags):"I"(bit))
#		define	CLR_i2cflag(bit)		asm volatile("CBR %0,(1 << %1)":"=r"(runflags):"I"(bit))

#		define set(port,x) port |= (x)
#		define clr(port,x) port &= ~(x)

#		define	SET_OUTPUT(port,pin)	asm volatile("SBI %0, %1" : :"I"(_SFR_IO_ADDR(port)), "I"(pin))
#		define	CLR_OUTPUT(port,pin)	asm volatile("CBI %0, %1" : :"I"(_SFR_IO_ADDR(port)), "I"(pin))

		typedef enum LED2472G_LE_KEY {
			ENUMKEYVALUE(DAT_LATCH,1)
			ENUMKEYVALUE(CONF_WRITE,3)
			ENUMKEYVALUE(CONF_READ,5)
			ENUMKEYVALUE(GAIN_WRITE,7)
			ENUMKEYVALUE(GAIN_READ,9)
			ENUMKEYVALUE(DET_OPEN,11)
			ENUMKEYVALUE(DET_SHORT,12)
			ENUMKEYVALUE(DET_OPEN_SHORT,13)
			ENUMKEYVALUE(THERM_READ,14)
		}	le_key;

		typedef union pagebuffer_enum
		{
			uint8_t		ui8 [0x100];
			uint16_t	ui16[0x100 >>1];
			uint32_t	ui32[0x100 >>2];
		}	I2C_PAGE_BUFFER;
		enum REG_ADDR {
			// LED pixels buffers
			ENUMKEYVALUE(REG_PIXELS,0)
			ENUMKEYVALUE(REG_PIXELS_MAX,191)
			// administrative registers
			ENUMKEYVALUE(REG_WAI,0xF0)
			ENUMKEYVALUE(REG_VER,0xF1)
			ENUMKEYVALUE(REG_KEYS,0xF2)
			ENUMKEYVALUE(REG_EE_WP,0xF3)
			ENUMKEYVALUE(REG_WDTCSR,0xF4)
			ENUMKEYVALUE(REG_SMCR,0xF5)
			ENUMKEYVALUE(REG_WDC,0xF6)
			ENUMKEYVALUE(REG_TOV0C,0xF7)
			// data from LED2472G shift register (high byte at high address, evenly aligned)
			ENUMKEYVALUE(REG_LED_CONF,0xC0)		// configuration registers (24+8 bit)
			ENUMKEYVALUE(REG_LED_GAIN,0xC4)		// gain (24+8 bit)
			ENUMKEYVALUE(REG_LED_ERROR,0xC8)	// error open/short detection (24+8 bit)
			ENUMKEYVALUE(REG_LED_THERMAL,0xCC)	// thermal alert (24+8 bit)
			// debugging registers
			ENUMKEYVALUE(REG_PINA,0xE0)
			ENUMKEYVALUE(REG_DDRA,0xE1)
			ENUMKEYVALUE(REG_PORTA,0xE2)
			ENUMKEYVALUE(REG_PINB,0xE3)
			ENUMKEYVALUE(REG_DDRB,0xE4)
			ENUMKEYVALUE(REG_PORTB,0xE5)
			ENUMKEYVALUE(REG_PINC,0xE6)
			ENUMKEYVALUE(REG_DDRC,0xE7)
			ENUMKEYVALUE(REG_PORTC,0xE8)
			ENUMKEYVALUE(REG_PIND,0xE9)
			ENUMKEYVALUE(REG_DDRD,0xEA)
			ENUMKEYVALUE(REG_PORTD,0xEB)
			ENUMKEYVALUE(REG_MCUSR,0xEC)
			ENUMKEYVALUE(REG_MCUCR,0xED)
			ENUMKEYVALUE(REG_PORTCR,0xEE)
			ENUMKEYVALUE(REG_PRR,0xEF)
		};
#		if !defined(_VARIABLES_C_)
			extern I2C_PAGE_BUFFER	registers[];
#		else
			I2C_PAGE_BUFFER registers[I2C_MAXPAGE +1] __attribute__((address (RAMSTART))) = {
				[0].ui8 = {
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
					0x40, 0x00, 0x00, 0x01,			// LED2472G CFG LL,LH,HL, HH=write marker
					0x00, 0x00, 0x00, 0x01,			// LED2472G GAIN LL,LH,HL, HH=write marker
					0xC8, 0xC9, 0xCA, 0xCB,			// LED2472G ERROR LL,LH,HL, HH=unused
					0xCC, 0xCD, 0xCE, 0xCF,			// LED2472G THERMAL LL,LH,HL, HH=unused
					//	offset 0xD0
					0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
					0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
					//	offset 0xE0
					0xE0, 0xE1, 0xE2,				// PINA,DDRA,PORTA
					0xE3, 0xE4, 0xE5,				// PINB,DDRB,PORTB
					0xE6, 0xE7, 0xE8,				// PINC,DDRC,PORTC
					0xE9, 0xEA, 0xEB,				// PIND,DDRD,PORTD
					0xEC,							// MCUSR
					0xED,							// MCUCR
					0xEE,							// PORTCR
					0xEF,							// PRR
					//	offset 0xF0
					FW_I2CID, FW_VERSION,			// Who-Am-I and firmware version
					0xF2,							// keys
					0xF3,							// EEPROM write protect disabled
					0xF4,							// watchdog timer control register wdt_reg
					0xF5,							// SMCR Sleep Mode Control Register
					0xF6,							// I2C_WDC watchdog interrupt counter
					0xF7,							// TOV0 interrupt counter (+1 every 32768ns)
					0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
				}
			};
#		endif
		//	some defines, to ease access
#		define	reg_ui8(page,pos)	(registers[page].ui8[pos])
#		define	reg_ui16(page,pos)	(registers[page].ui16[pos >>1])
#		define	reg_ui32(page,pos)	(registers[page].ui32[pos >>2])
#		define	pixels				(reg_ui8(0,REG_PIXELS))
#		define	keys				(reg_ui8(0,REG_KEYS))
#		define	eewp				(reg_ui8(0,REG_EE_WP))
#		define	wdt_reg				(reg_ui8(0,REG_WDTCSR))
#		define	sm_reg				(reg_ui8(0,REG_SMCR))
#		define	watchdog			(reg_ui8(0,REG_WDC))
#		define	counttmr			(reg_ui8(0,REG_TOV0C))
#		if	defined(USE_LEDREAD) || defined(USE_LEDWRITE)
#			define	LED_CFG			(reg_ui32(0,REG_LED_CONF))
#			define	LED_GAIN		(reg_ui32(0,REG_LED_GAIN))
#			define	LED_ERROR		(reg_ui32(0,REG_LED_ERROR))
#			define	LED_THERMAL		(reg_ui32(0,REG_LED_THERMAL))
#		endif

		extern void SYSTEM_RESET(void);
		extern void draw_loop(void);
		extern void check_keys(void);
		extern void delay(uint8_t ticks);
		extern void write_data(uint32_t data, le_key type);
		extern uint32_t read_data(le_key type);
		extern void write_registers(void);
		extern uint8_t ld_RAMPY(void);
		extern void st_RAMPY(uint8_t DATA);

#	endif	/* defined(__ASSEMBLER__) */

#	define watchdog_prescalerbits(wdp)	(((wdp >> 3 &1) << WDP3) | ((wdp >> 2 &1) << WDP2) | ((wdp >> 1 &1) << WDP1) | ((wdp &1) << WDP0))
#	define clock_prescalerbits(clkps)	(((clkps >> 3 &1) << CLKPS3) | ((clkps >> 2 &1) << CLKPS2) | ((clkps >> 1 &1) << CLKPS1) | ((clkps &1) << CLKPS0))
#	define timer0_prescalerbits(cs0)	(((cs0 >> 2 &1) << CS02) | ((cs0 >> 1 &1) << CS01) | ((cs0 &1) << CS00))

#	define	SET_EE_WP		SET_OUTPUT(PORTB,EE_WP)	// EE_WP high==EE_WP enabled
#	define	CLR_EE_WP		CLR_OUTPUT(PORTB,EE_WP)	// EE_WP low==EE_WP disabled
#	define	SET_KEYS_INT	SET_OUTPUT(PORTB,KEYS_INT)
#	define	CLR_KEYS_INT	CLR_OUTPUT(PORTB,KEYS_INT)
#	define	SET_FRAME_INT	SET_OUTPUT(PORTB,FRAME_INT)
#	define	CLR_FRAME_INT	CLR_OUTPUT(PORTB,FRAME_INT)
#	define	SET_DEBUG_TP1	SET_OUTPUT(PORTA,DEBUG_TP1)
#	define	CLR_DEBUG_TP1	CLR_OUTPUT(PORTA,DEBUG_TP1)
#	define	SET_DEBUG_TP2	SET_OUTPUT(PORTB,DEBUG_TP2)
#	define	CLR_DEBUG_TP2	CLR_OUTPUT(PORTB,DEBUG_TP2)

#	define	SETFLAG_INPROGRESS	SET_i2cflag(I2C_inprogress)
#	define	CLRFLAG_INPROGRESS	CLR_i2cflag(I2C_inprogress)
#	define	SSFLAG_INPROGRESS	SFS_i2cflag(I2C_inprogress)
#	define	SCFLAG_INPROGRESS	SFC_i2cflag(I2C_inprogress)
#	define	SETFLAG_MODEADDR	SET_i2cflag(I2C_modeADDR)
#	define	CLRFLAG_MODEADDR	CLR_i2cflag(I2C_modeADDR)
#	define	SSFLAG_MODEADDR		SFS_i2cflag(I2C_modeADDR)
#	define	SCFLAG_MODEADDR		SFC_i2cflag(I2C_modeADDR)
#	define	SETFLAG_VALIDREG	SET_i2cflag(I2C_validreg)
#	define	CLRFLAG_VALIDREG	CLR_i2cflag(I2C_validreg)
#	define	SSFLAG_VALIDREG		SFS_i2cflag(I2C_validreg)
#	define	SCFLAG_VALIDREG		SFC_i2cflag(I2C_validreg)
#	define	SETFLAG_NACK		SET_i2cflag(I2C_NACK)
#	define	CLRFLAG_NACK		CLR_i2cflag(I2C_NACK)
#	define	SSFLAG_NACK			SFS_i2cflag(I2C_NACK)
#	define	SCFLAG_NACK			SFC_i2cflag(I2C_NACK)
#endif // _RPI_SENSE_H_

/*	special registers
**	__SREG__		Status register at address 0x3F
**	__SP_H__		Stack pointer high byte at address 0x3E
**	__SP_L__		Stack pointer low byte at address 0x3D
**	__tmp_reg__		Register r0, used for temporary storage
**	__zero_reg__	Register r1, always zero
*/
