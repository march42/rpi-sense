/*
** rpi-sense-twi.c - Atmel TWI macros, variables and ISR code
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

#	include	"rpi-sense.h"
#	include <stdio.h>
#	include <stdlib.h>

/*	DEFINEs for code definition
**	TWI_VECTOR_M	TWI_vect for MASTER
**	TWI_VECTOR_S	TWI_vect for SLAVE
*/

/*	check given register address if valid for writing
*/
#		if !defined(I2C_VALIDATE_ADDRESS)
#			define	TWI_SRA_valid(address) (TRUE)
#		else // defined(I2C_VALIDATE_ADDRESS)
uint8_t TWI_SRA_valid(uint8_t address)
{
	//	check for valid receiver address
	if(REG_PIXELS_MAX >= address)
	{
		return(TRUE);
	}

#	if (0 < I2C_MAXPAGE)
		//	page>0 is always valid
		if(0 < i2cpage)
		{
			return(TRUE);
		}
#	endif

#	if	defined(USE_REGWRITE)
		if(REG_WDTCSR == address)
		{
			return(TRUE);
		}
		if(REG_SMCR == address)
		{
			return(TRUE);
		}
		if((REG_PINA & 0xF0) == (address &0xF0))
		{
			return(TRUE);
		}
#	endif

#	if	defined(USE_LEDWRITE)
		if((REG_LED_CONF & 0xF0) == (address &0xF8))
		{
			return(TRUE);
		}
#	endif

	return(FALSE);
}
#		endif // defined(I2C_VALIDATE_ADDRESS)

/*	check given register address if valid for reading
*/
#		if !defined(I2C_VALIDATE_ADDRESS)
#			define	TWI_address_valid(address) (TRUE)
#		else // defined(I2C_VALIDATE_ADDRESS)
uint8_t TWI_address_valid(uint8_t address)
{
	if(REG_PIXELS_MAX >= address)
	{
		return(TRUE);
	}
	if(REG_WAI == (address & REG_WAI))	//if((REG_WAI <= address) && (REG_TOV0C >= address))
	{
		return(TRUE);
	}

#	if (0 < I2C_MAXPAGE)
		//	page>0 is always valid
		if(0 < i2cpage)
		{
			return(TRUE);
		}
#	endif

#	if	!defined(NDEBUG) || defined(USE_REGWRITE)
		if(REG_PINA == (address & 0xF0))
		{
			return(TRUE);
		}
#	endif

#	if	defined(USE_LEDREAD) || defined(USE_LEDWRITE)
		if(REG_LED_CONF == (address & 0xF0))
		{
			return(TRUE);
		}
#	endif

	return(FALSE);
}
#		endif // defined(I2C_VALIDATE_ADDRESS)

#	if defined(TWI_VECTOR_M) || defined(TWI_VECTOR_S)
		// clear TWEA and TWINT flag and start TWI operation
#		define	TWI_NACK		{ i2cflags.NACK = 1; TWCR &= ~_BV(TWEA); }
		// clear TWINT flag and start TWI operation
#		define	TWI_ACK			{ TWCR |= _BV(TWEA) | _BV(TWINT); }
#		define	CLR_TWINT		{ TWCR = _BV(TWINT); }
ISR(TWI_vect)
{
	uint8_t		DATA;

	i2cflags.inprogress	= 1;					// I2C transaction in progress
	i2cflags.NACK		= 0;					// clear NACK flag

	DATA				= TW_STATUS;			// load status and mask out pre-scaler bits
	if(TW_NO_INFO == DATA)						// no information on condition
	{
		//	just ignore and do nothing
	}

	// SLAVE
#if defined(TWI_VECTOR_S)
	else if(TW_SR_DATA_ACK == DATA)				// data received, ACK returned
	{
		DATA				= TWDR;				// load DATA from hardware

		if(0 != i2cflags.modeADDR)
		{
			TWI_ACK;							// clear TWINT flag and restart TWI operation
			i2cflags.modeADDR	= 0;			// clear DATA==ADDRESS receive flag
			i2caddr				= DATA - 1;		// copy received DATA -1 to address register
			//	i2caddr gets incremented before read/write so it needs to be DATA-1
		}

		else if(REG_WAI == ++i2caddr)			// increment and check address
		{
			if(0x42 == DATA)
			{
				TWI_NACK;						// clear TWINT flag and continue TWI operation
				SYSTEM_RESET();					// execute SYSTEM_RESET
			}
#		if (0 < I2C_MAXPAGE)
			else if(0xA0 == (DATA & ~I2C_MAXPAGE))	// mask out I2C_MAXPAGE bits (1,3,7)
			{
				TWI_ACK;					// clear TWINT flag and continue TWI operation
				i2cpage	= (DATA & I2C_MAXPAGE);	// set I2C page address
				reg_ui8(i2cpage,REG_WAI)	= FW_I2CID +i2cpage;	// set Who-Am-I response
			}
#		endif // (0 < I2C_MAXPAGE)
			else
			{
				TWI_NACK;						// clear TWINT flag and continue TWI operation
			}
		}
		else if(TWI_SRA_valid(i2caddr))			// validate address
		{
			TWI_ACK;							// clear TWINT flag and continue TWI operation
			registers[i2cpage].ui8[i2caddr]	= DATA;		// store value to buffer
			i2cflags.validreg	= 1;			// new DATA received and stored to buffer
		}
		else
		{
			TWI_NACK;							// clear TWINT flag and continue TWI operation
		}
	}

	else if(TW_ST_DATA_ACK == DATA)				// data transmitted, ACK received
	{
lbl_TWI_ST_DATA_ACK:
#	if (0 < I2C_MAXPAGE)
		if(REG_WAI == ++i2caddr)
		{
			TWDR	= FW_I2CID + i2cpage;		// I2C Who-Am-I firmware ID
		}
		else if(0 < i2cpage)					// only check on first page
		{
			TWDR	= reg_ui8(i2cpage,i2caddr);	// load value from buffer to transmission buffer
			TWI_ACK;
		}
#	else // (0 < I2C_MAXPAGE)
		if(REG_WAI == ++i2caddr)
		{
			TWDR	= FW_I2CID;					// I2C Who-Am-I firmware ID
			// no need to add i2cpage on single page firmware
		}
#	endif // (0 < I2C_MAXPAGE)
		else if(TWI_address_valid(++i2caddr))	// increment and validate address
		{
			TWDR	= reg_ui8(0,i2caddr);		// load value from buffer to transmission buffer
			// no need to use i2cpage, only first page gets checked
			TWI_ACK;							// continue TWI
			if(REG_KEYS == i2caddr)				// joystick status
			{
				CLR_KEYS_INT;					// clear KEYS_INT interrupt line to Raspberry Pi
			}
		}
		else
		{
			TWDR	= 0x00;						// clear transmission buffer
			TWI_NACK;
		}
	}
	else if(TW_ST_SLA_ACK == DATA)				// SLA+R received, ACK returned
	{
		goto lbl_TWI_ST_DATA_ACK;
	}
	else if(TW_SR_SLA_ACK == DATA)				// SLA+W received, ACK returned
	{
		i2cflags.modeADDR	= 1;				// next DATA received is register address
		TWI_ACK;
	}

	else if(TW_ST_LAST_DATA == DATA)			// last data byte transmitted, ACK received
	{
		TWI_ACK;
	}
	else if(TW_SR_STOP == DATA)					// stop or repeated start condition received while selected
	{
		i2cflags.inprogress	= 0;				// clear inprogress flag
		TWI_ACK;
	}

	else if(TW_ST_DATA_NACK == DATA)			// data transmitted, NACK received
	{
		goto lbl_TWI_ST_DATA_ACK;
	}
	else if(TW_SR_DATA_NACK == DATA)			// data received, NACK returned
	{
		TWI_ACK;
	}

#	if defined(TWI_SLAVE_GCALL)
		else if(TW_SR_ARB_LOST_GCALL_ACK == TW_STATUS)	// arbitration lost in SLA+RW, general call received, ACK returned
		{
			goto	lbl_TWI_RESET;				// stop/reset the TWI system
		}

		else if(TW_SR_GCALL_DATA_NACK == TW_STATUS)	// general call data received, NACK returned
		{
			TWI_NACK;
		}

		else if(TW_SR_GCALL_ACK == TW_STATUS)	// general call received, ACK returned
		{
			TWI_NACK;
		}

		else if(TW_SR_GCALL_DATA_ACK == TW_STATUS)	// general call data received, ACK returned
		{
			TWI_NACK;
		}
#	endif // defined(TWI_SLAVE_GCALL)

	else if(TW_BUS_ERROR == DATA)				// illegal START or STOP condition
	{
lbl_TWI_RESET:	// stop/reset the TWI system
		do
		{
			TWCR	|= _BV(TWSTO);				// MASTER generate STOP, SLAVE release to recover from error
			// in SLAVE mode this will not generate a STOP condition, but the TWI returns to unaddressed and releases lines
			_delay_us(5);						// 1 byte at bus clock 400kHz is about 22.5ys
		}
		while(0 != (TWCR & _BV(TWSTO)));
		// wait for bit cleared and TWI has been reinitialized
		CLR_i2cflags;							// clear all flags
		TWI_ACK;
	}

	else if(TW_SR_ARB_LOST_SLA_ACK == DATA)		// arbitration lost in SLA+RW, SLA+W received, ACK returned
	{
		goto	lbl_TWI_RESET;					// stop/reset the TWI system
	}

	else if(TW_ST_ARB_LOST_SLA_ACK == DATA)		// arbitration lost in SLA+RW, SLA+R received, ACK returned
	{
		goto	lbl_TWI_RESET;					// stop/reset the TWI system
	}
#endif // defined(TWI_VECTOR_S)

#	if defined(TWI_VECTOR_M)
/*
	else if( == TW_STATUS)			// 
	{
	}

	//	MASTER
	case TW_START:								// START condition transmitted
	case TW_REP_START:							// REPEATED START condition transmitted
		break;

	// MASTER RECEIVER
	case TW_MR_ARB_LOST:						// arbitration lost in SLA+R or NACK
		goto	lbl_TWI_RESET;					// stop/reset the TWI system
	case TW_MR_SLA_ACK:							// SLA+R transmitted, ACK received
	case TW_MR_DATA_ACK:						// DATA received, ACK returned
		break;
	case TW_MR_SLA_NACK:						// SLA+R transmitted, NACK received
	case TW_MR_DATA_NACK:						// DATA received, NACK returned
		break;

	// MASTER TRANSMITTER
	case TW_MT_ARB_LOST:						// arbitration lost in SLA+W or DATA
		goto	lbl_TWI_RESET;					// stop/reset the TWI system
	case TW_MT_SLA_ACK:							// SLA+W transmitted, ACK received
	case TW_MT_DATA_ACK:						// DATA transmitted, ACK received
		break;
	case TW_MT_SLA_NACK:						// SLA+W transmitted, NACK received
	case TW_MT_DATA_NACK:						// DATA transmitted, NACK received
		break;
*/
#	endif // defined(TWI_VECTOR_M)

	// clear TWINT flag
	else	//if(0 != (TWSR & _BV(TWINT)))
	{
		TWI_NACK;								// NACK and continue TWI system
	}
}
#	endif // defined(TWI_VECTOR_M) || defined(TWI_VECTOR_S)

