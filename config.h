/*
** config.h
**
** Created: 12.06.2017 12:42:48
**  Author: Marc Hefter
*/


#ifndef CONFIG_H_
#define CONFIG_H_

	/*	general configurations
	**	TWI_DATA_RAMPY	always use RAMPY registers for I2C data buffers
	**	USE_SLEEP		never use SLEEP to prevent flickering LEDs
	**	I2C_PAGES		restrict to single page I2C registers
	*/
#	define TWI_DATA_RAMPY
//#	define USE_SLEEP
#	define I2C_PAGES (1)

#	if defined(DISABLE_EXTRAS)
		//	no code extra should be enabled
#		undef USE_SLEEP
#		define F_I2C (100ull*1000)
#		undef I2C_VALIDATE_ADDRESS
#		undef USE_LEDREAD
#		undef USE_REGWRITE
#		undef USE_LEDWRITE
#		undef I2C_PAGES
#	else // defined(DISABLE_EXTRAS)
#		define F_I2C (400ull*1000)
//#		define I2C_VALIDATE_ADDRESS
#		if defined(NDEBUG)
			//	release build
#			define USE_LEDREAD
#		else // defined(NDEBUG)
			//	debug build
#			define USE_REGWRITE
#			define USE_LEDWRITE
#		endif // defined(NDEBUG)
#	endif // defined(DISABLE_EXTRAS)

#	define I2C_HIGHSPEED (100000ull < F_I2C)

#endif /* CONFIG_H_ */