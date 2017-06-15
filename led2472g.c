/*
** led2472g.c - code routines for handling of LED2472G driver
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

#if defined(USE_LEDWRITE)
void write_led(uint8_t _looping)
{
	switch(_looping &0x3)
	{
	case 0:
		if(1 == reg_ui8(0, REG_LED_CONF +3))		// check write indicator flag
		{
			//	unused byte3 (bit31-24) used as write indicator flag
			write_data(LED_CFG,CONF_WRITE);
			//reg_ui8(0, REG_LED_CONF +3)		= 0;	// clear write indicator flag
		}
		LED_CFG		= read_data(CONF_READ);
		break;
	case 1:
		LED_ERROR	= read_data(DET_OPEN_SHORT);
		break;
	case 2:
		if(1 == reg_ui8(0, REG_LED_GAIN +3))		// check write indicator flag
		{
			//	unused byte3 (bit31-24) used as write indicator flag
			write_data(LED_GAIN,GAIN_WRITE);
			//reg_ui8(0, REG_LED_GAIN +3)		= 0;	// clear write indicator flag
		}
		LED_GAIN	= read_data(GAIN_READ);
		break;
	case 3:
		LED_THERMAL	= read_data(THERM_READ);
		break;
	}
}
#endif // defined(USE_LEDWRITE)

#if	defined(USE_LEDREAD) || defined(USE_LEDWRITE)
void read_led(uint8_t _looping)
{
	if(0xFF == _looping || 0 == (_looping &0x3))
		LED_CFG		= read_data(CONF_READ);
	if(0xFF == _looping || 1 == (_looping &0x3))
		LED_ERROR	= read_data(DET_OPEN_SHORT);
	if(0xFF == _looping || 2 == (_looping &0x3))
		LED_GAIN	= read_data(GAIN_READ);
	if(0xFF == _looping || 3 == (_looping &0x3))
		LED_THERMAL	= read_data(THERM_READ);
}
#endif // USE_LEDREAD || USE_LEDWRITE
