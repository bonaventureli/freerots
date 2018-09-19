/*
 * FreeRTOS Kernel V10.1.0
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes. */
#include "partest.h"

#define partstNUM_LEDs	2

#define LED0_MASK		( ( unsigned short ) 0x05 )
#define LED1_MASK		( ( unsigned short ) 0x03 )

static const unsigned short xLEDs[ partstNUM_LEDs ] = { LED0_MASK, LED1_MASK };

/*-----------------------------------------------------------*/

void vParTestInitialise( void )
{
	/* LED Port Initialization */
	//PMCM &= ~( LED0_MASK | LED1_MASK );
		/* LED Port Initialization */
	//PMCM &= ~( LED0_MASK | LED1_MASK );
	PMC10 &= ~ 1<<3;//Port mode
  PM10 &= ~(1<<3);//Output
	//P10 &= ~(1<<3);//low level
	P10 |=  (1<<3);//high level
	
	PMC8 &= ~(1<<5);
  PM8 &= ~ (1<<5);
	P8 |= (1<<5);//high level
}
/*-----------------------------------------------------------*/

void vParTestSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue )
{
unsigned portBASE_TYPE uxLEDMask;

	if( uxLED < partstNUM_LEDs )
	{
		uxLEDMask = xLEDs[ uxLED ];
		
		taskENTER_CRITICAL();
		{
			if( xValue )
			{
				P8 &= ~(1<<uxLEDMask);//low level
				P10 &= ~(1<<uxLEDMask);//low level
			}
			else
			{
				P8 |= (1<<uxLEDMask);//high level
				P10 |= (1<<uxLEDMask);//high level
			}
		}
		taskEXIT_CRITICAL();
	}
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned portBASE_TYPE uxLED )
{
unsigned portBASE_TYPE uxLEDMask;

	if( uxLED < partstNUM_LEDs )
	{
		uxLEDMask = xLEDs[ uxLED ];
		
		taskENTER_CRITICAL();
		{
			if( P8 & uxLEDMask )
			{
				P8 &= ~uxLEDMask;
			}
			else
			{
				P8 |= uxLEDMask;
			}
		}
		taskEXIT_CRITICAL();
	}
}

