/** 
Copyright (c) 2012-2014, International Business Machines Corp.
Copyright (c) 2009-2011, Texas Instruments Incorporated
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met: 
1) Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer: 2) Redistributions in binary 
form must reproduce the above copyright notice, this list of conditions and the 
following disclaimer in the documentation and/or other materials provided with the 
distribution, and; 3) Neither the name of the Texas Instruments nor the names of its 
contributors may be used to endorse or promote products derived from this software 
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS, 
STATUTORY OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
All Rights Reserved
 */


#ifndef _APSS_DEFINES_H
#define _APSS_DEFINES_H

/****************************************************************************
 * Defines
 ****************************************************************************/

/** Number of GPIO Ports supported */
#define NUM_GPIO_PORTS 2

/** Number of ADC Channels supported */
#define NUM_ADC_CHANNELS 16

/** Number of PWM Channels supported */
#define NUM_PWM_CHANNELS 8

/** The default I2C Address of the APSS */
#define APSS_I2C_ADDRESS 0x70


/****************************************************************************
 * Macros
 ****************************************************************************/

/** Disable interrupts Globally for atomic operations */
#define DISABLE_ALL_INTERRUPTS() DINT

/** Enable interrupts Globally after atomic operations */
#define ENABLE_ALL_INTERRUPTS()  EINT

/** Enable or Disable Debug statements */
#define DEBUG (void*)

#define __LITTLE_ENDIAN 1 


/****************************************************************************
 * Typedefs
 ****************************************************************************/
typedef int 	            bool;
typedef signed char 	    int8_t;
typedef unsigned char 	    uint8_t;
typedef signed short int 	int16_t;
typedef unsigned short int 	uint16_t;
typedef signed long int 	int32_t;
typedef unsigned long int 	uint32_t;


/****************************************************************************
 * Globals
 ****************************************************************************/

/** @brief Holds Software Revision Code */
extern uint8_t G_apssSoftwareRev;

/* @brief Holds the mask for interrupts that have occured via IRQ since the last read */
extern uint8_t G_gpioIrqInterrupt[NUM_GPIO_PORTS];

//----------------------------------------------------------------------------
//
// TI F28035 Fakeouts
//-----------------------------------------------
extern uint8_t  G_gpioPort[NUM_GPIO_PORTS];          
extern uint8_t  G_gpioIoConfig[NUM_GPIO_PORTS];      
extern uint8_t  G_gpioDriveConfig[NUM_GPIO_PORTS];  
extern uint8_t  G_gpioInterruptMask[NUM_GPIO_PORTS]; 

extern uint8_t  G_softOcPins; 

extern uint16_t G_adcChan[NUM_ADC_CHANNELS];
extern uint8_t  G_pwmChan[NUM_PWM_CHANNELS];

//----------------------------------------------------------------------------


#endif
