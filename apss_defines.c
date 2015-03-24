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


/****************************************************************************
 * Includes
 ****************************************************************************/
#include "apss_defines.h"


/****************************************************************************
 * Defines
 ****************************************************************************/

/** @brief Holds Software Revision Code 
 *  
 *  Major = 4 MSB 
 *  Minor = 4 LSB
 */
#define APSS_REV_MAJOR 0
#define APSS_REV_MINOR 5
uint8_t G_apssSoftwareRev  = ((APSS_REV_MAJOR << 4) + APSS_REV_MINOR);


/****************************************************************************
 * Globals
 ****************************************************************************/

/* @brief Holds the mask for interrupts that have occured via IRQ since the last read */
uint8_t G_gpioIrqInterrupt[NUM_GPIO_PORTS]          = {0x00,0x00};

/* @brief Holds the soft OC pin status */
uint8_t  G_softOcPins = 0x00; 

//----------------------------------------------------------------------------
//
// TI F28035 Fakeouts
//-----------------------------------------------

// TODO:  TI F28035 Fake of GPIO Registers
uint8_t G_gpioPort[NUM_GPIO_PORTS]          = {0x11,0x22};
uint8_t G_gpioIoConfig[NUM_GPIO_PORTS]      = {0x00,0x00};
uint8_t G_gpioDriveConfig[NUM_GPIO_PORTS]   = {0xFF,0xFF};
uint8_t G_gpioInterruptMask[NUM_GPIO_PORTS] = {0x00,0x00};


// TODO:  TI F28035 Fake Values of ADC Channels
uint16_t G_adcChan[NUM_ADC_CHANNELS] =
{
  0x0000, 
  0x0111, 
  0x0222, 
  0x0333, 
  0x0444, 
  0x0555, 
  0x0666, 
  0x0777, 
  0x0888, 
  0x0999, 
  0x0aaa, 
  0x0bbb, 
  0x0ccc, 
  0x0ddd, 
  0x0eee, 
  0x0fff, 
};

// TODO:  TI F28035 Fake Values of PWM Channels
uint8_t G_pwmChan[NUM_PWM_CHANNELS] = 
{
  0x80,
  0x80,
  0x80,
  0x80,
  0x80,
  0x80,
  0x80,
  0x80,
};

//----------------------------------------------------------------------------

