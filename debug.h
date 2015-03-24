/*
Copyright (c) 2009-2011, Texas Instruments Incorporated
Copyright (c) 2012-2014, International Business Machines Corp.
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
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
#ifndef DEBUG_H_
#define DEBUG_H_

/****************************************************************************
 * Includes
 ****************************************************************************/
#include "apss_defines.h"
#include "DSP2803x_Device.h"
#include "CLAShared.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
typedef struct
{
  union
  {
    struct fields
    {	
    uint32_t enable_debug_latch_0  :1;
    uint32_t enable_debug_latch_1  :1;
    uint32_t enable_debug_latch_2  :1;  	
    uint32_t adc_chan_stick        :1;
    uint32_t pwm_chan_stick        :1;
    uint32_t gpio_port_stick       :1;
    uint32_t bypass_iir_filter     :1;
    uint32_t iir2pwm0_7            :1;
    uint32_t iir2pwm8_15           :1;
    }bit;
    uint32_t word;
  }fields;
} debugMode_t; 

/****************************************************************************
 * Globals
 ****************************************************************************/
extern debugMode_t G_debug;

/****************************************************************************
 * Functions
 ****************************************************************************/
void debugModeEnable(uint8_t i_data);

#endif /*DEBUG_H_*/
