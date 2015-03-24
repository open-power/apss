/** Copyright (c) 2009-2011, Texas Instruments Incorporated
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
*/
#ifndef PWM_H_
#define PWM_H_

/****************************************************************************
 * Includes
 ****************************************************************************/

/****************************************************************************
 * Defines
 ****************************************************************************/
// 
// 
// ADC_SAMPLE_PERIOD is used to configure ePWM1 in 
// up count mode with TBPRD = 1/2 SYSCLKOUT = 30 MHz
// 
//      An ADC SOC will be started every ePWM1 period. 
//      For a 20 KHz sampling rate:
//      -> Tpwm = (TBPRD + 1) x (1/30 MHz)
//      -> TBPRD = (1/20KHz)x(30MHz)-1 = 1499 
// 
#define ADC_SAMPLE_PERIOD   1499 


/****************************************************************************
 * Globals
 ****************************************************************************/

/****************************************************************************
 * Functions
 ****************************************************************************/
void init_epwm1(void);
void init_epwm2(void);
void init_epwm3(void);
void init_epwm4(void);
void init_epwm5(void);

void start_epwm(void);

void pwmDoPwmWrites(void);

interrupt void epwm5_isr(void) ;

#endif /*PWM_H_*/
