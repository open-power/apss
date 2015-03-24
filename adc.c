/** 
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
 *
 */

/****************************************************************************
 * Includes
 ****************************************************************************/
#include "DSP2803x_Device.h"
#include "DSP2803x_Examples.h"

#include "CLAShared.h"

#include "adc.h"

#include "apss_defines.h"

#include "debug.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Globals
 ****************************************************************************/

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* Initialize the ADC Module   
*
* EPWM1 will be used to generate the ADC Start of conversion (SOC)
*
* @param  none
*
* @return none 
*/
void init_adc(void)
{

   // Assumes ADC clock is already enabled in InitSysCtrl();

   // 
   // Call the InitAdc function in the DSP2803x_Adc.c file
   //
   // This function calibrates and powers up the ADC to 
   // into a known state.
   //
   InitAdc();  
   
   //
   // Specific ADC configuration for this example
   //
   // ADC interrupt will trigger early - before the ADC conversion starts
   // Enable ADCINT1
   // Disable ADC 1 continuous mode
   // Set the SOC1 channel select to ADCINTA0.  
   // The interrupt triggers task 1 which is where the CLA IIR filter is located
   EALLOW;

   // ADC interrupt comes early (end of sample window)
   // SOC0 will trigger ADCINT1
   // Enable ADCINT1
   // Disable ADCINT1 Continuous mode	    
   //
   AdcRegs.ADCCTL1.bit.INTPULSEPOS = 0;
   AdcRegs.INTSEL1N2.bit.INT1SEL   = 0;
   AdcRegs.INTSEL1N2.bit.INT1E     = 1;	    
   AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;
   
   // set SOC0 channel select to ADCINA0, SOC1->ADCINA1, etc.
   // set SOC[15:0] start trigger on EPWM1A interrupt  	    
   // set SOC[15:0] S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
   //
   AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0;     
   AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC1CTL.bit.CHSEL    = 1;     
   AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC2CTL.bit.CHSEL    = 2;     
   AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC3CTL.bit.CHSEL    = 3;     
   AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC4CTL.bit.CHSEL    = 4;     
   AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC4CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC5CTL.bit.CHSEL    = 5;     
   AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC5CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC6CTL.bit.CHSEL    = 6;     
   AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC6CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC7CTL.bit.CHSEL    = 7;     
   AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC7CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC8CTL.bit.CHSEL    = 8;     
   AdcRegs.ADCSOC8CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC8CTL.bit.ACQPS    = 6;
   	    
   AdcRegs.ADCSOC9CTL.bit.CHSEL    = 9;     
   AdcRegs.ADCSOC9CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC9CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC10CTL.bit.CHSEL    = 10;     
   AdcRegs.ADCSOC10CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC10CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC11CTL.bit.CHSEL    = 11;     
   AdcRegs.ADCSOC11CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC11CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC12CTL.bit.CHSEL    = 12;     
   AdcRegs.ADCSOC12CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC12CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC13CTL.bit.CHSEL    = 13;     
   AdcRegs.ADCSOC13CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC13CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC14CTL.bit.CHSEL    = 14;     
   AdcRegs.ADCSOC14CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC14CTL.bit.ACQPS    = 6;

   AdcRegs.ADCSOC15CTL.bit.CHSEL    = 15;     
   AdcRegs.ADCSOC15CTL.bit.TRIGSEL  = 0x0D;     
   AdcRegs.ADCSOC15CTL.bit.ACQPS    = 6;

   EDIS;
}          

/**
* Populate the G_adcChan array with ADC results   
*
* 
*
* @param  none
*
* @return none 
*/
void adcDoPopulateArray(void)
{
    uint8_t l_idx = 0;

    if(1 == G_debug.fields.bit.bypass_iir_filter)
    {
        // ---------------------------------------------------------
        // If Debug flag set, Populate with Raw ADC values
        // ---------------------------------------------------------
        G_adcChan[0]  = AdcResult.ADCRESULT0;
        G_adcChan[1]  = AdcResult.ADCRESULT1;
        G_adcChan[2]  = AdcResult.ADCRESULT2;
        G_adcChan[3]  = AdcResult.ADCRESULT3;
        G_adcChan[4]  = AdcResult.ADCRESULT4;
        G_adcChan[5]  = AdcResult.ADCRESULT5;
        G_adcChan[6]  = AdcResult.ADCRESULT6;
        G_adcChan[7]  = AdcResult.ADCRESULT7;
        G_adcChan[8]  = AdcResult.ADCRESULT8;
        G_adcChan[9]  = AdcResult.ADCRESULT9;
        G_adcChan[10] = AdcResult.ADCRESULT10;
        G_adcChan[11] = AdcResult.ADCRESULT11;
        G_adcChan[12] = AdcResult.ADCRESULT12;
        G_adcChan[13] = AdcResult.ADCRESULT13;
        G_adcChan[14] = AdcResult.ADCRESULT14;
        G_adcChan[15] = AdcResult.ADCRESULT15;			
    }
    else if (1 == G_debug.fields.bit.adc_chan_stick)
    {
        // ---------------------------------------------------------
        // If set, and IIR bypassed, then populate as chan[3] = 0x0333
        // ---------------------------------------------------------
        for(l_idx=0; l_idx<NUM_ADC_CHANNELS; l_idx++)
        {
            G_adcChan[l_idx] = (l_idx + l_idx*0x10 + l_idx*0x100);
        }		
    }
    else
    {
        // ---------------------------------------------------------
        // If no debug flags are set, populate with values from IIR 
        // ---------------------------------------------------------
        for(l_idx=0; l_idx<NUM_ADC_CHANNELS; l_idx++)
        {
            G_adcChan[l_idx] = (uint16_t) yn_array[l_idx];
        }
    }
}

