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

/****************************************************************************
 * Includes
 ****************************************************************************/
#include "apss_defines.h"
#include "debug.h"
#include "spi.h"
#include "cla.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Globals
 ****************************************************************************/
debugMode_t G_debug;

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* Enable Debug Modes for APS
*
* These bits are useless now, but may be used for other purposes.
* To enable these bits, you must send
*    I2C:  [START][ADDR][FE][56][STOP]
*    I2C:  [START][ADDR][FE][A5][STOP]
*    I2C:  [START][ADDR][FE][78][STOP]
* before you can set the bits you want to send
*    I2C:  [START][ADDR][FE][XX][STOP]
*
* @param  none
*
* @return none 
*/
void debugModeEnable(uint8_t i_data)
{

	if(G_debug.fields.bit.enable_debug_latch_0
	   && G_debug.fields.bit.enable_debug_latch_1
	   && G_debug.fields.bit.enable_debug_latch_2)
	{
	switch(i_data)
	{
		case 0x00:
		    // After sending this, you will have to resend the latch bits
		    G_debug.fields.word = 0;  break;
		case 0x01:
		    // After sending this, you won't have to resend the latch bits
		    G_debug.fields.word = 0;  
		    G_debug.fields.bit.enable_debug_latch_0 = 1;
		    G_debug.fields.bit.enable_debug_latch_1 = 1;
		    G_debug.fields.bit.enable_debug_latch_2 = 1;
		    break;
	    case 0x02:
	        G_debug.fields.bit.adc_chan_stick    = 1;  break;
	    case 0x03:
	        G_debug.fields.bit.pwm_chan_stick    = 1;  break;
	    case 0x04:
	        G_debug.fields.bit.gpio_port_stick   = 1;  break;
	    case 0x05:
	        G_debug.fields.bit.bypass_iir_filter = 1;  break;
	    case 0x06:
	        G_debug.fields.bit.iir2pwm8_15 = 0;
	        G_debug.fields.bit.iir2pwm0_7  = 1;  
	        break;
	    case 0x07:
	        G_debug.fields.bit.iir2pwm0_7 = 0;
	        G_debug.fields.bit.iir2pwm8_15  = 1;  
	        break;
        case 0x08:
            ENABLE_ALL_INTERRUPTS(); //JRK testing
            break;
        case 0x09:
            pInitialyn_array = &yn_array[0]; //JRK testing
	        break;
        case 0x0A:
            SpiaRegs.SPICCR.bit.SPISWRESET=0; //reset spi
            SpiaRegs.SPICCR.bit.SPISWRESET=1; //resume spi
            break;
        case 0x0B:
            spi_fifo_init();
            break;
        case 0x0C:
        	// Clear the ADC interrupt flag so the next SOC (start of conversion) can occur
			AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

			// Peform the necessary clean up in the PIE to allow another interrupt can be taken
			// ADC related PIE acknowledgment
			PieCtrlRegs.PIEACK.bit.ACK1 = 1 ;
			PieCtrlRegs.PIEACK.bit.ACK10 = 1 ;
			// CLA related PIE acknowledgment
			PieCtrlRegs.PIEACK.bit.ACK11 = 1 ;

			// Setup the IIR filter set for the next ADC conversion set.  These pointers are in the
			// CPUtoCLA Message RAM.  It is much easier to reinitialize the pointers in the main CPU.
			pINITIALIIRFILTER = &IIRFILTERSET.Channels[0] ;
			pInitialADCResult = (Uint16 *)&AdcResult.ADCRESULT0 ;
			pInitialyn_array = &yn_array[0] ;
			pInitialxn_array = &xn_array[0] ;

			// Signal the CLA to copy these to its working copy in its own Data RAM.
			Cla1ForceTask8();
			break;
        case 0x0D:
        	EALLOW;
        	//
		   // Task 1 has the option to be started by either EPWM1_INT or ADCINT1
		   // In this case we will allow ADCINT1 to start CLA Task 1
		   //

		   Cla1Regs.MPISRCSEL1.bit.PERINT1SEL = CLA_INT1_ADCINT1;

		   Cla1Regs.MMEMCFG.bit.PROGE = 1;
		   Cla1Regs.MMEMCFG.bit.RAM1E = 1;

		   //
		   // Enable the IACK instruction to start a task
		   // Enable the CLA interrupt 8 and interrupt 1
		   //

		   Cla1Regs.MCTL.bit.IACKE = 1;
		   Cla1Regs.MIER.all = (M_INT8 | M_INT1);
			EDIS;
			break;
        case 0x0E:
        	EALLOW;
        	Cla1Regs.MCTL.bit.SOFTRESET = 1;
        	EDIS;
        	break;
        case 0x0F:
			EALLOW;
			Cla1Regs.MCTL.bit.HARDRESET = 1;
			EDIS;
			break;
        case 0x10:
        	init_cla();
        	break;
        case 0x11:
		   //
		   // Force CLA task 8 using the IACK instruction
		   // Task 8 will initialize the IIR Biquad.
		   // No need to wait, the task will finish by the time
		   // we configure the ePWM and ADC modules
		   //
		   Cla1ForceTask8();
		   break;
        case 0x12:
		   EALLOW;
		   Cla1Regs.MVECT1 = (Uint16) (&Cla1Task1 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   Cla1Regs.MVECT2 = (Uint16) (&Cla1Task2 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   Cla1Regs.MVECT3 = (Uint16) (&Cla1Task3 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   Cla1Regs.MVECT4 = (Uint16) (&Cla1Task4 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   Cla1Regs.MVECT5 = (Uint16) (&Cla1Task5 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   Cla1Regs.MVECT6 = (Uint16) (&Cla1Task6 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   Cla1Regs.MVECT7 = (Uint16) (&Cla1Task7 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   Cla1Regs.MVECT8 = (Uint16) (&Cla1Task8 - &Cla1Prog_asm_Start)*sizeof(Uint32);
		   EDIS;
		   break;
        case 0x13:
        	EALLOW;
		   //
		   // Task 1 has the option to be started by either EPWM1_INT or ADCINT1
		   // In this case we will allow ADCINT1 to start CLA Task 1
		   //

		   Cla1Regs.MPISRCSEL1.bit.PERINT1SEL = CLA_INT1_ADCINT1;

		   Cla1Regs.MMEMCFG.bit.PROGE = 1;
		   Cla1Regs.MMEMCFG.bit.RAM1E = 1;

		   //
		   // Enable the IACK instruction to start a task
		   // Enable the CLA interrupt 8 and interrupt 1
		   //

		   Cla1Regs.MCTL.bit.IACKE = 1;
		   Cla1Regs.MIER.all = (M_INT8 | M_INT1);
		   EDIS;
		   break;
        case 0x14:
        	EALLOW;
        	Cla1Regs.MIFRC.bit.INT1 = 1;
        	EDIS;
        	break;
        case 0x15:
			EALLOW;
			Cla1Regs.MIFRC.bit.INT8 = 1;
			EDIS;
			break;
        default:
	        break;	        
	}	     	
	}
	else
	{
	switch(i_data)
	{
	    case 0x56:
	        G_debug.fields.bit.enable_debug_latch_0 = 1;  
	        break;
	    case 0xA5:
	        if(G_debug.fields.bit.enable_debug_latch_0)
	        {
	            G_debug.fields.bit.enable_debug_latch_1 = 1;  
	        }
	        break;
	    case 0x78:
	        if(G_debug.fields.bit.enable_debug_latch_1)
	        {
	            G_debug.fields.bit.enable_debug_latch_2 = 1;  
	        }
	        break;
		default:
	        G_debug.fields.word = 0;  break;	        
	}
	}
	    
}
