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
#include "DSP2803x_Device.h"
#include "DSP2803x_Examples.h"

#include "CLAShared.h"      // includes all of the shared variables (shared between
                            // C28x C source and CLA assembly code)
                         
#include "cla.h"
                      
/****************************************************************************
 * Defines
 ****************************************************************************/
#define CLARAM1_ENABLE	1

/****************************************************************************
 * Globals
 ****************************************************************************/
volatile Uint32  IIRFilterCompleteCount = 0 ;

extern Uint16 Cla1funcsLoadStart;
extern Uint16 Cla1funcsLoadEnd;
extern Uint16 Cla1funcsRunStart;
//Cla data memory locations
//extern Uint16 Cla1DataLoadStart;
//extern Uint16 Cla1DataLoadEnd;
//extern Uint16 Cla1DataRunStart;

/****************************************************************************
 * Functions
 ****************************************************************************/

void MemCopy2(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}

/**
* ISR for CLA Task 1 
*
* This interrupt will be taken by the main CPU when CLA task 1 completes
*
* @param  none
*
* @return none 
*/
interrupt void cla1_isr1()
{
    // This interrupt signals the completion of the set of IIR biquads.
    // This interrupt handler will rearm the CLA IIR biquad setup to prepare for the
    // next run.
    // 

    // Clear the ADC interrupt flag so the next SOC (start of conversion) can occur
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;       

    // Peform the necessary clean up in the PIE to allow another interrupt can be taken
    // ADC related PIE acknowledgment
    PieCtrlRegs.PIEACK.bit.ACK1 = 1 ;
    PieCtrlRegs.PIEACK.bit.ACK10 = 1 ;
    // CLA related PIE acknowledgment
    PieCtrlRegs.PIEACK.bit.ACK11 = 1 ;
    PieCtrlRegs.PIEACK.bit.ACK12 = 1 ;

    // Setup the IIR filter set for the next ADC conversion set.  These pointers are in the
    // CPUtoCLA Message RAM.  It is much easier to reinitialize the pointers in the main CPU.
    pINITIALIIRFILTER = &IIRFILTERSET.Channels[0] ;
    pInitialADCResult = (Uint16 *)&AdcResult.ADCRESULT0 ;
    pInitialyn_array = &yn_array[0] ;
    pInitialxn_array = &xn_array[0] ;

    // Signal the CLA to copy these to its working copy in its own Data RAM.
    Cla1ForceTask8();

    // Toggle GPIO for status
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;		

    // Meant for instrumenting the operation.
    IIRFilterCompleteCount++ ;

    // If desired, should add code here to set a flag to the application layer that results
    // of the IIR biquad operations are ready.
}


/**
* CLA module initialization   
*
* 1) Enable the CLA clock
* 2) Init the CLA interrupt vectors
* 3) Allow the IACK instruction to flag a CLA interrupt/start a task
*    This is used to force Task 8 in this example
* 4) Copy CLA code from its load address to CLA program RAM
*
*    Note: during debug the load and run addresses could be 
*    the same as Code Composer Studio can load the CLA program
*    RAM directly. 
* 
*    The ClafuncsLoadStart, ClafuncsLoadEnd, and ClafuncsRunStart
*    symbols are created by the linker. 
* 4) Assign CLA program memory to the CLA
* 5) Enable CLA interrupts (MIER)
*
* @param  none
*
* @return none 
*/
void init_cla()
{

   MemCopy2(&Cla1funcsLoadStart, &Cla1funcsLoadEnd, &Cla1funcsRunStart);
//   MemCopy(&Cla1DataLoadStart, &Cla1DataLoadEnd, &Cla1DataRunStart);

   //
   // This code assumes the CLA clock is already enabled in 
   // the call to InitSysCtrl();
   //
   // EALLOW: is needed to write to EALLOW protected registers
   // EDIS: is needed to disable write to EALLOW protected registers
   //
   // Initalize the interrupt vectors for Task 1 (CLA IIR Filter)
   // and for Task 8 (FIR filter initalization)
   //
   // The symbols used in this calculation are defined in the CLA 
   // assembly code and in the CLAShared.h header file
   //

   EALLOW;
   Cla1Regs.MVECT1 = (Uint16) (&Cla1Task1 - &Cla1Prog_asm_Start)*sizeof(Uint32);
   Cla1Regs.MVECT2 = (Uint16) (&Cla1Task2 - &Cla1Prog_asm_Start)*sizeof(Uint32);
   Cla1Regs.MVECT3 = (Uint16) (&Cla1Task3 - &Cla1Prog_asm_Start)*sizeof(Uint32);
   Cla1Regs.MVECT4 = (Uint16) (&Cla1Task4 - &Cla1Prog_asm_Start)*sizeof(Uint32);
   Cla1Regs.MVECT5 = (Uint16) (&Cla1Task5 - &Cla1Prog_asm_Start)*sizeof(Uint32);
   Cla1Regs.MVECT6 = (Uint16) (&Cla1Task6 - &Cla1Prog_asm_Start)*sizeof(Uint32);
   Cla1Regs.MVECT7 = (Uint16) (&Cla1Task7 - &Cla1Prog_asm_Start)*sizeof(Uint32);
   Cla1Regs.MVECT8 = (Uint16) (&Cla1Task8 - &Cla1Prog_asm_Start)*sizeof(Uint32);

   //
   // Task 1 has the option to be started by either EPWM1_INT or ADCINT1 
   // In this case we will allow ADCINT1 to start CLA Task 1
   //
   
   Cla1Regs.MPISRCSEL1.bit.PERINT1SEL = CLA_INT1_ADCINT1;
   ///////////Cla1Regs.MPISRCSEL1.bit.PERINT1SEL = 0xF; // disable interrupt from adc
   
   Cla1Regs.MMEMCFG.bit.PROGE = 1;
   Cla1Regs.MMEMCFG.bit.RAM1E = CLARAM1_ENABLE;
      
   //
   // Enable the IACK instruction to start a task
   // Enable the CLA interrupt 8 and interrupt 1
   //     
   
   Cla1Regs.MCTL.bit.IACKE = 1;            
   Cla1Regs.MIER.all = (M_INT8 | M_INT1);

   //
   // Force CLA task 8 using the IACK instruction
   // Task 8 will initialize the IIR Biquad.
   // No need to wait, the task will finish by the time
   // we configure the ePWM and ADC modules
   //
   Cla1ForceTask8(); 
   EDIS;
}

