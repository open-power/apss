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
                            
#include "spi.h"            // SPI   Hardware low-level routines & interrupts
#include "pwm.h"            // PWM   Hardware low-level routines & interrupts
#include "adc.h"            // ADC   Hardware low-level routines & interrupts
#include "cla.h"            // CLA   Hardware low-level routines & interrupts
#include "timer.h"          // Timer Hardware low-level routines & interrupts
#include "gpio.h"           // GPIO  Hardware low-level routines & interrupts
#include "i2c.h"            // I2C   Hardware low-level routines & interrupts

#include "apss_defines.h"

#include "debug.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Globals
 ****************************************************************************/

/*****************************************************************************
 * Functions Declarations
 ****************************************************************************/
void init_pie(void);

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* main() -- entry point of the program that is run  
*
* @param  none
*
* @return none 
*/
void main (void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2803x_SysCtrl.c file.
    //

    InitSysCtrl();


    // Copy time critical code and Flash setup code to RAM
    // This includes the following ISR functions: cla1_isr7 and InitFlash();
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the F2808.cmd file.
    //

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);


    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    //

    InitFlash(); 

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2803x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //

    InitGpio();

    //    
    // Specific setup for this example:
    //
    
    G_debug.fields.word = 0;              // Clear any debug that might be set.
    
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO39   = 1;	  // GPIO39 is an output (connected to LED)
    GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;	  // GPIO39 pin is set to 0 (turn LED on)
    GpioCtrlRegs.GPBDIR.bit.GPIO44   = 1;	  // GPIO44 is an output (connected to LED)
    gpioResetLatch();                             // Reset the latch        
    EDIS;  


    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    init_pie() ;


    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in DSP2803x_InitPeripherals.c
    // InitPeripherals(); // Not required for this example

    InitSpiaGpio();      
    spi_fifo_init();    

    InitCpuTimers();     
    cpu_timer0_init();

    init_cla();  
    init_adc();        

    init_epwm5();
    init_epwm1();
    init_epwm2();
    init_epwm3();
    init_epwm4();

    I2CInterruptConfig();
    I2CA_Init_Interrupts(0x0038);

    // Step 5. User specific code, enable interrupts:

    start_epwm() ;
    
    // Read HW Strapping Pins on Boot Only
    gpioReadHwSettingPins();

    // Step 6. Forever Loop

    //
    // The main CPU will recieve an interrupt from the 
    // CLA each time task 1 (CLA IIR filter set) completes 
    //
    // In the meantime the main CPU can do other work
    //

    for(;;)
    {
        if(1)
        {
            // ------------------------------------------------------        	
            // ADC Reads
            // ------------------------------------------------------
            adcDoPopulateArray();
        }

        if(0)
        {
            // Check for sync issues
            spi_check_sync();
        }

        if(1)
        {
            // ------------------------------------------------------        	
            // Handle PWM Writes
            // ------------------------------------------------------
            pwmDoPwmWrites();      	
        }

        if(1)
        {
            // ------------------------------------------------------        	
            // GPIO Reads (including Soft OC)
            // ------------------------------------------------------
            gpioPortDoReads();

            // ------------------------------------------------------
            // GPIO Writes
            // ------------------------------------------------------             
            gpioPortDoWrites();

            // ------------------------------------------------------
            // GPIO Direction
            // ------------------------------------------------------ 
            gpioPortSetDirection();
        }

        if(1)
        {
            // ------------------------------------------------------        	
            // I2C Slave Commands
            // ------------------------------------------------------
            I2CProcessCmd();
        }
    }
}


/**
* Peripheral Interrupt Expansion module initialization   
*
* 
*
* @param  none
*
* @return none 
*/
void init_pie()
{
    // Disable CPU interrupts
    //

    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2803x_PieCtrl.c file.
    //

    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2803x_DefaultIsr.c.
    // This function is found in DSP2803x_PieVect.c.
    //

    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    // EALLOW: is needed to write to EALLOW protected registers
    // EDIS: is needed to disable write to EALLOW protected registers
    //

    EALLOW;  
    PieVectTable.CLA1_INT1 = &cla1_isr1;   
    PieVectTable.EPWM5_INT = &epwm5_isr ;
    PieVectTable.TINT0     = &cpu_timer0_isr; 
    spi_setup_interrupt_vectors();
    EDIS;    

    //
    // Enable INT 11.1 in the PIE (CLA Task 1)
    // Enable INT 11 at the CPU level
    // Enable Global interrupts with INTM
    // Enable Global realtime interrupts with DBGM
    //

    PieCtrlRegs.PIEIER11.bit.INTx1 = 1;      
    IER |= M_INT11;                        

    PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
    IER |= M_INT3 ; 

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // @tgh - LED
    IER |= M_INT1;                      // @tgh - LED

    spi_interrupts_enable();

    EINT;          						   
    ERTM;          						   
}

