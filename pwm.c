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
/** 
 *
 *  PWM-DAC RC Filter Cutoff Frequency is 1.591 kHz
 *    R =   1 kOhm
 *    C = 100 nF
 * 
 */

/****************************************************************************
 * Includes
 ****************************************************************************/
#include "DSP2803x_Device.h"
#include "DSP2803x_Examples.h"
#include "CLAShared.h"
#include "pwm.h"

#include "apss_defines.h"
#include "debug.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
/* @brief Determines the period of the PWM waveform and inversely Fpwm 
 * 
 * The following calculations assume 
 *   Fsysclkout = 60MHz
 *        TBCLK = 0 (No Prescalar Division of Sysclk)
 *        MODE  = Up/Down (If Up Only, multiply freq x2 below)
 * 
 *      Fpwm  =  TBPRD (PWM_PERIOD)
 *   0.5 kHz     60000 
 *   1   kHz     30000
 *   2   kHz     15000
 *   3.66kHz      8192
 *   7.34kHz      4096
 *  14.65 kHz     2048
 *  29.30         1024
 *  58.59 kHz      512
 * 117.19 kHz      256
 *   !!! NEVER SET THIS LOWER THAN 256 !!!
 * 
 * */
#define PWM_PERIOD	       512  

/* Initial PWM Duty Cycle */
#define PWM_INITIAL_DUTY_CYCLE (PWM_DUTY_50_00)

/* PWM Duty Cycles Values for a Given PWM % */
#define PWM_DUTY_03_12         (PWM_PERIOD/32)                    // 0.103 V
#define PWM_DUTY_06_25         (PWM_PERIOD/16)                    // 0.206 V
#define PWM_DUTY_12_50         (PWM_PERIOD/8)                     // 0.412 V
#define PWM_DUTY_25_00         (PWM_PERIOD/4)                     // 0.825 V
#define PWM_DUTY_50_00         (PWM_PERIOD/2)                     // 1.65  V
#define PWM_DUTY_75_00         (PWM_DUTY_50_00 + PWM_DUTY_25_00)  // 2.475 V
#define PWM_DUTY_87_50         (PWM_DUTY_75_00 + PWM_DUTY_12_50)  // 2.887 V
#define PWM_DUTY_93_75         (PWM_DUTY_87_50 + PWM_DUTY_06_25)  // 3.093 V
#define PWM_DUTY_100_00        (PWM_PERIOD)                       // 3.300 V

/* Calculate factor for translating I2C PWM to PWM */
#define I2C_MAX_PWM            (0xFF)
#define PWM_I2C_GAIN_FACTOR    (PWM_PERIOD/(I2C_MAX_PWM + 1))

/* Calculate factor for translating IIR Output to PWM */
#define IIR_MAX                0xFFF
#define PWM_IIR_MATH_FACTOR    16
#define PWM_IIR_VREF           2048
#define PWM_IIR_VPWM           3300
#define PWM_IIR_GAIN_FACTOR    (IIR_MAX*PWM_IIR_MATH_FACTOR) / ((PWM_IIR_VREF/PWM_IIR_VPWM) * PWM_PERIOD)   
   
/****************************************************************************
 * Macros
 ****************************************************************************/   

/* @brief Convert the I2C PWM Value into the Actual PWM register 
 * 
 * Convert the I2C PWM Value into the Actual PWM register value that 
 * corresponds to the I2C Command's intention 
 */
#define I2C2PWM(value)   (((uint16_t) value) * PWM_I2C_GAIN_FACTOR)

/* @brief Convert the IIR Output Value into the Actual PWM register 
 * 
 * Convert the IIR Output Value into the Actual PWM register value that 
 * corresponds to the IIR Output Voltage 
 */
#define IIR2PWM(value)   ((((uint16_t) value) / 16) * PWM_I2C_GAIN_FACTOR)        


/****************************************************************************
 * Globals
 ****************************************************************************/
volatile Uint32  EPwm5TimerIntCount = 0;
volatile Uint32 StopTest = 0 ;


/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* ISR for ePWM 5 
*
* This interrupt will be taken by the main CPU when ePWM 1 completes
*
* @param  none
*
* @return none 
*/
interrupt void epwm5_isr()
{
    // This interrupt is simply used for instrumenting the Start of Conversion for the ADC.
    // It does not need to be implemented.
    //
    EPwm5TimerIntCount++;

    // Clear INT flag for this timer
    EPwm5Regs.ETCLR.bit.INT = 1;

    // Peform the necessary clean up in the PIE to allow another interrupt can be taken
    // ePWM1 related PIE acknowledgment
    PieCtrlRegs.PIEACK.bit.ACK3 = 1 ;
}


/**
* Initialize ePWM 5  
*
* EPWM1 will be used to generate the ADC Start of conversion
*
* @param  none
*
* @return none 
*/
void init_epwm5(void)
{
   //                                         
   // Assumes ePWM1 clock is already enabled in InitSysCtrl();
   // 
   // Before configuring the ePWMs, halt the counters
   // After configuration they can all be started again
   // in syncronization by setting this bit.
   //

   EALLOW;					   
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   //
   // EALLOW: is needed to write to EALLOW protected registers
   // EDIS: is needed to disable write to EALLOW protected registers   
   //
   // Enable start of conversion (SOC) on A
   // An SOC event will occur when the ePWM counter is zero
   // The ePWM will generate an SOC on the first event
   // 

   EALLOW;

   //
   // Set the period for ePWM1  
   // By default TBPRD = 1/2 SYSCLKOUT 
   // Set the counter for up count mode
   //

   EPwm5Regs.TBPRD 		= ADC_SAMPLE_PERIOD;  
   EPwm5Regs.TBCTL.bit.CTRMODE 	= TB_COUNT_UP;	   
   EPwm5Regs.ETSEL.bit.SOCAEN	= 1;	   
   EPwm5Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;	   
   EPwm5Regs.ETPS.bit.SOCAPRD 	= ET_1ST;	   

   // Not necessary for creating the Start of Conversion (SOC) signal to the ADC.
   // The interrupt is enabled for instrumentation of this process.
   EPwm5Regs.ETSEL.bit.INTSEL   = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm5Regs.ETSEL.bit.INTEN    = 1;  // Enable INT
   EPwm5Regs.ETPS.bit.INTPRD    = ET_1ST;           // Generate INT on 1st event

   EDIS;
}


/**
* Start ePWM   
*
* EPWM1 will be used to generate the ADC Start of conversion
*
* @param  none
*
* @return none 
*/
void start_epwm(void)
{
   //
   // Start the ePWM counters                   
   // Note: this should be done after all ePWM modules are configured
   // to ensure synchronization between the ePWM modules.
   //

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
}


/**
* Initialize ePWM 1 
*
* EPWM4 will be used to ...
*
* @param  none
*
* @return none 
*/
void init_epwm1(void)
{


    // EALLOW: is needed to write to EALLOW protected registers
    // EDIS: is needed to disable write to EALLOW protected registers
    // Configure the GPIO4 pin to be EPWM4A output

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    EDIS;

    //
    // Disable the timer (counter mode is halt)
    // Set the free/soft emulation bits to ignore the
    // emulation halt (PWM will continue to count when
    // the CPU is halted)
    //

    EPwm1Regs.TBCTL.bit.CTRMODE = 0x3; 		
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 3; 

    //
    // Clear the counter
    // Set the period and timer phase
    // Specify when the compare A event will occur
    //

    EPwm1Regs.TBCTR = 0x0000;			   
    EPwm1Regs.TBPRD = PWM_PERIOD;	   
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;   
    EPwm1Regs.CMPA.half.CMPA = PWM_INITIAL_DUTY_CYCLE;
    EPwm1Regs.CMPB           = PWM_INITIAL_DUTY_CYCLE;

    //
    // On compare A, when counting up, pull the EPWM A output high
    // On compare A, when counting down, pull the EPWM A outpout low
    // Set the counter to up/down count mode
    //

    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    //EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
    
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;
    //EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
     
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;	
    
    	
} 


/**
* Initialize ePWM 3 
*
* EPWM3 will be used to ...
*
* @param  none
*
* @return none 
*/
void init_epwm3(void)
{

    // EALLOW: is needed to write to EALLOW protected registers
    // EDIS: is needed to disable write to EALLOW protected registers
    // Configure the GPIO4 pin to be EPWM4A output

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
    EDIS;

    //
    // Disable the timer (counter mode is halt)
    // Set the free/soft emulation bits to ignore the
    // emulation halt (PWM will continue to count when
    // the CPU is halted)
    //

    EPwm3Regs.TBCTL.bit.CTRMODE = 0x3;		
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 3; 

    //
    // Clear the counter
    // Set the period and timer phase
    // Specify when the compare A event will occur
    //

    EPwm3Regs.TBCTR = 0x0000;			   
    EPwm3Regs.TBPRD = PWM_PERIOD;	   
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;   
    EPwm3Regs.CMPA.half.CMPA = PWM_INITIAL_DUTY_CYCLE;
    EPwm3Regs.CMPB           = PWM_INITIAL_DUTY_CYCLE;

    //
    // On compare A, when counting up, pull the EPWM A output high
    // On compare A, when counting down, pull the EPWM A outpout low
    // Set the counter to up/down count mode
    //

    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;    
    
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		

} 


/**
* Initialize ePWM 2 
*
* EPWM2 will be used to ...
*
* @param  none
*
* @return none 
*/
void init_epwm2(void)
{
    // EALLOW: is needed to write to EALLOW protected registers
    // EDIS: is needed to disable write to EALLOW protected registers
    // Configure the GPIO4 pin to be EPWM4A output

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    EDIS;

    //
    // Disable the timer (counter mode is halt)
    // Set the free/soft emulation bits to ignore the
    // emulation halt (PWM will continue to count when
    // the CPU is halted)
    //

    EPwm2Regs.TBCTL.bit.CTRMODE = 0x3;		
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 3; 

    //
    // Clear the counter
    // Set the period and timer phase
    // Specify when the compare A event will occur
    //

    EPwm2Regs.TBCTR = 0x0000;			   
    EPwm2Regs.TBPRD = PWM_PERIOD;	   
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000;   
    EPwm2Regs.CMPA.half.CMPA = PWM_INITIAL_DUTY_CYCLE;
    EPwm2Regs.CMPB           = PWM_INITIAL_DUTY_CYCLE;

    //
    // On compare A, when counting up, pull the EPWM A output high
    // On compare A, when counting down, pull the EPWM A outpout low
    // Set the counter to up/down count mode
    //

    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;      
    
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		

} 

/**
* Initialize ePWM 4 
*
* EPWM4 will be used to ...
*
* @param  none
*
* @return none 
*/
void init_epwm4(void)
{
    // EALLOW: is needed to write to EALLOW protected registers
    // EDIS: is needed to disable write to EALLOW protected registers
    // Configure the GPIO4 pin to be EPWM4A output

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
    EDIS;

    //
    // Disable the timer (counter mode is halt)
    // Set the free/soft emulation bits to ignore the
    // emulation halt (PWM will continue to count when
    // the CPU is halted)
    //

    EPwm4Regs.TBCTL.bit.CTRMODE = 0x3;		
    EPwm4Regs.TBCTL.bit.FREE_SOFT = 3; 

    //
    // Clear the counter
    // Set the period and timer phase
    // Specify when the compare A event will occur
    //

    EPwm4Regs.TBCTR = 0x0000;			   
    EPwm4Regs.TBPRD = PWM_PERIOD;	   
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;   
    EPwm4Regs.CMPA.half.CMPA = PWM_INITIAL_DUTY_CYCLE;
    EPwm4Regs.CMPB           = PWM_INITIAL_DUTY_CYCLE;

    //
    // On compare A, when counting up, pull the EPWM A output high
    // On compare A, when counting down, pull the EPWM A outpout low
    // Set the counter to up/down count mode
    //

    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
    
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;      
    
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		
} 


/**
* Write the values from Global to all PWMs
*
* 
*
* @param  none
*
* @return none 
*/
void pwmDoPwmWrites(void)
{
    if(G_debug.fields.bit.pwm_chan_stick)
    {
        EALLOW;
        EPwm1Regs.CMPA.half.CMPA = PWM_DUTY_03_12;
        EPwm1Regs.CMPB           = PWM_DUTY_06_25;   
        EDIS;
        EALLOW;	        
        EPwm2Regs.CMPA.half.CMPA = PWM_DUTY_12_50;
        EPwm2Regs.CMPB           = PWM_DUTY_25_00;
        EDIS;
        EALLOW;	
        EPwm3Regs.CMPA.half.CMPA = PWM_DUTY_50_00;
        EPwm3Regs.CMPB           = PWM_DUTY_75_00;
        EDIS; 
        EALLOW;	
        EPwm4Regs.CMPA.half.CMPA = PWM_DUTY_87_50;
        EPwm4Regs.CMPB           = PWM_DUTY_93_75;
        EDIS;   	
    }
    else if (G_debug.fields.bit.iir2pwm0_7)
    {
        EALLOW;
        EPwm1Regs.CMPA.half.CMPA = IIR2PWM(yn_array[0]);
        EPwm1Regs.CMPB           = IIR2PWM(yn_array[1]);   
        EDIS;
        EALLOW;
        EPwm2Regs.CMPA.half.CMPA = IIR2PWM(yn_array[2]);
        EPwm2Regs.CMPB           = IIR2PWM(yn_array[3]);   
        EDIS;
        EALLOW;	
        EPwm3Regs.CMPA.half.CMPA = IIR2PWM(yn_array[4]);
        EPwm3Regs.CMPB           = IIR2PWM(yn_array[5]);
        EDIS; 
        EALLOW;	
        EPwm4Regs.CMPA.half.CMPA = IIR2PWM(yn_array[6]);
        EPwm4Regs.CMPB           = IIR2PWM(yn_array[7]);
        EDIS; 
    }
    else if (G_debug.fields.bit.iir2pwm8_15)
    {
        EALLOW;
        EPwm1Regs.CMPA.half.CMPA = IIR2PWM(yn_array[8]);
        EPwm1Regs.CMPB           = IIR2PWM(yn_array[9]);   
        EDIS;
        EALLOW;
        EPwm2Regs.CMPA.half.CMPA = IIR2PWM(yn_array[10]);
        EPwm2Regs.CMPB           = IIR2PWM(yn_array[11]);   
        EDIS;
        EALLOW;	
        EPwm3Regs.CMPA.half.CMPA = IIR2PWM(yn_array[12]);
        EPwm3Regs.CMPB           = IIR2PWM(yn_array[13]);
        EDIS; 
        EALLOW;	
        EPwm4Regs.CMPA.half.CMPA = IIR2PWM(yn_array[14]);
        EPwm4Regs.CMPB           = IIR2PWM(yn_array[15]);
        EDIS; 
    }
    else
    {
        EALLOW;	        
        EPwm1Regs.CMPA.half.CMPA = I2C2PWM(G_pwmChan[0]);
        EPwm1Regs.CMPB           = I2C2PWM(G_pwmChan[1]);   
        EDIS;
        EALLOW;	        
        EPwm2Regs.CMPA.half.CMPA = I2C2PWM(G_pwmChan[2]);
        EPwm2Regs.CMPB           = I2C2PWM(G_pwmChan[3]);   
        EDIS;
        EALLOW;	
        EPwm3Regs.CMPA.half.CMPA = I2C2PWM(G_pwmChan[4]);
        EPwm3Regs.CMPB           = I2C2PWM(G_pwmChan[5]);
        EDIS; 
        EALLOW;	
        EPwm4Regs.CMPA.half.CMPA = I2C2PWM(G_pwmChan[6]);
        EPwm4Regs.CMPB           = I2C2PWM(G_pwmChan[7]);
        EDIS;   	
    }
}


