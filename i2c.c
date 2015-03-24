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
#include "DSP2803x_I2C_defines.h"
#include "apss_defines.h"
#include "i2c_cmds.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Globals
 ****************************************************************************/
Uint16 Register;
Uint16 Reg[6] = {0,0,0,0,0,0};
Uint16 InData[3];
Uint16 OutData[3];
Uint16 I2cIndex = 0;
Uint16 I2cProcessCmd = 0;


/****************************************************************************
 * Functions Declarations
 ****************************************************************************/
interrupt void i2c_int1a_isr(void);


/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * @brief   Initialize I2C module in slave mode with interrupts.
 * 
 * @param   I2CSlave_OwnAddress The slave device's own address.
 * 
 * @return  None
 */
void I2CA_Init_Interrupts(Uint16 I2CSlave_OwnAddress)
{
	InitI2CGpio();
	
   // Initialize I2C
	I2caRegs.I2COAR = I2CSlave_OwnAddress;		// Own address
	I2caRegs.I2CPSC.all = 9;		// Prescaler - need 7-12 Mhz on module clk
	I2caRegs.I2CCLKL = 10;			// NOTE: must be non zero
	I2caRegs.I2CCLKH = 5;			// NOTE: must be non zero
	I2caRegs.I2CCNT = 1;	        // Get 1 byte
	I2caRegs.I2CIER.all = 0x18;		// Clear interrupts (was 0)
	I2caRegs.I2CSTR.bit.RRDY = 1;	// Clear flag
	I2caRegs.I2CIER.bit.RRDY = 1;   // Enable Receive Interrupt
	I2caRegs.I2CIER.bit.XRDY = 1;   // Enable Receive Interrupt
	I2caRegs.I2CIER.bit.AAS  = 1;   // Enable AAS (START) Interrupt
	I2caRegs.I2CMDR.all = 0x0020;	// Take I2C out of reset
   									// Stop I2C when suspended
   	I2caRegs.I2CEMDR.bit.BCM = 0;   // Disable (Enable?) Backwards Compatibility Mode
	return;   
}


/**
 * @brief   Initialize I2C slave mode interrupts.
 * 
 * @param   None
 * 
 * @return  None
 */
void I2CInterruptConfig(void)
{
    EALLOW;
    PieVectTable.I2CINT1A    = &i2c_int1a_isr;
    EDIS;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;    // Enable the PIE block

    // Enable I2C interrupt 1  in the PIE: Group 8
    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;

    // Enable CPU INT8 which is connected to PIE group 8
    IER |= M_INT8;
}


/**
 * @brief   ISR for I2C slave
 * 
 * @param   None
 * 
 * @return  None
 */
interrupt void i2c_int1a_isr(void)     // I2C-A
{
    Uint16 IntSource;

    // Read interrupt source
    IntSource = I2caRegs.I2CISRC.bit.INTCODE & 0x7;

    switch(IntSource)
    {
        case I2C_NO_ISRC:   // =0  None
            break;

        case I2C_ARB_ISRC:  // =1  Arbitration Lost
            break;

        case I2C_NACK_ISRC: // =2  NACK condition detected
            break;

        case I2C_ARDY_ISRC: // =3  Registers ready for access
            break;

        case I2C_RX_ISRC:   // =4  Receive Data Ready
        
            // Get Data from Register
            InData[I2cIndex++] = I2caRegs.I2CDRR;
            
#if 0
            // Setup Command/Register variable with 1st byte sent over I2C
            // START..ADDRESS..InData[0]..InData[1]..Indata[2]..InData[2]..
            Register = InData[0];           // Used on data transmit.
            
            // If this was a 2 byte command, it contains data; the data is
            // handled by calling the i2cCommandWrite function
            if (I2cIndex == 2)
            {
            	i2cCommandWrite(Register,InData[1]);
            }
            
            // Process the I2C Command
            i2cCommandRead(Register);
            
            // Build the I2C Response
            OutData[0] = i2cResponse(); //Get most significant byte.	
#else
            I2cProcessCmd = 1;
#endif
            // If index is at 3, we have received 2 data bytes.
            // Keep index @ 2 so we always just overwrite the dummy InData[2]
            if (I2cIndex == 3){  I2cIndex = 2;  }
            break;

        case I2C_TX_ISRC:   // =5  Transmit Data Ready
            // Output Data to Host, only 1 byte of data ever
            I2caRegs.I2CDXR = OutData[0];
            break;

        case I2C_SCD_ISRC:  // =6  Stop Condition Detected
            break;

        case I2C_AAS_ISRC:  // =7  Addressed as Slave
            // Start Condition Detected, reset I2cIndex to receive command/reg
            I2cIndex = 0;
            break;

        default:
            asm("   ESTOP0"); // Halt on invalid number.

    }

    // Enable further I2C (PIE Group 8) interrupts by acknowledging this one.
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP8;
}


/**
 * @brief   Initialize I2C slave mode interrupts.
 * 
 * @param   None
 * 
 * @return  None
 */
void I2CProcessCmd(void)
{
    if(I2cProcessCmd)
    {
        // Setup Command/Register variable with 1st byte sent over I2C
        // START..ADDRESS..InData[0]..InData[1]..Indata[2]..InData[2]..
        Register = InData[0];           // Used on data transmit.

        // If this was a 2 byte command, it contains data; the data is
        // handled by calling the i2cCommandWrite function
        if (I2cIndex == 2)
        {
            i2cCommandWrite(Register,InData[1]);
        }

        // Process the I2C Command
        i2cCommandRead(Register);

        // Build the I2C Response
        OutData[0] = i2cResponse(); //Get most significant byte.	

        // Update critical flag
        ////DISABLE_ALL_INTERRUPTS();
        I2cProcessCmd = 0;
        ////ENABLE_ALL_INTERRUPTS();
    }
}
