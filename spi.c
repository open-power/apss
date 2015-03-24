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
#include "spi.h"
#include "spi_cmds.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Globals
 ****************************************************************************/
spi_cmd_msg_t sdata;     // Send data buffer
spi_cmd_msg_t rdata;     // Receive data buffer
Uint16 outgoing;     // last data transfer word for spi
uint8_t reset_flag = 1; // flag indicating spi reset completed

/****************************************************************************
 * Forward Declarations
 ****************************************************************************/
interrupt void spiTxFifoIsr(void); 
interrupt void spiRxFifoIsr(void); 


/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* Setup the SPI Interrupt Vectors
*
* 
*
* @param  none
*
* @return none 
*/
void spi_setup_interrupt_vectors(void)
{
   EALLOW;
   PieVectTable.SPIRXINTA = &spiRxFifoIsr;  // @tgh - this is the only SPI int that is used
   PieVectTable.SPITXINTA = &spiTxFifoIsr;  // @tgh
   EDIS;
}


/**
* Enable the SPI Interrupts
*
* 
*
* @param  none
*
* @return none 
*/
void spi_interrupts_enable(void)
{
   PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  // @tgh -- spi
   PieCtrlRegs.PIEIER6.bit.INTx2 = 1;  // @tgh -- spi   
   IER |= 0x20;                        // @tgh -- spi	
}


/**
* SPI Module Initialization
*
* FIFO is no longer used.  Standard 240x Mode is used.
*
* @param  none
*
* @return none 
*/
void spi_fifo_init()
{
   // Initialize SPI FIFO registers
   SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI

   SpiaRegs.SPICCR.all=0x004F;       // 16-bit character, No Loopback mode - 20121210 should be 0x4F, not 0x0F 
   SpiaRegs.SPICTL.all=0x0003;       // Interrupt enabled, Slave XMIT enabled, JRK - removed overrun interrupt
   SpiaRegs.SPISTS.all=0x0000;
   SpiaRegs.SPIBRR=0x0063;           // Baud rate - doesn't matter for Slave SPI
   SpiaRegs.SPIFFTX.all=0x8020;      // Disable FIFO's, set TX FIFO level to 0
   SpiaRegs.SPIFFRX.all=0x0022;      // Set RX FIFO level to 4
   SpiaRegs.SPIFFCT.all=0x00;
   SpiaRegs.SPIPRI.all=0x0010;

   SpiaRegs.SPICCR.bit.SPISWRESET=1; // Enable SPI

   SpiaRegs.SPIFFTX.bit.TXFIFO=1;
   SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
   
   // Initialize Data Structures
   sdata.word = 0;
   rdata.word = 0;
}


/**
* ISR for SPI FIFO Transmit - deprecated
*
* FIFO is no longer used.  Standard 240x Mode is used instead, so
* this interrupt won't be run.
*
* @param  none
*
* @return none 
*/
interrupt void spiTxFifoIsr(void)
{
    // This interrupt is not used in Standard 240x SPI Mode
    // but would be used if we ever switched back to FIFO Mode,
    // which seems unlikely.
    SpiaRegs.SPITXBUF=0xFFFF;           // Send data

    SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ACK
}

/**
* check if SPISTE de-asserted, if so then check global
* spi-data register to what is in SPIDAT, if it is not equal
* and SPIDAT also is not 00 then reset the SPI interface...
* This is to help reduce risk of single bit errors
* locking up the state machine between master/slave
* since a spi reset should clear out the buffers and start
* with a fresh frame.
*
* @param none
*
* @return none
**/
void spi_check_sync(void)
{
    // check if reset is not already done and check if SPISTE is high (deactive)
    if (GpioDataRegs.GPADAT.bit.GPIO18 == 1 && reset_flag > 0 && SpiaRegs.SPIDAT != outgoing) {
    	reset_flag--;
    	// make sure we have had sufficient time to catch the receive interrupt, so we decrement the
    	// reset_flag until it reaches the value '1'.
        if (reset_flag == 1) {
            // reset spi
            SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI
            SpiaRegs.SPICCR.bit.SPISWRESET=1; // Enable SPI
            // setup outgoing buffer to match the last request
            SpiaRegs.SPITXBUF = outgoing;
            reset_flag = 0;
        }
    }
}

/**
* ISR for SPI 240x Mode Transmit & Receive
*
* FIFO is no longer used.  Standard 240x Mode is used.
*
* @param  none
*
* @return none 
*/
interrupt void spiRxFifoIsr(void)
{
	// Read Command in from SPI
    rdata.word = SpiaRegs.SPIRXBUF;
      
    spiCommand(&rdata);
    
    // Place response into TX Buffer
    outgoing = spiResponse(&rdata);
    SpiaRegs.SPITXBUF = outgoing;
    
    // Toggle Debug Pin indicating we are talking SPI 
    GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1;  

    // reset flag indicating we have sent data
    reset_flag = 4;

    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack

    // JRK - try moving these to the ending....
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
}



