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

#include "gpio.h"
#include "apss_defines.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Globals
 ****************************************************************************/
hwSettingPins_t G_hwSetPin;

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* Read the pins that are strapped by HW to indicate settings to APSS 
*
* These pins are useless now, but may be used for other purposes
*
* @param  none
*
* @return none 
*/
void gpioReadHwSettingPins(void)
{
  G_hwSetPin.altitudeEnable    = GpioDataRegs.GPBDAT.bit.GPIO44; 
  G_hwSetPin.temperatureEnable = GpioDataRegs.GPBDAT.bit.GPIO39;
  G_hwSetPin.temperatureCount0 = GpioDataRegs.GPADAT.bit.GPIO30;
  G_hwSetPin.temperatureCount1 = GpioDataRegs.GPADAT.bit.GPIO31;
  G_hwSetPin.temperatureCount2 = GpioDataRegs.GPBDAT.bit.GPIO34;	
} 


/**
* Set GPIO Direction Registers based on Global Variable Setting 
*
* 
*
* @param  none
*
* @return none 
*/
void gpioPortSetDirection(void)
{
    uint8_t l_gpio_0 = G_gpioIoConfig[0];
    uint8_t l_gpio_1 = G_gpioIoConfig[1];
    uint8_t l_idx = 0;
    
    for(l_idx = 0; l_idx<8; l_idx++)
    {
    	EALLOW;	
    	switch(l_idx)
    	{
    	  case 0:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO8  = (l_gpio_0 & 0x01);  	
    	     GpioCtrlRegs.GPADIR.bit.GPIO20 = (l_gpio_1 & 0x01); 
    	     break; 
    	  case 1:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO9  = (l_gpio_0 & 0x01); 
    	     GpioCtrlRegs.GPADIR.bit.GPIO21 = (l_gpio_1 & 0x01); 
    	     break; 
    	  case 2:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO10 = (l_gpio_0 & 0x01); 
    	     GpioCtrlRegs.GPADIR.bit.GPIO22 = (l_gpio_1 & 0x01); 
    	     break; 
    	  case 3:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO11 = (l_gpio_0 & 0x01); 
    	     GpioCtrlRegs.GPADIR.bit.GPIO23 = (l_gpio_1 & 0x01); 
    	     break; 
    	  case 4:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO12 = (l_gpio_0 & 0x01); 
    	     GpioCtrlRegs.GPADIR.bit.GPIO24 = (l_gpio_1 & 0x01); 
    	     break; 
    	  case 5:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO13 = (l_gpio_0 & 0x01); 
    	     GpioCtrlRegs.GPADIR.bit.GPIO25 = (l_gpio_1 & 0x01); 
    	     break; 
    	  case 6:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO14 = (l_gpio_0 & 0x01); 
    	     GpioCtrlRegs.GPADIR.bit.GPIO26 = (l_gpio_1 & 0x01); 
    	     break; 
    	  case 7:  
    	     GpioCtrlRegs.GPADIR.bit.GPIO15 = (l_gpio_0 & 0x01); 
    	     GpioCtrlRegs.GPADIR.bit.GPIO27 = (l_gpio_1 & 0x01); 
    	     break; 
    	  default:
    	     break;
    	}
    	EDIS;
    	
    	// Shift the bits by 1
    	l_gpio_0 >>= 1;
    	l_gpio_1 >>= 1;
    }    
}


/**
* Write GPIO Drive Registers based on Global Variable Setting 
*
* 
*
* @param  none
*
* @return none 
*/
void gpioPortDoWrites(void)
{
    // ------------------------------------------------------
    // GPIO Writes (Set to 1)
    // ------------------------------------------------------
    uint32_t l_gpio = G_gpioDriveConfig[0];
    GpioDataRegs.GPASET.all = (((uint32_t) l_gpio) <<  8) & 0x0000FF00;
    l_gpio = G_gpioDriveConfig[1];
    GpioDataRegs.GPASET.all = (((uint32_t) l_gpio) << 20) & 0x0FF00000;
    
    // ------------------------------------------------------
    // GPIO Writes (Clear to 0)
    // ------------------------------------------------------
    l_gpio = G_gpioDriveConfig[0];
    GpioDataRegs.GPACLEAR.all = (((uint32_t) ~l_gpio) <<  8) & 0x0000FF00;
    l_gpio = G_gpioDriveConfig[1];
    GpioDataRegs.GPACLEAR.all = (((uint32_t) ~l_gpio) << 20) & 0x0FF00000;
}


/**
* Read GPIO Data Registers; place in Global Variable
*
* 
*
* @param  none
*
* @return none 
*/
void gpioPortDoReads(void)
{
	// ------------------------------------------------------        	
	// GPIO Reads
	// ------------------------------------------------------
	uint32_t l_gpio = GpioDataRegs.GPADAT.all;
    G_gpioPort[0] = (uint16_t) ((l_gpio & 0x0000FF00) >> 8);
    G_gpioPort[1] = (uint16_t) ((l_gpio & 0x0FF00000) >> 20);
    
	// ------------------------------------------------------        	
	// GPIO Soft OC Reads
	// ------------------------------------------------------
	l_gpio = GpioDataRegs.GPBDAT.all;
    G_softOcPins = (uint8_t) ((l_gpio & 0x00000F00)) >> 8;
}

/**
* Toggle a GPIO pin to reset a HW latch ...  ~125ns 
*
* 
*
* @param  none
*
* @return none 
*/
void gpioResetLatch(void)
{
   GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1;	  // GPIO44 pin is set to 0 (to reset latch)

   asm("   NOP");
   asm("   NOP");
   asm("   NOP");
   asm("   NOP");  
   asm("   NOP");
 
   GpioDataRegs.GPBSET.bit.GPIO44 = 1;	          // GPIO44 pin is set to 1 (enable latch)   
}
