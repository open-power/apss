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



#include "apss_defines.h"
#include "i2c_cmds.h"
#include "debug.h"
#include "gpio.h"

/****************************************************************************
 * Globals
 ****************************************************************************/

/** @brief Holds the response to the i2c commands, based on last i2c cmd */
uint8_t G_i2cResponseRegisters[256];

/** @brief Holds the response to invalid i2c commands */
const uint8_t G_i2cInvalid = 0xFF;

/** @brief Holds the i2c address of the APSS */
const uint8_t G_i2cAddress = APSS_I2C_ADDRESS;

/** @brief Holds the pointers to data that will be used to fill in response */
const void * G_i2cResponseDataPtrs[256] = 
{
    /*  [00]: NO_CHANGE           */ &G_i2cInvalid,
    /*  [01]: REV_CODE            */ &G_apssSoftwareRev,
    /*  [02]: DAC0                */ &G_pwmChan[0],
    /*  [03]: DAC1                */ &G_pwmChan[1],
    /*  [04]: DAC2                */ &G_pwmChan[2],
    /*  [05]: DAC3                */ &G_pwmChan[3],
    /*  [06]: DAC4                */ &G_pwmChan[4],
    /*  [07]: DAC5                */ &G_pwmChan[5],
    /*  [08]: DAC6                */ &G_pwmChan[6],
    /*  [09]: DAC7                */ &G_pwmChan[7],
    /*  [0a]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [0b]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [0c]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [0d]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [0e]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [0f]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [10]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [11]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [12]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [13]: NO_COMMAND          */ &G_i2cInvalid,
    /*  [14]: ADC_CH0_LSB         */ (uint8_t *) &G_adcChan[0],
    /*  [15]: ADC_CH0_MSB         */ &G_i2cInvalid,
    /*  [16]: ADC_CH1_LSB         */ (uint8_t *) &G_adcChan[1],
    /*  [17]: ADC_CH1_MSB         */ &G_i2cInvalid,
    /*  [18]: ADC_CH2_LSB         */ (uint8_t *) &G_adcChan[2],
    /*  [19]: ADC_CH2_MSB         */ &G_i2cInvalid,
    /*  [1a]: ADC_CH3_LSB         */ (uint8_t *) &G_adcChan[3],
    /*  [1b]: ADC_CH3_MSB         */ &G_i2cInvalid,
    /*  [1c]: ADC_CH4_LSB         */ (uint8_t *) &G_adcChan[4],
    /*  [1d]: ADC_CH4_MSB         */ &G_i2cInvalid,
    /*  [1e]: ADC_CH5_LSB         */ (uint8_t *) &G_adcChan[5],
    /*  [1f]: ADC_CH5_MSB         */ &G_i2cInvalid,
    /*  [20]: ADC_CH6_LSB         */ (uint8_t *) &G_adcChan[6],
    /*  [21]: ADC_CH6_MSB         */ &G_i2cInvalid,
    /*  [22]: ADC_CH7_LSB         */ (uint8_t *) &G_adcChan[7],
    /*  [23]: ADC_CH7_MSB         */ &G_i2cInvalid,
    /*  [24]: ADC_CH8_LSB         */ (uint8_t *) &G_adcChan[8],
    /*  [25]: ADC_CH8_MSB         */ &G_i2cInvalid,
    /*  [26]: ADC_CH9_LSB         */ (uint8_t *) &G_adcChan[9],
    /*  [27]: ADC_CH9_MSB         */ &G_i2cInvalid,
    /*  [28]: ADC_CH10_LSB        */ (uint8_t *) &G_adcChan[10],
    /*  [29]: ADC_CH10_MSB        */ &G_i2cInvalid,
    /*  [2a]: ADC_CH11_LSB        */ (uint8_t *) &G_adcChan[11],
    /*  [2b]: ADC_CH11_MSB        */ &G_i2cInvalid,
    /*  [2c]: ADC_CH12_LSB        */ (uint8_t *) &G_adcChan[12],
    /*  [2d]: ADC_CH12_MSB        */ &G_i2cInvalid,
    /*  [2e]: ADC_CH13_LSB        */ (uint8_t *) &G_adcChan[13],
    /*  [2f]: ADC_CH13_MSB        */ &G_i2cInvalid,
    /*  [30]: ADC_CH14_LSB        */ (uint8_t *) &G_adcChan[14],
    /*  [31]: ADC_CH14_MSB        */ &G_i2cInvalid,
    /*  [32]: ADC_CH15_LSB        */ (uint8_t *) &G_adcChan[15],
    /*  [33]: ADC_CH15_MSB        */ &G_i2cInvalid,
    /*  [34]: GPIO_IN0            */ &G_gpioPort[0],
    /*  [35]: GPIO_IN1            */ &G_gpioPort[1],
    /*  [36]: GPIO_MODE0          */ &G_gpioIoConfig[0],
    /*  [37]: GPIO_MODE1          */ &G_gpioIoConfig[1],
    /*  [38]: GPIO_READONLY       */ &G_softOcPins,
    /*  [39]: INVALID_CMD         */ &G_i2cInvalid,
    /*  [3a]: INVALID_CMD         */ &G_i2cInvalid,
};

/** @brief Holds the last I2C Cmd we received, used as index on registers */
uint8_t G_i2cLastCommand = 0;

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* Grab an ADC Value, and put into into response registers 
*
* This function uses the Data Pointer as a uint16_t * and copies the value
* into 2 registers at once
*
* @param i_cmd - the i2c cmd, used to index into the table
*
* @return none 
*/
void i2cGrabAdcChannel(uint8_t i_cmd)
{
    uint16_t l_adc = 0;

    // Read the adc data out as a "uint16_t *"
    l_adc = *((uint16_t *) G_i2cResponseDataPtrs[i_cmd]);    

    // Copy data into response register
    G_i2cResponseRegisters[i_cmd]   = (uint8_t)  (l_adc & 0x00FF);          // LSB
    G_i2cResponseRegisters[i_cmd+1] = (uint8_t) ((l_adc & 0xFF00) >> 8);    // MSB
}


/**
* Put the PWM value that was sent over I2C into the PWM
*
* @param i_chan - the pwm channel that was received over i2c
*
* @param i_data - the 8-bit payload that was received over I2C
*
* @return none 
*/
void i2cWritePwmChannel(uint8_t i_chan, uint8_t i_data)
{
    DEBUG("PWM[%d] Changed to Data: 0x%02x\n",i_chan, i_data);
    if(i_chan < NUM_PWM_CHANNELS)
    {
       G_pwmChan[i_chan] = i_data;
    }
}


/**
* Parse the I2C command, and handle the write
*
* This function is used to handle the writes to the PWMs, all other write commands just
* bounce off
*
* @param i_cmd - the 8-bit command that was received over I2C
*
* @param i_data - the 8-bit payload that was received over I2C
*
* @return none 
*/
void i2cCommandWrite(uint8_t i_cmd, uint8_t i_data)
{
    DEBUG("I2C Command Write: 0x%02x Data: 0x%02x\n",i_cmd, i_data);

    // This is an ADC Command, and we have to treat it special so that the 
    // MSB is locked on a read of the LSB.
    switch(i_cmd)
    {
        case DAC0:
            i2cWritePwmChannel(0,i_data);  break;
        case DAC1:
            i2cWritePwmChannel(1,i_data);  break;
        case DAC2:
            i2cWritePwmChannel(2,i_data);  break;
        case DAC3:
            i2cWritePwmChannel(3,i_data);  break;
        case DAC4:
            i2cWritePwmChannel(4,i_data);  break;
        case DAC5:
            i2cWritePwmChannel(5,i_data);  break;
        case DAC6:
            i2cWritePwmChannel(6,i_data);  break;
        case DAC7:
            i2cWritePwmChannel(7,i_data);  break;
        case GPIO_IN0:
        case GPIO_IN1:
             gpioResetLatch();        break;
        case 0xFE:
            debugModeEnable(i_data);  break;
        default:
            // No other commands support writes
            break;
    }
}

/**
* Parse the I2C command, and set up response 
*
* This function is used to filter the NOCHANGE command out, to handle the 
* adc weirdness of locking the MSB, and steup/copy all other commands' data.
*
* @param i_cmd - the 8-bit command that was received over I2C
*
* @return none 
*/
void i2cCommandRead(uint8_t i_cmd)
{
    DEBUG("I2C Command Read: 0x%02x\n",i_cmd);

    if( NO_CHANGE == i_cmd )
    {
        // Leave the command at what is was set to last time and don't copy
        // anything
    } 
    else if( i_cmd == 0xFE)
    {
    	// Debug Mode Setting
        // Set to invalid command to get a canned zero response
        G_i2cLastCommand = INVALID_CMD;

        // Copy data into response register
        G_i2cResponseRegisters[G_i2cLastCommand] = (G_debug.fields.word > 0) ? (G_debug.fields.word & 0x000000FF) : 0xFF;    	
    }    
    else if( i_cmd >= INVALID_CMD)
    {
        // Set to invalid command to get a canned zero response
        G_i2cLastCommand = INVALID_CMD;

        // Copy data into response register
        G_i2cResponseRegisters[G_i2cLastCommand] = *((uint8_t *) G_i2cResponseDataPtrs[G_i2cLastCommand]);
    }
    else if(  (i_cmd >= ADC_CH0_LSB) && (i_cmd <= ADC_CH15_MSB) )
    {
        // This is an ADC Command, and we have to treat it special so that the 
        // MSB is locked on a read of the LSB.
        switch(i_cmd)
        {
            case ADC_CH0_LSB:   // FALLTHROUGH   
            case ADC_CH1_LSB:   // FALLTHROUGH
            case ADC_CH2_LSB:   // FALLTHROUGH
            case ADC_CH3_LSB:   // FALLTHROUGH
            case ADC_CH4_LSB:   // FALLTHROUGH
            case ADC_CH5_LSB:   // FALLTHROUGH
            case ADC_CH6_LSB:   // FALLTHROUGH
            case ADC_CH7_LSB:   // FALLTHROUGH
            case ADC_CH8_LSB:   // FALLTHROUGH
            case ADC_CH9_LSB:   // FALLTHROUGH
            case ADC_CH10_LSB:  // FALLTHROUGH
            case ADC_CH11_LSB:  // FALLTHROUGH
            case ADC_CH12_LSB:  // FALLTHROUGH
            case ADC_CH13_LSB:  // FALLTHROUGH
            case ADC_CH14_LSB:  // FALLTHROUGH
            case ADC_CH15_LSB:  // FALLTHROUGH
                // Copy data into response register on a LSB access
                // but on a MSB access, don't do any copy
                i2cGrabAdcChannel(i_cmd); break;
            default:
                // On a MSB access, don't do any copy because the
                // data should have been locked on the LSB access.
                break;
        }

        // Set up global for register index
        G_i2cLastCommand = i_cmd;
    }
    else
    {
        // Copy data into response register
        G_i2cResponseRegisters[i_cmd] = *((uint8_t *) G_i2cResponseDataPtrs[i_cmd]);

        // Set up global for register index
        G_i2cLastCommand = i_cmd;
    }
    DEBUG("Copied 0x%02x into [0x%02x:0x%02x]\n",G_i2cResponseRegisters[i_cmd], i_cmd,G_i2cLastCommand );
}


/**
* Fill out the I2C Response when asked for it
*
* @param none - but G_i2cResponseRegisters & G_i2cLastCommand used
*
* @return l_rsp - the i2c response data
*/
uint8_t i2cResponse(void)
{
  uint8_t l_rsp = 0;

  DEBUG("Response from Idx: 0x%02x\n",G_i2cLastCommand);
  l_rsp = G_i2cResponseRegisters[G_i2cLastCommand];

  DEBUG("Response:  0x%02x\n",l_rsp);

  return l_rsp;
}




/**
* Handle a I2C command from the FSP
*
* @param i_msg - 16-bit SPI command
*
* @return none 
*/
uint8_t i2cTransaction(uint8_t * i_buf, uint8_t i_len)
{
    uint8_t l_addr = i_buf[0];
    uint8_t l_cmd  = i_buf[1];
    uint8_t l_data = i_buf[2];

    DEBUG("Buffer Length: %d\n",i_len);

    if(G_i2cAddress == l_addr)
    {
        if( i_len >= 3)
        {
           i2cCommandWrite(l_cmd,l_data);
        }
        i2cCommandRead(l_cmd);
    }
    else
    { 
       DEBUG("I2C Address Error!\n");
    }
        
    return i2cResponse();
}
