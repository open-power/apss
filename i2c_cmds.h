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

#ifndef _I2C_CMDS_H
#define _I2C_CMDS_H

//---------------------------------------------------------
// Defines
//---------------------------------------------------------

/**
 * @enum  eApssI2cCommand
 * @brief APSS Commands Enumeration
 */
typedef enum
{   
    NO_CHANGE     = 0x00,    
    REV_CODE      = 0x01,
    DAC0          = 0x02,
    DAC1          = 0x03,
    DAC2          = 0x04,
    DAC3          = 0x05,
    DAC4          = 0x06,
    DAC5          = 0x07,
    DAC6          = 0x08,
    DAC7          = 0x09,
    NO_COMMAND_0A = 0x0a,
    NO_COMMAND_0B = 0x0b,
    NO_COMMAND_0C = 0x0c,
    NO_COMMAND_0D = 0x0d,
    NO_COMMAND_0E = 0x0e,
    NO_COMMAND_0F = 0x0f,
    NO_COMMAND_10 = 0x10,
    NO_COMMAND_11 = 0x11,
    NO_COMMAND_12 = 0x12,
    NO_COMMAND_13 = 0x13,
    ADC_CH0_LSB   = 0x14,
    ADC_CH0_MSB   = 0x15,
    ADC_CH1_LSB   = 0x16,
    ADC_CH1_MSB   = 0x17,
    ADC_CH2_LSB   = 0x18,
    ADC_CH2_MSB   = 0x19,
    ADC_CH3_LSB   = 0x1a,
    ADC_CH3_MSB   = 0x1b,
    ADC_CH4_LSB   = 0x1c,
    ADC_CH4_MSB   = 0x1d,
    ADC_CH5_LSB   = 0x1e,
    ADC_CH5_MSB   = 0x1f,
    ADC_CH6_LSB   = 0x20,
    ADC_CH6_MSB   = 0x21,
    ADC_CH7_LSB   = 0x22,
    ADC_CH7_MSB   = 0x23,
    ADC_CH8_LSB   = 0x24,
    ADC_CH8_MSB   = 0x25,
    ADC_CH9_LSB   = 0x26,
    ADC_CH9_MSB   = 0x27,
    ADC_CH10_LSB  = 0x28,
    ADC_CH10_MSB  = 0x29,
    ADC_CH11_LSB  = 0x2a,
    ADC_CH11_MSB  = 0x2b,
    ADC_CH12_LSB  = 0x2c,
    ADC_CH12_MSB  = 0x2d,
    ADC_CH13_LSB  = 0x2e,
    ADC_CH13_MSB  = 0x2f,
    ADC_CH14_LSB  = 0x30,
    ADC_CH14_MSB  = 0x31,
    ADC_CH15_LSB  = 0x32,
    ADC_CH15_MSB  = 0x33,
    GPIO_IN0      = 0x34,
    GPIO_IN1      = 0x35,
    GPIO_MODE0    = 0x36,
    GPIO_MODE1    = 0x37,
    GPIO_READONLY = 0x38,
    // These should always be the last 2 members
    INVALID_CMD   = 0x39,
    NUM_OF_I2C_COMMANDS
} eApssI2cCommand;

//-------------------------------------
// Globals
//-------------------------------------
extern const uint8_t G_i2cAddress;

//-------------------------------------
// Functions
//-------------------------------------
void i2cCommandRead(uint8_t i_cmd);
void i2cCommandWrite(uint8_t i_cmd, uint8_t i_data);

uint8_t i2cResponse(void);

#endif
