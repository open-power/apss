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
#include "spi_cmds.h"
#include "gpio.h"
#include "DSP2803x_Device.h"

/****************************************************************************
 * Globals
 ****************************************************************************/

/** @brief Holds the current mode that the APSS is in, based on last cmd */
uint8_t  G_apssMode        = APSS_MODE_INVALID;

/** @brief Holds config of current mode that APSS is in, based on last cmd */
uint16_t G_apssModeConfig  = 0;

/** @brief Holds value of last toggle for when spi rsp is set to toggle */
uint8_t G_spiToggle        = 0x01;


/****************************************************************************
 * Functions
 ****************************************************************************/

/**
* Return toggle code for a spi response
*
* For responses that suppot toggle codes, need to send back "01" or "10"
* with "01" as the starting toggle count when a new command frame is sent.
* Only the lowest 2 bits of G_spiToggle are used
*
* @param i_reset - reset the spi Toggle to "01"
*
* @return (G_spiToggle & 0x03)
*          return 2 bits, alternating between '01' or '10'
*/
uint8_t spiGetToggleCode(uint8_t i_reset)
{
  if (0 != i_reset)
  { 
      G_spiToggle = 0x01; 
  }
  else
  {
      G_spiToggle += 0x02; 
  }

  return (G_spiToggle & 0x03);
}


/**
* Return interrupt bits for a spi response
*
* @param none
*
* @return 0
*          
*/
uint8_t spiGetInterrupt()
{
  uint8_t l_interrupt = 0;
  uint8_t l_irq = 0;
  uint8_t l_int_mask  = 0;
  uint8_t l_idx       = 0;
  
  for(l_idx=0; l_idx<NUM_GPIO_PORTS; l_idx++)
  {
      // Grab any IRQ interrupts that occured
      ////DISABLE_ALL_INTERRUPTS();
      l_irq = G_gpioIrqInterrupt[l_idx];
      G_gpioIrqInterrupt[l_idx] = 0;
      ////ENABLE_ALL_INTERRUPTS();

      // Start with the Mask as set by SPI mode
      l_int_mask = G_gpioInterruptMask[l_idx];
      // Then mask off all outputs (output = 1)
      l_int_mask &= ~G_gpioIoConfig[l_idx];

      // Then apply mask to the gpio pins
      l_interrupt |= (l_irq & l_int_mask);
      l_interrupt |= (G_gpioPort[l_idx] & l_int_mask);
  }

  return ( (l_interrupt) ? 1 : 0 );
}


/**
* Return interrupt bits for a spi response
*
* @param none
*
* @return 0
*          
*/
uint8_t spiGetSoftOCAlarm()
{
  return 0;
}


/**
* Return zeros for invalid SPI command
*
* At reset, if the master tries to read from the APSS (by sending command 
* code 0x0000) the APSS will return 0's on MISO until a proper command code 
* has been sent
*
* @param none
*
* @return
*   0x0000 This function will always return zero
*/
uint16_t spiStreamReturnZero(uint16_t i_command)
{
  DEBUG("Response with Zeros\n");

  return 0x0000;
}


/**
* Return value of manually specified ADC Channel
*
* This command allows the master to specify which channels should
* be returned in the following frame.  It can be used to get an  
* individual channel once, or the same channel over & over via stream.
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      The MSB  4 bits will be Interrupt, SoftOC, Toggle
*                      The LSB 12 bits will be the ADC channel value.
*/
uint16_t spiStreamManualMode(uint16_t i_command)
{
  spi_cmd_manual_mode_t * l_mode = (spi_cmd_manual_mode_t *) &i_command;
  spi_rsp_msg_t           l_rsp;
  static uint8_t          L_adcChan = 0;

  // Handle Mode Changes
  if(0 != l_mode->cmd)
  {
     L_adcChan = l_mode->adc_chan;
  }

  // Build Up Reeponse
  l_rsp.word      = 0;
  l_rsp.fields.payload   = G_adcChan[L_adcChan];
  l_rsp.fields.interrupt_bit = spiGetInterrupt(); 
  l_rsp.fields.softoc    = spiGetSoftOCAlarm();
  l_rsp.fields.toggle    = 0;  // This isn't used for Manual Mode

  return l_rsp.word;
}


/**
* Return current Altitude as read by APSS
*
* This command is deprecated and should not be used.
* 
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      The MSB  4 bits will be Interrupt, SoftOC, Toggle
*                      The LSB 12 bits will be the Altitude
*/
uint16_t spiStreamAltitudeMode(uint16_t i_command)
{
  //spi_cmd_msg_t * l_mode = (spi_cmd_msg_t *) &i_command;
  spi_rsp_msg_t l_rsp;

  l_rsp.word    = 0;
  l_rsp.fields.payload = G_apssSoftwareRev;  // Altitude

  return l_rsp.word;
}


/**
* Return value of manually specified GPIO Port
*
* This command allows the master to specify which gpio port should
* be returned in the following frame.  It can be used to get an  
* individual port once, or the same port over & over via stream.
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      The MSB  2 bits will be Interrupt, SoftOC
*                      The D13:D8 bits will be 0, they are reserved
*                      The LSB  8 bits will be the GPIO port values.
*/
uint16_t spiStreamGpioPortMode(uint16_t i_command)
{
  static uint8_t L_gpioPortNum = 0;

  spi_cmd_gpio_data_mode_t * l_mode = (spi_cmd_gpio_data_mode_t *) &i_command;
  spi_rsp_gpio_data_mode_t l_rsp;

  // Handle Mode Changes
  if(0 != l_mode->cmd)
  {
      if(NUM_GPIO_PORTS > l_mode->gpio_port)
      {
          L_gpioPortNum = l_mode->gpio_port;
      }
      DEBUG("Gpio Mode: Cfg=%d, Port=%d\n", l_mode->gpio_port, L_gpioPortNum);
  }

  // Build response
  l_rsp.fields.gpio_port  = G_gpioPort[L_gpioPortNum];

  l_rsp.fields.interrupt_bit     = spiGetInterrupt(); 
  l_rsp.fields.softoc        = spiGetSoftOCAlarm();
  l_rsp.fields._reserved0    = 0;

  l_rsp.fields.soft_oc_0     = (G_softOcPins & 0x08) ? 1 : 0;
  l_rsp.fields.soft_oc_1     = (G_softOcPins & 0x04) ? 1 : 0;
  l_rsp.fields.soft_oc_2     = (G_softOcPins & 0x02) ? 1 : 0;
  l_rsp.fields.soft_oc_3     = (G_softOcPins & 0x01) ? 1 : 0;

  if(l_mode->clear_latch == 0)
  {
  	gpioResetLatch();
  }

  return l_rsp.word;
}


/**
* Return next value in stream of specified ADC Channels
*
* This command allows the master to specify which ADC channels should
* be returned in the following frames.  It is used to grab all specified 
* ADC values via stream.
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      Depending on bit D5 of SPI Command:
*                         The MSB  4 bits will be Interrupt, SoftOC, Toggle
*                         The MSB  4 bits will be ADC Channel Number
*                      The LSB 12 bits will be the next ADC channel value.
*/
uint16_t spiStreamAuto2Mode(uint16_t i_command)
{
  static spi_cmd_auto2_mode_t L_auto2ModeLastConfig = {0};
  static uint8_t L_adcChan = 0;

  spi_cmd_auto2_mode_t * l_mode = (spi_cmd_auto2_mode_t *) &i_command;
  spi_rsp_msg_t l_rsp;

  // Handle Mode Changes
  if(0 != l_mode->cmd)
  {
      DEBUG("Auto-2: Cfg=%d, Rst=%d, Adc=%d, Tog=%d\n", 
              l_mode->new_cfg_bits,
              l_mode->adc_chan_reset,
              l_mode->adc_last_chan, 
              l_mode->toggle);

      // reset spi
	  SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI
	  SpiaRegs.SPICCR.bit.SPISWRESET=1; // Enable SPI

      if(l_mode->new_cfg_bits)
      {
          if(l_mode->adc_chan_reset)
          {
              L_adcChan = 0;
              l_mode->adc_chan_reset = 0;
          }

          // Other bits can just be handled as is.
          L_auto2ModeLastConfig = *l_mode;
      }
  }
          
  // Build response
  l_rsp.fields.payload  = G_adcChan[L_adcChan];

  if(L_auto2ModeLastConfig.toggle)
  {
    l_rsp.fields.interrupt_bit = spiGetInterrupt(); 
    l_rsp.fields.softoc    = spiGetSoftOCAlarm(); 
    l_rsp.fields.toggle    = spiGetToggleCode(l_mode->cmd);  
  }
  else
  {
    l_rsp.fields2.seq_count = L_adcChan;
  }

  L_adcChan++;
  if(L_adcChan > L_auto2ModeLastConfig.adc_last_chan){L_adcChan = 0;}
  
  return l_rsp.word;
}


/**
* Return next value in stream of specified ADC Channels & GPIO Ports
*
* This command allows the master to specify which ADC channels & GPIO ports
* be returned in the following frames.  It is used to grab all specified 
* ADC values & GPIO ports via stream.
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      Depending on bit D5 of SPI Command:
*                         The MSB  4 bits will be Interrupt, SoftOC, Toggle
*                         The MSB  4 bits will be ADC Channel Number
*                      The LSB 12 bits will be the next ADC channel value or
*                        the next GPIO port
*/
uint16_t spiStreamCompositeMode(uint16_t i_command)
{
  static spi_cmd_composite_mode_t L_compositeModeLastConfig = {0};
  static uint8_t L_adcChan = 0;
  static uint8_t L_gpioPort = 0;
  static uint8_t L_doGpio = 0;

  spi_cmd_composite_mode_t * l_mode = (spi_cmd_composite_mode_t *) &i_command;
  spi_rsp_msg_t l_rsp;

  // Handle Mode Changes
  if(0 != l_mode->cmd)
  {
      DEBUG("Composite: Cfg=%d, Rst=%d, Adc=%d, Gpio=%d\n", 
              l_mode->new_cfg_bits,
              l_mode->adc_chan_reset,
              l_mode->adc_last_chan, 
              l_mode->gpio_last_port);

      // NOTE - we should be resetting L_doGpio, L_gpioPort to 0 in here
      // JRK - also checking that L_adcChan never exceeds 15 (maximum readings)
      L_gpioPort = 0;
      L_doGpio = 0;
      // We always aassume a new mode change results in resetting the ADC channels for composite mode
      // This is a simplification to avoid the user having to set the new_cfg_bits/adc_chan_reset parameter
      L_adcChan = 0;
      l_mode->adc_chan_reset = 0;

      if(l_mode->new_cfg_bits)
      {
          if(l_mode->adc_chan_reset)
          {
              L_adcChan = 0;
              l_mode->adc_chan_reset = 0;
          }

          L_compositeModeLastConfig = *l_mode;
      }
  }

  // Build response
  if(L_doGpio)
  {
    l_rsp.fields.payload  = ((G_softOcPins & 0x0f) << 8) | G_gpioPort[L_gpioPort];
    l_rsp.fields2.seq_count = 0;
  }
  else
  {
    l_rsp.fields.payload  = G_adcChan[L_adcChan];
    l_rsp.fields2.seq_count = L_adcChan;
  }

  if(0 == L_doGpio)
  {
      L_adcChan++;
      if(L_adcChan > L_compositeModeLastConfig.adc_last_chan)
      {
          if( L_compositeModeLastConfig.gpio_last_port > 0)
          {
              // Transition to GPIO mode, since we sent all ADCs
              L_doGpio  = 1;
          }
          L_adcChan = 0;
      }
  }
  else
  {
      L_gpioPort++;
      if(L_gpioPort >= L_compositeModeLastConfig.gpio_last_port)
      {
      	  // Transition back to ADC mode, since we sent all GPIOs
          L_doGpio  = 0;
          L_gpioPort = 0;
          // Reset GPIO latch, since we just finished reading GPIO pins
          gpioResetLatch();
      }
  }
  
  // ensure we never go outside of bounds - double check
//  if(L_adcChan > L_compositeModeLastConfig.adc_last_chan)
//  {
//    L_adcChan = 0;
//  }
  return l_rsp.word;
}


/**
* Set up GPIO Configuration
*
* This command allows the master to specify which gpio pins should
* be used as inputs vs outputs.  
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      All response bits will be set to zero
*
* @todo Should the LSB 8 bits return the GPIO port config 
*/
uint16_t spiStreamGpioConfigMode(uint16_t i_command)
{
  spi_cmd_gpio_config_mode_t * l_mode = (spi_cmd_gpio_config_mode_t *) &i_command;
  spi_rsp_msg_t l_rsp;

  // Handle Mode Changes
  if(0 != l_mode->cmd)
  {
      if(NUM_GPIO_PORTS > l_mode->gpio_port)
      {
          G_gpioIoConfig[l_mode->gpio_port] = l_mode->payload;
      }
  }
        
  // Build Response
  l_rsp.word = 0;

  // Only if we changed modes, should we send a response
  if(0 != l_mode->cmd)
  {
    l_rsp.fields.payload = G_gpioIoConfig[l_mode->gpio_port];
  }
  
  return l_rsp.word;
}


/**
* Set up GPIO Output Drive
*
* This command allows the master to specify which gpio pins should
* be driven to high-Z or GND 
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      All response bits will be set to zero
*
* @todo Should the LSB 8 bits return the GPIO port read? 
*/
uint16_t spiStreamGpioDriveSelMode(uint16_t i_command)
{
  spi_cmd_gpio_drive_sel_mode_t * l_mode = (spi_cmd_gpio_drive_sel_mode_t *) &i_command;
  spi_rsp_msg_t l_rsp;

  // Handle Mode Changes
  if(0 != l_mode->cmd)
  {
      if(NUM_GPIO_PORTS > l_mode->gpio_port)
      {
          G_gpioDriveConfig[l_mode->gpio_port] = l_mode->payload;
      }
  }

  // Build Response
  l_rsp.word = 0;

  // Only if we changed modes, should we send a response
  if(0 != l_mode->cmd)
  {
    l_rsp.fields.payload = G_gpioDriveConfig[l_mode->gpio_port];
  }

  return l_rsp.word;
}


/**
* Set up GPIO Interrupt Mask 
*
* This command allows the master to specify which gpio pins should
* be used to trip the interrupt bit.
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*                      All response bits will be set to zero
*/
uint16_t spiStreamGpioInterruptMaskMode(uint16_t i_command)
{
  spi_cmd_gpio_int_mask_mode_t * l_mode = (spi_cmd_gpio_int_mask_mode_t *) &i_command;
  spi_rsp_msg_t l_rsp;


  // Handle Mode Changes
  if(0 != l_mode->cmd)
  {
      if(NUM_GPIO_PORTS > l_mode->gpio_port)
      {
          G_gpioInterruptMask[l_mode->gpio_port] = l_mode->payload;

      }
  }

  // Build Response
  l_rsp.word = 0;

  // Only if we changed modes, should we send a response
  if(0 != l_mode->cmd)
  {
      l_rsp.fields.payload = G_gpioInterruptMask[l_mode->gpio_port];
  }

  return l_rsp.word;
}


/**
* Function Pointer Table for SPI Commands 
*
* This table holds the functions that will be called for a given
* commands that comes in over the SPI Bus
*
* All functions will use the following command, and send the following reponse
*
* @param i_command - the command that was received over SPI
*
* @return l_rsp.word - This function will return the 16bit value that will
*                      be sent back over SPI to the master.  
*/
const stream_func_ptr G_streamModeFuncPtrs[APSS_MAX_MODES] = 
{
  &spiStreamReturnZero,             // APSS_MODE_NOCHANGE           
  &spiStreamManualMode,             // APSS_MODE_MANUAL_ADC         
  &spiStreamReturnZero,             // APSS_MODE_STREAM_TEMPERATURE - obsolete?
  &spiStreamAuto2Mode,              // APSS_MODE_AUTO_2_STREAM_DATA 
  &spiStreamGpioConfigMode,         // APSS_MODE_GPIO_CONFIG        
  &spiStreamGpioDriveSelMode,       // APSS_MODE_GPIO_DRIVE_SEL     
  &spiStreamGpioInterruptMaskMode,  // APSS_MODE_GPIO_INTERRUPT_MSK 
  &spiStreamGpioPortMode,           // APSS_MODE_GPIO_PORT_STREAM   
  &spiStreamCompositeMode,          // APSS_MODE_COMPOSITE_DATA     
  &spiStreamAltitudeMode,           // APSS_MODE_ALTITUDE           - obsolete?     
  &spiStreamReturnZero,             // APSS_MODE_RESERVED_10        
  &spiStreamReturnZero,             // APSS_MODE_RESERVED_11        
  &spiStreamReturnZero,             // APSS_MODE_RESERVED_12        
  &spiStreamReturnZero,             // APSS_MODE_RESERVED_13        
  &spiStreamReturnZero,             // APSS_MODE_RESERVED_14        
  &spiStreamReturnZero,             // APSS_MODE_RESERVED_15        
};


/**
* Parse the SPI command, and set up Global Commands based on it 
*
* This function is basically used to filter the NOCHANGE command out
* from the other commands, and set up the global variables accordingly.
*
* @param i_cmd - the 16-bit command that was received over SPI
*
* @return none 
*/
void spiCommand(spi_cmd_msg_t * i_cmd)
{
  DEBUG("Command: 0x%04x  Cmd: %01x Payload: %03x\n",i_cmd->word,i_cmd->fields.cmd,i_cmd->fields.payload);

  if(i_cmd->fields.cmd != APSS_MODE_NOCHANGE)
  {
    G_apssMode       = i_cmd->fields.cmd;
    G_apssModeConfig = i_cmd->word;

    DEBUG("Switched Mode to 0x%x, w/ Config 0x%x\n",G_apssMode,G_apssModeConfig);
  }
}


/**
* Parse the SPI command, and call the correct function pointer based
* on it.
*
* This function will also filter out invalid commands.
*
* @param none - but G_apssMode & G_apssModeConfig are used
*
* @return none 
*/
uint16_t spiResponse(spi_cmd_msg_t * i_cmd)
{
  uint16_t l_rsp = 0;

  if(G_apssMode < APSS_MODE_INVALID)
  {
    //l_rsp = (*G_streamModeFuncPtrs[G_apssMode])(G_apssModeConfig);
    l_rsp = (*G_streamModeFuncPtrs[G_apssMode])(i_cmd->word);
  }
//  else {
//      // reset spi for unrecognized command
//      SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI
//      SpiaRegs.SPICCR.bit.SPISWRESET=1; // Enable SPI
//  }

  DEBUG("Response:  0x%04x\n",l_rsp);

  return l_rsp;
}


/**
* Handle a SPI command from the OCC
*
* @param i_msg - 16-bit SPI command
*
* @return none 
*/
uint16_t spiTransaction(uint8_t * i_buf, uint8_t i_len)
{
    uint8_t l_msb = i_buf[0];
    uint8_t l_lsb = i_buf[1];

    uint16_t i_msg = (l_msb);
    i_msg = i_msg << 8;
    i_msg += l_lsb;

    spi_cmd_msg_t * l_cmd = (spi_cmd_msg_t *) &i_msg;
    spiCommand(l_cmd);
    
    return spiResponse(l_cmd);
}

