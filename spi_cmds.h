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



#ifndef _SPI_CMDS_H
#define _SPI_CMDS_H

#include "apss_defines.h"

//---------------------------------------------------------
// Defines
//---------------------------------------------------------

/** Number of SPI Commands supported */
#define APSS_MAX_MODES 16

/**
 * @enum  eApssMode
 * @brief APSS Commands Enumeration
 */
typedef enum
{                                    // D15:D12
  APSS_MODE_NOCHANGE           = 0,  // 0000
  APSS_MODE_MANUAL_ADC         = 1,  // 0001
  APSS_MODE_STREAM_TEMPERATURE = 2,  // 0010
  APSS_MODE_AUTO_2_STREAM_DATA = 3,  // 0011
  APSS_MODE_GPIO_CONFIG        = 4,  // 0100
  APSS_MODE_GPIO_DRIVE_SEL     = 5,  // 0101
  APSS_MODE_GPIO_INTERRUPT_MSK = 6,  // 0110
  APSS_MODE_GPIO_PORT_STREAM   = 7,  // 0111
  APSS_MODE_COMPOSITE_DATA     = 8,  // 1000
  APSS_MODE_INVALID
} eApssMode;


//---------------------------------------------------------
// Structs
//---------------------------------------------------------

typedef uint16_t (*stream_func_ptr)(uint16_t);

/** @struct spi_rsp_msg_t
 *  @brief This structure holds the generic SPI response type
 *  @var spi_rsp_msg_t::payload
 *    Member 'payload' contains the data that APSS is returning
 */
typedef union
{
  struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t payload   : 12;  // D11-D0
    uint16_t toggle    :  2;  // D15-D12
    uint16_t softoc    :  1;  // D15-D12
    uint16_t interrupt_bit :  1;  // D15-D12
#else
    uint16_t interrupt_bit :  1;  // D15-D12
    uint16_t softoc    :  1;  // D15-D12
    uint16_t toggle    :  2;  // D15-D12
    uint16_t payload   : 12;  // D11-D0
#endif
  }fields;
  struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t __payload : 12;   // D11-D0
    uint16_t seq_count :  4;  // D15-D12
#else
    uint16_t seq_count :  4;  // D15-D12
    uint16_t __payload : 12;   // D11-D0  
#endif
  }fields2;
  uint16_t word;
} spi_rsp_msg_t;


/** @struct spi_cmd_msg_t
 *  @brief This structure holds the generic SPI message type
 *  @var spi_cmd_msg_t::cmd
 *    Member 'cmd' contains the cmd that APSS should handle
 *  @var spi_cmd_msg_t::payload
 *    Member 'payload' contains the data that APSS is receiving
 */
typedef union
{
  struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t payload : 12;  // D11-D0
    uint16_t cmd     :  4;  // D15-D12
#else
    uint16_t cmd     :  4;  // D15-D12
    uint16_t payload : 12;  // D11-D0
#endif
  }fields;
  uint16_t word;
} spi_cmd_msg_t;


//-------------------------------------
// Manual Mode
//-------------------------------------

/** @struct spi_cmd_manual_mode_t
 *  @brief The command structure for Manual Mode
 *  @var spi_cmd_manual_mode_t::cmd
 *    Member 'cmd' contains the cmd that APSS should handle
 */
typedef struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t _unused_1 :  7;  // D6-0
    uint16_t adc_chan  :  4;  // D10-D7
    uint16_t _unused_0 :  1;  // D11
    uint16_t cmd       :  4;  // D15-D12
#else
    uint16_t cmd       :  4;  // D15-D12
    uint16_t _unused_0 :  1;  // D11
    uint16_t adc_chan  :  4;  // D10-D7
    uint16_t _unused_1 :  7;  // D6-0
#endif
} spi_cmd_manual_mode_t;


//-------------------------------------
// Auto-2 Mode
//-------------------------------------

/** @struct spi_cmd_auto2_mode_t
 *  @brief The command structure for Auto-2 Mode
 *  @var spi_cmd_auto2_mode_t::cmd
 *    Member 'cmd' contains the cmd that APSS should handle
 */
typedef struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t _unused_0       :  5;  // D4-D0
    uint16_t toggle          :  1;  // D5
    uint16_t adc_last_chan   :  4;  // D9-6
    uint16_t adc_chan_reset  :  1;  // D10
    uint16_t new_cfg_bits    :  1;  // D11
    uint16_t cmd             :  4;  // D15-D12
#else
    uint16_t cmd             :  4;  // D15-D12
    uint16_t new_cfg_bits    :  1;  // D11
    uint16_t adc_chan_reset  :  1;  // D10
    uint16_t adc_last_chan   :  4;  // D9-6
    uint16_t toggle          :  1;  // D5
    uint16_t _unused_0       :  5;  // D4-D0
#endif
} spi_cmd_auto2_mode_t;


//-------------------------------------
// Composite Mode
//-------------------------------------

/** @struct spi_cmd_composite_mode_t
 *  @brief The command structure for Auto-2 Mode
 *  @var spi_cmd_auto2_mode_t::cmd
 *    Member 'cmd' contains the cmd that APSS should handle
 */
typedef struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t gpio_last_port  :  2;  // D1-D0
    uint16_t _unused_0       :  4;  // D5-D2
    uint16_t adc_last_chan   :  4;  // D9-D6
    uint16_t adc_chan_reset  :  1;  // D10
    uint16_t new_cfg_bits    :  1;  // D11
    uint16_t cmd             :  4;  // D15-D12
#else
    uint16_t cmd             :  4;  // D15-D12
    uint16_t new_cfg_bits    :  1;  // D11
    uint16_t adc_chan_reset  :  1;  // D10
    uint16_t adc_last_chan   :  4;  // D9-D6
    uint16_t _unused_0       :  4;  // D5-D2
    uint16_t gpio_last_port  :  2;  // D1-D0
#endif
} spi_cmd_composite_mode_t;


//-------------------------------------
// Gpio Data Stream Mode
//-------------------------------------

/** @struct spi_cmd_gpio_data_mode_t
 *  @brief The command structure for GPIO Mode
 *  @var spi_cmd_gpio_data_mode_t::cmd
 *    Member 'cmd' contains the cmd that APSS should handle
 */
typedef struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t clear_latch     :  1;  // D0
    uint16_t _unused_0       :  7;  // D7-D1
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t cmd             :  4;  // D15-D12
#else
    uint16_t cmd             :  4;  // D15-D12
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t _unused_0       :  7;  // D7-D1
    uint16_t clear_latch     :  1;  // D0
#endif
} spi_cmd_gpio_data_mode_t;


/** @struct spi_rsp_gpio_data_mode_t
 *  @brief The command structure for GPIO Data Mode Response
 */
typedef union
{
struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t gpio_port  :  8;  // D7-D0
    uint16_t soft_oc_3  :  1;  // D8
    uint16_t soft_oc_2  :  1;  // D9
    uint16_t soft_oc_1  :  1;  // D10
    uint16_t soft_oc_0  :  1;  // D11
    uint16_t _reserved0 :  2;  // D13-D12
    uint16_t softoc     :  1;  // D14
    uint16_t interrupt_bit  :  1;  // D15
#else
    uint16_t interrupt_bit  :  1;  // D15
    uint16_t softoc     :  1;  // D14
    uint16_t _reserved0 :  2;  // D13-D12
    uint16_t soft_oc_0  :  1;  // D11
    uint16_t soft_oc_1  :  1;  // D10
    uint16_t soft_oc_2  :  1;  // D9
    uint16_t soft_oc_3  :  1;  // D8
    uint16_t gpio_port  :  8;  // D7-D0
#endif
  }fields;
uint16_t word;
} spi_rsp_gpio_data_mode_t;

//-------------------------------------
// Gpio Port Config Mode
//-------------------------------------

/** @struct spi_cmd_gpio_config_mode_t
 *  @brief The command structure for GPIO Port Config cmd
 */
typedef struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t payload         :  8;  // D7-D0
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t cmd             :  4;  // D15-D12
#else
    uint16_t cmd             :  4;  // D15-D12
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t payload         :  8;  // D7-D0
#endif
} spi_cmd_gpio_config_mode_t;


//-------------------------------------
// Gpio Port Drive Sel Mode
//-------------------------------------

/** @struct spi_cmd_gpio_drive_sel_mode_t
 *  @brief The command structure for GPIO Drive Sel Mode
 */
typedef struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t payload         :  8;  // D7-D0
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t cmd             :  4;  // D15-D12
#else
    uint16_t cmd             :  4;  // D15-D12
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t payload         :  1;  // D7-D0
#endif
} spi_cmd_gpio_drive_sel_mode_t;


//-------------------------------------
// Gpio Port Interrupt Mask Config Mode
//-------------------------------------

/** @struct spi_cmd_gpio_int_mask_mode_t
 *  @brief The command structure for GPIO Interrupt Mask
 */
typedef struct
  {
#ifdef __LITTLE_ENDIAN
    uint16_t payload         :  8;  // D7-D0
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t cmd             :  4;  // D15-D12
#else
    uint16_t cmd             :  4;  // D15-D12
    uint16_t gpio_port       :  4;  // D11-D8
    uint16_t payload         :  8;  // D7-D0
#endif
} spi_cmd_gpio_int_mask_mode_t;

/****************************************************************************
 * Functions
 ****************************************************************************/
void spiCommand(spi_cmd_msg_t * i_cmd);
uint16_t spiResponse(spi_cmd_msg_t * i_cmd);


#if 0
//-------------------------------------
// Endian-ness Test
//-------------------------------------
typedef union
{
  struct 
  {
    uint32_t a:4;
    uint32_t b:4;
    uint32_t c:4;
    uint32_t d:4;
    uint32_t e:4;
    uint32_t f:4;
    uint32_t g:4;
    uint32_t h:4;
  } ;
  uint32_t word;
}u32;

typedef union
{
  struct 
  {
    uint32_t a:4;
    uint32_t b:4;
    uint32_t c:4;
    uint32_t d:4;
  } ;
  uint16_t word;
}u16;
#endif

#endif
