/***************************************************************************//**
 *   @file   ad74416h.h
 *   @brief  Header file of AD74416h Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef _AD74416H_H
#define _AD74416H_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "stdint.h"
#include "stdbool.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define AD74416H_N_CHANNELS             4

#define AD74416H_CH_A                   0
#define AD74416H_CH_B                   1
#define AD74416H_CH_C                   2
#define AD74416H_CH_D                   3

/** The value of the sense resistor in ohms */
#define AD74416H_RSENSE                 12
/** 16 bit ADC */
#define AD74416H_ADC_MAX_VALUE		16777215

/** Register map */
#define AD74416H_NOP				0x00
#define AD74416H_CH_FUNC_SETUP(x)               (0x01 + (x * 12))
#define AD74416H_ADC_CONFIG(x)                  (0x02 + (x * 12))
#define AD74416H_DIN_CONFIG0(x)			(0x03 + (x * 12))
#define AD74416H_DIN_CONFIG1(x)			(0x04 + (x * 12))
#define AD74416H_OUTPUT_CONFIG(x)               (0x05 + (x * 12))
#define AD74416H_RTD_CONFIG(x)			(0x06 + (x * 12))
#define AD74416H_FET_LKG_COMP(x)		(0x07 + (x * 12))
#define AD74416H_DO_EXT_CONFIG(x)		(0x08 + (x * 12))
#define AD74416H_I_BURNOUT_CONFIG(x)		(0x09 + (x * 12))
#define AD74416H_DAC_CODE(x)			(0x0A + (x * 12))
#define AD74416H_DAC_ACTIVE(x)			(0x0C + (x * 12))
#define AD74416H_GPIO_CONFIG(x)			(0x32 + x)
#define AD74416H_PWR_OPTIM_CONFIG		0x38
#define AD74416H_ADC_CONV_CTRL			0x39
#define AD74416H_DIAG_ASSIGN			0x3A
#define AD74416H_WTD_CONFIG			0x3B
#define AD74416H_DIN_COMP_OUT			0x3E
#define AD74416H_ALERT_STATUS			0x3F
#define AD74416H_LIVE_STATUS			0x40
#define AD74416H_ADC_RESULT_UPR(x)		(0x41 + (x * 2))
#define AD74416H_ADC_RESULT(x)			(0x42 + (x * 2))
#define AD74416H_ADC_DIAG_RESULT(x)		(0x49 + x)
#define AD74416H_LAST_ADC_RESULT_UPR		0x4D
#define AD74416H_LAST_ADC_RESULT		0x4E
#define AD74416H_DIN_COUNTER(x)			(0x50 + (x * 2))
#define AD74416H_SUPPLY_ALERT_STATUS		0x57
#define AD74416H_CHANNEL_ALERT_STATUS(x)	(0x58 + x)
#define AD74416H_ALERT_MASK			0x5C
#define AD74416H_SUPPLY_ALERT_MASK		0x5D
#define AD74416H_CHANNEL_ALERT_MASK(x)		(0x5E + x)
#define AD74416H_READ_SELECT			0x6A
#define AD74416H_BURST_READ_SEL			0x6B
#define AD74416H_THERM_RST			0x75
#define AD74416H_CMD_KEY			0x76
#define AD74416H_BORADCAST_CMD_KEY		0x77
#define AD74416H_SCRATCH(x)			(0x78 + x)
#define AD74416H_GENERIC_ID			0x7A
#define AD74416H_SILICON_REV			0x7B
#define AD74416H_SILICON_ID0			0x7D
#define AD74416H_SILICON_ID1			0x7E
#define AD74416H_HART_ALERT_STATUS(x)		(0x80 + (x * 16))
#define AD74416H_HART_RX(x)			(0x81 + (x * 16))
#define AD74416H_HART_TX(x)			(0x82 + (x * 16))
#define AD74416H_HART_FCR(x)			(0x83 + (x * 16))
#define AD74416H_HART_MCR(x)			(0x84 + (x * 16))
#define AD74416H_HART_RFC(x)			(0x85 + (x * 16))
#define AD74416H_HART_TFC(x)			(0x86 + (x * 16))
#define AD74416H_HART_ALERT_MASK(x)		(0x87 + (x * 16))
#define AD74416H_HART_CONFIG(x)			(0x88 + (x * 16))
#define AD74416H_HART_TX_PREM(x)		(0x89 + (x * 16))
#define AD74416H_HART_EVDET(x)			(0x8A + (x * 16))
#define AD74416H_HART_TX_GAIN(x)		(0x8B + (x * 16))
#define AD74416H_HART_GPIO_IF_CONFIG		0xC0
#define AD74416H_HART_GPIO_MON_CONFIG(x)	(0xC1 + x)

/** Software reset sequence */
#define AD74416H_CMD_KEY_RESET_1                0x15FA
#define AD74416H_CMD_KEY_RESET_2                0xAF51

#define AD74416H_SPI_RD_RET_INFO_MSK		NO_OS_BIT(8)
#define AD74416H_ERR_CLR_MSK			NO_OS_GENMASK(15, 0)
#define AD74416H_SPI_CRC_ERR_MSK		NO_OS_BIT(13)

/* AD74416H_CH_FUNC_SETUP */
#define AD74416H_CH_FUNC_SETUP_MSK		NO_OS_GENMASK(3, 0)

/* AD74416H_ADC_CONFIG */
#define AD74416H_ADC_CONV_RATE_MSK		NO_OS_GENMASK(10, 8)
#define AD74416H_ADC_CONV_RANGE_MSK		NO_OS_GENMASK(6, 4)
#define AD74416H_CONV_MUX_MSK			NO_OS_GENMASK(2, 0)

/* AD74416H_DIN_CONFIG0 */
#define AD74416H_COUNT_EN_MSK			NO_OS_BIT(15)
#define AD74416H_DIN_INV_COMP_OUT_MSK		NO_OS_BIT(14)
#define AD74416H_COMPARATOR_EN_MSK		NO_OS_BIT(13)
#define AD74416H_DIN_SINK_RANGE_MSK		NO_OS_BIT(12)
#define AD74416H_DIN_SINK_MSK			NO_OS_GENMASK(11, 7)
#define AD74416H_DEBOUNCE_MODE_MSK		NO_OS_BIT(6)
#define AD74416H_DEBOUNCE_TIME_MSK		NO_OS_BIT(4, 0)

#define AD74416H_FRAME_SIZE 			5

#endif // _AD74416H_H
