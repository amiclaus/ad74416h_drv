/***************************************************************************//**
 *   @file   ad74416h.c
 *   @brief  Source file of AD74416H Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ad74416h.h"
#include "no_os_crc8.h"
#include "no_os_delay.h"
#include "no_os_error.h"
#include "no_os_util.h"
#include "no_os_alloc.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD74416H_CRC_POLYNOMIAL 	0x7
#define AD74416H_DIN_DEBOUNCE_LEN 	NO_OS_BIT(5)
#define AD77416H_DEV_ADDRESS_MSK	NO_OS_GENMASK(5, 4)

/******************************************************************************/
/************************ Variable Declarations ******************************/
/******************************************************************************/
NO_OS_DECLARE_CRC8_TABLE(_crc_table);

static const unsigned int ad74416h_debounce_map[AD74416H_DIN_DEBOUNCE_LEN] = {
	0,     13,    18,    24,    32,    42,    56,    75,
	100,   130,   180,   240,   320,   420,   560,   750,
	1000,  1300,  1800,  2400,  3200,  4200,  5600,  7500,
	10000, 13000, 18000, 24000, 32000, 42000, 56000, 75000,
};

/** The time required for an ADC conversion by rejection (us) */
static const uint32_t conv_times_ad74416h[] = { 208, 833, 50000, 100000 };


/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Load the address and value in a communication buffer using
 * the format that the device expects.
 * @param reg - Register address.
 * @param val - Register value.
 * @param buff - The communication buffer.
 */
static void ad74416h_format_reg_write(uint8_t addr, uint8_t reg, uint16_t val,
				uint8_t *buff)
{
	buff[0] = no_os_field_prep(AD77416H_DEV_ADDRESS_MSK, addr);
	buff[1] = reg;
	no_os_put_unaligned_be16(val, &buff[2]);
	buff[4] = no_os_crc8(_crc_table, buff, 4, 0);
}

/**
 * @brief Read a raw communication frame
 * @param desc - The device structure.
 * @param addr - The register's address.
 * @param val - A raw comm frame.
 * @return 0 in case of success, negative error otherwise.
 */
int ad74416h_reg_read_raw(struct ad74416h_desc *desc, uint32_t addr,
			  uint8_t *val)
{
	int ret;
	/**
	 * Reading a register on AD74416H requires writing the address to the READ_SELECT
	 * register first and then doing another spi read, which will contain the requested
	 * register value.
	 */
	ad74416h_format_reg_write(desc->dev_addr, AD74416H_READ_SELECT, addr, desc->comm_buff);

	ret = no_os_spi_write_and_read(desc->spi_desc, desc->comm_buff,
				       AD74416H_FRAME_SIZE);
	if (ret)
		return ret;

	return no_os_spi_write_and_read(desc->spi_desc, val, AD74416H_FRAME_SIZE);
}

/**
 * @brief Write a register's value
 * @param desc  - The device structure.
 * @param addr - The register's address.
 * @param val - The register's value.
 * @return 0 in case of success, negative error otherwise
 */
int ad74416h_reg_write(struct ad74416h_desc *desc, uint32_t addr, uint16_t val)
{
	ad74416h_format_reg_write(desc->dev_addr, addr, val, desc->comm_buff);

	return no_os_spi_write_and_read(desc->spi_desc, desc->comm_buff,
					AD74416H_FRAME_SIZE);
}


/**
 * @brief Read a register's value
 * @param desc  - The device structure.
 * @param addr - The register's address.
 * @param val - The register's read value.
 * @return 0 in case of success, negative error otherwise
 */
int ad74416h_reg_read(struct ad74416h_desc *desc, uint32_t addr, uint16_t *val)
{
	int ret;
	uint8_t expected_crc;

	ret = ad74416h_reg_read_raw(desc, addr, desc->comm_buff);
	if (ret)
		return ret;

	expected_crc = no_os_crc8(_crc_table, desc->comm_buff, 4, 0);
	if (expected_crc != desc->comm_buff[4])
		return -EINVAL;

	*val = no_os_get_unaligned_be16(&desc->comm_buff[2]);

	return 0;
}

/**
 * @brief Update a register's field.
 * @param desc  - The device structure.
 * @param addr - The register's address.
 * @param val - The register's value.
 * @param mask - The mask for a specific register field.
 * @return 0 in case of success, negative error otherwise.
 */
int ad74416h_reg_update(struct ad74416h_desc *desc, uint32_t addr,
			uint16_t mask,
			uint16_t val)
{
	int ret;
	uint16_t data;

	ret = ad74416h_reg_read(desc, addr, &data);
	if (ret)
		return ret;

	data &= ~mask;
	data |= no_os_field_prep(mask, val);

	return ad74416h_reg_write(desc, addr, data);
}

/**
 * @brief Perform a soft reset.
 * @param desc - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad74416h_reset(struct ad74416h_desc *desc)
{
	int ret;

	ret = ad74416h_reg_write(desc, AD74416H_CMD_KEY, AD74416H_CMD_KEY_RESET_1);
	if (ret)
		return ret;

	return ad74416h_reg_write(desc, AD74416H_CMD_KEY, AD74416H_CMD_KEY_RESET_2);
}

/**
 * @brief Comm test function
 * @param desc - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int ad74416h_scratch_test(struct ad74416h_desc *desc)
{
	int ret;
	uint16_t val;
	uint16_t test_val = 0x1234;

	ret = ad74416h_reg_write(desc, AD74416H_SCRATCH(0), test_val);
	if (ret)
		return ret;

	ret = ad74416h_reg_read(desc, AD74416H_SCRATCH(0), &val);
	if (ret)
		return ret;

	if (val != test_val)
		return -EINVAL;

	return 0;
}

/**
 * @brief Initialize the device structure.
 * @param desc - The device structure to be initialized.
 * @param init_param - Initialization parameter for the device descriptor.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad74416h_init(struct ad74416h_desc **desc,
		  struct ad74416h_init_param *init_param)
{
	int ret;
	struct ad74416h_desc *descriptor;

	if (!init_param)
		return -EINVAL;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = no_os_spi_init(&descriptor->spi_desc, &init_param->spi_ip);
	if (ret)
		goto err;

	descriptor->dev_addr = init_param->dev_addr;

	no_os_crc8_populate_msb(_crc_table, AD74416H_CRC_POLYNOMIAL);

	ret = ad74416h_reset(descriptor);
	if (ret)
		goto comm_err;

	ret = ad74416h_scratch_test(descriptor);
	if (ret)
		goto comm_err;

	*desc = descriptor;

	return 0;

comm_err:
	no_os_spi_remove(descriptor->spi_desc);
err:
	no_os_free(descriptor);

	return ret;
}

/**
 * @brief Free the device descriptor.
 * @param desc - The device structure.
 * @return 0 in case of success, -EINVAL otherwise.
 */
int ad74416h_remove(struct ad74416h_desc *desc)
{
	int ret;

	if (!desc)
		return -EINVAL;

	ret = no_os_spi_remove(desc->spi_desc);
	if (ret)
		return ret;

	no_os_free(desc);

	return 0;
}


