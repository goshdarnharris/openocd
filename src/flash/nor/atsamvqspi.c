/***************************************************************************
 *   Copyright (C) 2022 by George Harris                                   *
 *   george@luminairecoffee.com                                            *
 *																		   *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#include "../../../contrib/loaders/flash/atsamvqspi/executive.h"
#include "../../../contrib/loaders/flash/atsamvqspi/atsamv7qspi_types.h"

/* Offsets from ssp_base into config & data registers */
// #define SSP_CR0		(0x00)  /* Control register 0 */
// #define SSP_CR1		(0x04)  /* Control register 1 */
// #define SSP_DATA	(0x08)  /* Data register (TX and RX) */
// #define SSP_SR		(0x0C)  /* Status register */
// #define SSP_CPSR	(0x10)  /* Clock prescale register */

/* Status register fields */
// #define SSP_BSY		(0x00000010)

/* Timeout in ms */
// #define SSP_CMD_TIMEOUT   (100)
// #define SSP_PROBE_TIMEOUT (100)
// #define SSP_MAX_TIMEOUT  (3000)

/* Size of the stack to alloc in the working area for the execution of
 * the ROM spifi_init() function */
#define SPIFI_INIT_STACK_SIZE  512

struct atsamvqspi_flash_bank {
	bool probed;
	uint32_t bank_num;
	uint32_t max_spi_clock_mhz;
	const struct flash_device *dev;
};


struct atsamvqspi_executive {
	struct target* target;
	struct working_area* text_working_area;
	struct working_area* param_working_area;
	struct working_area* fifo_working_area;
	struct working_area* stack_working_area;
};

#define EXECUTIVE_PARAM_WORKING_AREA_SIZE 24
#define EXECUTIVE_STACK_WORKING_AREA_SIZE 1000

static void atsamvqspi_executive_free(struct atsamvqspi_executive* exec)
{
	if(exec == NULL) return;
	if(exec->text_working_area != NULL) target_free_working_area(exec->target, exec->text_working_area);
	if(exec->param_working_area != NULL) target_free_working_area(exec->target, exec->param_working_area);
	if(exec->fifo_working_area != NULL) target_free_working_area(exec->target, exec->fifo_working_area);
	if(exec->stack_working_area != NULL) target_free_working_area(exec->target, exec->stack_working_area);
	free(exec);
	return;
}

static struct atsamvqspi_executive* atsamvqspi_executive_alloc(struct flash_bank* bank)
{
	struct atsamvqspi_executive* exec = malloc(sizeof(struct atsamvqspi_executive));
	if(exec == NULL) return NULL;

	exec->target = bank->target;
	exec->text_working_area = NULL;
	exec->param_working_area = NULL;
	exec->fifo_working_area = NULL;

	const size_t min_working_area = atsamv7qspi_executive_bin_len + 
		EXECUTIVE_PARAM_WORKING_AREA_SIZE + 
		EXECUTIVE_STACK_WORKING_AREA_SIZE + 1;

	int retval = target_alloc_working_area(exec->target, atsamv7qspi_executive_bin_len,
		&exec->text_working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Insufficient working area. You must configure a working"
			" area of at least %zuB in order to use QSPI flash on this target.",
			min_working_area);
		atsamvqspi_executive_free(exec);
		return NULL;
	}

	retval = target_alloc_working_area(exec->target, EXECUTIVE_PARAM_WORKING_AREA_SIZE,
		&exec->param_working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Insufficient working area. You must configure a working"
			" area of at least %zuB in order to use QSPI flash on this target.",
			min_working_area);
		atsamvqspi_executive_free(exec);
		return NULL;
	}

	retval = target_alloc_working_area(exec->target, EXECUTIVE_STACK_WORKING_AREA_SIZE,
		&exec->stack_working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Insufficient working area. You must configure a working"
			" area of at least %zuB in order to use QSPI flash on this target.",
			min_working_area);
		atsamvqspi_executive_free(exec);
		return NULL;
	}

	/* Write algorithm to working area */
	retval = target_write_buffer(exec->target, exec->text_working_area->address,
		atsamv7qspi_executive_bin_len, atsamv7qspi_executive_bin);
	LOG_DEBUG("wrote atsamv7qspi_executive_bin to %lx", exec->text_working_area->address);
	if (retval != ERROR_OK) {
		atsamvqspi_executive_free(exec);
		return NULL;
	}

	return exec;
}

static int atsamvqspi_executive_set_params(struct atsamvqspi_executive* exec, const uint32_t* params, uint32_t n_params)
{
	if(exec == NULL) return ERROR_FAIL;
	if(params == NULL) return ERROR_FAIL;
	if(n_params > EXECUTIVE_PARAM_WORKING_AREA_SIZE/sizeof(uint32_t)) return ERROR_FAIL;
	
	//Write params to working area
	int retval = target_write_buffer(exec->target, exec->param_working_area->address,
		sizeof(uint32_t)*n_params, (const uint8_t*)params);
	if (retval != ERROR_OK) return retval;

	LOG_DEBUG("set flash executive parameters:");
	for(uint8_t i=0; i<n_params; i++) {
		LOG_DEBUG("param[%d] = %x", i, params[i]);
	}

	return retval;
}

static uint32_t calculate_stack_top(struct atsamvqspi_executive* exec)
{
	uint32_t unaligned_stack_address = exec->stack_working_area->address + exec->stack_working_area->size - 1;
	uint32_t aligned_stack_address = unaligned_stack_address - (unaligned_stack_address % 32);
	return aligned_stack_address;
}

static int atsamvqspi_executive_run(struct atsamvqspi_executive* exec, uint32_t* return_value, uint32_t timeout)
{
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[4];
	if(exec == NULL) return ERROR_FAIL;
	int retval = ERROR_OK;

	//Initialize register parameters
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* param struct address (in), return code (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_IN);	/* return value */
	init_reg_param(&reg_params[2], "msp", 32, PARAM_OUT);	/* stack pointer */
	init_reg_param(&reg_params[3], "sp", 32, PARAM_OUT);	/* stack pointer */

	buf_set_u32(reg_params[0].value, 0, 32, exec->param_working_area->address);
	buf_set_u32(reg_params[2].value, 0, 32, calculate_stack_top(exec));
	buf_set_u32(reg_params[3].value, 0, 32, calculate_stack_top(exec));

	LOG_DEBUG("set r0 to %x", buf_get_u32(reg_params[0].value, 0, 32));
	LOG_DEBUG("set msp to %x", buf_get_u32(reg_params[2].value, 0, 32));
	LOG_DEBUG("set sp to %x", buf_get_u32(reg_params[3].value, 0, 32));

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_algorithm(exec->target, 
		0, NULL, 
		4, reg_params,
		exec->text_working_area->address,
		0x0, //this architecture terminates by the algorithm issuing a break point.
		timeout, 
		&armv7m_info
	);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error executing flash algorithm %d", retval);
	} else if(return_value != NULL) {
		*return_value = buf_get_u32(reg_params[1].value, 0, 32);
	}

	LOG_DEBUG("executive completed with error %u, return value %u",
		buf_get_u32(reg_params[0].value, 0, 32), buf_get_u32(reg_params[1].value, 0, 32));

	int executive_retval = buf_get_u32(reg_params[0].value, 0, 32);
	if(executive_retval != EXECUTIVE_ERROR_OK) return ERROR_FAIL;

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	return retval;
}

static int atsamvqspi_executive_run_async(struct atsamvqspi_executive* exec, const uint8_t* buffer, uint32_t size, uint32_t* return_value)
{
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[4];
	if(exec == NULL) return ERROR_FAIL;
	int retval = ERROR_OK;

	//Initialize register parameters
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* param struct address (in), return code (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_IN);	/* return value */
	init_reg_param(&reg_params[2], "msp", 32, PARAM_OUT);	/* stack pointer */
	init_reg_param(&reg_params[3], "sp", 32, PARAM_OUT);	/* stack pointer */

	buf_set_u32(reg_params[0].value, 0, 32, exec->param_working_area->address);
	buf_set_u32(reg_params[2].value, 0, 32, calculate_stack_top(exec));
	buf_set_u32(reg_params[3].value, 0, 32, calculate_stack_top(exec));

	LOG_DEBUG("set r0 to %x", buf_get_u32(reg_params[0].value, 0, 32));
	LOG_DEBUG("set msp to %x", buf_get_u32(reg_params[2].value, 0, 32));
	LOG_DEBUG("set sp to %x", buf_get_u32(reg_params[3].value, 0, 32));

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(exec->target, 
		buffer, size, 1,
		0, NULL, 
		4, reg_params,
		exec->fifo_working_area->address, exec->fifo_working_area->size,
		exec->text_working_area->address,
		0x0, //this architecture terminates by the algorithm issuing a break point.
		&armv7m_info
	);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error executing flash async algorithm");
	} else if(return_value != NULL) {
		*return_value = buf_get_u32(reg_params[1].value, 0, 32);
	}

	LOG_DEBUG("executive completed with error %u, return value %u",
		buf_get_u32(reg_params[0].value, 0, 32), buf_get_u32(reg_params[1].value, 0, 32));

	int executive_retval = buf_get_u32(reg_params[0].value, 0, 32);
	if(executive_retval != EXECUTIVE_ERROR_OK) return ERROR_FAIL;

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	return retval;
}

/* flash_bank lpcspifi <base> <size> <chip_width> <bus_width> <target>
 */
FLASH_BANK_COMMAND_HANDLER(atsamvqspi_flash_bank_command) //OK
{
	struct atsamvqspi_flash_bank *atsamvqspi_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	atsamvqspi_info = malloc(sizeof(struct atsamvqspi_flash_bank));
	if (!atsamvqspi_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = atsamvqspi_info;
	atsamvqspi_info->probed = false;

	return ERROR_OK;
}

static int atsamvqspi_bulk_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct atsamvqspi_flash_bank *atsamvqspi_info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("full chip erase");

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(atsamvqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	struct atsamvqspi_executive* executive = atsamvqspi_executive_alloc(bank);
	if(executive == NULL) return ERROR_FAIL;

	uint32_t params[1] = { EXECUTIVE_COMMAND_ERASE_CHIP };
	retval = atsamvqspi_executive_set_params(executive, params, 1);

	if(retval == ERROR_OK) {
		retval = atsamvqspi_executive_run(executive, NULL, 3000*bank->num_sectors);
	}	

	atsamvqspi_executive_free(executive);
	return retval;
}

static int atsamvqspi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct atsamvqspi_flash_bank *atsamvqspi_info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("erase from sector %u to sector %u", first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(atsamvqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	//If we're erasing the whole chip, just use a bulk erase.
	if (first == 0 && last == (bank->num_sectors - 1)) {
		return atsamvqspi_bulk_erase(bank);
	} else {	
		struct atsamvqspi_executive* executive = atsamvqspi_executive_alloc(bank);
		if(executive == NULL) return ERROR_FAIL;

		uint32_t params[3] = {
			EXECUTIVE_COMMAND_ERASE_RANGE,		//command code
			bank->sectors[first].offset,	//starting offset
			last - first + 1				//n blocks to erase
		};
		retval = atsamvqspi_executive_set_params(executive, params, 3);
		
		if(retval == ERROR_OK) {
			retval = atsamvqspi_executive_run(executive, NULL, 3000*(last - first + 1));
		}	

		atsamvqspi_executive_free(executive);
		return retval;
	}
}

static int atsamvqspi_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int atsamvqspi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct atsamvqspi_flash_bank *atsamvqspi_info = bank->driver_priv;
	uint32_t page_size, fifo_size;
	int retval = ERROR_OK;

	LOG_DEBUG("write %u bytes to offset %x", count, offset);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > atsamvqspi_info->dev->size_in_bytes) {
		LOG_WARNING("Writes past end of flash. Extra data discarded.");
		count = atsamvqspi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
				(bank->sectors[sector].offset + bank->sectors[sector].size))
			&& ((offset + count - 1) >= bank->sectors[sector].offset)
			&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	struct atsamvqspi_executive* executive = atsamvqspi_executive_alloc(bank);
	if(executive == NULL) return ERROR_FAIL;

	/* if no valid page_size, use reasonable default */
	page_size = atsamvqspi_info->dev->pagesize ?
		atsamvqspi_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	//Allocate the write FIFO
	fifo_size = target_get_working_area_avail(target);

	if (fifo_size == 0) {
		/* if we already allocated the writing code but failed to get fifo
		 * space, free the algorithm */
		atsamvqspi_executive_free(executive);

		// LOG_ERROR("Insufficient working area. Please allocate at least"
		// 	" %zdB of working area to enable flash writes.",
		// 	sizeof(lpcspifi_flash_write_code) + 1
		// );

		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (fifo_size < page_size) {
		// LOG_WARNING("Working area size is limited; flash writes may be"
		// 	" slow. Increase working area size to at least %zdB"
		// 	" to reduce write times.",
		// 	(size_t)(sizeof(lpcspifi_flash_write_code) + page_size)
		// );
	} else if (fifo_size > 0x2000) { /* Beyond this point, we start to get diminishing returns */
		fifo_size = 0x2000;
	}

	if (target_alloc_working_area(target, fifo_size, &executive->fifo_working_area) != ERROR_OK) {
		atsamvqspi_executive_free(executive);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	uint32_t params[6] = {
		EXECUTIVE_COMMAND_WRITE,
		executive->fifo_working_area->address,
		executive->fifo_working_area->address + 2*sizeof(uint32_t),
		executive->fifo_working_area->address + executive->fifo_working_area->size,
		offset,
		count
	};
	retval = atsamvqspi_executive_set_params(executive, params, 6);
	
	if(retval == ERROR_OK) retval = atsamvqspi_executive_run_async(executive, buffer, count, NULL);

	atsamvqspi_executive_free(executive);
	return retval;
}

/* Return ID of flash device */
static int atsamvqspi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct atsamvqspi_executive* executive = atsamvqspi_executive_alloc(bank);
	if(executive == NULL) return ERROR_FAIL;

	uint32_t params[1] = { EXECUTIVE_COMMAND_PROBE };
	retval = atsamvqspi_executive_set_params(executive, params, 1);
	if(retval == ERROR_OK) {
		retval = atsamvqspi_executive_run(executive, id, 5000);
	} else {
		LOG_ERROR("unable to set parameters for atsamvqspi flash executive");
	}
	atsamvqspi_executive_free(executive);
	return retval;
}

static int atsamvqspi_probe(struct flash_bank *bank)
{
	struct atsamvqspi_flash_bank *atsamvqspi_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	int retval;
	uint32_t sectorsize;

	/* If we've already probed, we should be fine to skip this time. */
	if (atsamvqspi_info->probed)
		return ERROR_OK;

	retval = atsamvqspi_read_flash_id(bank, &id);
	if(retval != ERROR_OK) return retval;

	atsamvqspi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			atsamvqspi_info->dev = p;
			break;
		}

	if (!atsamvqspi_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		atsamvqspi_info->dev->name, atsamvqspi_info->dev->device_id);

	/* Set correct size value */
	bank->size = atsamvqspi_info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = atsamvqspi_info->dev->sectorsize ?
		atsamvqspi_info->dev->sectorsize : atsamvqspi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = atsamvqspi_info->dev->size_in_bytes / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;

	atsamvqspi_info->probed = true;
	return ERROR_OK;
}

static int atsamvqspi_auto_probe(struct flash_bank *bank)
{
	struct atsamvqspi_flash_bank *atsamvqspi_info = bank->driver_priv;
	if (atsamvqspi_info->probed)
		return ERROR_OK;
	return atsamvqspi_probe(bank);
}

static int atsamvqspi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_atsamvqspi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct atsamvqspi_flash_bank *atsamvqspi_info = bank->driver_priv;

	if (!(atsamvqspi_info->probed)) {
		command_print_sameline(cmd, "\nQSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "\nQSPI flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		atsamvqspi_info->dev->name, atsamvqspi_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver atsamvqspi_flash = {
	.name = "atsamvqspi",
	.flash_bank_command = atsamvqspi_flash_bank_command,
	.erase = atsamvqspi_erase,
	.protect = atsamvqspi_protect,
	.write = atsamvqspi_write,
	.read = default_flash_read,
	.probe = atsamvqspi_probe,
	.auto_probe = atsamvqspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = atsamvqspi_protect_check,
	.info = get_atsamvqspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
