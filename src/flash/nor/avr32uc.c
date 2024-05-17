// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 Paul Kutukov                                       *
 ***************************************************************************/

//TODO: turn pseudo-code from 32070A–AVR32–11/07 into actual C code

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/avr32_ap7k.h>
#include <target/avr32_jtag.h>
#include <inttypes.h>

#define ERR_CHECK(x) { int __err; if ((__err = (x)) != ERROR_OK) { return __err; } }

/* Public Instructions: */
#define AVR32UC_JTAG_INS_IDCODE 0x01
#define AVR32UC_JTAG_INS_SAMPLE_PRELOAD 0x02
#define AVR32UC_JTAG_INS_EXTEST 0x03
#define AVR32UC_JTAG_INS_INTEST 0x04
#define AVR32UC_JTAG_INS_CLAMP 0x06
#define AVR32UC_JTAG_INS_BYPASS 0x1F
/* AVR Specified Public Instructions: */
#define AVR32UC_JTAG_INS_CHIP_ERASE 0x0F
#define AVR32UC_JTAG_INS_NEXUS_ACCESS 0x10
#define AVR32UC_JTAG_INS_MEMORY_WORD_ACCESS 0x11
#define AVR32UC_JTAG_INS_MEMORY_BLOCK_ACCESS 0x12
#define AVR32UC_JTAG_INS_CANCEL_ACCESS 0x13
#define AVR32UC_JTAG_INS_MEMORY_SERVICE 0x14
#define AVR32UC_JTAG_INS_MEMORY_SIZED_ACCESS 0x15
#define AVR32UC_JTAG_INS_SYNC 0x17
#define AVR32UC_JTAG_INS_HALT 0x1C
#define AVR32UC_JTAG_INS_AVR_RESET 0x0C

/* Data Registers: */
#define AVR32UC_JTAG_REG_BYPASS_LEN 1
#define AVR32UC_JTAG_REG_DEVICEID_LEN 32

#define AVR32UC_JTAG_REG_RESET_LEN 5
#define AVR32UC_JTAG_REG_JTAGID_LEN 32
#define AVR32UC_JTAG_REG_MEMORY_SIZED_ACCESS_LEN 35

#define AVR32UC_FLASH_CTRL_BASE 0xFFFE1400

#define FCR 0x00
#define FCMD 0x04
#define FSR 0x08
#define FGPFRHI 0x0C
#define FGPFRLO 0x10

#define FSR_FRDY_MASK 1
#define FSR_FRDY_offset 0
#define FSR_PROGE_MASK 0x08
#define FSR_PROGE_offset 3
#define FSR_LOCKE_MASK 0x04
#define FSR_LOCKE_offset 2
#define FSR_FSZ_MASK 0x0000E000
#define FSR_FSZ_offset 13
#define FCMD_FCMD_MASK 0x1F
#define FCMD_FCMD_offset 0
#define FCMD_PAGEN_MASK 0x00FFFF00
#define FCMD_PAGEN_offset 8
#define FCMD_KEY_MASK 0xFF000000
#define FCMD_KEY_offset 24
#define FGPFR_LOCK_MASK 0x0000FFFF
#define FGPFR_LOCK_offset 0
#define WORDS_PER_PAGE 128
#define BYTES_PER_PAGE (WORDS_PER_PAGE * 4)
#define USER_PAGE_OFFSET 0x00800000
#define WRITE_PROTECT_KEY 0xA5000000
#define CMD_WRITE_PAGE 1
#define CMD_ERASE_PAGE 2
#define CMD_CLEAR_PAGE_BUFFER 3
#define CMD_LOCK_REGION 4
#define CMD_UNLOCK_REGION 5
#define CMD_ERASE_ALL 6
#define CMD_WRITE_GP_FUSE_BIT 7
#define CMD_ERASE_GP_FUSE_BIT 8
#define CMD_SET_SECURITY_BIT 9
#define CMD_PROGRAM_GP_FUSE_BYTE 10
#define CMD_WRITE_USER_PAGE 13
#define CMD_ERASE_USER_PAGE 14

#define EXTRACT_MFG(X) (((X) & 0xffe) >> 1)
#define EXTRACT_PART(X) (((X) & 0xffff000) >> 12)
#define EXTRACT_VER(X) (((X) & 0xf0000000) >> 28)

struct avr32uc_type
{
	char name[15];
	uint16_t chip_id;
	int flash_page_size;
	int flash_page_num;
};

struct avr32uc_flash_bank
{
	int ppage_size;
	bool probed;
};

static const struct avr32uc_type avr32uc_chips_info[] = {
	/*	name, chip_id,	flash_page_size, flash_page_num
	 */
	{"AT32UC3B0512", 0x2050, 128, 1024},
	{"AT32UC3B1512", 0x2052, 128, 1024},
	{"AT32UC3B0256", 0x1EE4, 128, 512},
	{"AT32UC3B1256", 0x1EE5, 128, 512},
	{"AT32UC3B0128", 0x1EE6, 128, 256},
	{"AT32UC3B1128", 0x1EE9, 128, 256},
	{"AT32UC3B064", 0x1EEA, 128, 128},
	{"AT32UC3B164", 0x1EEB, 128, 128}};

/* avr program functions */
static int AVR32UC_JTAG_reset(struct avr32_ap7k_common *avr, uint32_t reset)
{
	avr32_jtag_set_instr(&(avr->jtag), NULL, AVR32UC_JTAG_INS_AVR_RESET);
	avr32_jtag_send_dat(avr->jtag.tap, NULL, reset, AVR32UC_JTAG_REG_RESET_LEN);
	ERR_CHECK(mcu_execute_queue());

	return ERROR_OK;
}

static int AVR32UC_JTAG_read_jtagid(struct avr32_ap7k_common *avr, uint32_t *id)
{
	avr32_jtag_set_instr(avr->jtag.tap, NULL, AVR32UC_JTAG_INS_IDCODE);
	avr32_jtag_send_dat(avr->jtag.tap, id, 0, AVR32UC_JTAG_REG_JTAGID_LEN);
	ERR_CHECK(mcu_execute_queue());

	return ERROR_OK;
}

static int avr32uc_jtagprg_chiperase(struct avr32_ap7k_common *avr)
{
	uint32_t poll_value;

	avr32_jtag_queue_instruction(avr->jtag.tap, &poll_value, AVR32UC_JTAG_INS_CHIP_ERASE);
	avr32_jtag_send_dat(avr->jtag.tap, NULL, 0, AVR32UC_JTAG_REG_BYPASS_LEN);
	ERR_CHECK(mcu_execute_queue());

	return (poll_value & (1 << 2)) ? ERROR_FLASH_BUSY : ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(avr32uc_flash_bank_command)
{
	struct avr32uc_flash_bank *avr32uc_flash_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	avr32uc_flash_bank = malloc(sizeof(struct avr32uc_flash_bank));
	bank->driver_priv = avr32uc_flash_bank;

	avr32uc_flash_bank->probed = false;

	return ERROR_OK;
}

static int avr32uc_flash_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct avr32uc_flash_bank *avr32uc_flash_bank = bank->driver_priv;
	struct avr32_ap7k_common *avr = target->arch_info;
	const struct avr32uc_type *type = NULL;
	uint32_t device_id;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	avr32uc_flash_bank->probed = false;

	ERR_CHECK(AVR32UC_JTAG_read_jtagid(avr, &device_id));

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	if (EXTRACT_MFG(device_id) != 0x1F)
		LOG_ERROR("0x%" PRIx32 " is invalid Manufacturer for avr, 0x%X is expected",
				  EXTRACT_MFG(device_id),
				  0x1F);

	for (size_t i = 0; i < ARRAY_SIZE(avr32uc_chips_info); i++)
	{
		if (avr32uc_chips_info[i].chip_id == EXTRACT_PART(device_id))
		{
			type = &avr32uc_chips_info[i];
			LOG_INFO("target device is %s", type->name);
			break;
		}
	}

	if (type)
	{
		free(bank->sectors);

		/* chip found */
		bank->base = 0x00000000;
		bank->size = (type->flash_page_size * type->flash_page_num);
		bank->num_sectors = type->flash_page_num;
		bank->sectors = malloc(sizeof(struct flash_sector) * type->flash_page_num);

		for (int i = 0; i < type->flash_page_num; i++)
		{
			bank->sectors[i].offset = i * type->flash_page_size;
			bank->sectors[i].size = type->flash_page_size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = -1;
		}

		avr32uc_flash_bank->probed = true;
		return ERROR_OK;
	}
	else
	{
		/* chip not supported */
		LOG_ERROR("0x%" PRIx32 " is not support for avr", EXTRACT_PART(device_id));

		avr32uc_flash_bank->probed = true;
		return ERROR_FAIL;
	}
}

static int avr32uc_flash_auto_probe(struct flash_bank *bank)
{
	struct avr32uc_flash_bank *avr32uc_bank = bank->driver_priv;
	if (avr32uc_bank->probed)
		return ERROR_OK;
	return avr32uc_flash_probe(bank);
}

static int avr32uc_flash_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct target *target = bank->target;
	struct avr32_ap7k_common *avr = target->arch_info;
	const struct avr32uc_type *type = NULL;
	uint32_t device_id;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ERR_CHECK(AVR32UC_JTAG_read_jtagid(avr, &device_id));

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	if (EXTRACT_MFG(device_id) != 0x1F)
		LOG_ERROR("0x%" PRIx32 " is invalid Manufacturer for avr32uc, 0x%X is expected",
				  EXTRACT_MFG(device_id),
				  0x1F);

	for (size_t i = 0; i < ARRAY_SIZE(avr32uc_chips_info); i++)
	{
		if (avr32uc_chips_info[i].chip_id == EXTRACT_PART(device_id))
		{
			type = &avr32uc_chips_info[i];
			LOG_INFO("target device is %s", type->name);

			break;
		}
	}

	if (type)
	{
		/* chip found */
		command_print_sameline(cmd, "%s - Rev: 0x%" PRIx32 "", type->name,
							   EXTRACT_VER(device_id));
		return ERROR_OK;
	}
	else
	{
		/* chip not supported */
		command_print_sameline(cmd, "Cannot identify target as a avr\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}
}

COMMAND_HANDLER(avrf_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	//bank->target

	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32uc_read_flash_controller_reg(struct flash_bank *bank, uint8_t addr, uint32_t* data)
{
	struct avr32_ap7k_common *avr = bank->target->arch_info;
	uint8_t ir_in = 0;
	uint64_t dr_in = 0;
	uint64_t absolute_addr = ((addr + AVR32UC_FLASH_CTRL_BASE) << 1) | 0b1;

	ERR_CHECK(avr32_jtag_set_instr(&(avr->jtag), &ir_in, AVR32UC_JTAG_INS_MEMORY_WORD_ACCESS));
	if (ir_in & 0b11100)
	{
		LOG_ERROR("%s: failed to acquire memory access (%" PRIX8 ")", ir_in);
		if (ir_in & 0b10000) return ERROR_FLASH_PROTECTED;
		if (ir_in & 0b01000) return ERROR_TARGET_FAILURE;
		if (ir_in & 0b00100) return ERROR_FLASH_BUSY;
	}
	else if (!(ir_in & 0b00001)) return ERROR_FAIL;
	//Address phase
	ERR_CHECK(avr32_jtag_send_dat(&(avr->jtag), &dr_in, absolute_addr, AVR32UC_JTAG_REG_MEMORY_SIZED_ACCESS_LEN));
	if (dr_in & 0b01) return ERROR_FLASH_BUSY;
	if (dr_in & 0b10) return ERROR_TARGET_FAILURE;
	//Data read phase
	ERR_CHECK(avr32_jtag_send_dat(&(avr->jtag), &dr_in, 0, AVR32UC_JTAG_REG_MEMORY_SIZED_ACCESS_LEN));
	if (dr_in & (1 << (AVR32UC_JTAG_REG_MEMORY_SIZED_ACCESS_LEN - 3))) return ERROR_FLASH_BUSY;
	if (dr_in & (1 << (AVR32UC_JTAG_REG_MEMORY_SIZED_ACCESS_LEN - 2))) return ERROR_TARGET_FAILURE;
	if (data) *data = (uint32_t)(dr_in & 0xFFFFFFFF);

	return ERROR_OK;
}
static int avr32uc_write_flash_controller_reg(struct flash_bank *bank, uint8_t addr, uint32_t data)
{
	struct avr32_ap7k_common *avr = bank->target->arch_info;
	uint8_t ir_in = 0;
	uint64_t dr_in = 0;
	uint64_t absolute_addr = (addr + AVR32UC_FLASH_CTRL_BASE) << 1;

	ERR_CHECK(avr32_jtag_set_instr(&(avr->jtag), &ir_in, AVR32UC_JTAG_INS_MEMORY_WORD_ACCESS));
		return ERROR_FAIL;
	if (ir_in & 0b11100)
	{
		LOG_ERROR("%s: failed to acquire memory access (%" PRIX8 ")", ir_in);
		if (ir_in & 0b10000) return ERROR_FLASH_PROTECTED;
		if (ir_in & 0b01000) return ERROR_TARGET_FAILURE;
		if (ir_in & 0b00100) return ERROR_FLASH_BUSY;
	}
	else if (!(ir_in & 0b00001)) return ERROR_FAIL;
	//Address phase
	ERR_CHECK(avr32_jtag_send_dat(&(avr->jtag), &dr_in, absolute_addr, AVR32UC_JTAG_REG_MEMORY_SIZED_ACCESS_LEN));
	if (dr_in & 0b01) return ERROR_FLASH_BUSY;
	if (dr_in & 0b10) return ERROR_TARGET_FAILURE;
	//Data read phase
	ERR_CHECK(avr32_jtag_send_dat(&(avr->jtag), &dr_in, (uint64_t)data << 3, AVR32UC_JTAG_REG_MEMORY_SIZED_ACCESS_LEN));
	if (dr_in & 0b01) return ERROR_FLASH_BUSY;
	if (dr_in & 0b10) return ERROR_TARGET_FAILURE;

	return ERROR_OK;
}

int avr32uc_wait_flash_ready(struct flash_bank *bank)
{
	int Timeout = 1000;
	while (Timeout--)
	{
		uint32_t fsrReg;
		ERR_CHECK(avr32uc_read_flash_controller_reg(bank, FSR, &fsrReg));
		// If LOCKE bit in FSR set
		if ((fsrReg & FSR_LOCKE_MASK) >> FSR_LOCKE_offset)
			return 2;
		// If PROGE bit in FSR set
		if ((fsrReg & FSR_PROGE_MASK) >> FSR_PROGE_offset)
			return 3;
		// Read FRDY bit in FSR
		if ((fsrReg & FSR_FRDY_MASK) >> FSR_FRDY_offset)
			return 0;
		// FLASH ready for next operation
	}
}

int avr32uc_send_flash_command(struct flash_bank *bank, uint32_t cmd)
{
	return avr32uc_write_flash_controller_reg(bank, FCMD, cmd);
}

int avr32uc_flash_clear_page_buf(struct flash_bank *bank)
{
	uint32_t command = WRITE_PROTECT_KEY | CMD_CLEAR_PAGE_BUFFER;
	if (avr32uc_wait_flash_ready(bank) != 0) return ERROR_FAIL;
	int err;
	ERR_CHECK(avr32uc_send_flash_command(bank, command));
	avr32uc_wait_flash_ready(bank);
}

int avr32uc_flash_get_size(struct flash_bank *bank, uint32_t* size)
{
	uint32_t fsrReg;
	ERR_CHECK(avr32uc_read_flash_controller_reg(bank, FSR, &fsrReg));
	unsigned int fsz = (fsrReg & FSR_FSZ_MASK) >> FSR_FSZ_offset;
	unsigned int size = 0;
	switch (fsz)
	{
	case 0:
		size = 32 * 1024;
		break;
	case 1:
		size = 64 * 1024;
		break;
	case 2:
		size = 128 * 1024;
		break;
	case 3:
		size = 256 * 1024;
		break;
	case 4:
		size = 384 * 1024;
		break;
	case 5:
		size = 512 * 1024;
		break;
	case 6:
		size = 768 * 1024;
		break;
	case 7:
		size = 1024 * 1024;
		break;
	default:
		return -1;
	}
	return size;
}

int avr32uc_flash_unlock_region(struct flash_bank *bank, uint32_t offset, uint32_t size)
{
	if (offset >= USER_PAGE_OFFSET && offset < USER_PAGE_OFFSET + BYTES_PER_PAGE)
		return ERROR_OK; // the user page doesn't need unlocking
	if (offset + size > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;
	int lastpagetounlock = ((offset + size) / BYTES_PER_PAGE);
	// compute start offset of page to write to
	uint32_t page = offset & ~(BYTES_PER_PAGE - 1);
	int pagenr = ((offset) / BYTES_PER_PAGE);
	while (pagenr <= lastpagetounlock)
	{
		uint32_t command = WRITE_PROTECT_KEY | CMD_UNLOCK_REGION;
		// include the correct page number in the command
		command |= ((pagenr << FCMD_PAGEN_offset) & FCMD_PAGEN_MASK);
		// Unlocking page: pagenr
		if (avr32uc_wait_flash_ready(bank) != 0) return ERROR_FAIL;
		ERR_CHECK(avr32uc_send_flash_command(bank, command)); // execute unlock page command
		avr32uc_wait_flash_ready(bank);
		page += BYTES_PER_PAGE;
		offset = page;
		pagenr = ((offset) / BYTES_PER_PAGE);
	}
}

int avr32uc_flash_unlock(struct flash_bank *bank)
{
	int DeviceSize;
	ERR_CHECK(avr32uc_flash_get_size(bank, &DeviceSize));
	ERR_CHECK(avr32uc_flash_unlock_region(bank, 0, DeviceSize));
}

int avr32uc_flash_erase(struct flash_bank *bank)
{
	if (avr32uc_wait_flash_ready(bank) != 0) return ERROR_FAIL;
	uint32_t Command = WRITE_PROTECT_KEY | CMD_ERASE_ALL;
	ERR_CHECK(avr32uc_send_flash_command(bank, Command));
	avr32uc_wait_flash_ready(bank);
}

int avr32uc_flash_erase_userpage(struct flash_bank *bank)
{
	uint32_t command = WRITE_PROTECT_KEY | CMD_ERASE_USER_PAGE;
	if (avr32uc_wait_flash_ready(bank) != 0) return ERROR_FAIL;
	ERR_CHECK(avr32uc_send_flash_command(bank, command)); // execute user page erase command
	avr32uc_wait_flash_ready(bank);
}

int avr32uc_flash_erase_region(struct flash_bank *bank, uint32_t offset, uint32_t size)
{
	if (offset >= USER_PAGE_OFFSET && offset < USER_PAGE_OFFSET + BYTES_PER_PAGE)
		ERR_CHECK(avr32uc_flash_erase_userpage(bank));
	int lastpagetoerase = (offset + size) / BYTES_PER_PAGE;
	int page = offset & ~(BYTES_PER_PAGE - 1); // compute start offset of page to write to
	int pagenr = (offset) / BYTES_PER_PAGE;
	while (pagenr <= lastpagetoerase)
	{
		uint32_t Command = WRITE_PROTECT_KEY | CMD_ERASE_PAGE;
		Command |= ((pagenr << FCMD_PAGEN_offset) & FCMD_PAGEN_MASK);
		// include the correct page number in the command
		if (avr32uc_wait_flash_ready(bank) != 0) return ERROR_FAIL;
		ERR_CHECK(avr32uc_send_flash_command(bank, Command)); // execute page erase command
		avr32uc_wait_flash_ready(bank);
		page += BYTES_PER_PAGE;
		offset = page;
		pagenr = (offset) / BYTES_PER_PAGE;
	}
}

int ProgramUserPage(offset - USER_PAGE_OFFSET, DataBuffer)
{
	if (offset >= BYTES_PER_PAGE || offset + Length(DataBuffer) > BYTES_PER_PAGE)
		return -1;
	// Packet bufferPacket(BYTES_PER_PAGE) define a buffer packet
	// to manipulate the data
	// If the packet to be written is smaller than the user page we fill the
	// remaining space with existing data
	if (offset > 0 || Length(DataBuffer) < BYTES_PER_PAGE)
		ReadMemory(mBaseAddress + USER_PAGE_OFFSET, bufferPacket, 0);
	// Must clear the page buffer before writing to it.
	avr32uc_flash_clear_page_buf();
	int bytesLeftInPacket = Length(DataBuffer);
	int i = 0; // data packet index
	// Fill buffer packet
	while (bytesLeftInPacket > 0)
	{
		bufferPacket.writeSingleByte(offset++, ReadSingleByte(DataBuffer));
		i++;
		bytesLeftInPacket--;
	}
	// Write page buffer
	WriteMemory(mBaseAddress + USER_PAGE_OFFSET, bufferPacket);
	uint32_t command = WRITE_PROTECT_KEY | CMD_WRITE_USER_PAGE;
	avr32uc_wait_flash_ready();
	avr32uc_send_flash_command(command); // execute user page write command
	avr32uc_wait_flash_ready();
}
int ProgramSequence(offset, DataBuffer)
{
	if (offset >= USER_PAGE_OFFSET && offset < USER_PAGE_OFFSET + BYTES_PER_PAGE)
		ProgramUserPage(offset - USER_PAGE_OFFSET, DataBuffer);

	if (offset >= mDeviceSize || offset + Length(DataBuffer) > mDeviceSize)
		return -1;
	// compute start offset of page to write to
	uint32_t page = offset & ~(BYTES_PER_PAGE - 1);
	unsigned int bytesLeft = (Length(DataBuffer));
	int dataoffset = 0; // current offset in the data packet
	Packet bufferPacket(BYTES_PER_PAGE);
	// we write one page at a time
	// Loop until all bytes in data has been written
	while (bytesLeft > 0)
	{
		bufferPacket.clear(0xff);
		// Must clear the page buffer before writing to it.
		avr32uc_flash_clear_page_buf();
		/* Keeps track of how many bytes to write to the bufferPacket.
		 * If the start offset is not aligned on a page boundary, we will not fill
		 * the bufferPacket completely. This is also the case when the number of
		 * bytes left to write is less than the size of a page. If the bufferPacket
		 * is not filled completely we first read the current flash content into
		 * the packet. This way we will always preserve existing flash data
		 * adjacent to the new data we wish to write.
		 */
		int bytesLeftInPacket = min((page + BYTES_PER_PAGE - offset), bytesLeft);
		int bufferoffset = offset % BYTES_PER_PAGE;
		if (bufferoffset != 0 || bytesLeftInPacket != (BYTES_PER_PAGE))
		{
			ReadMemory(mBaseAddress + page, bufferPacket, 0);
		}
		for (int i = 0; i < bytesLeftInPacket; ++i)
		{
			bufferPacket.writeSingleByte(bufferoffset++,
										 ReadSingleByte(DataBuffer));
			offset++;
			WriteMemory(mBaseAddress + page, bufferPacket);
			int pagenr = ((offset) / BYTES_PER_PAGE)
				uint32_t command = WRITE_PROTECT_KEY | CMD_WRITE_PAGE;
			// include the correct page number in the command
			command |= pagenr << FCMD_PAGEN_offset;
			avr32uc_wait_flash_ready();
			avr32uc_send_flash_command(command); // execute page write command
			avr32uc_wait_flash_ready();
			page += BYTES_PER_PAGE;
			offset = page;
			bytesLeft -= bytesLeftInPacket;
		}
	}
}
bool GetGeneralPurposeFuseBit(int Index)
{
	if (Index > 31 || Index < 0)
		return;
	uint32_t fgpfrReg = avr32uc_read_flash_controller_reg(FGPFR);
	return fgpfrReg & (1 << Index);
}
void SetGeneralPurposeFuseBit(int Index, bool Value)
{
	if (Index > 31 || Index < 0)
		return;
	uint32_t command = WRITE_PROTECT_KEY | (index << FCMD_PAGEN_offset);
	if (Value) // erase bit
		command |= CMD_ERASE_GP_FUSE_BIT;
	else
		// program bit
		command |= CMD_WRITE_GP_FUSE_BIT;
	avr32uc_wait_flash_ready();
	avr32uc_send_flash_command(command);
	avr32uc_wait_flash_ready();
}

void SetGeneralPurposeFuseByte(int Index, unsigned char Value)
{
	if (Index > 3 || Index < 0)
		return;
	uint32_t command = WRITE_PROTECT_KEY | CMD_PROGRAM_GP_FUSE_BYTE | Index << FCMD_PAGEN_offset | Value << (FCMD_PAGEN_offset + 2);
	avr32uc_wait_flash_ready();
	avr32uc_send_flash_command(command);
	avr32uc_wait_flash_ready();
}

static const struct command_registration avrf_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.usage = "<bank>",
		.handler = avrf_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "erase entire device",
	},
	COMMAND_REGISTRATION_DONE};
static const struct command_registration avrf_command_handlers[] = {
	{
		.name = "avrf",
		.mode = COMMAND_ANY,
		.help = "AVR flash command group",
		.usage = "",
		.chain = avrf_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE};

const struct flash_driver avr32uc_flash = {
	.name = "avr",
	.commands = avrf_command_handlers,
	.flash_bank_command = avr32uc_flash_bank_command,
	.erase = avrf_erase,
	.write = avrf_write,
	.read = default_flash_read,
	.probe = avr32uc_flash_probe,
	.auto_probe = avr32uc_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = avr32uc_flash_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
