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

/* AVR32_UC_JTAG_Instructions */
#define AVR_JTAG_INS_LEN 5
/* Public Instructions: */
#define AVR_JTAG_INS_IDCODE 0x01
#define AVR_JTAG_INS_SAMPLE_PRELOAD 0x02
#define AVR_JTAG_INS_EXTEST 0x03
#define AVR_JTAG_INS_INTEST 0x04
#define AVR_JTAG_INS_CLAMP 0x06
#define AVR_JTAG_INS_CHIP_ERASE 0x0F
#define AVR_JTAG_INS_NEXUS_ACCESS 0x10
#define AVR_JTAG_INS_MEMORY_WORD_ACCESS 0x11
#define AVR_JTAG_INS_MEMORY_BLOCK_ACCESS 0x12
#define AVR_JTAG_INS_CANCEL_ACCESS 0x13
#define AVR_JTAG_INS_MEMORY_SERVICE 0x14
#define AVR_JTAG_INS_MEMORY_SIZED_ACCESS 0x15
#define AVR_JTAG_INS_SYNC 0x17
#define AVR_JTAG_INS_HALT 0x1C
#define AVR_JTAG_INS_BYPASS 0x1F
/* AVR Specified Public Instructions: */
#define AVR_JTAG_INS_AVR_RESET 0x0C

/* Data Registers: */
#define AVR_JTAG_REG_BYPASS_LEN 1
#define AVR_JTAG_REG_DEVICEID_LEN 32

#define AVR_JTAG_REG_RESET_LEN 5
#define AVR_JTAG_REG_JTAGID_LEN 32
#define AVR_JTAG_REG_MEMORY_SIZED_ACCESS_LEN 35

#define FCR 0x00
#define FCMD 0x04
#define FSR 0x08
#define FGPFRHI 0x0C
#define FGPFRLO 0x10

#define FSR_FRDY_MASK 1
#define FSR_FRDY_OFFSET 0
#define FSR_PROGE_MASK 0x08
#define FSR_PROGE_OFFSET 3
#define FSR_LOCKE_MASK 0x04
#define FSR_LOCKE_OFFSET 2
#define FSR_FSZ_MASK 0x0000E000
#define FSR_FSZ_OFFSET 13
#define FCMD_FCMD_MASK 0x1F
#define FCMD_FCMD_OFFSET 0
#define FCMD_PAGEN_MASK 0x00FFFF00
#define FCMD_PAGEN_OFFSET 8
#define FCMD_KEY_MASK 0xFF000000
#define FCMD_KEY_OFFSET 24
#define FGPFR_LOCK_MASK 0x0000FFFF
#define FGPFR_LOCK_OFFSET 0
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

struct avrf_flash_bank
{
	int ppage_size;
	bool probed;
};

static const struct avrf_type avft_chips_info[] = {
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
static int avr_jtag_reset(struct avr_common *avr, uint32_t reset)
{
	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_AVR_RESET);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, reset, AVR_JTAG_REG_RESET_LEN);

	return ERROR_OK;
}

static int avr_jtag_read_jtagid(struct avr_common *avr, uint32_t *id)
{
	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_IDCODE);
	avr_jtag_senddat(avr->jtag_info.tap, id, 0, AVR_JTAG_REG_JTAGID_LEN);

	return ERROR_OK;
}

static int avr_jtagprg_chiperase(struct avr_common *avr)
{
	uint32_t poll_value;

	do
	{
		poll_value = 0;
		avr_jtag_sendinstr(avr->jtag_info.tap, &poll_value, AVR_JTAG_INS_CHIP_ERASE);
		avr_jtag_senddat(avr->jtag_info.tap,
						 &poll_value,
						 0x3380,
						 AVR_JTAG_REG_PROGRAMMING_COMMAND_LEN);
		if (mcu_execute_queue() != ERROR_OK)
			return ERROR_FAIL;
		LOG_DEBUG("poll_value = 0x%04" PRIx32 "", poll_value);
	} while (!(poll_value & 0x0200));

	// avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x00, AVR_JTAG_REG_BYPASS_LEN);

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(avrf_flash_bank_command)
{
	struct avrf_flash_bank *avrf_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	avrf_info = malloc(sizeof(struct avrf_flash_bank));
	bank->driver_priv = avrf_info;

	avrf_info->probed = false;

	return ERROR_OK;
}

static int avrf_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct avrf_flash_bank *avrf_info = bank->driver_priv;
	struct avr_common *avr = target->arch_info;
	const struct avrf_type *avr_info = NULL;
	uint32_t device_id;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	avrf_info->probed = false;

	avr_jtag_read_jtagid(avr, &device_id);
	if (mcu_execute_queue() != ERROR_OK)
		return ERROR_FAIL;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	if (EXTRACT_MFG(device_id) != 0x1F)
		LOG_ERROR("0x%" PRIx32 " is invalid Manufacturer for avr, 0x%X is expected",
				  EXTRACT_MFG(device_id),
				  0x1F);

	for (size_t i = 0; i < ARRAY_SIZE(avft_chips_info); i++)
	{
		if (avft_chips_info[i].chip_id == EXTRACT_PART(device_id))
		{
			avr_info = &avft_chips_info[i];
			LOG_INFO("target device is %s", avr_info->name);
			break;
		}
	}

	if (avr_info)
	{
		free(bank->sectors);

		/* chip found */
		bank->base = 0x00000000;
		bank->size = (avr_info->flash_page_size * avr_info->flash_page_num);
		bank->num_sectors = avr_info->flash_page_num;
		bank->sectors = malloc(sizeof(struct flash_sector) * avr_info->flash_page_num);

		for (int i = 0; i < avr_info->flash_page_num; i++)
		{
			bank->sectors[i].offset = i * avr_info->flash_page_size;
			bank->sectors[i].size = avr_info->flash_page_size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = -1;
		}

		avrf_info->probed = true;
		return ERROR_OK;
	}
	else
	{
		/* chip not supported */
		LOG_ERROR("0x%" PRIx32 " is not support for avr", EXTRACT_PART(device_id));

		avrf_info->probed = true;
		return ERROR_FAIL;
	}
}

static int avrf_auto_probe(struct flash_bank *bank)
{
	struct avrf_flash_bank *avrf_info = bank->driver_priv;
	if (avrf_info->probed)
		return ERROR_OK;
	return avrf_probe(bank);
}

static int avrf_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	const struct avrf_type *avr_info = NULL;
	uint32_t device_id;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	avr_jtag_read_jtagid(avr, &device_id);
	if (mcu_execute_queue() != ERROR_OK)
		return ERROR_FAIL;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	if (EXTRACT_MFG(device_id) != 0x1F)
		LOG_ERROR("0x%" PRIx32 " is invalid Manufacturer for avr, 0x%X is expected",
				  EXTRACT_MFG(device_id),
				  0x1F);

	for (size_t i = 0; i < ARRAY_SIZE(avft_chips_info); i++)
	{
		if (avft_chips_info[i].chip_id == EXTRACT_PART(device_id))
		{
			avr_info = &avft_chips_info[i];
			LOG_INFO("target device is %s", avr_info->name);

			break;
		}
	}

	if (avr_info)
	{
		/* chip found */
		command_print_sameline(cmd, "%s - Rev: 0x%" PRIx32 "", avr_info->name,
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

	if (avrf_mass_erase(bank) == ERROR_OK)
		command_print(CMD, "avr mass erase complete");
	else
		command_print(CMD, "avr mass erase failed");

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

int WaitFlashReady()
{
	int Timeout = 1000;
	while (Timeout--)
	{
		uint32_t fsrReg = getRegister(FSR);
		// If LOCKE bit in FSR set
		if ((fsrReg & FSR_LOCKE_MASK) >> FSR_LOCKE_OFFSET)
			return -2;
		// If PROGE bit in FSR set
		if ((fsrReg & FSR_PROGE_MASK) >> FSR_PROGE_OFFSET)
			return -3;
		// Read FRDY bit in FSR
		if ((fsrReg & FSR_FRDY_MASK) >> FSR_FRDY_OFFSET)
			return 0;
		// FLASH ready for next operation
	}
}
void ClearPageBuffer()
{
	uint32_t command = WRITE_PROTECT_KEY | CMD_CLEAR_PAGE_BUFFER;
	WaitFlashReady();
	WriteCommand(Command);
	WaitFlashReady();
}
int GetInternalFlashSize(void)
{
	uint32_t fsrReg = getRegister(FSR);
	unsigned int fsz = (fsrReg & FSR_FSZ_MASK) >> FSR_FSZ_OFFSET;
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
int UnlockRegion(unsigned int Offset, unsigned int Size)
{
	if (Offset >= USER_PAGE_OFFSET && Offset < USER_PAGE_OFFSET + BYTES_PER_PAGE)
		return 0; // the user page doesn't need unlocking
	if (Offset >= mDeviceSize || Offset + size > mDeviceSize)
		return -1;
	int lastpagetounlock = ((Offset + Size) / BYTES_PER_PAGE);
	// compute start offset of page to write to
	uint32_t page = Offset & ~(BYTES_PER_PAGE - 1);
	int pagenr = ((Offset) / BYTES_PER_PAGE);
	while (pagenr <= lastpagetounlock)
	{
		uint32_t command = WRITE_PROTECT_KEY | CMD_UNLOCK_REGION;
		// include the correct page number in the command
		command |= ((pagenr << FCMD_PAGEN_OFFSET) & FCMD_PAGEN_MASK);
		// Unlocking page: pagenr
		WaitFlashReady();
		WriteCommand(command); // execute unlock page command
		WaitFlashReady();
		page += BYTES_PER_PAGE;
		Offset = page;
		pagenr = ((Offset) / BYTES_PER_PAGE);
	}
}
void UnlockEntireFlash(void)
{
	DeviceSize = GetInternalFlashSize;
	UnlockRegion(0, DeviceSize);
}
void EraseSequence(void)
{
	WaitForFlashReady();
	Command = WRITE_PROTECT_KEY | CMD_ERASE_ALL;
	WriteCommand(Command);
	WaitForFlashReady();
}
void EraseUserPage(void)
{
	uint32_t command = WRITE_PROTECT_KEY | CMD_ERASE_USER_PAGE;
	WaitFlashReady();
	WriteCommand(command); // execute user page erase command
	WaitFlashReady();
}
void EraseRegionSequence(Offset, Size)
{
	if (Offset >= USER_PAGE_OFFSET && Offset < USER_PAGE_OFFSET + BYTES_PER_PAGE)
		EraseUserPage();
	int lastpagetoerase = (Offset + Size) / BYTES_PER_PAGE;
	int page = Offset & ~(BYTES_PER_PAGE - 1); // compute start offset of page to write to
	int pagenr = (Offset) / BYTES_PER_PAGE;
	while (pagenr <= lastpagetoerase)
	{
		Command = WRITE_PROTECT_KEY | CMD_ERASE_PAGE;
		Command |= ((pagenr << FCMD_PAGEN_OFFSET) & FCMD_PAGEN_MASK);
		// include the correct page number in the command
		WaitFlashReady();
		WriteCommand(Command); // execute page erase command
		WaitFlashReady();
		page += BYTES_PER_PAGE;
		Offset = page;
		pagenr = (Offset) / BYTES_PER_PAGE;
	}
}
int ProgramUserPage(Offset - USER_PAGE_OFFSET, DataBuffer)
{
	if (Offset >= BYTES_PER_PAGE || Offset + Length(DataBuffer) > BYTES_PER_PAGE)
		return -1;
	// Packet bufferPacket(BYTES_PER_PAGE) define a buffer packet
	// to manipulate the data
	// If the packet to be written is smaller than the user page we fill the
	// remaining space with existing data
	if (Offset > 0 || Length(DataBuffer) < BYTES_PER_PAGE)
		ReadMemory(mBaseAddress + USER_PAGE_OFFSET, bufferPacket, 0);
	// Must clear the page buffer before writing to it.
	ClearPageBuffer();
	int bytesLeftInPacket = Length(DataBuffer);
	int i = 0; // data packet index
	// Fill buffer packet
	while (bytesLeftInPacket > 0)
	{
		bufferPacket.writeSingleByte(Offset++, ReadSingleByte(DataBuffer));
		i++;
		bytesLeftInPacket--;
	}
	// Write page buffer
	WriteMemory(mBaseAddress + USER_PAGE_OFFSET, bufferPacket);
	uint32_t command = WRITE_PROTECT_KEY | CMD_WRITE_USER_PAGE;
	WaitFlashReady();
	WriteCommand(command); // execute user page write command
	WaitFlashReady();
}
int ProgramSequence(Offset, DataBuffer)
{
	if (Offset >= USER_PAGE_OFFSET && Offset < USER_PAGE_OFFSET + BYTES_PER_PAGE)
		ProgramUserPage(Offset - USER_PAGE_OFFSET, DataBuffer);

	if (Offset >= mDeviceSize || Offset + Length(DataBuffer) > mDeviceSize)
		return -1;
	// compute start offset of page to write to
	uint32_t page = Offset & ~(BYTES_PER_PAGE - 1);
	unsigned int bytesLeft = (Length(DataBuffer));
	int dataOffset = 0; // current offset in the data packet
	Packet bufferPacket(BYTES_PER_PAGE);
	// we write one page at a time
	// Loop until all bytes in data has been written
	while (bytesLeft > 0)
	{
		bufferPacket.clear(0xff);
		// Must clear the page buffer before writing to it.
		ClearPageBuffer();
		/* Keeps track of how many bytes to write to the bufferPacket.
		 * If the start offset is not aligned on a page boundary, we will not fill
		 * the bufferPacket completely. This is also the case when the number of
		 * bytes left to write is less than the size of a page. If the bufferPacket
		 * is not filled completely we first read the current flash content into
		 * the packet. This way we will always preserve existing flash data
		 * adjacent to the new data we wish to write.
		 */
		int bytesLeftInPacket = min((page + BYTES_PER_PAGE - Offset), bytesLeft);
		int bufferOffset = Offset % BYTES_PER_PAGE;
		if (bufferOffset != 0 || bytesLeftInPacket != (BYTES_PER_PAGE))
		{
			ReadMemory(mBaseAddress + page, bufferPacket, 0);
		}
		for (int i = 0; i < bytesLeftInPacket; ++i)
		{
			bufferPacket.writeSingleByte(bufferOffset++,
										 ReadSingleByte(DataBuffer));
			Offset++;
			WriteMemory(mBaseAddress + page, bufferPacket);
			int pagenr = ((Offset) / BYTES_PER_PAGE)
				uint32_t command = WRITE_PROTECT_KEY | CMD_WRITE_PAGE;
			// include the correct page number in the command
			command |= pagenr << FCMD_PAGEN_OFFSET;
			WaitFlashReady();
			WriteCommand(command); // execute page write command
			WaitFlashReady();
			page += BYTES_PER_PAGE;
			Offset = page;
			bytesLeft -= bytesLeftInPacket;
		}
	}
}
bool GetGeneralPurposeFuseBit(int Index)
{
	if (Index > 31 || Index < 0)
		return;
	uint32_t fgpfrReg = getRegister(FGPFR);
	return fgpfrReg & (1 << Index);
}
void SetGeneralPurposeFuseBit(int Index, bool Value)
{
	if (Index > 31 || Index < 0)
		return;
	uint32_t command = WRITE_PROTECT_KEY | (index << FCMD_PAGEN_OFFSET);
	if (Value) // erase bit
		command |= CMD_ERASE_GP_FUSE_BIT;
	else
		// program bit
		command |= CMD_WRITE_GP_FUSE_BIT;
	WaitFlashReady();
	WriteCommand(command);
	WaitFlashReady();
}

void SetGeneralPurposeFuseByte(int Index, unsigned char Value)
{
	if (Index > 3 || Index < 0)
		return;
	uint32_t command = WRITE_PROTECT_KEY | CMD_PROGRAM_GP_FUSE_BYTE | Index << FCMD_PAGEN_OFFSET | Value << (FCMD_PAGEN_OFFSET + 2);
	WaitFlashReady();
	WriteCommand(command);
	WaitFlashReady();
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

const struct flash_driver avr_flash = {
	.name = "avr",
	.commands = avrf_command_handlers,
	.flash_bank_command = avrf_flash_bank_command,
	.erase = avrf_erase,
	.write = avrf_write,
	.read = default_flash_read,
	.probe = avrf_probe,
	.auto_probe = avrf_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = avrf_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
