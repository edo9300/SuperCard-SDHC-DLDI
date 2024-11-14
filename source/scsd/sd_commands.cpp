#include "sd_commands.h"

static inline void drop_response(int bytesToDrop)
{
	bytesToDrop++; // +  8 clocks

	// Wait for the card to be non-busy
	while (((REG_SCSD_CMD<uint16_t> & 0x01) != 0));

	// 实际上，当跳出这个循环的时候，已经读了一个bit了，后续会多读一个bit，但是这是抛弃的rsp,因此多读一个bit也就是多一个时钟周期罢了
	while (--bytesToDrop)
	{
		dummy_read(REG_SCSD_CMD<uint32_t>);
		dummy_read(REG_SCSD_CMD<uint32_t>);
		dummy_read(REG_SCSD_CMD<uint32_t>);
		dummy_read(REG_SCSD_CMD<uint32_t>);
	}
}

inline bool read_response(uint8_t* dest, uint32_t length) {
	constexpr uint32_t BUSY_WAIT_TIMEOUT = 500000;
	
	for(auto i = BUSY_WAIT_TIMEOUT; (REG_SCSD_CMD<uint16_t> & 0x01) != 0; --i) {
		if(i == 0)
			return false;
	}
	
	int numBits = length * 8;
	// The first bit is always 0
	uint32_t partial_result = ((REG_SCSD_CMD<uint16_t>) & 0x01) << 16;
	numBits-=2;
	// Read the remaining bits in the response.
	// It's always most significant bit first
	const uint32_t mask_2bit = 0x10001;
	while (numBits) {
		numBits-=2;
		partial_result = (partial_result << 2) | (REG_SCSD_CMD<uint32_t> & mask_2bit);
		if ((numBits & 0x7) == 0) {
			//_1_3_5_7 _0_2_4_6 
			*dest++ = ((partial_result >> 16) | (partial_result<<1));
			partial_result = 0;
		}
	}
	for (int i = 0; i < 4; ++i) {//8clock
		dummy_read(REG_SCSD_CMD<uint32_t>);
	}
	return true;
}

namespace {
constexpr uint32_t sd_crc_bit = 0x100000;
constexpr uint32_t sd_nocrc_bit = 0;
constexpr uint32_t sd_st = 0x040000+sd_crc_bit;
constexpr uint32_t sd_rw4 = 0x200000;
constexpr uint32_t sd_rw3 = 0x080000;
constexpr uint32_t sd_rw2 = 0x280000;
constexpr uint32_t sd_rw1 = 0;
constexpr uint32_t sd_buff_bit = 0x400000;
constexpr uint32_t sd_command_bit = 0x800000;
}

void SDCommand(SD_COMMANDS command, uint32_t argument)
{
	uint8_t databuff[5];
	uint8_t *tempDataPtr = databuff;

	*tempDataPtr++ = command | 0x40;
	*tempDataPtr++ = argument >> 24;
	*tempDataPtr++ = argument >> 16;
	*tempDataPtr++ = argument >> 8;
	*tempDataPtr++ = argument;

	while (((REG_SCSD_CMD<uint16_t> & 0x01) == 0));

	dummy_read(REG_SCSD_CMD<uint16_t>);
	
	// clear crc7
	*(volatile uint16_t*)(0x9000000+sd_buff_bit) = 0;
	
	auto* r3 = (volatile uint32_t*)(0x9000000 + sd_command_bit + sd_rw4);
	for(uint32_t byte : databuff) {
		byte |= byte << 20;
		*r3 = byte;
	}
	*(volatile uint32_t*)(0x9000000 + sd_command_bit + sd_crc_bit + sd_rw4) = 0;

	return;
}

void SDCommandAndDropResponse(SD_COMMANDS command, uint32_t argument, uint32_t bytesToDrop) {
	SDCommand(command, argument);
	drop_response(bytesToDrop);
}

bool SDCommandAndReadResponse(SD_COMMANDS command, uint32_t argument, uint8_t* responseBuffer, uint32_t bytesToRead) {
	SDCommand(command, argument);
	return read_response(responseBuffer, bytesToRead);
}

APP_COMMAND_RESULT SDAppCommand(SD_APP_COMMANDS app_command, uint32_t relative_card_address, uint32_t argument) {
	uint8_t responseBuffer[6]; // purposely uninitialized
	SDCommandAndReadResponse(APP_CMD, relative_card_address, responseBuffer, 6);
	if (responseBuffer[0] != APP_CMD)
		return FAILED_TO_SEND;
	SDCommand(static_cast<SD_COMMANDS>(app_command), argument);
	return OK;
}

APP_COMMAND_RESULT SDAppCommandAndDropResponse(SD_APP_COMMANDS app_command, uint32_t relative_card_address, uint32_t argument, uint32_t bytesToDrop) {
	if(auto res = SDAppCommand(app_command, relative_card_address, argument); res != OK)
		return res;
	drop_response(bytesToDrop);
	return OK;
}

APP_COMMAND_RESULT SDAppCommandAndReadResponse(SD_APP_COMMANDS app_command, uint32_t relative_card_address, uint32_t argument, uint8_t* responseBuffer, uint32_t bytesToRead) {
	if(auto res = SDAppCommand(app_command, relative_card_address, argument); res != OK)
		return res;
	return read_response(responseBuffer, bytesToRead) ? OK : FAILED_TO_PARSE_RESPONSE;
}