#include <utility>
#include "sc_commands.h"
#include <nds/memory.h>

bool isSCLite = false;

#define SC_FLASH_IDLE			((u16) 0xF0)

static std::pair<vu16*, vu16*> get_magic_addrs(bool lite) {
	auto* const SCLITE_FLASH_MAGIC_ADDR_1 = (vu16*)0x08000AAA;
	auto* const SCLITE_FLASH_MAGIC_ADDR_2 = (vu16*)0x08000554;
	auto* const SC_FLASH_MAGIC_ADDR_1	 = (vu16*)0x08000b92;
	auto* const SC_FLASH_MAGIC_ADDR_2	 = (vu16*)0x0800046c;
	if(lite)
		return { SCLITE_FLASH_MAGIC_ADDR_1, SCLITE_FLASH_MAGIC_ADDR_2 };
	return { SC_FLASH_MAGIC_ADDR_1, SC_FLASH_MAGIC_ADDR_2 };
}

void sc_change_mode(u16 mode) {
	auto* const SC_MODE_REG = (vu16*)0x09FFFFFE;
	const u16 SC_MODE_MAGIC			= 0xA55A;
	*SC_MODE_REG = SC_MODE_MAGIC;
	*SC_MODE_REG = SC_MODE_MAGIC;
	*SC_MODE_REG = mode;
	*SC_MODE_REG = mode;
}

void sc_flash_rw_enable(bool lite) {
	constexpr u16 SC_MODE_FLASH_RW		= 0x0004;
	constexpr u16 SC_MODE_FLASH_RW_LITE	= 0x1510;
	sc_change_mode(lite ? SC_MODE_FLASH_RW_LITE : SC_MODE_FLASH_RW);
}

void send_command(SC_FLASH_COMMAND command, bool lite) {
	constexpr u16 SC_FLASH_MAGIC_1 = 0x00aa;
	constexpr u16 SC_FLASH_MAGIC_2 = 0x0055;
	auto [magic_addr_1, magic_addr_2] = get_magic_addrs(lite);
	*magic_addr_1 = SC_FLASH_MAGIC_1;
	*magic_addr_2 = SC_FLASH_MAGIC_2;
	*magic_addr_1 = static_cast<u16>(command);
}

bool try_guess_lite() {
	auto get_flash_id = [](bool lite) -> u32 {
		static constexpr u16 COMMAND_ERROR = 0x002e;
		sc_flash_rw_enable(lite);
		auto [magic_addr_1, magic_addr_2] = get_magic_addrs(lite);
		send_command(SC_FLASH_COMMAND::IDENTIFY, lite);
		auto upper_half = *GBA_BUS;
		auto magic = *magic_addr_1;
		*GBA_BUS = SC_FLASH_IDLE;
		if(upper_half == COMMAND_ERROR)
			return 0;
		return (upper_half << 16) | magic;
	};
	if(auto id = get_flash_id(true); id != 0)
		return (isSCLite = true);
	
	return false;
}
