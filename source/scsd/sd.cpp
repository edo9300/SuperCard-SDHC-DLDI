#include "memcnt_guard.h"
#include "sc_commands.h"
#include "sd.h"
#include "sd_commands.h"

void SDSendClock(uint32_t num) {
	while (num--)
	{
		dummy_read(REG_SCSD_CMD<uint16_t>);
	}
}

uint16_t isSDHC = false;

inline constexpr auto NUM_STARTUP_CLOCKS = 30000;
inline constexpr auto GO_IDLE_STATE = 0;
inline constexpr auto CMD8 = 8;
inline constexpr auto APP_CMD = 55;
inline constexpr auto CMD58 = 58;
inline constexpr auto SD_APP_OP_COND = 41;
inline constexpr auto MAX_STARTUP_TRIES = 5000;
inline constexpr auto SD_OCR_VALUE = 0x00030000;
//2.8V to 3.0V
inline constexpr auto ALL_SEND_CID = 2;
inline constexpr auto SEND_RELATIVE_ADDR = 3;
inline constexpr auto SD_STATE_STBY = 3;
inline constexpr auto SEND_CSD = 9;
inline constexpr auto SELECT_CARD = 7;
inline constexpr auto SET_BUS_WIDTH = 6;
inline constexpr auto SET_BLOCKLEN = 16;
inline constexpr auto RESPONSE_TIMEOUT = 256;
inline constexpr auto SEND_STATUS = 13;
inline constexpr auto SD_STATE_TRAN = 4;
inline constexpr auto READY_FOR_DATA = 1;

void SDResetCard()
{
	auto* SD_RESET_ADDR = (volatile uint16_t*)0x09440000;
	*SD_RESET_ADDR = 0xFFFF;
}

bool SDInit(){
	MemcntGuard guard{true};
	try_guess_lite();
    sc_change_mode(en_sdram + en_sdcard);
	SDResetCard(); //do we really need that?
	SDSendClock (NUM_STARTUP_CLOCKS);
	SDCommand (GO_IDLE_STATE, 0);
	SDSendClock (NUM_STARTUP_CLOCKS);

	uint8_t responseBuffer[17] = {0};
	isSDHC = false;
	uint16_t cmd8Response = SDCommandAndReadResponse(CMD8, 0x1AA, responseBuffer, 17);
	if (cmd8Response && responseBuffer[0] == CMD8 && responseBuffer[1] == 0 && responseBuffer[2] == 0 && responseBuffer[3] == 0x1 && responseBuffer[4] == 0xAA) {
		isSDHC = true;//might be
	}
	{
		int i;
		for (i = 0; i < MAX_STARTUP_TRIES; ++i) {
			SDCommandAndReadResponse(APP_CMD, 0, responseBuffer, 6);
			if (responseBuffer[0] != APP_CMD) {	
				return false;
			}

			// uint32_t arg = SD_OCR_VALUE;
			uint32_t arg = 0;
			arg |= (1<<28); //Max performance
			arg |= (1<<20); //3.3v
			if(isSDHC)
				arg |= (1<<30); // Set HCS bit,Supports SDHC

			if (SDCommandAndReadResponse(SD_APP_OP_COND, arg, responseBuffer, 6) &&//ACMD41
				((responseBuffer[1] & (1<<7)) != 0)/*Busy:0b:initing 1b:init completed*/) {
				uint16_t CCS = responseBuffer[1] & (1<<6);//0b:SDSC  1b:SDHC/SDXC
				if(!CCS && isSDHC)
					isSDHC = false;
				break; // Card is ready
			}
			SDSendClock (NUM_STARTUP_CLOCKS);
		}

		if (i >= MAX_STARTUP_TRIES) {
			return false;
		}
	}

	// The card's name, as assigned by the manufacturer
	SDCommandAndDropResponse(ALL_SEND_CID, 0, 17);
	// Get a new address
	uint32_t relativeCardAddress = 0;
	{
		int i;
		for (i = 0; i < MAX_STARTUP_TRIES ; i++) {
			SDCommandAndReadResponse(SEND_RELATIVE_ADDR, 0, responseBuffer, 6);
			relativeCardAddress = (responseBuffer[1] << 24) | (responseBuffer[2] << 16);
			if ((responseBuffer[3] & 0x1e) != (SD_STATE_STBY << 1)) {
				break;
			}
		}
		if (i >= MAX_STARTUP_TRIES) {
			return false;
		}
	}

	// Some cards won't go to higher speeds unless they think you checked their capabilities
	SDCommandAndDropResponse(SEND_CSD, relativeCardAddress, 17);
 
	// Only this card should respond to all future commands
	SDCommandAndDropResponse(SELECT_CARD, relativeCardAddress, 6);
 
	// Set a 4 bit data bus
	SDCommandAndDropResponse(APP_CMD, relativeCardAddress, 6);
	SDCommandAndDropResponse(SET_BUS_WIDTH, 2, 6); // 4-bit mode.
	
	return true;
}