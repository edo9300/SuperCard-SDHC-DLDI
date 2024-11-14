#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum SD_COMMANDS {
	// Basic Commands (class 0)
	CMD0 = 0,
	CMD2 = 2,
	CMD3,
	CMD4,
	CMD7 = 7,
	CMD8,
	CMD9,
	CMD10,
	CMD11,
	CMD12,
	CMD13,
	CMD15 = 15,
	// Block-Oriented Read Commands (class 2)
	CMD16,
	CMD17,
	CMD18,
	CMD19,
	CMD20,
	CMD22 = 22,
	CMD23,
	// Block-Oriented Write Commands (class 4)
	// CMD16
	// CMD20
	// CMD22
	// CMD23
	CMD24,
	CMD25,
	CMD27 = 27,
	// Application-Specific Commands (class 8)
	// CMD23
	CMD55 = 55,
	CMD56,
	// aliases
	GO_IDLE_STATE = CMD0,
	ALL_SEND_CID = CMD2,
	SEND_RELATIVE_ADDR = CMD3,
	SET_DSR = CMD4,
	SELECT_DESELECT_CARD = CMD7,
	SEND_IF_COND = CMD8,
	SEND_CSD = CMD9,
	SEND_CID = CMD10,
	VOLTAGE_SWITCH = CMD11,
	STOP_TRANSMISSION = CMD12,
	SEND_STATUS_SEND_TASK_STATUS = CMD13,
	GO_INACTIVE_STATE = CMD15,
	SET_BLOCKLEN = CMD16,
	READ_SINGLE_BLOCK = CMD17,
	READ_MULTIPLE_BLOCK = CMD18,
	SEND_TUNING_BLOCK = CMD19,
	SPEED_CLASS_CONTROL = CMD20,
	ADDRESS_EXTENSION = CMD22,
	SET_BLOCK_COUNT = CMD23,
	WRITE_BLOCK = CMD24,
	WRITE_MULTIPLE_BLOCK = CMD25,
	PROGRAM_CSD = CMD27,
	APP_CMD = CMD55,
	GEN_CMD = CMD56,
} SD_COMMANDS;

typedef enum SD_APP_COMMANDS {
	ACMD6 = 6,
	ACMD13 = 13,
	ACMD22 = 22,
	ACMD23,
	ACMD41 = 41,
	ACMD42,
	ACMD51 = 51,
	// aliases
	SET_BUS_WIDTH = ACMD6,
	SD_STATUS = ACMD13,
	SEND_NUM_WR_BLOCKS = ACMD22,
	SET_WR_BLK_ERASE_COUNT = ACMD23,
	SD_SEND_OP_COND = ACMD41,
	SET_CLR_CARD_DETECT = ACMD42,
	SEND_SCR = ACMD51,
} SD_APP_COMMANDS;

void SDCommand(SD_COMMANDS command, uint32_t argument);
void SDCommandAndDropResponse(SD_COMMANDS command, uint32_t argument, uint32_t bytesToDrop = 6);
bool SDCommandAndReadResponse(SD_COMMANDS command, uint32_t argument, uint8_t* responseBuffer, uint32_t bytesToRead = 6);



typedef enum APP_COMMAND_RESULT {
	OK,
	FAILED_TO_SEND,
	FAILED_TO_PARSE_RESPONSE,
} APP_COMMAND_RESULT;
APP_COMMAND_RESULT SDAppCommand(SD_APP_COMMANDS app_command, uint32_t relative_card_address, uint32_t argument);
APP_COMMAND_RESULT SDAppCommandAndDropResponse(SD_APP_COMMANDS app_command, uint32_t relative_card_address, uint32_t argument, uint32_t bytesToDrop = 6);
APP_COMMAND_RESULT SDAppCommandAndReadResponse(SD_APP_COMMANDS app_command, uint32_t relative_card_address, uint32_t argument, uint8_t* responseBuffer, uint32_t bytesToRead = 6);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
template<typename T>
static inline auto& REG_SCSD_CMD = *(volatile T*)0x09800000;
inline void dummy_read(auto){}
#endif