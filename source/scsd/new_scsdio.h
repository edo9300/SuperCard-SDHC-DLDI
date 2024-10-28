#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "nds/ndstypes.h"

// extern void sc_InitSCMode (void);
bool MemoryCard_IsInserted (void);
// extern void sc_sdcard_reset(void);

void SDCommand(u8 command,u32 sector);
void sd_data_write(u16 *buff,u8* crc16buff);
void WriteSector(u8 *buff,u32 sector,u32 writenum);
bool ReadSector (u8 *buff,u32 sector,u32 readnum);

#ifdef __cplusplus
}
#endif