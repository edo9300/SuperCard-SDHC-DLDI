#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "nds/ndstypes.h"

// extern void sc_InitSCMode (void);
bool MemoryCard_IsInserted (void);
// extern void sc_sdcard_reset(void);

extern void sd_crc16_s(u16* buff,u16 num,u16* crc16buff);

extern void sd_data_read_s(u16 *buff);


bool init_sd();
void send_clk(u32 num);
void sc_sdcard_reset(void);
void SDCommand(u8 command,u32 sector);
void sd_data_write(u16 *buff,u8* crc16buff);
void WriteSector(u8 *buff,u32 sector,u32 writenum);
bool ReadSector (u8 *buff,u32 sector,u32 readnum);

#ifdef __cplusplus
}
#endif