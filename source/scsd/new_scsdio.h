#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool MemoryCard_IsInserted(void);

void WriteSector(uint8_t* buff, uint32_t sector, uint32_t writenum);
bool ReadSector(uint8_t* buff, uint32_t sector, uint32_t readnum);

#ifdef __cplusplus
}
#endif