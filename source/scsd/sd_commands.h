#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void SDCommand(uint8_t command, uint32_t argument);
void SDCommandAndDropResponse(uint8_t command, uint32_t argument, uint32_t bytesToDrop = 6);
bool SDCommandAndReadResponse(uint8_t command, uint32_t argument, uint8_t* responseBuffer, uint32_t bytesToRead = 6);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
template<typename T>
static inline auto& REG_SCSD_CMD = *(volatile T*)0x09800000;
inline void dummy_read(auto){}
#endif