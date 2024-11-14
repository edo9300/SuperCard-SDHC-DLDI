#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define en_firmware 0
#define en_sdram 1
#define en_sdcard 2
#define en_write 4

void sc_change_mode(uint16_t mode);

#ifdef __cplusplus
}
#endif