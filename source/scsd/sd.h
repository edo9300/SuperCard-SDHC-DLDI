#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool SDInit(void);
void SDSendClock(uint32_t num);
void SDResetCard();

bool is_sdhc(void);

#ifdef __cplusplus
}
#endif