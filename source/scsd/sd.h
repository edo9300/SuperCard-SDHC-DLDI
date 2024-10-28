#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool SDInit(void);
void SDSendClock(uint32_t num);
void SDResetCard();

extern uint16_t isSDHC;

#ifdef __cplusplus
}
#endif