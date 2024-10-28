#pragma once
#include <cstdint>

class MemcntGuard {
	static inline auto& EXMEMCNT_ADDR = *(volatile uint16_t*)0x04000204;
	uint16_t old_memcnt{};
	/*  2-3   32-pin GBA Slot ROM 1st Access Time (0-3 = 10, 8, 6, 18 cycles)
		4	 32-pin GBA Slot ROM 2nd Access Time (0-1 = 6, 4 cycles)*/
	static uint16_t setFastCNT(uint16_t originData) {
		const uint16_t mask = ~0b00011100;//~ 000011100, clear bit 2-3 + 4
		const uint16_t setVal = ((2) << 2) | (1<<4);
		return (originData & mask) | setVal;
	}
	static uint16_t setSlowCNT(uint16_t originData) {
		const uint16_t mask = ~0b00011100;//~ 000011100, clear bit 2-3 + 4
		return (originData & mask);
	}
public:
	MemcntGuard(bool slow) {
		old_memcnt = EXMEMCNT_ADDR;
		if(slow) {
			EXMEMCNT_ADDR = setSlowCNT(old_memcnt);
		} else {
			EXMEMCNT_ADDR = setFastCNT(old_memcnt);
		}
	}
	~MemcntGuard() {
		EXMEMCNT_ADDR = old_memcnt;
	}
};