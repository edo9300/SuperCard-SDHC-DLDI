#include "new_scsdio.h"
#include "sc_commands.h"
#include "sd_commands.h"
#include "memcnt_guard.h"
#include "sd.h"

#ifdef __BLOCKSDS__
extern "C" void __aeabi_memcpy4(volatile void *dest, const volatile void *src, int n);
#else
#define INLINE_ASM 1
#endif

template<typename T>
static inline auto& REG_SCSD_DATAADD = *(volatile T*)0x09000000;

template<typename T>
static inline auto& REG_SCSD_DATAREAD = *(volatile T*)0x09100000;

static bool SCSD_readData(void* buffer);
static void SCSD_writeData(void* buff, void* crc16buff);

uint64_t sdio_crc16_4bit_checksum(void* data, uint32_t num_words);

static constexpr auto SCSD_STS_BUSY = 0x0100;

inline void WaitOnWrite(bool needsBusy) {
	while (true) {
		auto isBusy = (REG_SCSD_DATAADD<uint16_t> & SCSD_STS_BUSY) != 0;
		if(isBusy != needsBusy)
			return;
	}
}

extern "C" void sdms_data_write_s(void* buff, void* crc16buff);

[[gnu::optimize("Os")]]
void WriteSector(uint8_t* buff, uint32_t sector, uint32_t writenum)
{
	MemcntGuard guard{true};
	uint64_t crc16;//并行4个
	sc_change_mode(en_sdram + en_sdcard);
	SDResetCard();
	auto param = is_sdhc() ? sector : (sector << 9);
	SDCommandAndDropResponse(WRITE_MULTIPLE_BLOCK, param);
	SDSendClock(0x10);
	for (auto buffEnd = buff + writenum * 512 ; buff < buffEnd; buff += 512)
	{
		crc16 = sdio_crc16_4bit_checksum(buff, 512 / sizeof(uint32_t));
		union {
			uint64_t val;
			uint16_t arr[4];
		} u{crc16};
		SCSD_writeData(buff, u.arr);
		SDSendClock(0x10);
	}
	SDCommandAndDropResponse(STOP_TRANSMISSION, 0);
	SDSendClock(0x10);
	WaitOnWrite(false);
	sc_change_mode(en_sdram);
	return;
}

extern "C" void sdms_data_read_s(void* buff);

[[gnu::optimize("Os")]]
bool ReadSector(uint8_t *buff, uint32_t sector, uint32_t readnum)
{
	MemcntGuard guard{true};
	auto param = is_sdhc() ? sector : (sector << 9);
    sc_change_mode(en_sdram + en_sdcard);
	SDResetCard();
	SDCommandAndDropResponse(READ_MULTIPLE_BLOCK, param);
	{
		for(auto buffer_end = buff + readnum * (512); buff < buffer_end; buff += 512)
		{
			SCSD_readData(buff);
		}
	}
	SDCommandAndDropResponse(STOP_TRANSMISSION, 0);

	SDSendClock(0x10);	   // Send clock signal
	sc_change_mode(en_sdram);
	return true;
}

namespace {
constexpr uint32_t sd_crc_bit = 0x100000;
constexpr uint32_t sd_nocrc_bit = 0;

constexpr uint32_t sd_st = 0x040000+sd_crc_bit;
constexpr uint32_t sd_rw4 = 0x200000;
constexpr uint32_t sd_rw3 = 0x080000;
constexpr uint32_t sd_rw2 = 0x280000;
constexpr uint32_t sd_rw1 = 0;
constexpr uint32_t sd_buff_bit = 0x400000;

template<typename T>
static inline auto& REG_SCSD_DATAREAD_4 = *(volatile T*)(0x9000000 + sd_rw4);
template<typename T>
static inline auto& REG_SCSD_DATAREAD_2 = *(volatile T*)(0x9000000 + sd_rw2);
template<typename T>
static inline auto& REG_SCSD_DATAREAD_1 = *(volatile T*)(0x9000000 + sd_rw1);

template<typename T>
static inline auto& REG_SCSD_DATAWRITE_4 = *(volatile T*)(0x9000000 + sd_rw4);
template<typename T>
static inline auto& REG_SCSD_DATAWRITE_2 = *(volatile T*)(0x9000000 + sd_rw2);
template<typename T>
static inline auto& REG_SCSD_DATAWRITE_1 = *(volatile T*)(0x9000000 + sd_rw1);

#define SD_STATUS (*(volatile uint16_t*)(0x9000000+sd_st))
#define SD_BUFF_BIT (*(volatile uint16_t*)(0x9000000+sd_buff_bit))

#define BUSY_WAIT_TIMEOUT 500000
#define SCSD_STS_BUSY 0x100

}

union ptr_cast {
	void* ptr;
	uint8_t* u8;
	uint16_t* u16;
	uint32_t* u32;
};

#if !defined(__BLOCKSDS__) && !INLINE_ASM
static void SCSD_writeData32(uint32_t* buff_u32) {
	for(int i = (512/32); i; --i) {
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
		REG_SCSD_DATAWRITE_4<uint32_t> = *buff_u32++;
	}
}
#endif

[[gnu::optimize(0)]] // we don't really want gcc to unroll this loop
static void SCSD_writeData16(uint16_t* buff_u16) {
	auto readU16Aligned = [&]() -> uint32_t {
		uint32_t res = *buff_u16++;
		res |= (*buff_u16++) << 16;
		return res;
	};
	for(int i = (512/32); i; --i) {
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU16Aligned();
	}
}

[[gnu::optimize(0)]] // we don't really want gcc to unroll this loop
static void SCSD_writeData8(uint8_t* buff_u8) {
	auto readU8Aligned = [&]() -> uint32_t {
		uint32_t res = *buff_u8++;
		res |= (*buff_u8++) << 8;
		res |= (*buff_u8++) << 16;
		res |= (*buff_u8++) << 24;
		return res;
	};
	for(int i = (512/32); i; --i) {
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
		REG_SCSD_DATAWRITE_4<uint32_t> = readU8Aligned();
	}
}

void SCSD_writeData(void* buffer, void* crc_buff)
{
	WaitOnWrite(false);
	dummy_read(REG_SCSD_DATAADD<uint16_t>);
	REG_SCSD_DATAADD<uint16_t> = 0; // start bit

	if(((intptr_t)buffer & 0x03) == 0) [[likely]] {
#if defined(__BLOCKSDS__)
		__aeabi_memcpy4(&REG_SCSD_DATAWRITE_4<uint32_t>, buffer, 512);
#elif !INLINE_ASM
		SCSD_writeData32(ptr_cast{buffer}.u32);
#else
		asm volatile(
			"write_loop:\n"
			"	ldmia   %1!,{r3-r10}\n"
			"	stmia   %2,{r3-r10}\n"
			"	subs    %3, %3, #1\n"
			"	bne     write_loop\n"
			: "=r"(buffer)
			: "0"(buffer), "r"((intptr_t)&REG_SCSD_DATAWRITE_4<uint32_t>), "r"(512/32)
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "cc"
		);
#endif
	} else if(((intptr_t)buffer & 0x01) == 0) {
		SCSD_writeData16(ptr_cast{buffer}.u16);
	} else {
		SCSD_writeData8(ptr_cast{buffer}.u8);
	}

	auto* crc_u16 = ptr_cast{crc_buff}.u16;
	REG_SCSD_DATAWRITE_4<uint16_t> = *crc_u16++;
	REG_SCSD_DATAWRITE_4<uint16_t> = *crc_u16++;
	REG_SCSD_DATAWRITE_4<uint16_t> = *crc_u16++;
	REG_SCSD_DATAWRITE_4<uint16_t> = *crc_u16++;

	REG_SCSD_DATAADD<uint16_t> = 0xFF; // end bit

	WaitOnWrite(true);
	// dummy writes
	REG_SCSD_DATAADD<uint32_t> = 0;
	REG_SCSD_DATAADD<uint32_t> = 0;
}

#if !defined(__BLOCKSDS__) && !INLINE_ASM
static void SCSD_readData32(uint32_t* buff_u32) {
	for(int i = (512/32); i; --i) {
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
		*buff_u32++ = REG_SCSD_DATAREAD_4<uint32_t>;
	}
}
#endif

[[gnu::optimize(0)]] // we don't really want gcc to unroll this loop
static void SCSD_readData16(uint16_t* buff_u16) {
	auto writeU16Aligned = [&](uint32_t value){
		*buff_u16++ = value & 0xFFFF;
		*buff_u16++ = value >> 16;
	};
	for(int i = (512/32); i; --i) {
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU16Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
	}
}

[[gnu::optimize(0)]] // we don't really want gcc to unroll this loop
static void SCSD_readData8(uint8_t* buff_u8) {
	auto writeU8Aligned = [&](uint32_t value){
		*buff_u8++ = value & 0xFF;
		*buff_u8++ = (value >> 8) & 0xFF;
		*buff_u8++ = (value >> 16) & 0xFF;
		*buff_u8++ = (value >> 24) & 0xFF;
	};
	for(int i = (512/32); i; --i) {
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
		writeU8Aligned(REG_SCSD_DATAREAD_4<uint32_t>);
	}
}

static bool SCSD_readData(void* buffer) {
	dummy_read(SD_STATUS);

	while(SD_BUFF_BIT & 1);
	dummy_read(REG_SCSD_DATAREAD_4<uint16_t>);

	if(((intptr_t)buffer & 0x03) == 0) [[likely]] {
#if defined(__BLOCKSDS__)
		__aeabi_memcpy4(buffer, &REG_SCSD_DATAREAD_4<uint32_t>, 512);
#elif !INLINE_ASM
		SCSD_readData32(ptr_cast{buffer}.u32);
#else
		asm volatile(
			"read_loop:\n"
			"	ldmia   %2,{r3-r10}\n"	// currently fills whole 256K of RAM,
			"	stmia   %1!,{r3-r10}\n" // even though the proggy is smaller
			"	subs    %3, %3, #1\n"
			"	bne     read_loop\n"
			: "=r"(buffer)
			: "0"(buffer), "r"(&REG_SCSD_DATAREAD_4<uint32_t>), "r"(512/32)
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "cc", "memory"
		);
#endif
	} else if(((intptr_t)buffer & 0x01) == 0) {
		SCSD_readData16(ptr_cast{buffer}.u16);
	} else {
		SCSD_readData8(ptr_cast{buffer}.u8);
	}

	//crc16
	dummy_read(REG_SCSD_DATAREAD_4<uint32_t>);
	dummy_read(REG_SCSD_DATAREAD_4<uint16_t>);
	//end bit
	dummy_read(REG_SCSD_DATAREAD_1<uint16_t>);

	return true;
}

bool MemoryCard_IsInserted(void) {
    sc_change_mode(en_sdram + en_sdcard);
	return (REG_SCSD_CMD<uint16_t> & 0x300)==0; // 读取状态寄存器的值
}

uint64_t inline calSingleCRC16(uint64_t crc, uint32_t data_in){
	// Shift out 8 bits for each line
	uint32_t data_out = crc >> 32;
	crc <<= 32;

	// XOR outgoing data to itself with 4 bit delay
	data_out ^= (data_out >> 16);

	// XOR incoming data to outgoing data with 4 bit delay
	data_out ^= (data_in >> 16);

	// XOR outgoing and incoming data to accumulator at each tap
	uint64_t xorred = data_out ^ data_in;
	crc ^= xorred;
	crc ^= xorred << (5 * 4);
	crc ^= xorred << (12 * 4);
	return crc;
}

uint32_t loadBigEndU32_u8(uint8_t* &dataBuf){
	uint32_t data;
	data = (*dataBuf++) << 24;
	data |= (*dataBuf++) << 16;
	data |= (*dataBuf++) << 8;
	data |= (*dataBuf++);
	return data;
}

uint64_t sdio_crc16_4bit_checksum(void* dataBuf, uint32_t num_words)
{
	uint64_t crc = 0;
	if((uintptr_t)dataBuf & 3){//uint8_t align
		auto* data = static_cast<uint8_t*>(dataBuf);
		auto* end = data + (num_words * sizeof(uint32_t));
		while (data < end)
		{
			uint32_t data_in = loadBigEndU32_u8(data);
			crc = calSingleCRC16(crc, data_in);
		}
	} else {
		auto* data = static_cast<uint32_t*>(dataBuf);
		auto* end = data + num_words;
		while (data < end)
		{
			uint32_t data_in = __builtin_bswap32(*data++);
			crc = calSingleCRC16(crc, data_in);
		}
	}

	return __builtin_bswap64(crc);
}
