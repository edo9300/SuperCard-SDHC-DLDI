#include "new_scsdio.h"
#include "sc_commands.h"
#include "sd_commands.h"
#include "memcnt_guard.h"
#include "sd.h"

template<typename T>
static inline auto& REG_SCSD_DATAADD = *(volatile T*)0x09000000;

template<typename T>
static inline auto& REG_SCSD_DATAREAD = *(volatile T*)0x09100000;

bool SCSD_readData(void* buffer);

uint64_t sdio_crc16_4bit_checksum(void* data, uint32_t num_words);

static constexpr auto SCSD_STS_BUSY = 0x0100;

inline void WaitOnRead(bool needsBusy) {
	while (true) {
		auto isBusy = (REG_SCSD_DATAREAD<uint16_t> & SCSD_STS_BUSY) != 0;
		if(isBusy != needsBusy)
			return;
	}
}

inline void WaitOnWrite(bool needsBusy) {
	while (true) {
		auto isBusy = (REG_SCSD_DATAADD<uint16_t> & SCSD_STS_BUSY) != 0;
		if(isBusy != needsBusy)
			return;
	}
}

void WriteSector(uint8_t* buff, uint32_t sector, uint32_t writenum)
{
	MemcntGuard guard{true};
	uint64_t crc16;//并行4个
	sc_change_mode(en_sdram + en_sdcard);
	SDResetCard();
	auto param = is_sdhc() ? sector : (sector << 9);
	SDCommandAndDropResponse(WRITE_MULTIPLE_BLOCK, param);
	for (auto buffEnd = buff + writenum * 512 ; buff < buffEnd; buff += 512)
	{
		crc16 = sdio_crc16_4bit_checksum((uint32_t*)(buff), 512 / sizeof(uint32_t));
		sd_data_write((uint16_t*)(buff), (uint8_t*)(&crc16));
		SDSendClock(0x10);
	}
	SDCommandAndDropResponse(STOP_TRANSMISSION, 0);
	SDSendClock(0x10);
	WaitOnWrite(false);
	return;
}

bool ReadSector(uint8_t *buff, uint32_t sector, uint32_t readnum)
{
	uint16_t res = true;
	MemcntGuard guard{true};
	sc_change_mode(en_sdram + en_sdcard);
	auto param = is_sdhc() ? sector : (sector << 9);
	SDCommand(READ_MULTIPLE_BLOCK, param); // R0 = 0x12, R1 = 0, R2 as calculated above
	{
		MemcntGuard guard{is_scLite()};
		for(auto buffer_end = buff + readnum*(512); buff < buffer_end; buff += 512)
		{
			res = SCSD_readData(buff); // Add R6, left shifted by 9, to R4 before casting
			if(!res)
				break;
		}
	}
	SDCommandAndDropResponse(STOP_TRANSMISSION, 0); // Command to presumably stop reading

	SDSendClock(0x10);	   // Send clock signal
	return res;
}

#define BUSY_WAIT_TIMEOUT 500000
#define SCSD_STS_BUSY 0x100

void sd_data_write(uint16_t *buff, uint8_t *crc16buff)
{
	WaitOnWrite(false); // Note:两边的等待是不一致的
	dummy_read(REG_SCSD_DATAADD<uint16_t>);
	REG_SCSD_DATAADD<uint16_t> = 0; // start bit
	{
		MemcntGuard guard{false};

		auto writeuint16_t = [](uint32_t data)//lambda Function
			{
				
			// Write the data to the card
			// 4 halfwords are transmitted to the Supercard at once, for timing purposes
			// Only the first halfword needs to contain data for standard SuperCards
			// For the SuperCard Lite, the data is split into 4 nibbles, one per halfword, with this arrangment:
			//		 The first nibble is taken from bits 4-7 of the second halfword
			//		 The second nibble is taken from bits 4-7 of the first halfword
			//		 The third nibble is taken from bits 4-7 of the fourth halfword
			//		 The fourth nibble is taken from bits 4-7 of the third halfword
				data |= (data << 20);
				REG_SCSD_DATAADD<uint32_t> = data;
				REG_SCSD_DATAADD<uint32_t> = (data >> 8);
			};

#define LOAD_ORR_CONSTANT \
	"mov     r4, #255\n" \
	"orr     r4, r4, #65280\n"
#define WRITE_U16 \
	"ldrh r0, [%0], #2\n" \
	"add r0, r0, r0, lsl #20\n" \
	"lsr r1, r0, #8\n" \
	"stmia %1, {r0-r1}\n"
#define WRITE_U32 \
	"ldr r0, [%0], #4\n"   \
	"lsr r2, r0, #16\n"	\
	"and r0, r0, r4\n" \
	"add r0, r0, r0, lsl #20\n" \
	"lsr r1, r0, #8\n"	 \
	"add r2, r2, r2, lsl #20\n" \
	"lsr r3, r2, #8\n" \
	"stmia %1, {r0-r1}\n"   \
	"stmia %1, {r2-r3}\n"

		if((uint32_t)buff & 3){//unaligned
			uint8_t* buff_u8 = (uint8_t*)buff;
			uint16_t byteHI;
			uint16_t byteLo;
			for (int i = 0; i < 512; i += 2){
				byteLo = *buff_u8++;
				byteHI = *buff_u8++;
				writeuint16_t((byteHI << 8) | byteLo);
			}
		}
		else if((uint32_t)buff & 2){//uint16_t aligned
			asm volatile(
				LOAD_ORR_CONSTANT
				WRITE_U16
				WRITE_U32
			"1:\n"
				WRITE_U32
				WRITE_U32
				"cmp %0, %2\n"
				"blt 1b\n"
				WRITE_U16
				: // 没有输出
				: "r"((uint32_t)buff),"r"((uint32_t)&REG_SCSD_DATAADD<uint32_t>),"r"(((uint32_t)buff) + 510/*512-2*/)
				: "r0", "r1", "r2", "r3", "r4", "cc"// 破坏列表
			);
		}
		else{//uint32_t aligned
			asm volatile(
				LOAD_ORR_CONSTANT
			"2:\n"
				WRITE_U32
				WRITE_U32
				"cmp %0, %2\n"
				"blt 2b"
				: // 没有输出
				: "r"((uint32_t)buff),"r"((uint32_t)&REG_SCSD_DATAADD<uint32_t>),"r"(((uint32_t)buff) + 512)
				: "r0", "r1", "r2", "r3", "r4", "cc"// 破坏列表
			);
			// uint32_t *buff_u32 = (uint32_t*)buff;
			// for (int i = 0; i < 512; i += 4){
			//	 writeuint32_t(*buff_u32++);
			// }//or?
			// for (int i = 0; i < 512; i += 2){
			//	 writeuint16_t(*buff++);
			// }
		}

		
		if ((uint32_t)crc16buff & 1)
		{   
			uint16_t byteHI;
			uint16_t byteLo;
			for (int i = 0; i < 4; i ++){
				byteLo = *crc16buff++;
				byteHI = *crc16buff++;
				writeuint16_t((byteHI << 8) | byteLo);
			}
		}
		else if ((uint32_t)crc16buff & 2){
			asm volatile(
				WRITE_U16
				WRITE_U16
				WRITE_U16
				WRITE_U16
				: // 没有输出
				: "r"((uint32_t)crc16buff),"r"((uint32_t)&REG_SCSD_DATAADD<uint32_t>)
				: "r0", "r1", "r2", "r3", "cc"// 破坏列表
			);
		}
		else{
			asm volatile(
				LOAD_ORR_CONSTANT
				WRITE_U32
				WRITE_U32
				: // 没有输出
				: "r"((uint32_t)crc16buff),"r"((uint32_t)&REG_SCSD_DATAADD<uint32_t>)
				: "r0", "r1", "r2", "r3", "r4", "cc"// 破坏列表
			);
			// for (int i = 0; i < 4; i++)
			// {
			//	 writeuint16_t(*crc16buff++);
			// }
		}
		REG_SCSD_DATAADD<uint16_t> = 0xFF; // end bit
	}
	WaitOnWrite(true); // Note:这个部分与上个部分是不一样的
}

bool SCSD_readData (void* buffer) {
	uint8_t* buff_u8 = (uint8_t*)buffer;
	// FIXME: There might be the need for a timeout, 500000 was attempted but wasn't enough
	WaitOnRead(true);
	const uint32_t maskHi = 0xFFFF0000;
	#define LOAD_U32_ALIGNED_2WORDS \
			"ldmia  %2, {r0-r7} \n"   /*从REG_SCSD_DATAREAD_32_ADDR读取8个32位值到r0-r7*/  \
			"and	r3, r3, %3 \n"	 /*r3 &= maskHi*/									 \
			"and	r7, r7, %3 \n"	 \
			"orr	r3, r3, r1, lsr #16 \n"	 /*r3 |= (r1 >> 16)*/						\
			"orr	r7, r7, r5, lsr #16 \n"	 \
			"stmia  %0!, {r3,r7} \n"  /*将r3和r7的值存储到buff_u32指向的位置，并将buff_u32增加8字节*/

	#define LOAD_U32_ALIGNED_1WORDS \
			"ldmia  %2, {r0-r3} \n"   /*从REG_SCSD_DATAREAD_32_ADDR读取8个32位值到r0-r7*/  \
			"and	r3, r3, %3 \n"	 /*r3 &= maskHi*/									 \
			"orr	r3, r3, r1, lsr #16 \n"	 /*r3 |= (r1 >> 16)*/						\
			"str	r3, [%0], #4 \n"  /*将r3和r7的值存储到buff_u32指向的位置，并将buff_u32增加8字节*/

	#define LOAD_U32_ALIGNED_U16 \
			"ldmia  %2, {r0-r1} \n"   /*从REG_SCSD_DATAREAD_32_ADDR读取8个32位值到r0-r7*/  \
			"lsr r1, r1, #16\n"	 \
			"strh r1, [%0], #2\n"	   //uint16_t


	if (((uint32_t)buff_u8 & 0x03) == 0){//uint32_t aligned
		asm volatile(
		"1: \n"
			LOAD_U32_ALIGNED_2WORDS
			LOAD_U32_ALIGNED_2WORDS
			"cmp	%0, %1 \n"
			"blt	1b \n"			  // if buffer<bufferEnd continue;
			:
			: "r" (buffer), "r" ((uint32_t)buffer+512), "r" (&REG_SCSD_DATAREAD<uint32_t>), "r" (maskHi)
			: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory","cc"
		);
		// i=256;
		// uint16_t* buff = (uint16_t*)buffer;
		// while(i--) {

		// 	dummy_read(REG_SCSD_DATAREAD<uint32_t>);
		// 	*buff++ = REG_SCSD_DATAREAD<uint32_t> >> 16; 
		// }
	}else if ((((uint32_t)buff_u8 & 0x01) == 0)) {//uint16_t aligned
		asm volatile(
			LOAD_U32_ALIGNED_U16
			LOAD_U32_ALIGNED_1WORDS
			LOAD_U32_ALIGNED_2WORDS
		"2: \n"
			LOAD_U32_ALIGNED_2WORDS
			LOAD_U32_ALIGNED_2WORDS
			"cmp	%0, %1 \n"
			"blt	2b \n"			  // if buffer<bufferEnd continue;
			LOAD_U32_ALIGNED_U16
			:
			: "r" (buffer), "r" ((uint32_t)buffer+510/*512-2*/), "r" (&REG_SCSD_DATAREAD<uint32_t>), "r" (maskHi)
			: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory","cc"
		);
	} else 
	{
		uint32_t temp;
		int i=256;
		while(i--) {
			dummy_read(REG_SCSD_DATAREAD<uint32_t>);
			temp = REG_SCSD_DATAREAD<uint32_t> >> 16; 
			*buff_u8++ = (uint8_t)temp;
			*buff_u8++ = (uint8_t)(temp >> 8);
		}
	}

	asm volatile(
			"ldmia  %0, {r0-r7} \n"   // drop the crc16
			"ldrh   r1, [%0] \n"   // end by reading as u16
			:
			: "r" (&REG_SCSD_DATAREAD<uint32_t>)
			: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
		);
	// for (i = 0; i < 8; i++) {
	// 	*REG_SCSD_DATAREAD_32_ADDR;
	// }
	// *REG_SCSD_DATAREAD_ADDR;

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
