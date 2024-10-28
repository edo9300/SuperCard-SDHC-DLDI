#include "new_scsdio.h"
#include "sc_commands.h"
#include "sd_commands.h"
#include "memcnt_guard.h"
#include "sd.h"

#define sd_dataadd 0x9000000  

bool SCSD_readData(void* buffer);

uint64_t sdio_crc16_4bit_checksum(uint32_t *data, uint32_t num_words);

static constexpr auto SCSD_STS_BUSY = 0x0100;

inline void WaitBusyOnRead(bool timeout) {
	#define BUSY_WAIT_TIMEOUT 500000
	vu16 *wait_busy = (vu16 *)sd_dataadd;
	while (((*wait_busy) & SCSD_STS_BUSY) == 0);
	if(extraRead)
		*wait_busy;
}

inline void WaitBusyOnWrite(bool extraRead = false) {
	auto* wait_busy = (volatile uint16_t*)sd_dataadd;
	while (((*wait_busy) & SCSD_STS_BUSY) == 0);
	if(extraRead)
		*wait_busy;
}

void WriteSector(u8 *buff, u32 sector, u32 writenum)
{
	MemcntGuard guard{true};
	u64 crc16;//并行4个
	sc_change_mode(en_sdram + en_sdcard);
	SDResetCard();
	auto param = isSDHC ? sector : (sector << 9);
	if(writenum == 1){
		SDCommandAndDropResponse(24, param);
		crc16 = sdio_crc16_4bit_checksum((u32 *)(buff),512/sizeof(u32));
		sd_data_write((u16 *)(buff), (u8 *)(&crc16));
	} else {
		SDCommandAndDropResponse(25, param);
		for (auto buffEnd = buff + writenum * 512 ; buff < buffEnd; buff += 512)
		{
			crc16 = sdio_crc16_4bit_checksum((u32 *)(buff),512/sizeof(u32));
			sd_data_write((u16 *)(buff), (u8 *)(&crc16));
			SDSendClock(0x10);
		}
		SDCommandAndDropResponse(12, 0);
	}
	SDSendClock(0x10);
	WaitBusyOnWrite();
	return;
}

bool ReadSector(uint8_t *buff, uint32_t sector, uint32_t readnum)
{
	bool res = true;
	MemcntGuard guard{true};
    sc_change_mode(en_sdram + en_sdcard);
	auto param = isSDHC ? sector : (sector << 9);
	if(readnum == 1){
		SDCommand(0x11, param);
		MemcntGuard guard{isSCLite};
		res = SCSD_readData(buff);
	}else{
		SDCommand(0x12, param); // R0 = 0x12, R1 = 0, R2 as calculated above
		{
			MemcntGuard guard{isSCLite};
			for(auto buffer_end = buff + readnum*(512);buff<buffer_end;buff+=512)
			{
				res = SCSD_readData(buff); // Add R6, left shifted by 9, to R4 before casting
				if(!res)
					break;
			}
		}
		SDCommandAndDropResponse(0xC, 0); // Command to presumably stop reading
	}

	SDSendClock(0x10);	   // Send clock signal
	return res;
}

#define BUSY_WAIT_TIMEOUT 500000
#define SCSD_STS_BUSY 0x100

void sd_data_write(u16 *buff, u8 *crc16buff)
{
	vu16 *const data_write_u16 = (vu16 *)sd_dataadd;
	vu32 *const data_write_u32 = (vu32 *)sd_dataadd;
	WaitBusyOnWrite(true); // Note:两边的等待是不一致的

	*data_write_u16 = 0; // start bit
	{
		MemcntGuard guard{false};

		auto writeU16 = [data_write_u32](uint32_t data)//lambda Function
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
				*data_write_u32 = data;
				*data_write_u32 = (data >> 8);
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

		if((u32)buff & 3){//unaligned
			u8* buff_u8 = (u8*)buff;
			u16 byteHI;
			u16 byteLo;
			for (int i = 0; i < 512; i += 2){
				byteLo = *buff_u8++;
				byteHI = *buff_u8++;
				writeU16((byteHI << 8) | byteLo);
			}
		}
		else if((u32)buff & 2){//u16 aligned
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
				: "r"((u32)buff),"r"((u32)data_write_u32),"r"(((u32)buff) + 510/*512-2*/)
				: "r0", "r1", "r2", "r3", "r4", "cc"// 破坏列表
			);
		}
		else{//u32 aligned
			asm volatile(
				LOAD_ORR_CONSTANT
			"2:\n"
				WRITE_U32
				WRITE_U32
				"cmp %0, %2\n"
				"blt 2b"
				: // 没有输出
				: "r"((u32)buff),"r"((u32)data_write_u32),"r"(((u32)buff) + 512)
				: "r0", "r1", "r2", "r3", "r4", "cc"// 破坏列表
			);
			// u32 *buff_u32 = (u32*)buff;
			// for (int i = 0; i < 512; i += 4){
			//	 writeU32(*buff_u32++);
			// }//or?
			// for (int i = 0; i < 512; i += 2){
			//	 writeU16(*buff++);
			// }
		}

		
		if ((u32)crc16buff & 1)
		{   
			u16 byteHI;
			u16 byteLo;
			for (int i = 0; i < 4; i ++){
				byteLo = *crc16buff++;
				byteHI = *crc16buff++;
				writeU16((byteHI << 8) | byteLo);
			}
		}
		else if ((u32)crc16buff & 2){
			asm volatile(
				WRITE_U16
				WRITE_U16
				WRITE_U16
				WRITE_U16
				: // 没有输出
				: "r"((u32)crc16buff),"r"((u32)data_write_u32)
				: "r0", "r1", "r2", "r3", "cc"// 破坏列表
			);
		}
		else{
			asm volatile(
				LOAD_ORR_CONSTANT
				WRITE_U32
				WRITE_U32
				: // 没有输出
				: "r"((u32)crc16buff),"r"((u32)data_write_u32)
				: "r0", "r1", "r2", "r3", "r4", "cc"// 破坏列表
			);
			// for (int i = 0; i < 4; i++)
			// {
			//	 writeU16(*crc16buff++);
			// }
		}
		*data_write_u16 = 0xFF; // end bit
	}
	WaitBusyOnWrite(); // Note:这个部分与上个部分是不一样的
}


vu16* const REG_SCSD_DATAREAD_ADDR	=	((vu16*)(0x09100000));
vu32* const REG_SCSD_DATAREAD_32_ADDR	=	((vu32*)(0x09100000));
bool SCSD_readData (void* buffer) {
	u8* buff_u8 = (u8*)buffer;
	int i;
	
	i = BUSY_WAIT_TIMEOUT;
	while (((*REG_SCSD_DATAREAD_ADDR) & SCSD_STS_BUSY));
	if (i == 0) {
		return false;
	}

	const u32 maskHi = 0xFFFF0000;
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
			"strh r1, [%0], #2\n"	   //u16


	if (((u32)buff_u8 & 0x03) == 0){//u32 aligned
		asm volatile(
		"1: \n"
			LOAD_U32_ALIGNED_2WORDS
			LOAD_U32_ALIGNED_2WORDS
			"cmp	%0, %1 \n"
			"blt	1b \n"			  // if buffer<bufferEnd continue;
			:
			: "r" (buffer), "r" ((u32)buffer+512), "r" (REG_SCSD_DATAREAD_32_ADDR), "r" (maskHi)
			: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory","cc"
		);
		// i=256;
		// u16* buff = (u16*)buffer;
		// while(i--) {

		// 	*(REG_SCSD_DATAREAD_32_ADDR);
		// 	*buff++ = (*(REG_SCSD_DATAREAD_32_ADDR)) >> 16; 
		// }
	}else if ((((u32)buff_u8 & 0x01) == 0)) {//u16 aligned
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
			: "r" (buffer), "r" ((u32)buffer+510/*512-2*/), "r" (REG_SCSD_DATAREAD_32_ADDR), "r" (maskHi)
			: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory","cc"
		);
	} else 
	{
		u32 temp;
		i=256;
		while(i--) {
			*REG_SCSD_DATAREAD_32_ADDR;
			temp = (*REG_SCSD_DATAREAD_32_ADDR) >> 16;
			*buff_u8++ = (u8)temp;
			*buff_u8++ = (u8)(temp >> 8);
		}
	}

	asm volatile(
			"ldmia  %0, {r0-r7} \n"   // drop the crc16
			"ldrh   r1, [%0] \n"   // end
			:
			: "r" ((u32)REG_SCSD_DATAREAD_32_ADDR)
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

uint64_t inline calSingleCRC16(uint64_t crc,uint32_t data_in){
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

uint32_t loadBigEndU32_u8(u8* &dataBuf){
	u32 data;
	data = (*dataBuf++) << 24;
	data |= (*dataBuf++) << 16;
	data |= (*dataBuf++) << 8;
	data |= (*dataBuf++);
	return data;
}
uint64_t sdio_crc16_4bit_checksum(uint32_t *dataBuf, uint32_t num_words)
{
	uint64_t crc = 0;
	if((uintptr_t)dataBuf & 3){//u8 align
		uint8_t *data = (u8*)dataBuf;
		uint8_t *end = (u8*)(dataBuf + num_words);
		while (data < end)
		{
			uint32_t data_in = loadBigEndU32_u8(data);
			crc = calSingleCRC16(crc,data_in);
		}
	}else{
		uint32_t *data = dataBuf;
		uint32_t *end = dataBuf + num_words;
		while (data < end)
		{
			uint32_t data_in = __builtin_bswap32(*data++);
			crc = calSingleCRC16(crc,data_in);
		}
	}

	return __builtin_bswap64(crc);
}
