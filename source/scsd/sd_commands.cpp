#include "sd_commands.h"

static inline void drop_response(int bytesToDrop)
{
	bytesToDrop++; // +  8 clocks

	// Wait for the card to be non-busy
	while (((REG_SCSD_CMD<uint16_t> & 0x01) != 0));

	// 实际上，当跳出这个循环的时候，已经读了一个bit了，后续会多读一个bit，但是这是抛弃的rsp,因此多读一个bit也就是多一个时钟周期罢了
	
   asm volatile(
		"2: \n\t"
			"ldmia %0, {r0-r3} \n\t"   
			"subs %1, %1, #1 \n\t"   // bytesToDrop--
			"bne 2b \n\t"			// 如果byteNum不为0，则继续循环
		: // 没有输出
		: "r" ((uint32_t)&REG_SCSD_CMD<uint16_t>), "r" (bytesToDrop) // 输入
		: "r0", "r1", "r2", "r3", "cc" // 破坏列表
	);
	// while (bytesToDrop--)
	// {
	//	 *cmd_addr_u32;
	//	 *cmd_addr_u32;
	//	 *cmd_addr_u32;
	//	 *cmd_addr_u32;
	// }
}

inline bool read_response(uint8_t* dest, uint32_t length) {
	constexpr uint32_t BUSY_WAIT_TIMEOUT = 500000;
	
	for(auto i = BUSY_WAIT_TIMEOUT; (REG_SCSD_CMD<uint16_t> & 0x01) != 0; --i) {
		if(i == 0)
			return false;
	}
	
	int numBits = length * 8;
	// The first bit is always 0
	uint32_t partial_result = ((REG_SCSD_CMD<uint16_t>) & 0x01) << 16 ;
	numBits-=2;
	// Read the remaining bits in the response.
	// It's always most significant bit first
	const uint32_t mask_2bit = 0x10001;
	while (numBits) {
		numBits-=2;
		partial_result = (partial_result << 2) | (REG_SCSD_CMD<uint32_t> & mask_2bit);
		if ((numBits & 0x7) == 0) {
			//_1_3_5_7 _0_2_4_6 
			*dest++ = ((partial_result >> 16) | (partial_result<<1));
			partial_result = 0;
		}
	}
	for (int i = 0; i < 4; ++i) {//8clock
		dummy_read(REG_SCSD_CMD<uint32_t>);
	}
	return true;
}

const static uint8_t crc7_lut[]{
	 0x0, 0x12, 0x24, 0x36, 0x48, 0x5A, 0x6C, 0x7E, 0x90, 0x82, 0xB4, 0xA6, 0xD8, 0xCA, 0xFC, 0xEE,
	 0x32, 0x20, 0x16, 0x4, 0x7A, 0x68, 0x5E, 0x4C, 0xA2, 0xB0, 0x86, 0x94, 0xEA, 0xF8, 0xCE, 0xDC,
	 0x64, 0x76, 0x40, 0x52, 0x2C, 0x3E, 0x8, 0x1A, 0xF4, 0xE6, 0xD0, 0xC2, 0xBC, 0xAE, 0x98, 0x8A,
	 0x56, 0x44, 0x72, 0x60, 0x1E, 0xC, 0x3A, 0x28, 0xC6, 0xD4, 0xE2, 0xF0, 0x8E, 0x9C, 0xAA, 0xB8,
	 0xC8, 0xDA, 0xEC, 0xFE, 0x80, 0x92, 0xA4, 0xB6, 0x58, 0x4A, 0x7C, 0x6E, 0x10, 0x2, 0x34, 0x26,
	 0xFA, 0xE8, 0xDE, 0xCC, 0xB2, 0xA0, 0x96, 0x84, 0x6A, 0x78, 0x4E, 0x5C, 0x22, 0x30, 0x6, 0x14,
	 0xAC, 0xBE, 0x88, 0x9A, 0xE4, 0xF6, 0xC0, 0xD2, 0x3C, 0x2E, 0x18, 0xA, 0x74, 0x66, 0x50, 0x42,
	 0x9E, 0x8C, 0xBA, 0xA8, 0xD6, 0xC4, 0xF2, 0xE0, 0xE, 0x1C, 0x2A, 0x38, 0x46, 0x54, 0x62, 0x70,
	 0x82, 0x90, 0xA6, 0xB4, 0xCA, 0xD8, 0xEE, 0xFC, 0x12, 0x0, 0x36, 0x24, 0x5A, 0x48, 0x7E, 0x6C,
	 0xB0, 0xA2, 0x94, 0x86, 0xF8, 0xEA, 0xDC, 0xCE, 0x20, 0x32, 0x4, 0x16, 0x68, 0x7A, 0x4C, 0x5E,
	 0xE6, 0xF4, 0xC2, 0xD0, 0xAE, 0xBC, 0x8A, 0x98, 0x76, 0x64, 0x52, 0x40, 0x3E, 0x2C, 0x1A, 0x8,
	 0xD4, 0xC6, 0xF0, 0xE2, 0x9C, 0x8E, 0xB8, 0xAA, 0x44, 0x56, 0x60, 0x72, 0xC, 0x1E, 0x28, 0x3A,
	 0x4A, 0x58, 0x6E, 0x7C, 0x2, 0x10, 0x26, 0x34, 0xDA, 0xC8, 0xFE, 0xEC, 0x92, 0x80, 0xB6, 0xA4,
	 0x78, 0x6A, 0x5C, 0x4E, 0x30, 0x22, 0x14, 0x6, 0xE8, 0xFA, 0xCC, 0xDE, 0xA0, 0xB2, 0x84, 0x96,
	 0x2E, 0x3C, 0xA, 0x18, 0x66, 0x74, 0x42, 0x50, 0xBE, 0xAC, 0x9A, 0x88, 0xF6, 0xE4, 0xD2, 0xC0,
	 0x1C, 0xE, 0x38, 0x2A, 0x54, 0x46, 0x70, 0x62, 0x8C, 0x9E, 0xA8, 0xBA, 0xC4, 0xD6, 0xE0, 0xF2
};

inline uint8_t CRC7_one(uint8_t crcIn, uint8_t data)
{
	crcIn ^= data;
	return crc7_lut[crcIn];
}

inline uint8_t SD_CRC7(uint8_t *pBuf, int len) {
	uint8_t crc = 0;
	while (len--)
		crc = CRC7_one(crc, *pBuf++);
	crc |= 1;
	return crc;
}

void SDCommand(uint8_t command, uint32_t argument)
{
	uint8_t databuff[6];
	uint8_t *tempDataPtr = databuff;

	*tempDataPtr++ = command | 0x40;
	*tempDataPtr++ = argument >> 24;
	*tempDataPtr++ = argument >> 16;
	*tempDataPtr++ = argument >> 8;
	*tempDataPtr++ = argument;
	*tempDataPtr = SD_CRC7(databuff, 5);

	while (((REG_SCSD_CMD<uint16_t> & 0x01) == 0)){}

	dummy_read(REG_SCSD_CMD<uint16_t>);

	auto& send_command_addr = REG_SCSD_CMD<uint32_t>; // 假设sd_comadd也是数据写入地址

	#define SEND_ONE_COMMAND_BYTE \
		"ldrb r0, [%0], #1 \n"		 /* uint32_t data = *tempDataPtr++; */ \
		"orr  r0, r0, r0, lsl #17 \n"  /* data = data | (data << 17); */ \
		"mov  r1, r0, lsl #2 \n"	   \
		"mov  r2, r0, lsl #4 \n"	   \
		"mov  r3, r0, lsl #6 \n"	   \
		"stmia %1, {r0-r3}\n"
	asm volatile(//发送6次
	"1:\n"
		SEND_ONE_COMMAND_BYTE
		SEND_ONE_COMMAND_BYTE
		"cmp %0, %2\n"
		"blt 1b"
		: // 没有输出
		: "r"((uint32_t)databuff),"r"((uint32_t)&send_command_addr),"r"(((uint32_t)databuff) + 6)
		: "r0", "r1", "r2", "r3"// 破坏列表
	);
	// int length = 6;
	// while (length--)
	// {
	//	 uint32_t data = *tempDataPtr++;
	//	 data = data | (data << 17);
	//	 *send_command_addr = data;
	//	 *send_command_addr = data << 2;
	//	 *send_command_addr = data << 4;
	//	 *send_command_addr = data << 6;
	//	 // sd_dataadd[0] ~ [3]至少目前证明都是镜像的，可以随便用，可以用stmia来加速
	//	 // 本质上是将U16的写合并成uint32_t的写
	// }
}

void SDCommandAndDropResponse(uint8_t command, uint32_t argument, uint32_t bytesToDrop) {
	SDCommand(command, argument);
	drop_response(bytesToDrop);
}

bool SDCommandAndReadResponse(uint8_t command, uint32_t argument, uint8_t* responseBuffer, uint32_t bytesToRead) {
	SDCommand(command, argument);
	return read_response(responseBuffer, bytesToRead);
}