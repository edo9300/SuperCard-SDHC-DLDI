#include <utility>
#include "sc_commands.h"

void sc_change_mode(uint16_t mode) {
	auto* const SC_MODE_REG = (volatile uint16_t*)0x09FFFFFE;
	const uint16_t SC_MODE_MAGIC			= 0xA55A;
	*SC_MODE_REG = SC_MODE_MAGIC;
	*SC_MODE_REG = SC_MODE_MAGIC;
	*SC_MODE_REG = mode;
	*SC_MODE_REG = mode;
}
