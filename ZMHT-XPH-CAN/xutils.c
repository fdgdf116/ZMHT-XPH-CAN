#include "xutils.h"

uint32_t calculate_xor_checksum(void* data, uint32_t total_bytes)
{
#if 0
	uint32_t checksum = 0;
	uint8_t* sum_data;

	sum_data = (uint8_t*)data;

	if (!sum_data || total_bytes % 4 || total_bytes < 4)
		return 0xffffffff;

	for (uint32_t i = 0; i < total_bytes; i += 4)
		checksum ^= *(uint32_t*)(sum_data + i);

	return checksum;
#else
	uint32_t checksum;
	uint8_t* sum_data;
	sum_data = (uint8_t*)data;

	checksum = 0;
	if (!sum_data || total_bytes % 4 || total_bytes < 4)
		return 0xffffffff;

	for (uint32_t i = 0; i < total_bytes; i++)
		checksum += sum_data[i];

	return checksum;
#endif
}

uint32_t swap_endian_32bit(uint32_t value)
{
	return ((value & 0xFF000000) >> 24) |
		((value & 0x00FF0000) >> 8) |
		((value & 0x0000FF00) << 8) |
		((value & 0x000000FF) << 24);
}

uint32_t reverse_bytes(uint32_t value) {
	return ((value & 0x000000FF) << 24) |  // 将最低字节移到最高位
		((value & 0x0000FF00) << 8) |  // 将次低字节移到次高位
		((value & 0x00FF0000) >> 8) |  // 将次高字节移到次低位
		((value & 0xFF000000) >> 24);   // 将最高字节移到最低位
}

