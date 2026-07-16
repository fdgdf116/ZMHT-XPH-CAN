#ifndef _XUTILS_H_
#define _XUTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xil_printf.h"

#define CAN_DBG_ERROR          0x00000001
#define CAN_DBG_WARNING        0x00000002
#define CAN_DBG_DEBUG          0x00000004
#define CAN_DBG_ALL            0xffffffff
#define CAN_DBG_OFF            0x00000000

#define CAN_DEBUG_MASK      CAN_DBG_ALL

#define CAN_DBG(mask, fmt, ...) \
	do { \
		if ((mask) & CAN_DEBUG_MASK) { \
			xil_printf(fmt, ##__VA_ARGS__); \
		} \
	} while (0)


uint32_t calculate_xor_checksum(void* data, uint32_t total_bytes);
uint32_t swap_endian_32bit(uint32_t value);
uint32_t reverse_bytes(uint32_t value);
#endif

