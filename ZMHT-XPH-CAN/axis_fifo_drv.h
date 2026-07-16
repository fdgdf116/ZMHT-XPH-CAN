#ifndef _AXIS_FIFO_DRV_H_
#define _AXIS_FIFO_DRV_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xil_types.h"

#define AXIS_FIFO_ISR   (0x0)
#define AXIS_FIFO_IER   (0x4)
#define AXIS_FIFO_TDFR  (0x8)
#define AXIS_FIFO_TDFV  (0xc)

#define AXIS_FIFO_TDFD  (0x10)
#define AXIS_FIFO_TLR   (0x14)
#define AXIS_FIFO_RDFR  (0x18)
#define AXIS_FIFO_RDFO  (0x1c)
#define AXIS_FIFO_RDFD  (0x20)
#define AXIS_FIFO_RLR   (0x24)
#define AXIS_FIFO_SRR   (0x28)

void axis_fifo_reset(UINTPTR BaseAddress);
uint32_t axis_fifo_read_data_len(UINTPTR BaseAddress);
int axis_fifo_read_data(UINTPTR BaseAddress, unsigned char *buffer, int length);
int axis_fifo_write_data(UINTPTR BaseAddress, unsigned char *buffer, int length);

#endif

