#include "axis_fifo_drv.h"
#include "xil_io.h"
#include "xstatus.h"
#include "xutils.h"

#define axis_fifo_rd32  Xil_In32
#define axis_fifo_wr32 Xil_Out32

void axis_fifo_reset(UINTPTR BaseAddress)
{
	axis_fifo_wr32((BaseAddress + AXIS_FIFO_TDFR), 0xa5);
	axis_fifo_wr32((BaseAddress + AXIS_FIFO_RDFR), 0xa5);
	axis_fifo_wr32((BaseAddress + AXIS_FIFO_SRR), 0xa5);
}

uint32_t axis_fifo_read_data_len(UINTPTR BaseAddress)
{
	return axis_fifo_rd32(BaseAddress + AXIS_FIFO_RDFO) * 4;
}

int axis_fifo_read_data(UINTPTR BaseAddress, unsigned char *buffer, int length)
{
	if (length % 4)
		return -1;
	int i = 0;
	for(i = 0; i < length; i += 4)
		*(uint32_t*)(&buffer[i]) = axis_fifo_rd32(BaseAddress + AXIS_FIFO_RDFD);
    return i;
}

static int axis_fifo_write(UINTPTR BaseAddress, unsigned char *buffer, int length)
{
	int i = 0;

	for(i = 0; i < length; i += 4) {
		if(axis_fifo_rd32(BaseAddress + AXIS_FIFO_TDFV)){
			axis_fifo_wr32(BaseAddress + AXIS_FIFO_TDFD, *((uint32_t*)&buffer[i]));
		} else {
			break;
		}
	}

	axis_fifo_wr32(BaseAddress + AXIS_FIFO_TLR, i);
    return i;

}

int axis_fifo_write_data(UINTPTR BaseAddress, unsigned char *buffer, int length)
{
#if 0
	int done, count, ret;
	if ((length % 4) || (buffer == NULL) || (!length))
		return -1;


	CAN_DBG(CAN_DBG_ERROR,"### axis fifo write length:%d ##### \r\n", length);

	count = 0;
	while(1){
		done = length - count;
		if(done == 0){
			break;
		}

		ret = axis_fifo_write(BaseAddress, buffer + count, done);
		if(ret != done){
			CAN_DBG(CAN_DBG_ERROR,"### axis fifo write data (%d != %d) ##### \r\n", ret, done);
		}

		count += ret;
	}
#else

	int ret;
	if ((length % 4) || (buffer == NULL) || (!length))
		return -1;

	ret = axis_fifo_write(BaseAddress, buffer, length);
	if(ret != length){
		CAN_DBG(CAN_DBG_ERROR,"### axis fifo write data (%d != %d) ##### \r\n", ret, length);
	}
#endif
	return 0;
}


