#include "axi_can_drv.h"
#include "can_sys.h"
#include "can_common.h"
#include "axis_fifo_drv.h"
#include "xintc_drv.h"
#include "tmrctr_drv.h"
#include "xcan.h"
#include "sleep.h"
#include "xil_cache.h"
#include "xutils.h"

volatile u32 can_send_index = 0;
static can_drv_t can_drv[AXI_CAN_DRV_MAX];
XGpio Gpio[2];
uint32_t total_frame_count = 0;
uint32_t channel_frame_count[6] = { 0 };

static uint8_t intc_init_status;
static uint8_t can_channel_ready[AXI_CAN_DRV_MAX];
static uint8_t buffer_config_receive[CAN_CONFIG_FRAME_SIZE_MAX]
	__attribute__((aligned(4)));
ddr_manager_t ddr_channels[DDR_CHANNEL_COUNT];

typedef struct {
	can_request_frame_t frames[PPS_CAN_FRAME_COUNT_MAX];
	uint32_t frame_count;
	uint8_t valid;
} pps_channel_buffer_t;

static pps_channel_buffer_t pps_channel_buffers[AXI_CAN_DRV_MAX];
static volatile uint32_t pps_pending_count;
static uint32_t pps_processed_count;
static uint32_t pps_enable_mask;

#define PPS_GPIO_CHANNEL GPIO_CHANNEL_1
#define PPS_CAN_TX_WAIT_MAX 1000000U
#define PPS_GPIO_COUNT 4U

typedef struct {
	XGpio gpio;
	uint16_t device_id;
	uint8_t intr_id;
	uint8_t pps_id;
} pps_gpio_t;

static pps_gpio_t pps_gpios[PPS_GPIO_COUNT] = {
	{
		.device_id = XPAR_MICRO_AXI_GPIO_2_DEVICE_ID,
		.intr_id = XPAR_MICRO_AXI_INTC_0_MICRO_AXI_GPIO_2_IP2INTC_IRPT_INTR,
		.pps_id = 2,
	},
	{
		.device_id = XPAR_MICRO_AXI_GPIO_3_DEVICE_ID,
		.intr_id = XPAR_MICRO_AXI_INTC_0_MICRO_AXI_GPIO_3_IP2INTC_IRPT_INTR,
		.pps_id = 3,
	},
	{
		.device_id = XPAR_MICRO_AXI_GPIO_4_DEVICE_ID,
		.intr_id = XPAR_MICRO_AXI_INTC_0_MICRO_AXI_GPIO_4_IP2INTC_IRPT_INTR,
		.pps_id = 4,
	},
	{
		.device_id = XPAR_MICRO_AXI_GPIO_5_DEVICE_ID,
		.intr_id = XPAR_MICRO_AXI_INTC_0_MICRO_AXI_GPIO_5_IP2INTC_IRPT_INTR,
		.pps_id = 5,
	},
};

static void parse_and_init_can(can_config_frame_t* frame);

uint32_t Axi_fifo_init_addresses[] =
{
	0X44A00000,
	0X44A10000,
	0X44A20000,
	0X44A30000,
	0X44A40000,
	0X44A50000
};

static uint8_t can_drv_id[AXI_CAN_DRV_MAX] = {
	axi_can_drv_id0,
	axi_can_drv_id1,
	axi_can_drv_id2,
	axi_can_drv_id3,
	axi_can_drv_id4,
	axi_can_drv_id5
};
static uint8_t can_intc_drv_id[AXI_CAN_DRV_MAX] = {
	axi_can_drv_intc_id0,
	axi_can_drv_intc_id1,
	axi_can_drv_intc_id2,
	axi_can_drv_intc_id3,
	axi_can_drv_intc_id4,
	axi_can_drv_intc_id5
};
static uint32_t can_status_base_address[AXI_CAN_DRV_MAX] = {
	can_status_frame_base_address0,
	can_status_frame_base_address1,
	can_status_frame_base_address2,
	can_status_frame_base_address3,
	can_status_frame_base_address4,
	can_status_frame_base_address5
};
static const uint32_t pps_can_data_base_address[AXI_CAN_DRV_MAX] = {
	0x20800000U,
	0x22800000U,
	0x24800000U,
	0x26800000U,
	0x28800000U,
	0x2A800000U
};
static uint32_t can_fifo_base_address[AXI_CAN_DRV_MAX] = {
	can_axis_fifo_base_address0,
	can_axis_fifo_base_address1,
	can_axis_fifo_base_address2,
	can_axis_fifo_base_address3,
	can_axis_fifo_base_address4,
	can_axis_fifo_base_address5
};
void ddr_manager_init() {
	uint32_t base_addresses[DDR_CHANNEL_COUNT] = {
		0x21000000,  // 通道0基地址
		0x23000000,  // 通道1基地址
		0x25000000,  // 通道2基地址
		0x27000000,  // 通道3基地址
		0x29000000,  // 通道4基地址
		0x2b000000   // 通道5基地址
	};

	uint32_t channel_sizes[DDR_CHANNEL_COUNT] = {
		0x01000000,  // 通道0大小：16MB
		0x01000000,  // 通道1大小：16MB
		0x01000000,  // 通道2大小：16MB
		0x01000000,  // 通道3大小：16MB
		0x01000000,  // 通道4大小：16MB
		0x01000000   // 通道5大小：16MB
	};
	uint32_t fifo_baseaddr[DDR_CHANNEL_COUNT] = {
		0x00500000,  // 通道0的FIFO基地址
		0x00510000,  // 通道1的FIFO基地址
		0x00520000,  // 通道2的FIFO基地址
		0x00530000,  // 通道3的FIFO基地址
		0x00540000,  // 通道4的FIFO基地址
		0x00550000   // 通道5的FIFO基地址
	};


	// 初始化每个通道
	for (uint8_t i = 0; i < DDR_CHANNEL_COUNT; i++) {
		ddr_channels[i].chid = i;
		ddr_channels[i].base_address = base_addresses[i];
		ddr_channels[i].current_address = base_addresses[i];
		ddr_channels[i].total_size = channel_sizes[i];
		ddr_channels[i].used_size = 0;
		ddr_channels[i].fifo_baseaddr = fifo_baseaddr[i];
	}
}

static void axi_can_drv_info_init(can_drv_t* drv, int drv_id, int chn_id, int drv_intc_id, uintptr_t address, uintptr_t fifo_base_address)
{
	memset(drv, 0x0, sizeof(can_drv_t));
	drv->can_id = drv_id;
	drv->can_chn_id = chn_id;
	drv->intc_id = drv_intc_id;
	drv->baud = 500000;
	drv->status_phy_addr = address;
	drv->response_phy_addr = address + CAN_RESPONSE_FRAME_OFFSET_START;
	drv->axis_fifo_base_address = fifo_base_address;
	drv->rxframe_phy_addr = can_rxframe_base_address + (chn_id * CAN_RXFRAME_ADDR_OFFSET_MAX);
	CAN_DBG(CAN_DBG_DEBUG, "can drv id:%d response_phy_addr:0x%x \r\n", drv_id, drv->response_phy_addr);
}

static int axi_gpio_drv_init(void)
{

	int Status;

	// 初始化GPIO 0 (控制时间戳开关)
	Status = XGpio_Initialize(&Gpio[0], XPAR_MICRO_AXI_GPIO_0_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		xil_printf("GPIO 0 Initialization Failed\r\n");
		return XST_FAILURE;
	}

	// 初始化GPIO 1 (读取时间戳数据)
	Status = XGpio_Initialize(&Gpio[1], XPAR_MICRO_AXI_GPIO_1_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		xil_printf("GPIO 1 Initialization Failed\r\n");
		return XST_FAILURE;
	}

	// GPIO0 通道1设置为输出 (0表示输出)
	XGpio_SetDataDirection(&Gpio[0], GPIO_CHANNEL_1, 0x00);
	// GPIO1 通道1和通道2设置为输入 (1表示输入)
	XGpio_SetDataDirection(&Gpio[1], GPIO_CHANNEL_1, GPIO1_OUTPUT_MASK);
	XGpio_SetDataDirection(&Gpio[1], GPIO_CHANNEL_2, GPIO1_OUTPUT_MASK);
	xil_printf("Successfully GPIO init\r\n");
	XGpio_DiscreteWrite(&Gpio[0], GPIO_CHANNEL_1, 0x01);
	return XST_SUCCESS;
}

static void pps_gpio_interrupt_handler(void* callback_ref)
{
	pps_gpio_t* pps_gpio = (pps_gpio_t*)callback_ref;
	u32 interrupt_status = XGpio_InterruptGetStatus(&pps_gpio->gpio);

	if ((interrupt_status & XGPIO_IR_CH1_MASK) != 0U) {
		pps_pending_count++;
		xil_printf("PPS GPIO%d interrupt received\r\n", pps_gpio->pps_id);
	}

	XGpio_InterruptClear(&pps_gpio->gpio, interrupt_status);
}

static int pps_gpio_drv_init(void)
{
	int status;
	uint32_t i;

	for (i = 0; i < PPS_GPIO_COUNT; i++) {
		pps_gpio_t* pps_gpio = &pps_gpios[i];

		status = XGpio_Initialize(&pps_gpio->gpio, pps_gpio->device_id);
		if (status != XST_SUCCESS) {
			CAN_DBG(CAN_DBG_ERROR, "PPS GPIO%d initialization failed\r\n",
				pps_gpio->pps_id);
			return XST_FAILURE;
		}

		XGpio_SetDataDirection(&pps_gpio->gpio, PPS_GPIO_CHANNEL, 0xFFFFFFFFU);
		XGpio_InterruptDisable(&pps_gpio->gpio, XGPIO_IR_CH1_MASK);
		XGpio_InterruptClear(&pps_gpio->gpio, XGPIO_IR_CH1_MASK);

		status = xintc_drv_connect(pps_gpio->intr_id,
			(XInterruptHandler)pps_gpio_interrupt_handler, pps_gpio);
		if (status != XST_SUCCESS) {
			CAN_DBG(CAN_DBG_ERROR, "PPS GPIO%d interrupt connect failed\r\n",
				pps_gpio->pps_id);
			return XST_FAILURE;
		}

		XGpio_InterruptEnable(&pps_gpio->gpio, XGPIO_IR_CH1_MASK);
		XGpio_InterruptGlobalEnable(&pps_gpio->gpio);
		CAN_DBG(CAN_DBG_DEBUG, "PPS GPIO%d interrupt initialized\r\n",
			pps_gpio->pps_id);
	}

	return XST_SUCCESS;
}


static void axi_can_drv_init(void)
{
	int i;
	int Status;
	for (i = 0; i < AXI_CAN_DRV_MAX; i++) {
		axi_can_drv_info_init(&can_drv[i], can_drv_id[i], i, can_intc_drv_id[i], can_status_base_address[i], can_fifo_base_address[i]);
		if (i > 3) {
			axis_fifo_reset(can_drv[i].axis_fifo_base_address);
			continue;
		}
		Status = axi_can_init(&can_drv[i]);
		if (Status != 0) {
			//CAN_DBG(CAN_DBG_ERROR,"can drv id:%d init error \r\n", i);
			can_drv[i].init_flag = 1;
		}
		else {
			can_channel_ready[i] = 1U;
			if (can_drv[i].rb == NULL) {
				CAN_DBG(CAN_DBG_ERROR, "chn:%d rb init error  \r\n", i);
			}
			if (!intc_init_status) {
				Status = xintc_drv_connect(can_drv[i].intc_id, (Xil_ExceptionHandler)XCan_IntrHandler, &can_drv[i]);
				if (Status != 0) {
					//can_drv[i].init_flag = 1;
					CAN_DBG(CAN_DBG_ERROR, "can drv id:%d intc init error \r\n", i);
				}
			}
		}
	}
}

static uint32_t pps_protocol_xor_checksum(const void* data, uint32_t total_bytes)
{
	const uint8_t* bytes = (const uint8_t*)data;
	uint32_t checksum = 0U;
	uint32_t offset;

	if ((data == NULL) || (total_bytes < sizeof(uint32_t)) ||
		((total_bytes % sizeof(uint32_t)) != 0U)) {
		return 0xFFFFFFFFU;
	}

	for (offset = 0; offset < total_bytes; offset += sizeof(uint32_t)) {
		uint32_t word;
		memcpy(&word, bytes + offset, sizeof(word));
		checksum ^= word;
	}

	return checksum;
}

static int pps_can_frames_valid(const can_request_frame_t* frames,
	uint32_t frame_count, uint8_t channel_id)
{
	uint32_t i;

	for (i = 0; i < frame_count; i++) {
		const can_request_frame_t* frame = &frames[i];

		if ((frame->head != axi_can_data_frame_head) ||
			(frame->direction != DIR_HOST_TO_DEVICE) ||
			(frame->type != CMD_HOST_PPS_DATA) ||
			(frame->length != sizeof(can_request_frame_t)) ||
			(frame->ch_id != channel_id)) {
			CAN_DBG(CAN_DBG_ERROR,
				"PPS CAN frame invalid: ch=%d index=%d\r\n",
				channel_id, i);
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}

static void parse_and_update_pps_command(const can_pps_command_t* command)
{
	uint32_t protocol_checksum;
	uint32_t legacy_checksum;
	uint32_t update_mask;
	uint32_t frame_count;
	uint8_t channel_id;

	protocol_checksum = pps_protocol_xor_checksum(command,
		sizeof(can_pps_command_t) - sizeof(command->xor_checksum));
	legacy_checksum = calculate_xor_checksum((void*)command,
		sizeof(can_pps_command_t) - sizeof(command->xor_checksum));
	if ((command->head != axi_can_data_frame_head) ||
		(command->direction != DIR_HOST_TO_DEVICE) ||
		(command->type != CMD_HOST_PPS_DATA) ||
		(command->length != sizeof(can_pps_command_t)) ||
		((command->xor_checksum != protocol_checksum) &&
		 (command->xor_checksum != legacy_checksum))) {
		CAN_DBG(CAN_DBG_ERROR, "PPS command validation failed\r\n");
		return;
	}

	update_mask = command->update_mask & command->ch_mask & PPS_CHANNEL_MASK;

	if (command->can_count == 0U) {
		for (channel_id = 0; channel_id < AXI_CAN_DRV_MAX; channel_id++) {
			if ((update_mask & (1U << channel_id)) != 0U) {
				pps_channel_buffers[channel_id].frame_count = 0U;
				pps_channel_buffers[channel_id].valid = 0U;
			}
		}
		pps_enable_mask = command->enable_mask & PPS_CHANNEL_MASK;
		return;
	}

	if ((command->can_count % sizeof(can_request_frame_t)) != 0U) {
		CAN_DBG(CAN_DBG_ERROR, "PPS CAN count is not 48-byte aligned: %d\r\n",
			command->can_count);
		return;
	}

	frame_count = command->can_count / sizeof(can_request_frame_t);
	if (frame_count > PPS_CAN_FRAME_COUNT_MAX) {
		CAN_DBG(CAN_DBG_ERROR, "PPS CAN frame count exceeds limit: %d\r\n",
			frame_count);
		return;
	}

	/* Validate every selected channel before changing any cached data. */
	for (channel_id = 0; channel_id < AXI_CAN_DRV_MAX; channel_id++) {
		const can_request_frame_t* ddr_frames;

		if ((update_mask & (1U << channel_id)) == 0U) {
			continue;
		}

		ddr_frames = (const can_request_frame_t*)
			pps_can_data_base_address[channel_id];
		Xil_DCacheInvalidateRange((INTPTR)ddr_frames, command->can_count);
		if (pps_can_frames_valid(ddr_frames, frame_count, channel_id) != XST_SUCCESS) {
			return;
		}
	}

	for (channel_id = 0; channel_id < AXI_CAN_DRV_MAX; channel_id++) {
		const can_request_frame_t* ddr_frames;

		if ((update_mask & (1U << channel_id)) == 0U) {
			continue;
		}

		ddr_frames = (const can_request_frame_t*)
			pps_can_data_base_address[channel_id];
		memcpy(pps_channel_buffers[channel_id].frames, ddr_frames,
			command->can_count);
		pps_channel_buffers[channel_id].frame_count = frame_count;
		pps_channel_buffers[channel_id].valid = 1U;
		CAN_DBG(CAN_DBG_DEBUG, "PPS buffer updated: ch=%d frames=%d\r\n",
			channel_id, frame_count);
	}

	pps_enable_mask = command->enable_mask & PPS_CHANNEL_MASK;
}

static void parse_host_command(void* frame_data, uint32_t data_len)
{
	can_config_frame_t* frame = (can_config_frame_t*)frame_data;

	if (data_len < CAN_CONFIG_FRAME_SIZE) {
		CAN_DBG(CAN_DBG_ERROR, "host command too short: %d\r\n", data_len);
		return;
	}

	if (frame->type == CMD_HOST_PPS_DATA) {
		parse_and_update_pps_command((const can_pps_command_t*)frame_data);
	}
	else {
		parse_and_init_can(frame);
	}
}

static void pps_send_cached_frames(void)
{
	uint8_t channel_id;

	for (channel_id = 0; channel_id < AXI_CAN_DRV_MAX; channel_id++) {
		pps_channel_buffer_t* buffer = &pps_channel_buffers[channel_id];
		uint32_t frame_index;

		if (((pps_enable_mask & (1U << channel_id)) == 0U) ||
			(buffer->valid == 0U) || (can_channel_ready[channel_id] == 0U)) {
			continue;
		}

		for (frame_index = 0; frame_index < buffer->frame_count; frame_index++) {
			const can_request_frame_t* request = &buffer->frames[frame_index];
			u32 tx_frame[4];
			uint32_t wait_count = 0U;
			int status;

			tx_frame[0] = request->idr;
			tx_frame[1] = XCan_CreateDlcValue(request->drcr);
			tx_frame[2] = request->dw1r;
			tx_frame[3] = request->dw2r;

			while (XCan_IsTxFifoFull(&can_drv[channel_id].can) == TRUE) {
				if (++wait_count >= PPS_CAN_TX_WAIT_MAX) {
					CAN_DBG(CAN_DBG_ERROR,
						"PPS CAN TX FIFO timeout: ch=%d index=%d\r\n",
						channel_id, frame_index);
					break;
				}
			}

			if (wait_count >= PPS_CAN_TX_WAIT_MAX) {
				continue;
			}

			status = XCan_Send(&can_drv[channel_id].can, tx_frame);
			if (status != XST_SUCCESS) {
				CAN_DBG(CAN_DBG_ERROR, "PPS CAN send failed: ch=%d index=%d\r\n",
					channel_id, frame_index);
			}
		}
	}
}

static void pps_process_pending(void)
{
	if (pps_processed_count == pps_pending_count) {
		return;
	}

	pps_processed_count++;
	pps_send_cached_frames();
}
void send_can_tx_status(uint8_t ch_id, uint32_t data_array[], uint16_t status, uint8_t type)
{

	ddr_manager_t* channel = &ddr_channels[ch_id];
	can_axis_fifo_frame_t ddr_frame;
	uint32_t frame_size = sizeof(data_frame3_t);
	data_frame3_t status_frame;
	//    static data_frame3_t status_frame =
	//    {
	//    		0x499602D2,
	//			0x01,
	//			0x82,
	//			0x30,
	//			0x00
	//    };
	status_frame.head = 0x499602D2;
	status_frame.direction = 0x01;
	status_frame.type = type;
	status_frame.length = 0x30;
	status_frame.ch_id = ch_id;
	total_frame_count = (total_frame_count == 0xFFFFFFFF) ? 0 : total_frame_count + 1;
	status_frame.total_count = total_frame_count;
	channel_frame_count[status_frame.ch_id] = (channel_frame_count[status_frame.ch_id] == 0xFFFFFFFF) ? 0 : channel_frame_count[status_frame.ch_id] + 1;
	status_frame.ch_count = channel_frame_count[status_frame.ch_id];
	status_frame.idr = data_array[0];
	status_frame.drcr = data_array[1] >> 28;
	status_frame.dw1r = data_array[2];
	status_frame.dw2r = data_array[3];
	status_frame.states = status;
	status_frame.xor_states = 1;
	status_frame.reserved2 = 0;
	uint32_t low_bits, high_bits;
	low_bits = XGpio_DiscreteRead(&Gpio[1], GPIO_CHANNEL_1);
	high_bits = XGpio_DiscreteRead(&Gpio[1], GPIO_CHANNEL_2);
	(((uint64_t)high_bits << 32) | (uint64_t)low_bits);
	status_frame.timestamp = (((uint64_t)high_bits << 32) | (uint64_t)low_bits);
	//status_frame.timestamp = 0;
	status_frame.xor_checksum = calculate_xor_checksum((uint8_t*)&status_frame, sizeof(data_frame3_t) - 4);
	//将以上数据写进ddr里面
	uint32_t next_address = channel->current_address + frame_size;

	// 检查是否超出16M边界
	if (next_address > channel->base_address + 0x100000) {
		// 回卷到起始地址
		channel->current_address = channel->base_address;
		// 可以选择是否重置已使用大小计数
		// channel->used_size = 0;
	}

	//		CAN_DBG(CAN_DBG_DEBUG,"CAN SECV rxFrame_along[0]:0x%08x \n\r",status_frame.idr);
	//		CAN_DBG(CAN_DBG_DEBUG,"CAN SECV rxFrame_along[1]:0x%08x \n\r",status_frame.drcr);
	//		CAN_DBG(CAN_DBG_DEBUG,"CAN SECV rxFrame_along[2]:0x%08x \n\r",status_frame.dw1r);
	//		CAN_DBG(CAN_DBG_DEBUG,"CAN SECV rxFrame_along[3]:0x%08x \n\r",status_frame.dw2r);

	memcpy((void*)channel->current_address, &status_frame, sizeof(status_frame));
	//  CAN_DBG(CAN_DBG_DEBUG,"DDR address:%d \n\r",channel->current_address);

	  //写axi_fifo
	ddr_frame.head = axi_can_fifo_frame_head;
	ddr_frame.direction = DIR_DEVICE_TO_HOST;
	ddr_frame.count = status_frame.ch_count;
	ddr_frame.pack_ddr_base = channel->current_address;
	ddr_frame.pkt_len = sizeof(status_frame);
	axis_fifo_write_data(channel->fifo_baseaddr, (uint8_t*)&ddr_frame, sizeof(can_axis_fifo_frame_t));

	channel->current_address += frame_size;
	channel->used_size += frame_size;


}


static void parse_and_init_can(can_config_frame_t* frame)
{
	can_drv_t* pcan_drv_t;
	uint32_t expected_checksum;

	if (frame->head != axi_can_data_frame_head) {
		CAN_DBG(CAN_DBG_ERROR, "config head error:0x%08x ,\r\n", frame->head);
		return;
	}

	expected_checksum = calculate_xor_checksum((uint8_t*)frame, sizeof(can_config_frame_t) - 4);
	if ((frame->xor_checksum != expected_checksum) || (frame->ch_id > AXI_CAN_DRV_MAX) || (DIR_HOST_TO_DEVICE != frame->direction) || (0 != frame->type))
	{
		CAN_DBG(CAN_DBG_ERROR, "config error checksum:%d ch_id:%d direction:%d type:%d \r\n", frame->xor_checksum, frame->ch_id, frame->direction, frame->type);
		return;
	}

	pcan_drv_t = &can_drv[frame->ch_id];
	if (pcan_drv_t->init_flag) {
		CAN_DBG(CAN_DBG_ERROR, "chn id:%d no init \r\n", frame->ch_id);
		return;
	}

	axi_can_config(pcan_drv_t, frame->baud_rate);

	switch (frame->mode) {
	case 0x02:
		axi_can_set_mode(pcan_drv_t, XCAN_MODE_NORMAL);
		CAN_DBG(CAN_DBG_DEBUG, "CAN MODE SET SUCCESSFUL: XCAN_MODE_NORMAL\r\n");
		break;

	case 0x04:
		axi_can_set_mode(pcan_drv_t, XCAN_MODE_LOOPBACK);
		CAN_DBG(CAN_DBG_DEBUG, "CAN MODE SET SUCCESSFUL: XCAN_MODE_LOOPBACK\r\n");
		break;

	default:
		CAN_DBG(CAN_DBG_DEBUG, "CAN MODE UNKNOWN: 0x%02X\r\n", frame->mode);
		break;
	}

	//	if(frame->mode){
	//		axi_can_set_mode(pcan_drv_t, XCAN_MODE_NORMAL);
	//		CAN_DBG(CAN_DBG_DEBUG,"CAN MODE SET SUCCESSFUL XCAN_MODE_NORMAL \r\n");
	//	}else {
	//		axi_can_set_mode(pcan_drv_t, XCAN_MODE_LOOPBACK);
	//		CAN_DBG(CAN_DBG_DEBUG,"CAN MODE SET SUCCESSFUL XCAN_MODE_LOOPBACK \r\n" );
	//	}


	return;
}

static void can_send(can_drv_t* pcan_drv, can_request_frame_t* request_frame, can_status_frame_t* status_frame, uint32_t count)
{
	int i, ret;
	u32 Can_TxFrame[4];

	ret = 0;
	for (i = 0; i < count; i++) {
		status_frame[i].type = CMD_DEVICE_STATUS;

		if ((request_frame != NULL)) {
			uint32_t low_bits, high_bits;
			uint32_t wait_ms = 0;
			uint8_t tx_timeout = 0;

			Can_TxFrame[0] = request_frame[i].idr;
			Can_TxFrame[1] = XCan_CreateDlcValue(request_frame[i].drcr);
			Can_TxFrame[2] = request_frame[i].dw1r;
			Can_TxFrame[3] = request_frame[i].dw2r;
				
			//CAN_DBG(CAN_DBG_DEBUG,"request_frame[i].idr:0X%08X \r\n",request_frame[i].idr);
			ret = XST_SUCCESS;
			while (XCan_IsTxFifoFull(&pcan_drv->can) == TRUE)
			{
				if (wait_ms >= 1000)
				{
					tx_timeout = 1;
					ret = XST_FAILURE;   // 或者任意非 XST_SUCCESS 的值
					CAN_DBG(CAN_DBG_DEBUG, "CAN TX FIFO WAIT TIMEOUT \r\n");
					break;
				}

				//usleep(1000);              // 或 usleep(1000)，看你工程里现成接口
				wait_ms++;
			}

			if (tx_timeout == 0)
			{
				ret = XCan_Send(&pcan_drv->can, Can_TxFrame);
			}

			low_bits = XGpio_DiscreteRead(&Gpio[1], GPIO_CHANNEL_1);
			// 读取高32位（GPIO1通道2）
			high_bits = XGpio_DiscreteRead(&Gpio[1], GPIO_CHANNEL_2);
			status_frame[i].timestamp = (((uint64_t)high_bits << 32) | (uint64_t)low_bits);
			if (ret != XST_SUCCESS) {
				CAN_DBG(CAN_DBG_DEBUG, "CAN SEND NOT SUCCESSFUL \r\n");
				status_frame[i].xor_states = 2;
				if (count > 1) {
					status_frame[i].type = CMD_CAN_SINGLE_ERROR;
				}
				else {
					status_frame[i].type = CMD_CAN_MULTI_ERROR;
				}
			}
			else {
				//CAN_DBG(CAN_DBG_DEBUG,"CAN SEND SUCCESSFUL \r\n");
				status_frame[i].xor_states = 0;
			}
		}
		else {

			status_frame[i].xor_states = 1;
			if (count > 1) {
				status_frame[i].type = CMD_CAN_SINGLE_ERROR;
			}
			else {
				status_frame[i].type = CMD_CAN_MULTI_ERROR;
			}
		}
		status_frame[i].xor_checksum = calculate_xor_checksum((void*)&status_frame[i], sizeof(can_status_frame_t) - 4);

	}
}


static void can_send_status_farme(void)
{
	int i, j;
	can_drv_t* pcan_drv;
	can_frame_info_t frame_info;
	uint32_t offset, rb_count;
	can_response_frame_t response_frame;
	can_axis_fifo_frame_t axis_fifo_frame;
	u32 rxFrame[4];

	memset(&response_frame, 0x0, sizeof(response_frame));

	for (i = 0; i < AXI_CAN_DRV_MAX; i++) {
		pcan_drv = &can_drv[i];

		if (pcan_drv->rb == NULL) {
			continue;
		}

		if (!ringbuffer_is_empty(pcan_drv->rb)) {
			rb_count = ringbuffer_len(pcan_drv->rb);


			offset = pcan_drv->response_frame_offset;
			if (pcan_drv->response_frame_offset % CAN_RESPONSE_FRAME_OFFSET) {
				offset = ((pcan_drv->response_frame_offset / CAN_RESPONSE_FRAME_OFFSET) + 1) * CAN_RESPONSE_FRAME_OFFSET;
			}

			if (offset + (rb_count * CAN_ONE_STATUS_FRAME_SIZE) >= CAN_RESPONSE_FRAME_OFFSET_START) {
				offset = 0;
			}

			for (j = 0; j < rb_count; j++) {

				ringbuffer_get(pcan_drv->rb, (uint8_t*)&frame_info, sizeof(can_frame_info_t));
				memcpy((void*)rxFrame, (void*)(pcan_drv->rxframe_phy_addr + (frame_info.offset * CAN_FIFO_ONE_FRAME_SIZE)), CAN_FIFO_ONE_FRAME_SIZE);

				response_frame.head = axi_can_data_frame_head;
				response_frame.direction = DIR_DEVICE_TO_HOST;
				response_frame.type = CMD_DEVICE_NORMAL;
				response_frame.length = CAN_ONE_STATUS_FRAME_SIZE;
				response_frame.ch_id = pcan_drv->can_chn_id;
				response_frame.idr = rxFrame[0];
				response_frame.drcr = rxFrame[1] >> 28;
				response_frame.dw1r = rxFrame[2];
				response_frame.dw2r = rxFrame[3];
				response_frame.states = frame_info.states;

				uint32_t low_bits, high_bits;
				low_bits = XGpio_DiscreteRead(&Gpio[1], GPIO_CHANNEL_1);
				high_bits = XGpio_DiscreteRead(&Gpio[1], GPIO_CHANNEL_2);
				(((uint64_t)high_bits << 32) | (uint64_t)low_bits);
				response_frame.timestamp = (((uint64_t)high_bits << 32) | (uint64_t)low_bits);
				response_frame.total_count = rb_count;
				response_frame.ch_count = j;
				response_frame.xor_checksum = calculate_xor_checksum((void*)&response_frame, sizeof(can_response_frame_t) - 4);
				// CAN_DBG(CAN_DBG_ALL," response_frame.xor_checksum:0x%08x\r\n", response_frame.xor_checksum);

				memcpy((can_response_frame_t*)(pcan_drv->response_phy_addr + offset + (j * sizeof(can_response_frame_t))), &response_frame, sizeof(response_frame));
			}

			pcan_drv->response_frame_offset = offset + (rb_count * sizeof(can_response_frame_t));
			axis_fifo_frame.head = axi_can_fifo_frame_head;
			axis_fifo_frame.direction = DIR_DEVICE_TO_HOST;
			axis_fifo_frame.count = pcan_drv->fifo_frame_count;
			axis_fifo_frame.pack_ddr_base = pcan_drv->response_phy_addr + offset;
			axis_fifo_frame.pkt_len = rb_count * CAN_ONE_STATUS_FRAME_SIZE;

			axis_fifo_write_data(pcan_drv->axis_fifo_base_address, (unsigned char*)&axis_fifo_frame, sizeof(can_axis_fifo_frame_t));
			pcan_drv->response_frame_offset = offset + (rb_count * sizeof(can_response_frame_t));
		}
	}
}



static void can_send_request_frame(can_drv_t* pcan_drv, uint32_t pack_ddr_base, uint32_t pkt_len)
{
	can_axis_fifo_frame_t axis_fifo_frame;
	can_request_frame_t* request_frame;
	can_status_frame_t* status_frame;
	uint32_t rem, count, done, offset;

	uint32_t calculated_xor, packets_count, packets_sum;
	uint32_t frame_error, request_len;

	uint8_t* frame_buffer;


	request_len = pkt_len;

	frame_buffer = (uint8_t*)malloc(pkt_len);
	if (frame_buffer == NULL) {
		CAN_DBG(CAN_DBG_ERROR, "malloc size %d error \r\n", pkt_len);
		return;
	}


	if (pkt_len % sizeof(can_request_frame_t)) {
		request_len = (pkt_len / sizeof(can_request_frame_t)) * sizeof(can_request_frame_t);
	}


	offset = pcan_drv->status_frame_offset;
	if (pcan_drv->status_frame_offset % CAN_CMD_STATUS_FRAME_OFFSET) {
		offset = ((pcan_drv->status_frame_offset / CAN_CMD_STATUS_FRAME_OFFSET) + 1) * CAN_CMD_STATUS_FRAME_OFFSET;
	}



	if ((offset + request_len) >= CAN_RESPONSE_FRAME_OFFSET_START) {
		//CAN_DBG(CAN_DBG_DEBUG,"#chn:%d offset:%d-%d pkt_len:(%d) ###\r\n", pcan_drv->can_chn_id, pcan_drv->status_frame_offset, offset, pkt_len);
		offset = 0;
	}


	memcpy((void*)(frame_buffer), (void*)pack_ddr_base, request_len);

	done = 0;
	count = sizeof(can_request_frame_t);

	packets_count = 0;
	frame_error = 0;

	bool first_recv = true;
	uint32_t pre_frm_cnt = 0;
	uint32_t pre_frm_sum = 0;

	uint32_t continus_cnts = 0;

	uint32_t bag_cnts = request_len / sizeof(can_request_frame_t);

	while (1)
	{
		can_send_status_farme();

		rem = request_len - done;

		if (rem == 0)
		{

			if (bag_cnts == (continus_cnts + 1))
			{
				if (!frame_error) {
					can_send(pcan_drv, (can_request_frame_t*)(frame_buffer),
						(can_status_frame_t*)(frame_buffer), (request_len / count));
				}
				else {
					can_send(pcan_drv, NULL,
						(can_status_frame_t*)(frame_buffer), (request_len / count));
				}
			}

			break;
		}

		request_frame = (can_request_frame_t*)(frame_buffer + done);
		status_frame = (can_status_frame_t*)(frame_buffer + done);

		calculated_xor = calculate_xor_checksum((void*)request_frame, sizeof(can_request_frame_t) - 4);

		if ((calculated_xor != request_frame->xor_checksum) || (request_frame->direction != 0) || (request_frame->head != axi_can_data_frame_head)) {
			frame_error = 1;
		}

		packets_sum = request_frame->packets_sum;

		done += count;
		if (first_recv)
		{
			pre_frm_cnt = request_frame->packets_no;
			pre_frm_sum = request_frame->packets_sum;
			first_recv = false;
			continue;
		}
		u_int32_t cur_frm_cnt = request_frame->packets_no;

		//			if(cur_frm_cnt - pre_frm_cnt != 1  )
		//			{
		//				frame_error = 1;
		//				CAN_DBG(CAN_DBG_DEBUG," cur_frm_cnt %d != pre_frm_cnt:%d  \r\n", cur_frm_cnt, pre_frm_cnt);
		//			}
		//			else
		//			{
		//				pre_frm_cnt =  cur_frm_cnt;
		//				continus_cnts++;
		//			}
		if ((cur_frm_cnt - pre_frm_cnt == 1) || (((pre_frm_cnt + 1) == pre_frm_sum) && (cur_frm_cnt == 0)))
		{
			continus_cnts++;
			pre_frm_cnt = cur_frm_cnt;
			pre_frm_sum = request_frame->packets_sum;
		}
		else
		{
			frame_error = 1;
			CAN_DBG(CAN_DBG_DEBUG, " cur_frm_cnt %d != pre_frm_cnt:%d ,request_frame->packets_sum: %d\r\n", cur_frm_cnt, pre_frm_cnt, request_frame->packets_sum);
		}
		//			if (first_recv)
		//			{
		//				pre_frm_cnt = request_frame->packets_no;
		//				first_recv = false;
		//				continue;
		//			}
		//			uint32_t cur_frm_cnt = request_frame->packets_no;
		//			        // 修改连续性判断逻辑
		//			if (cur_frm_cnt < request_frame->packets_sum)
		//			{
		//			    if (cur_frm_cnt == (axififo_index % axififo_unit_count))
		//			    {
		//			                axififo_index++;
		//			                pre_frm_cnt = cur_frm_cnt;
		//			                continus_cnts++;
		//			     }
		//			    else
		//			    {
		//			                frame_error = 1;
		//			                CAN_DBG(CAN_DBG_DEBUG, " cur_frm_cnt %d != expected:%d  \r\n", cur_frm_cnt, axififo_index % axififo_unit_count);
		//			    }
		//		 }
	}

	memcpy((void*)(pcan_drv->status_phy_addr + offset), (void*)frame_buffer, request_len);


	while (1) {

		axis_fifo_frame.head = axi_can_fifo_frame_head;
		axis_fifo_frame.direction = DIR_DEVICE_TO_HOST;
		axis_fifo_frame.count = pcan_drv->fifo_frame_count;
		axis_fifo_frame.pack_ddr_base = pcan_drv->status_phy_addr + offset;
		axis_fifo_frame.pkt_len = request_len;

		axis_fifo_write_data(pcan_drv->axis_fifo_base_address, (unsigned char*)&axis_fifo_frame, sizeof(can_axis_fifo_frame_t));

		pcan_drv->status_frame_offset = offset + pkt_len;
		pcan_drv->fifo_frame_count += 1;
		break;
	}

	free(frame_buffer);

}


void axi_can_frame_transfer(void)
{
	can_drv_t* pcan_drv;
	int i, ret, data_len;
	int send_cnt = 0;
	int chid;
	can_axis_fifo_frame_t axis_fifo_frame;
	axis_fifo_reset(CONFIG_FIFO_BASEADDRESS);
	for (i = 0; i < AXI_CAN_DRV_MAX; i++) {
		can_drv[i].along_more_int_couter = 0;
		can_drv[i].alonge_start = 0;
		can_drv[i].alonger_more_inter = 0;
		can_drv[i].alonge_inter = 0;
	}
	while (1)
	{

		for (int i = 0; i < AXI_CAN_DRV_MAX; i++)
		{
			data_len = axis_fifo_read_data_len(Axi_fifo_init_addresses[i]);
			if (data_len > 0)
			{
				if ((uint32_t)data_len > (uint32_t)sizeof(buffer_config_receive)) {
					CAN_DBG(CAN_DBG_ERROR, "host command too long: %d\r\n", data_len);
					axis_fifo_reset(Axi_fifo_init_addresses[i]);
					continue;
				}
				ret = axis_fifo_read_data(Axi_fifo_init_addresses[i], buffer_config_receive, data_len);
				if (ret == data_len) {
					parse_host_command(buffer_config_receive, data_len);
				}
				axis_fifo_reset(Axi_fifo_init_addresses[i]);
				if (ret != data_len)
				{
					CAN_DBG(CAN_DBG_ERROR, "read fifo config frame error,(%d !=%d) \r\n", data_len, ret);
				}

			}
		}

		pps_process_pending();

		for (i = 0; i < AXI_CAN_DRV_MAX; i++)
		{
			//CAN_DBG(CAN_DBG_DEBUG,"read GPIO data %d \r\n", gpio_data);


			pcan_drv = &can_drv[i];
			can_send_status_farme();
			chid = i;


			data_len = axis_fifo_read_data_len(pcan_drv->axis_fifo_base_address);

			if (data_len >= sizeof(can_axis_fifo_frame_t))
			{

				ret = axis_fifo_read_data(pcan_drv->axis_fifo_base_address, (unsigned char*)&axis_fifo_frame, sizeof(can_axis_fifo_frame_t));
				if (ret != sizeof(can_axis_fifo_frame_t))
				{
					CAN_DBG(CAN_DBG_ERROR, "read can config  error (%d != %d) \r\n", ret, sizeof(can_axis_fifo_frame_t));
					//axis_fifo_reset(pcan_drv->axis_fifo_base_address);
					continue;
				}
				if (axis_fifo_frame.direction == 0x3)
				{
					CAN_DBG(CAN_DBG_DEBUG, "START ALONG MODE:channel id %d\n\r", i);
					pcan_drv->alonge_start = 1;
					//初始化DDR的各个通道
					ddr_manager_init();
					while (1)
					{
						if (pcan_drv->alonge_inter == 1)
						{
							for (uint32_t offset = 0; offset < ALONG_SUM_OFFSET; offset += ALONG_OFFSET)
							{
								uint8_t* src_addr = (uint8_t*)(axis_fifo_frame.pack_ddr_base + offset);
								//CAN_DBG(CAN_DBG_DEBUG,"DDR_BASE_OFFSET:0X%08X,DDR_BASE_ADDR: 0X%08X , OFFSET:0X%08X \n\r",src_addr,axis_fifo_frame.pack_ddr_base,offset);
								uint8_t temp_buffer[sizeof(ALONG_DataPacket)];
								memcpy(temp_buffer, src_addr, sizeof(temp_buffer));

								ALONG_DataPacket* frame_t = (ALONG_DataPacket*)temp_buffer;
								if (frame_t->head != axi_can_data_frame_head || frame_t->type != ALONG_TYPE)
								{
									//CAN_DBG(CAN_DBG_DEBUG,"Along_Stand.head: 0x%08x, Along_Stand.type:0x%02x\r\n",frame_t->head, frame_t->type);
									break;
								}
								while (frame_t->can_id == rxFrame_along[0] && frame_t->data_type == rxFrame_along[2])
								{

									rxFrame_along[0] = rxFrame_along[0] << 21;
									send_can_tx_status(chid, rxFrame_along, 1, 0x82);
									while (pcan_drv->along_more_int_couter != (frame_t->rcv_len / 8))
									{

									}
									//表示多次中断完成
									if ((frame_t->rcv_len / 8) > 1)
									{
										for (int i = 0; i < ((frame_t->rcv_len / 8) - 1); i++)
										{
											CAN_SEND_ALONG_DAN[0] = CAN_SEND_ALONG[0 + i * 4];
											CAN_SEND_ALONG_DAN[1] = CAN_SEND_ALONG[1 + i * 4];
											CAN_SEND_ALONG_DAN[2] = CAN_SEND_ALONG[2 + i * 4];
											CAN_SEND_ALONG_DAN[3] = CAN_SEND_ALONG[3 + i * 4];
											send_can_tx_status(chid, CAN_SEND_ALONG_DAN, 1, 0x82);
										}
									}

									//等待所有中断完成，将除了第一包的数据存入相应的数组后，然后根据剩下的中断次数，来进行提取数据，发送给上位机

									can_send_index = 0;
									//memset(CAN_SEND_ALONG, 0, sizeof(CAN_SEND_ALONG));
									for (int i = 0; i < frame_t->send_len / CAN_FIFO_TWO_FRAME_SIZE; i++)
									{
										for (int j = 0; j < CAN_FIFO_THR_FRAME_SIZE; j++)
										{

											*(Can_TxFrame_t + j) = *((u32*)frame_t->send_data + j + i * CAN_FIFO_THR_FRAME_SIZE);
											//CAN_DBG(CAN_DBG_DEBUG,"Can_TxFrame_t[%d] :0x%08x\r\n",j,Can_TxFrame_t[j]);
										}
										while (XCan_IsTxFifoFull(&pcan_drv->can) == TRUE);
										Can_TxFrame_t[1] = XCan_CreateDlcValue(Can_TxFrame_t[1]);
										ret = XCan_Send(&pcan_drv->can, Can_TxFrame_t);
										if (ret != XST_SUCCESS)
										{
											CAN_DBG(CAN_DBG_DEBUG, "CAN SEND NOT SUCCESSFUL \r\n");
										}
										//这里将单机模拟中断过来的数据保存到数组里面，然后将回包的数据提取出来，放到数组里面
										//memcpy(CAN_SEND_DATA_PC,rxFrame_along,16);
										//组帧，上传到上位机
										send_can_tx_status(chid, Can_TxFrame_t, ret, 0x83);


										pcan_drv->along_more_int_couter = 0;
										pcan_drv->alonge_inter = 0;


									}
									break;
								}


							}
							pcan_drv->alonge_inter = 0;
							pcan_drv->along_more_int_couter = 0;

						}
					}

				}


				//CAN_DBG(CAN_DBG_DEBUG,"### pack_ddr_base:0x%x pkt_len:%d ## \r\n", axis_fifo_frame.pack_ddr_base, axis_fifo_frame.pkt_len);
				can_send_request_frame(pcan_drv, axis_fifo_frame.pack_ddr_base, axis_fifo_frame.pkt_len);
			}
		}
	}
}

void axi_can_system_init(void)
{
	int Status;

	intc_init_status = 0;

	Status = xintc_drv_init();
	if (Status) {
		CAN_DBG(CAN_DBG_ERROR, "intc drv init error \r\n");
		intc_init_status = 1;
	}
	if (axi_gpio_drv_init() != XST_SUCCESS) {

		CAN_DBG(CAN_DBG_ERROR, "intc drv_GPIO init error \r\n");
	}
	if (!intc_init_status && (pps_gpio_drv_init() != XST_SUCCESS)) {
		CAN_DBG(CAN_DBG_ERROR, "PPS GPIO init error\r\n");
	}

	if (!intc_init_status) {
		Status = tmrctr_drv_init();
		if (Status != XST_SUCCESS) {
			CAN_DBG(CAN_DBG_ERROR, "tmrctr drv init error \r\n");
		}
	}


	axi_can_drv_init();



	xintc_drv_start();
}
