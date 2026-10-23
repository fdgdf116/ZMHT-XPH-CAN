#ifndef _CAN_COMMON_H_
#define _CAN_COMMON_H_

#include <stddef.h>
#include "xparameters.h"

/****************axis fifo******************/
#define CONFIG_FIFO_BASEADDRESS (0x44A00000)
/*******************************************/
#define AXI_CAN_DRV_MAX (6)
#define DDR_CHANNEL_COUNT 6

#define CAN_INTR_VEC_ID		XPAR_INTC_0_CAN_0_VEC_ID
#define CAN_DEVICE_ID		XPAR_CAN_0_DEVICE_ID

#define CAN_CONFIG_FRAME_SIZE (48)
#define CAN_CONFIG_FRAME_SIZE_MAX (AXI_CAN_DRV_MAX * CAN_CONFIG_FRAME_SIZE)

#define CAN_FIFO_ONE_FRAME_SIZE (16)
#define CAN_FIFO_TWO_FRAME_SIZE (16)
#define CAN_FIFO_THR_FRAME_SIZE  (4)
#define CAN_FIFO_FRAME_SIZE_MAX (2048*10)

/******************************************/
#define CAN_RESPONSE_FRAME_OFFSET_START (8*1024*1024)
#define CAN_ONE_STATUS_FRAME_SIZE (48)
#define CAN_STATUS_FRAME_COUNT_MAX (128)

#define CAN_CMD_STATUS_FRAME_OFFSET (CAN_ONE_STATUS_FRAME_SIZE * CAN_STATUS_FRAME_COUNT_MAX)

/*********************************************/
#define CAN_RXFRAME_ADDR_OFFSET_MAX (2*1024*1024)
#define CAN_RESPONSE_FRAME_RB_COUNT_MAX (1*1024)
#define CAN_SEND_RESPONSE_FRAME_RB_COUNT_MAX (CAN_STATUS_FRAME_COUNT_MAX)
#define CAN_RESPONSE_FRAME_OFFSET (CAN_CMD_STATUS_FRAME_OFFSET)

//GPIO 偏移
#define GPIO_CHANNEL_1 1
#define GPIO_CHANNEL_2 2
#define GPIO0_OUTPUT_MASK 0x01
#define GPIO1_OUTPUT_MASK 0x00

#define PPS_CHANNEL_MASK ((1U << AXI_CAN_DRV_MAX) - 1U)
#define PPS_CAN_FRAME_COUNT_MAX CAN_STATUS_FRAME_COUNT_MAX
//单机模拟
u32 rxFrame_along[4];
u32 Can_TxFrame_t[4];
u32 CAN_SEND_DATA_PC[4];
u32 CAN_SEND_ALONG[256];
u32 CAN_SEND_ALONG_DAN[4];
extern volatile u32 can_send_index;

#define ALONG_SUM_OFFSET (32*1024)
#define ALONG_OFFSET 1024
#define ALONG_TX_BUFFER 256
#define ALONG_TYPE 0X03
#define ALONG_DATA_SE 32
//单机模拟结构体
#pragma pack(1)
typedef struct {
	uint32_t head;
	uint8_t direction;
	uint8_t type;
	uint16_t length;
	uint8_t channel_id;
	uint32_t total_count;
	uint32_t channel_count;
	uint16_t packet_type;
	uint32_t can_id;
	uint32_t data_type;
	uint16_t rcv_len;
	uint16_t send_len;
	uint8_t reserved;
	uint8_t send_data[512];
} ALONG_DataPacket;
#pragma pack()

typedef struct {
	uint32_t head;
	uint8_t direction;
	uint8_t type;
	uint8_t length;
	uint8_t ch_id;
	uint32_t total_count;
	uint32_t ch_count;
	uint32_t idr;
	uint32_t drcr;
	uint32_t dw1r;
	uint32_t dw2r;
	uint8_t states;
	uint8_t xor_states;
	uint16_t reserved2;
	uint64_t timestamp;
	uint32_t xor_checksum;
}data_frame3_t;




typedef enum {
	// 上位机发送的命令
	CMD_HOST_INIT = 0x00,  // 上位机发送初始化指令
	CMD_HOST_NORMAL_DATA = 0x01,  // 上位机发送正常数据
	CMD_HOST_PPS_DATA = 0x02,  // 上位机发送秒脉冲数据
	CMD_HOST_SIM_DATA = 0x03,  // 上位机发送单机模拟器数据

	// 下位机上传的命令
	CMD_DEVICE_STATUS = 0x80,  // 下位机上传状态反馈数据
	CMD_DEVICE_NORMAL = 0x81,  // 下位机上传正常采集数据
	CMD_DEVICE_SIM_DATA = 0x82,  // 下位机上传单机模拟器数据（原始定义0x81已修正为0x82）

	// 错误类命令
	CMD_CAN_MULTI_ERROR = 0xFE,  // 上位机发送CAN多包错误
	CMD_CAN_SINGLE_ERROR = 0xFF   // 上位机发送CAN单包错误
} CommandType;

typedef enum {
	DIR_HOST_TO_DEVICE = 0,    // 上位机 → 下位机
	DIR_DEVICE_TO_HOST = 1     // 下位机 → 上位机
} DataDirection;

enum axi_can_frame_head_e {
	axi_can_data_frame_head = 0x499602D2,
	axi_can_fifo_frame_head = 0x499601D1,
};

enum axi_can_drv_id_e {
	axi_can_drv_id0 = 0,
	axi_can_drv_id1 = 1,
	axi_can_drv_id2 = 2,
	axi_can_drv_id3 = 3,
	axi_can_drv_id4 = 4,
	axi_can_drv_id5 = 5,
	axi_can_drv_id_max
};

enum axi_can_drv_intc_id_e {
	axi_can_drv_intc_id0 = 1,
	axi_can_drv_intc_id1 = 15,
	axi_can_drv_intc_id2 = 16,
	axi_can_drv_intc_id3 = 17,
	axi_can_drv_intc_id4 = 18,
	axi_can_drv_intc_id5 = 19,
	axi_can_drv_intc_id_max
};

/* arm--->上位机*/
enum can_status_frame_base_address_e {
	can_status_frame_base_address0 = 0x21000000,
	can_status_frame_base_address1 = 0x23000000,
	can_status_frame_base_address2 = 0x25000000,
	can_status_frame_base_address3 = 0x27000000,
	can_status_frame_base_address4 = 0x29000000,
	can_status_frame_base_address5 = 0x2b000000,
	can_rxframe_base_address = 0x2c000000,
	can_status_frame_base_address_max
};

enum can_axis_fifo_base_address_e {
	can_axis_fifo_base_address0 = 0x00500000,
	can_axis_fifo_base_address1 = 0x00510000,
	can_axis_fifo_base_address2 = 0x00520000,
	can_axis_fifo_base_address3 = 0x00530000,
	can_axis_fifo_base_address4 = 0x00540000,
	can_axis_fifo_base_address5 = 0x00550000,
	can_axis_fifo_base_address_max
};

typedef struct {
	uint32_t head;
	uint8_t direction;
	uint8_t type;
	uint8_t length;
	uint8_t ch_id;
	uint32_t total_count;
	uint32_t ch_count;
	uint32_t baud_rate;
	uint8_t mode;
	uint8_t auto_send;
	uint16_t ier;
	uint32_t packet_interval_us;
	uint32_t pps_delay_ms;
	uint8_t reserved[4];
	uint64_t timestamp;
	uint32_t xor_checksum;
} can_config_frame_t;

typedef char can_config_interval_offset_must_be_24[
	(offsetof(can_config_frame_t, packet_interval_us) == 24) ? 1 : -1];
typedef char can_config_pps_delay_offset_must_be_28[
	(offsetof(can_config_frame_t, pps_delay_ms) == 28) ? 1 : -1];
typedef char can_config_frame_size_must_be_48[
	(sizeof(can_config_frame_t) == CAN_CONFIG_FRAME_SIZE) ? 1 : -1];

typedef struct __attribute__((packed)) {
	uint32_t head;
	uint8_t direction;
	uint8_t type;
	uint8_t length;
	uint8_t ch_mask;
	uint32_t can_count;
	uint32_t enable_mask;
	uint32_t update_mask;
	uint8_t reserved[16];
	uint64_t timestamp;
	uint32_t xor_checksum;
} can_pps_command_t;

typedef struct {
	uint32_t head;
	uint8_t direction;
	uint8_t type;
	uint8_t length;
	uint8_t ch_id;
	uint32_t total_count;
	uint32_t ch_count;
	uint32_t idr;
	uint32_t drcr;
	uint32_t dw1r;
	uint32_t dw2r;
	uint16_t packets_sum;
	uint16_t packets_no;
	uint64_t timestamp;
	uint32_t xor_checksum;
}can_request_frame_t;

typedef char can_pps_command_size_must_be_48[
	(sizeof(can_pps_command_t) == CAN_CONFIG_FRAME_SIZE) ? 1 : -1];
typedef char can_request_frame_size_must_be_48[
	(sizeof(can_request_frame_t) == CAN_CONFIG_FRAME_SIZE) ? 1 : -1];

typedef struct {
	uint32_t head;
	uint8_t direction;
	uint8_t type;
	uint8_t length;
	uint8_t ch_id;
	uint32_t total_count;
	uint32_t ch_count;
	uint32_t idr;
	uint32_t drcr;
	uint32_t dw1r;
	uint32_t dw2r;
	uint16_t xor_states;
	uint16_t reserved;
	uint64_t timestamp;
	uint32_t xor_checksum;
}can_status_frame_t;

typedef struct {
	uint32_t head;
	uint8_t direction;
	uint8_t type;
	uint8_t length;
	uint8_t ch_id;
	uint32_t total_count;
	uint32_t ch_count;
	uint32_t idr;
	uint32_t drcr;
	uint32_t dw1r;
	uint32_t dw2r;
	uint8_t states;
	uint8_t reserved1;
	uint16_t reserved2;
	uint64_t timestamp;
	uint32_t xor_checksum;
}can_response_frame_t;

typedef struct {
	uint32_t head;
	uint8_t  direction;
	uint8_t hp_ch_num;
	uint16_t count;
	uint32_t pack_ddr_base;
	uint32_t pkt_len;
}can_axis_fifo_frame_t;

typedef struct {
	uint8_t  chid;
	uint32_t base_address;
	uint32_t current_address;
	uint32_t total_size;
	uint32_t used_size;
	uint32_t fifo_baseaddr;
} ddr_manager_t;

//typedef struct {
//    unsigned int frame_offset;
//}can_frame_info_t;

typedef struct {
	unsigned int offset;
	unsigned char states;
}can_frame_info_t;

#endif
