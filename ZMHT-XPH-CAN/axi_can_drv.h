#ifndef _CAN_INIT_H_
#define _CAN_INIT_H_

#include <stdbool.h>

#include "xparameters.h"
#include "xstatus.h"
#include "can_common.h"
#include "xil_printf.h"
#include "ringbuffer.h"
#include "xcan.h"

/***************************************************************************/
#define MAX_PACKETS_PER_INTERRUPT 64
#define TEST_MESSAGE_ID			1024
#define FRAME_DATA_LENGTH	8

/************************** Constant Definitions ***************************/
#define TEST_BRPR_BAUD_PRESCALAR 2
#define TEST_BTR_SYNCJUMPWIDTH  0
#define TEST_BTR_SECOND_TIMESEGMENT 4
#define TEST_BTR_FIRST_TIMESEGMENT 9


/*CAN设备管理*/
typedef struct {
	u32 esr_acker_count;//XCAN_ESR_ACKER_MASK
	u32 esr_berr_count;//XCAN_ESR_BERR_MASK
	u32 esr_ster_count;//XCAN_ESR_STER_MASK
	u32 esr_fmer_count;//XCAN_ESR_FMER_MASK
	u32 esr_crcer_count;//XCAN_ESR_CRCER_MASK
}drv_can_error_t;

typedef struct {
	u32 ixr_bsoff_count;  	//IXR_BSOFF
	u32 ixr_rxoflw_count;	//XCAN_IXR_RXOFLW_MASK
	u32 ixr_rxuflw_count;	//XCAN_IXR_RXUFLW_MASK
	u32 ixr_txbfll_count; 	//XCAN_IXR_TXBFLL_MASK
	u32 ixr_txfll_count;	//XCAN_IXR_TXFLL_MASK
	u32 ixr_wkup_count;		//XCAN_IXR_WKUP_MASK
	u32 ixr_slp_count;		//XCAN_IXR_SLP_MASK
	u32 ixr_arblst_count;	//XCAN_IXR_ARBLST_MASK
}drv_can_event_t;


typedef struct {
	XCan can; //CAN设备
	u8 can_chn_id;
	u16 can_id; //can设备初始化id
	u8 intc_id; //can中断

	u8 init_flag; //初始化标志

	u32 baud; //波特率

	drv_can_event_t can_event; //event信息
	drv_can_error_t can_error; //error信息

	ring_buffer_t* rb; //接收外部CAN信息的ringbuffer
	u8 rb_count;
	u8 alonge_start;
	u8 alonge_inter;
	u8 alonger_more_inter;
	u32 along_more_int_couter;

	u32 rxframe_phy_addr;
	u32 status_phy_addr; //base_address; //arm--->上位机的 ddr的基地址
	u32 response_phy_addr;

	u32 status_frame_offset;
	u32 response_frame_offset;

	UINTPTR axis_fifo_base_address; //can fifo 的基地址
	u16 fifo_frame_count;
}can_drv_t;

/************************** Function Prototypes ******************************/

int axi_can_init(can_drv_t* pdrv_can);
void axi_can_config(can_drv_t* drv_can, u32 baud);
void axi_can_set_mode(can_drv_t* can_drv, u8 Mode);

#endif
