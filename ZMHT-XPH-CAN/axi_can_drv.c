#include "axi_can_drv.h"
#include "axis_fifo_drv.h"
#include "ringbuffer.h"
#include "xutils.h"
#include "can_common.h"

static void send_handler(void* CallBackRef)
{
	// XCan *can;
	// can_drv_t *pcan_drv;
	// pcan_drv = (can_drv_t *)CallBackRef;
	// can = &pcan_drv->can;
	// CAN_DBG(CAN_DBG_ERROR,"123456 send hander\n\r");
}


static void recv_handler(void* CallBackRef)
{
	XCan* can;
	can_drv_t* pcan_drv;
	can_frame_info_t frame_info;
	u32 rxFrame[4];

	pcan_drv = (can_drv_t*)CallBackRef;
	can = &pcan_drv->can;

	while (XCan_IsRxEmpty(can) != 1) {

		if ((pcan_drv->rb_count * CAN_FIFO_ONE_FRAME_SIZE) >= CAN_RXFRAME_ADDR_OFFSET_MAX) {
			pcan_drv->rb_count = 0;
		}

		frame_info.states = XCan_Recv(can, rxFrame);
		if (pcan_drv->rb != NULL && (pcan_drv->alonge_start == 0)) {
			//CAN_DBG(CAN_DBG_ERROR,"start interin normal \n\r");
			if (ringbuffer_is_full(pcan_drv->rb)) {
				CAN_DBG(CAN_DBG_ERROR, "can%d recv data ringbuffer error \n\r", pcan_drv->can_chn_id);
			}
			//			CAN_DBG(CAN_DBG_DEBUG,"CAN DIR rxFrame[0]:0x%08x \n\r",rxFrame[0]);
			//			CAN_DBG(CAN_DBG_DEBUG,"CAN DIR rxFrame[1]:0x%08x \n\r",rxFrame[1]);
			//			CAN_DBG(CAN_DBG_DEBUG,"CAN DIR rxFrame[2]:0x%08x \n\r",rxFrame[2]);
			//			CAN_DBG(CAN_DBG_DEBUG,"CAN DIR rxFrame[3]:0x%08x \n\r",rxFrame[3]);

			memcpy((void*)(pcan_drv->rxframe_phy_addr + (pcan_drv->rb_count * CAN_FIFO_ONE_FRAME_SIZE)), (void*)rxFrame, CAN_FIFO_ONE_FRAME_SIZE);

			frame_info.offset = pcan_drv->rb_count;
			ringbuffer_put(pcan_drv->rb, (uint8_t*)&frame_info, sizeof(can_frame_info_t));
			pcan_drv->rb_count += 1;

		}
		else if (pcan_drv->rb != NULL && (pcan_drv->alonge_start == 1))
		{
			//	CAN_DBG(CAN_DBG_ERROR,"start interin not normal \n\r");
			pcan_drv->alonge_inter = 1;
			pcan_drv->along_more_int_couter = pcan_drv->along_more_int_couter + 1;
			if (pcan_drv->along_more_int_couter == 1)
			{

				//pcan_drv->alonge_inter=1;
				rxFrame_along[0] = rxFrame[0] >> 21;
				rxFrame_along[1] = rxFrame[1];
				rxFrame_along[2] = rxFrame[2];
				rxFrame_along[3] = rxFrame[3];

				//	 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV rxFrame_along[0]:0x%08x \n\r",rxFrame_along[0]);
				//	 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV rxFrame_along[1]:0x%08x \n\r",rxFrame_along[1]);
				//	 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV rxFrame_along[2]:0x%08x \n\r",rxFrame_along[2]);
				//	 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV rxFrame_along[3]:0x%08x \n\r",rxFrame_along[3]);
			}
			else
			{
				// CAN_DBG(CAN_DBG_ERROR,"can send along  123\n\r");
				//如果进行第二次中断不会有任何·反馈，但是会将接受到的数据存储下来
				if (can_send_index + 4 <= 256)
				{
					CAN_SEND_ALONG[can_send_index] = rxFrame[0];
					CAN_SEND_ALONG[can_send_index + 1] = rxFrame[1];
					CAN_SEND_ALONG[can_send_index + 2] = rxFrame[2];
					CAN_SEND_ALONG[can_send_index + 3] = rxFrame[3];
					//
					//		 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV CAN_SEND_ALONG[%d]:0x%08x \n\r",can_send_index,CAN_SEND_ALONG[can_send_index]);
					//		 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV CAN_SEND_ALONG[%d+1]:0x%08x \n\r",can_send_index,CAN_SEND_ALONG[can_send_index+1]);
					//		 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV CAN_SEND_ALONG[%d+2]:0x%08x \n\r",can_send_index,CAN_SEND_ALONG[can_send_index+2]);
					//		 			CAN_DBG(CAN_DBG_DEBUG,"CAN RECV CAN_SEND_ALONG[%d+3]:0x%08x \n\r",can_send_index,CAN_SEND_ALONG[can_send_index+3]);

				}
				can_send_index += 4;
			}

		}
	}
}

void axi_can_config(can_drv_t* drv_can, u32 baud)
{
	XCan_EnterMode(&drv_can->can, XCAN_MODE_CONFIG);
	while (XCan_GetMode(&drv_can->can) != XCAN_MODE_CONFIG);

	drv_can->baud = baud;
	switch (baud) {
	case 1000000:  // 1Mbps
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 1M sucessfull \r\n");// 1M
		XCan_SetBaudRatePrescaler(&drv_can->can, 2);
		XCan_SetBitTiming(&drv_can->can, 0, 2, 3);
		break;
	case 800000:    //800k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 800K sucessfull \r\n");// 800k
		XCan_SetBaudRatePrescaler(&drv_can->can, 2);
		XCan_SetBitTiming(&drv_can->can, 0, 4, 3);
		break;
	case 500000:
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 500k sucessfull \r\n");// 500k
		XCan_SetBaudRatePrescaler(&drv_can->can, 2);
		XCan_SetBitTiming(&drv_can->can, 0, 4, 9);
		break;
	case 250000:   // 250k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 250k sucessfull \r\n");// 250k
		XCan_SetBaudRatePrescaler(&drv_can->can, 11);
		XCan_SetBitTiming(&drv_can->can, 0, 3, 2);
		break;
	case 125000:   // 125k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 125k sucessfull \r\n");// 125k
		XCan_SetBaudRatePrescaler(&drv_can->can, 11);
		XCan_SetBitTiming(&drv_can->can, 0, 5, 8);
		break;
	case 100000:   // 100k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 100k sucessfull \r\n");// 100k
		XCan_SetBaudRatePrescaler(&drv_can->can, 23);
		XCan_SetBitTiming(&drv_can->can, 0, 3, 4);
		break;
	case 50000:    // 50k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 50k sucessfull \r\n");// 50k
		XCan_SetBaudRatePrescaler(&drv_can->can, 47);
		XCan_SetBitTiming(&drv_can->can, 0, 3, 4);
		break;
	case 20000:    //20k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 20k sucessfull \r\n");// 20k
		XCan_SetBaudRatePrescaler(&drv_can->can, 119);
		XCan_SetBitTiming(&drv_can->can, 0, 3, 4);
		break;
	case 10000:    // 10k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 10k sucessfull \r\n");// 10k
		XCan_SetBaudRatePrescaler(&drv_can->can, 239);
		XCan_SetBitTiming(&drv_can->can, 0, 3, 4);
		break;
	case 5000:     // 5k
		CAN_DBG(CAN_DBG_DEBUG, " CAN_SetBaudRate set 5k sucessfull \r\n");// 5k
		XCan_SetBaudRatePrescaler(&drv_can->can, 239);
		XCan_SetBitTiming(&drv_can->can, 0, 3, 14);
		break;
	default:
		XCan_SetBaudRatePrescaler(&drv_can->can, 2);
		XCan_SetBitTiming(&drv_can->can, 0, 4, 9);
		break;
	}
}

static void event_handler(void* CallBackRef, u32 IntrMask)
{
	can_drv_t* CanPtr = (can_drv_t*)CallBackRef;

	if (IntrMask & XCAN_IXR_BSOFF_MASK) { /* Enter Bus off status */
		/*
		 * Entering Bus off status interrupt requires
		 * the CAN device be reset and re-configurated.
		 */
		XCan_Reset(&CanPtr->can);
		axi_can_config(CanPtr, CanPtr->baud);

		return;
	}

	if (IntrMask & XCAN_IXR_RXOFLW_MASK) { /* RX FIFO Overflow Interrupt */
		/*
		 * Code to handle RX FIFO Overflow
		 * Interrupt should be put here.
		 */
		CanPtr->can_event.ixr_rxoflw_count += 1;
	}

	if (IntrMask & XCAN_IXR_RXUFLW_MASK) { /* RX FIFO Underflow Interrupt */
		/*
		 * Code to handle RX FIFO Underflow
		 * Interrupt should be put here.
		 */
		CanPtr->can_event.ixr_rxuflw_count += 1;
	}

	if (IntrMask & XCAN_IXR_TXBFLL_MASK) { /* TX High Priority Full Intr */
		/*
		 * Code to handle TX High Priority Buffer Full
		 * Interrupt should be put here.
		 */
		CanPtr->can_event.ixr_txbfll_count += 1;
	}

	if (IntrMask & XCAN_IXR_TXFLL_MASK) { /* TX FIFO Full Interrupt */
		/*
		 * Code to handle TX FIFO Full
		 * Interrupt should be put here.
		 */
		CanPtr->can_event.ixr_txfll_count += 1;
	}

	if (IntrMask & XCAN_IXR_WKUP_MASK) { /* Wake up from sleep mode */
		/*
		 * Code to handle Wake up from sleep mode
		 * Interrupt should be put here.
		 */
		CanPtr->can_event.ixr_wkup_count += 1;
	}

	if (IntrMask & XCAN_IXR_SLP_MASK) { /* Enter sleep mode */
		/*
		 * Code to handle Enter sleep mode
		 * Interrupt should be put here.
		 */
		CanPtr->can_event.ixr_slp_count += 1;
	}

	if (IntrMask & XCAN_IXR_ARBLST_MASK) { /* Lost bus arbitration */

		/*
		 * Code to handle Lost bus arbitration
		 * Interrupt should be put here.
		 */
		CanPtr->can_event.ixr_arblst_count += 1;
	}

	return;
}
static void error_handler(void* CallBackRef, u32 ErrorMask)
{
	can_drv_t* CanPtr = (can_drv_t*)CallBackRef;

	if (ErrorMask & XCAN_ESR_ACKER_MASK) {
		/*
		 * ACK Error handling code should be put here.
		 */
		CanPtr->can_error.esr_acker_count += 1;
	}

	if (ErrorMask & XCAN_ESR_BERR_MASK) {
		/*
		 * Bit Error handling code should be put here.
		 */
		CanPtr->can_error.esr_berr_count += 1;
	}

	if (ErrorMask & XCAN_ESR_STER_MASK) {
		/*
		 * Stuff Error handling code should be put here.
		 */
		CanPtr->can_error.esr_ster_count += 1;
	}

	if (ErrorMask & XCAN_ESR_FMER_MASK) {
		/*
		 * Form Error handling code should be put here.
		 */
		CanPtr->can_error.esr_fmer_count += 1;
	}

	if (ErrorMask & XCAN_ESR_CRCER_MASK) {
		/*
		 * CRC Error handling code should be put here.
		 */
		CanPtr->can_error.esr_crcer_count += 1;
	}
	return;
}


void axi_can_set_mode(can_drv_t* can_drv, u8 Mode)
{
	XCan_EnterMode(&can_drv->can, Mode);
	while (XCan_GetMode(&can_drv->can) != Mode);
}


int axi_can_init(can_drv_t* pcan_drv)
{
	int Status;
	XCan* can;

	if (pcan_drv == NULL) {
		CAN_DBG(CAN_DBG_ERROR, "CAN pcan_drv error\r\n");
		return 1;
	}

	can = &pcan_drv->can;

	Status = XCan_Initialize(can, pcan_drv->can_id);
	if (Status != XST_SUCCESS) {
		CAN_DBG(CAN_DBG_ERROR, "CAN  0 Initialization Failed\r\n");
		return Status;
	}

	Status = XCan_SelfTest(can);
	if (Status != XST_SUCCESS) {
		//CAN_DBG(CAN_DBG_ERROR,"CAN Self-Test Failed\r\n");
		return Status;
	}

	axi_can_config(pcan_drv, pcan_drv->baud);

	XCan_SetHandler(can, XCAN_HANDLER_SEND,
		(void*)send_handler, (void*)pcan_drv);
	XCan_SetHandler(can, XCAN_HANDLER_RECV,
		(void*)recv_handler, (void*)pcan_drv);
	XCan_SetHandler(can, XCAN_HANDLER_ERROR,
		(void*)error_handler, (void*)pcan_drv);
	XCan_SetHandler(can, XCAN_HANDLER_EVENT,
		(void*)event_handler, (void*)pcan_drv);

	/****************************************/
	axi_can_set_mode(pcan_drv, XCAN_MODE_NORMAL);
	XCan_InterruptEnable(can, XCAN_IXR_ALL);

	pcan_drv->rb = ringbuffer_create(CAN_RESPONSE_FRAME_RB_COUNT_MAX * sizeof(can_frame_info_t), sizeof(can_frame_info_t));
	if (!pcan_drv->rb) {
		CAN_DBG(CAN_DBG_ERROR, "can%d tx buffer error \r\n", pcan_drv->can_chn_id);
		return 2;
	}
	ringbuffer_reset(pcan_drv->rb);

	CAN_DBG(CAN_DBG_DEBUG, "##### chn_id:%d-fifo_base_address:0x%x ##### \r\n", pcan_drv->can_chn_id, pcan_drv->axis_fifo_base_address);
	axis_fifo_reset(pcan_drv->axis_fifo_base_address);

	return XST_SUCCESS;
}
