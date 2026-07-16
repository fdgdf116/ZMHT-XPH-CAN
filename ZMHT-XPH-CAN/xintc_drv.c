#include"xintc_drv.h"
#include"xutils.h"

#define TMRCTR_INTERRUPT_ID	XPAR_INTC_0_TMRCTR_0_VEC_ID
/*定义了中断控制器的设备 ID，用于初始化中断控制器*/
#define DEVID_INTC0 XPAR_INTC_0_DEVICE_ID

static XIntc interrupt_controller;

int xintc_drv_init(void)
{
	int status;

	status = XIntc_Initialize(&interrupt_controller, DEVID_INTC0);
	if (status != XST_SUCCESS) {
		CAN_DBG(CAN_DBG_ERROR, "xintc inittialize error \r\n");
		return XST_FAILURE;
	}
	//	status = XIntc_SelfTest(&interrupt_controller);
	//	if (status != XST_SUCCESS) {
	//		CAN_DBG(CAN_DBG_ERROR,"xintc self test error \r\n");
	//		return XST_FAILURE;
	//	}

	return 0;
}

int xintc_drv_connect(u8 Id, XInterruptHandler Handler, void* CallBackRef)
{
	int status;
	status = XIntc_Connect(&interrupt_controller, Id, Handler, CallBackRef);
	if (status != XST_SUCCESS) {

		return XST_FAILURE;
	}
	XIntc_Enable(&interrupt_controller, Id);
	return 0;
}

void xintc_drv_start(void)
{
	/**/
	XIntc_Start(&interrupt_controller, XIN_REAL_MODE);
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XIntc_InterruptHandler, &interrupt_controller);
	Xil_ExceptionEnable();
}
