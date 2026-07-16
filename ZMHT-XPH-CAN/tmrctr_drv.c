#include "tmrctr_drv.h"

#include "xparameters.h"
#include "xtmrctr.h"
#include "xutils.h"
#include "xintc_drv.h"

#define TMRCTR_DEVICE_ID    XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID XPAR_INTC_0_TMRCTR_0_VEC_ID
#define TMRCTR_NUMBER       0U
/* 150MHz clock, 50us period: N = 150000000 * 50e-6 = 7500 counts */
#define TMRCTR_RESET_VALUE  0xFFFFE2B4U

static XTmrCtr timer_inst;
static volatile u32 timer_ticks;

static void timer_counter_handler(void *CallBackRef, u8 TmrCtrNumber)
{
    XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

    if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber)) {
        timer_ticks++;
    }
}

int tmrctr_drv_init(void)
{
    int status;

    timer_ticks = 0U;

    status = XTmrCtr_Initialize(&timer_inst, TMRCTR_DEVICE_ID);
    if (status != XST_SUCCESS) {
        CAN_DBG(CAN_DBG_ERROR, "tmrctr init failed\r\n");
        return XST_FAILURE;
    }

    status = XTmrCtr_SelfTest(&timer_inst, TMRCTR_NUMBER);
    if (status != XST_SUCCESS) {
        CAN_DBG(CAN_DBG_ERROR, "tmrctr self test failed\r\n");
        return XST_FAILURE;
    }

    XTmrCtr_SetHandler(&timer_inst, timer_counter_handler, &timer_inst);

    status = xintc_drv_connect((u8)TMRCTR_INTERRUPT_ID,
                               (XInterruptHandler)XTmrCtr_InterruptHandler,
                               &timer_inst);
    if (status != XST_SUCCESS) {
        CAN_DBG(CAN_DBG_ERROR, "tmrctr int connect failed\r\n");
        return XST_FAILURE;
    }

    XTmrCtr_SetOptions(&timer_inst,
                       TMRCTR_NUMBER,
                       XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);
    XTmrCtr_SetResetValue(&timer_inst, TMRCTR_NUMBER, TMRCTR_RESET_VALUE);
    XTmrCtr_Start(&timer_inst, TMRCTR_NUMBER);

    CAN_DBG(CAN_DBG_DEBUG, "tmrctr init success\r\n");
    return XST_SUCCESS;
}

u32 tmrctr_drv_get_ticks(void)
{
    return timer_ticks;
}
