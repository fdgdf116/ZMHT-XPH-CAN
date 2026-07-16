#ifndef _XINTC_DRV_H_
#define _XINTC_DRV_H_

#include "xintc.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xil_exception.h"
#include "xil_printf.h"

int xintc_drv_init(void);
int xintc_drv_connect(u8 Id, XInterruptHandler Handler, void* CallBackRef);
void xintc_drv_start(void);

#endif
