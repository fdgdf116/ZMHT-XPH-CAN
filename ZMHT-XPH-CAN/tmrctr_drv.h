#ifndef _TMRCTR_DRV_H_
#define _TMRCTR_DRV_H_

#include "xstatus.h"
#include "xil_types.h"

int tmrctr_drv_init(void);
u32 tmrctr_drv_get_ticks(void);

#endif
