/**
 * Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <runtime/int/interrupt.h>

/*
 * To debug exceptions, set a breakpoint in this function,
 * set EXCEPTION_WAIT to 0 in the debugger, then assembly single
 * step out of the exception handler and back to the application
 * where you can inspect the call stack.
 */
void _exception(uint32_t eid, void *usr)
{
    static volatile bool EXCEPTION_WAIT = true;
    adi_rtl_disable_interrupts();
    while (EXCEPTION_WAIT) {
        asm("nop");
    }
}

void exception_init(void)
{
    /*
     * Install exception handlers.  Use the RTL function to bypass OSAL IRQ
     * priority manipulation which fails for exception interrupts.
     */
#if defined(__ADSPCORTEXA5__)
    adi_rtl_register_dispatched_handler(ADI_RTL_XID_ABORT_DATA, _exception, NULL);
    adi_rtl_register_dispatched_handler(ADI_RTL_XID_ABORT_PREFETCH, _exception, NULL);
#elif defined(__ADSPCORTEXA55__)
    adi_rtl_register_dispatched_handler(ADI_RTL_EXCEPT_SYNC, _exception, NULL);
    adi_rtl_register_dispatched_handler(ADI_RTL_EXCEPT_SERR, _exception, NULL);
#endif
}
