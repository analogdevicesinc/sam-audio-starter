/**
 * Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <stdint.h>
#include <builtins.h>

#include "sae_lock.h"
#include "sae_ipc.h"

#if defined(__ADSPARM__)

bool sae_lock(volatile uint32_t *lock)
{
    __sync_synchronize();
#if defined(__ADSPCORTEXA55__)
    asm volatile(
        "    mov     w2, #1\n\t"
        "l1: ldaxr   w1, [%[lock]]\n\t"
        "    cbnz    w1, l1\n\t"
        "    stxr    w1, w2, [%[lock]]\n\t"
        "    cbnz    w1, l1\n\t"
        : /* No output operands */
        : [lock] "r" (lock)
        : "w2", "w1"
    );
    return(true);
#else
    uint32_t tmp = SAE_SHARC_ARM_IPC_UNLOCKED;
    return __atomic_compare_exchange_n(lock, &tmp, SAE_SHARC_ARM_IPC_LOCKED,
            false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
#endif
}

bool sae_unlock(volatile uint32_t *lock)
{
#if defined(__ADSPCORTEXA55__)
    asm volatile(
        "    stlr    wzr, [%[lock]]\n\t"
        : /* No output operands */
        : [lock] "r" (lock)
        : /* No clobber list */
    );
#else
    __atomic_store_n(lock, SAE_SHARC_ARM_IPC_UNLOCKED, __ATOMIC_SEQ_CST);
#endif
    __sync_synchronize();
    return(true);
}

#else  // __ADSPARM__

bool sae_lock(volatile uint32_t *lock)
{
    int err;
    uint32_t locked;
    bool ok = false;

    asm volatile ("SYNC;");
    locked = load_exclusive_32(lock, &err);
    if ((err == 0) && (locked == SAE_SHARC_ARM_IPC_UNLOCKED)) {
        err = store_exclusive_32(SAE_SHARC_ARM_IPC_LOCKED, lock);
        ok = (err == 0);
    }

    return(ok);
}

bool sae_unlock(volatile uint32_t *lock)
{
    int err;
    load_exclusive_32(lock, &err);
    err = store_exclusive_32(SAE_SHARC_ARM_IPC_UNLOCKED, lock);
    asm volatile ("SYNC;");
    return(err == 0);
}

#endif
