/**
 * Copyright (c) 2022 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#ifndef _lwipopts_h
#define _lwipopts_h

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Task config */
#include "task_cfg.h"

/* UMM malloc */
#include "umm_malloc.h"

/* Thread defines */
#define TCPIP_MBOX_SIZE                 128
#define DEFAULT_THREAD_STACKSIZE        (ETHERNET_TASK_STACK_SIZE)
#define DEFAULT_THREAD_PRIO             (ETHER_WORKER_PRIO)
#define DEFAULT_RAW_RECVMBOX_SIZE       10
#define DEFAULT_UDP_RECVMBOX_SIZE       10
#define DEFAULT_TCP_RECVMBOX_SIZE       64
#define DEFAULT_ACCEPTMBOX_SIZE         10

/* Enable APIs */
#define LWIP_NETIF_API                  1
#define LWIP_RAW                        0

/* Enable DHCP */
#define LWIP_DHCP                       1
#define LWIP_DHCP_CHECK_LINK_UP         1
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_STATUS_CALLBACK      1

/* Enable link up/dn callbacks */
#define LWIP_NETIF_LINK_CALLBACK        1

/* API configs */
#define MEMP_NUM_NETCONN                10
#define LWIP_SO_RCVTIMEO                1

/* Socket options */
#define SO_REUSE                        1

/* Heap configs */
#define MEM_LIBC_MALLOC                 1
#define mem_clib_malloc                 umm_malloc
#define mem_clib_free                   umm_free
#define mem_clib_calloc                 umm_calloc
#define MEMP_MEM_MALLOC                 1

/* Misc configs */
#define LWIP_STATS_DISPLAY              1
#define LWIP_STATS_LARGE                1

// if this is left to default 60s, we will need many TCP PCBs
// this ensures they die quickly
// http://savannah.nongnu.org/bugs/?49372
#define TCP_MSL 100UL /* The maximum segment lifetime in milliseconds */

// IPERF TCP tuning

#define TCP_MSS                        1460
#define TCP_WND                        (44 * TCP_MSS)

// Packet checksum offload

#define CHECKSUM_CHECK_IP               0
#define CHECKSUM_CHECK_UDP              0
#define CHECKSUM_CHECK_TCP              0
#define CHECKSUM_CHECK_ICMP             0

#define CHECKSUM_GEN_IP                 0
#define CHECKSUM_GEN_UDP                0
#define CHECKSUM_GEN_TCP                0
#define CHECKSUM_GEN_ICMP               0

// Ethernet 32-bit alignment

#if defined(__ADSPSC598_FAMILY__)
#define ETH_PAD_SIZE                    0
#else
#define ETH_PAD_SIZE                    2
#endif

/* TFTP options */
#define TFTP_MAX_FILENAME_LEN           30

/* mDNS options */
#define LWIP_MDNS_RESPONDER             1
#define LWIP_IGMP                       1
#define MEMP_NUM_UDP_PCB                10
#define LWIP_NUM_NETIF_CLIENT_DATA      5

/* Debug options */
#define LWIP_PLATFORM_DIAG(x)

#endif
