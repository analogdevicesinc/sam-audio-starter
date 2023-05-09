/**
 * Copyright (c) 2021 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* Simple service includes */
#include "syslog.h"
#include "telnet.h"

/* LwIP includes */
#include "lwip/inet.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "netif/etharp.h"

/* ADI Ethernet lwIP netif includes */
#include "lwip_adi_ether_netif.h"

/* Application includes */
#include "context.h"
#include "util.h"

/***********************************************************************
 * Ethernet variables
 **********************************************************************/
static uint8_t macaddr[6] = { 0x02, 0x04, 0x06, 0x08, 0x10, 0x12 };
static char hostname[32];

/***********************************************************************
 * Misc variables
 **********************************************************************/

/* This function converts a string to a network address struct */
static int net_aton(char *str_addr, ip_addr_t *net_addr)
{
    int i = inet_aton(str_addr, &net_addr->addr);
    if (!i)
        return -1;
    return 0;
}

/* This function prints a network address */
static void log_addr(char *name, ip_addr_t *net_addr)
{
    syslog_printf("%s: %d.%d.%d.%d\n", name,
        ip4_addr1(net_addr), ip4_addr2(net_addr),
        ip4_addr3(net_addr), ip4_addr4(net_addr));
}

static void ethernet_ready(APP_CONTEXT *context, ip_addr_t *ip_addr)
{
    ip4addr_ntoa_r(ip_addr, context->eth0_addr, sizeof(context->eth0_addr));
    xEventGroupSetBits(context->ethernetEvents, ETHERNET_READY);
}

#if LWIP_NETIF_STATUS_CALLBACK
static void netif_status_callback(struct netif *netif)
{
    struct adi_ether_netif *adi_ether = netif->state;
    APP_CONTEXT *context = (APP_CONTEXT *)adi_ether->usrPtr;

    /* Make sure there's a sane address */
    if (ip_addr_get_ip4_u32(&netif->ip_addr) == 0x00000000) {
        return;
    }
    log_addr("IP Address", &netif->ip_addr);
    log_addr("Gateway", &netif->gw);
    log_addr("Netmask", &netif->netmask);

    /* Now that we have an IP address, start Ethernet related tasks if
     * DHCP is enabled
     */
    if (context->cfg.static_ip == false) {
        ethernet_ready(context, &netif->ip_addr);
    }
}
#endif

#if LWIP_NETIF_LINK_CALLBACK
static void netif_link_status_callback(struct netif *netif)
{
    struct adi_ether_netif *adi_ether = netif->state;
    APP_CONTEXT *context = (APP_CONTEXT *)adi_ether->usrPtr;

    if (netif->flags & NETIF_FLAG_LINK_UP) {
        /*
         * Start Ethernet related tasks as soon as the link is up with
         * a static IP.
         */
        xEventGroupSetBits(context->ethernetEvents, ETHERNET_LINK_UP);
        if (context->cfg.static_ip == true) {
            ethernet_ready(context, &netif->ip_addr);
        }
        syslog_print("Ethernet link up\n");
    } else {
        syslog_print("Ethernet link down\n");
    }
}
#endif

/***********************************************************************
 * MAC Address generation
 **********************************************************************/
void makeMACAddr(APP_CONTEXT *context, uint8_t *macAddr)
{
    /*
     * Do something clever here to generate a unique MAC address
     */

    /* https://en.wikipedia.org/wiki/MAC_address */

    /* Set as locally administered */
    macAddr[0] |= 0x02;

    /* Set as unicast */
    macAddr[0] &= ~0x01;

    /* Log to the syslog */
    syslog_printf(
        "Ethernet MAC: %02x:%02x:%02x:%02x:%02x:%02x",
        macAddr[0], macAddr[1], macAddr[2],
        macAddr[3], macAddr[4], macAddr[5]
    );

}

/***********************************************************************
 * lwIP init
 **********************************************************************/
void ethernet_init(APP_CONTEXT *context)
{
    ip_addr_t sam_ip_addr, gw_addr, netmask;

    /* Set IP addresses */
    net_aton((char *)context->cfg.ip_addr, &sam_ip_addr);
    net_aton((char *)context->cfg.gateway_addr, &gw_addr);
    net_aton((char *)context->cfg.netmask, &netmask);

    /* Initialize lwIP and the TCP/IP subsystem */
    tcpip_init(NULL, NULL);

    /* Allocate a new ADI Ethernet network interface */
    context->adi_ether = adi_ether_netif_new(context);

    /* Generate a unique MAC address */
    makeMACAddr(context, macaddr);

    /* Configure the user parameters (hostname, EMAC port, MAC address) */
    sprintf(hostname, "%s-%02X%02X%02X%02X%02X%02X",
        "AK",
        macaddr[0], macaddr[1], macaddr[2],
        macaddr[3], macaddr[4], macaddr[5]
    );
    context->adi_ether->hostName = hostname;
    context->adi_ether->port = EMAC0;
    memcpy(context->adi_ether->ethAddr.addr, macaddr,
        sizeof(context->adi_ether->ethAddr.addr));

    /* Add the network interface */
    netif_add(&context->eth0_netif,
        &sam_ip_addr, &netmask, &gw_addr,
        context->adi_ether, adi_ether_netif_init, tcpip_input);

    /* Set this interface as the default interface */
    netif_set_default(&context->eth0_netif);

#if LWIP_NETIF_STATUS_CALLBACK
    /* Register a callback for DHCP address complete */
    netif_set_status_callback(&context->eth0_netif, netif_status_callback);
#else
    #error "Must enable LWIP_NETIF_STATUS_CALLBACK"
#endif

#if LWIP_NETIF_LINK_CALLBACK
    /* Register a callback for link status */
    netif_set_link_callback(&context->eth0_netif, netif_link_status_callback);
#else
    #error "Must enable LWIP_NETIF_LINK_CALLBACK"
#endif

    /* Set interface to up */
    netif_set_up(&context->eth0_netif);

    /* Enable DHCP */
    if (context->cfg.static_ip == false) {
        dhcp_start(&context->eth0_netif);
    }

    /* Start telnet server */
    telnet_start();

    syslog_print("Ethernet init complete");
}
