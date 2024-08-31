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

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
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
#include "lwip/apps/mdns.h"

/* ADI Ethernet lwIP netif includes */
#include "lwip_adi_ether_netif.h"

/* Application includes */
#include "context.h"
#include "util.h"

/***********************************************************************
 * Features
 **********************************************************************/

/***********************************************************************
 * Ethernet variables
 **********************************************************************/

/***********************************************************************
 * Forward declarations
 **********************************************************************/

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

static void log_mac(char *name, uint8_t *macAddr)
{
    syslog_printf(
        "%s: %02x:%02x:%02x:%02x:%02x:%02x", name,
        macAddr[0], macAddr[1], macAddr[2],
        macAddr[3], macAddr[4], macAddr[5]);
}

#if LWIP_NETIF_STATUS_CALLBACK
static void netif_status_callback(struct netif *netif)
{
    struct adi_ether_netif *adi_ether = netif->state;
    APP_CONTEXT *context = (APP_CONTEXT *)adi_ether->usrPtr;

    UNUSED(context);

    /* Make sure there's a sane address */
    if (ip_addr_get_ip4_u32(&netif->ip_addr) == 0x00000000) {
        return;
    }

    /* Tell mDNS */
    mdns_resp_netif_settings_changed(netif);

    /* Log the settings */
    syslog_printf(
        "Ethernet %c%c%u:",
        netif->name[0], netif->name[1], netif->num
    );
    syslog_printf("  Hostname: %s", adi_ether->hostName);
    log_mac("  MAC", adi_ether->ethAddr.addr);
    log_addr("  IP Address", &netif->ip_addr);
    log_addr("  Gateway", &netif->gw);
    log_addr("  Netmask", &netif->netmask);
}
#endif

#if LWIP_NETIF_LINK_CALLBACK
static void netif_link_status_callback(struct netif *netif)
{
    struct adi_ether_netif *adi_ether = netif->state;
    APP_CONTEXT *context = (APP_CONTEXT *)adi_ether->usrPtr;

    UNUSED(context);

    syslog_printf(
        "Ethernet %c%c%u link %s\n",
        netif->name[0], netif->name[1], netif->num,
        netif_is_link_up(netif) ? "up" : "down"
    );
}
#endif

/***********************************************************************
 * MAC Address generation
 **********************************************************************/
#include "flash.h"
#include "crc16.h"
void makeMACAddr(APP_CONTEXT *context, uint8_t *macAddr)
{
    static unsigned int seed = 0;
    unsigned int s;
    int i;

    /* Generate a random seed from the flash UID */
    if (seed == 0) {
        seed = crc16_ccitt(
            context->flashHandle->UID, sizeof(context->flashHandle->UID)
        );
    }

    /*
     * Generate a repeatable random MAC address.  It will also be used
     * for mDNS IP address so limit each octet to 254
     */
    s = seed; seed++;
    for (i = 0; i < 6; i++) {
        macAddr[i] = (rand_r(&s) & 0xFF) % 254;
    }

    /* https://en.wikipedia.org/wiki/MAC_address */

    /* Set as locally administered */
    macAddr[0] |= 0x02;

    /* Set as unicast */
    macAddr[0] &= ~0x01;
}

/***********************************************************************
 * lwIP init
 **********************************************************************/
void ethernet_init(APP_CONTEXT *context, ETH *eth, ETH_CFG *cfg)
{
    ip_addr_t ip_addr, gw_addr, netmask;
    static bool first = true;
    char mdns_ip_addr[32];

    /* Initialize lwIP and the TCP/IP subsystem */
    if (first) {
        tcpip_init(NULL, NULL);
    }

    /* Generate a unique MAC address and hostname */
    makeMACAddr(context, eth->macaddr);

    sprintf(eth->hostname,
        "%s-%02X%02X%02X%02X%02X%02X", cfg->base_hostname,
        eth->macaddr[0], eth->macaddr[1], eth->macaddr[2],
        eth->macaddr[3], eth->macaddr[4], eth->macaddr[5]
    );

    /* Set IP addresses */
    if (cfg->static_ip) {
        net_aton((char *)cfg->ip_addr, &ip_addr);
        net_aton((char *)cfg->gateway_addr, &gw_addr);
        net_aton((char *)cfg->netmask, &netmask);
    } else {
        /* Set initial mDNS IP/GW/MASK address if not static */
        sprintf(mdns_ip_addr,
            "169.254.%d.%d", eth->macaddr[4], eth->macaddr[5]
        );
        net_aton(mdns_ip_addr, &ip_addr);
        net_aton("0.0.0.0", &gw_addr);
        net_aton("255.255.0.0", &netmask);
    }

    /* Allocate a new ADI Ethernet network interface */
    eth->adi_ether = adi_ether_netif_new(context);

    /* Configure the user parameters (hostname, EMAC port) */
    eth->adi_ether->hostName = eth->hostname;
    eth->adi_ether->port = cfg->port;
    memcpy(eth->adi_ether->ethAddr.addr, eth->macaddr,
        sizeof(eth->adi_ether->ethAddr.addr));

    /* Add the network interface */
    netif_add(&eth->netif,
        &ip_addr, &netmask, &gw_addr,
        eth->adi_ether, adi_ether_netif_init, tcpip_input);

    /* Set this interface as the default interface */
    if (cfg->default_iface) {
        netif_set_default(&eth->netif);
    }

#if LWIP_NETIF_STATUS_CALLBACK
    /* Register a callback for DHCP address complete */
    netif_set_status_callback(&eth->netif, netif_status_callback);
#else
    #error "Must enable LWIP_NETIF_STATUS_CALLBACK"
#endif

#if LWIP_NETIF_LINK_CALLBACK
    /* Register a callback for link status */
    netif_set_link_callback(&eth->netif, netif_link_status_callback);
#else
    #error "Must enable LWIP_NETIF_LINK_CALLBACK"
#endif

    /* Set interface to up */
    netif_set_up(&eth->netif);

    /* Enable mDNS */
    if (first) {
        mdns_resp_init();
    }
    mdns_resp_add_netif(&eth->netif, eth->hostname, 60);

    /* Enable DHCP */
    if (cfg->static_ip == false) {
        dhcp_start(&eth->netif);
    }

    /* Start telnet server */
    if (first) {
        telnet_start(context);
    }

    /* Log init complete */
    syslog_printf(
        "Ethernet %c%c%u init complete\n",
        eth->netif.name[0], eth->netif.name[1], eth->netif.num
    );

    first = false;
}
