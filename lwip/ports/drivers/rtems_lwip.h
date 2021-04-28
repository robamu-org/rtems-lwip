/*
 * Copyright (c) 2013, 2015 Czech Technical University in Prague
 * Czech Republic
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * Author: Premysl Houdek <houdepre@fel.cvut.cz>
 * Mentor: Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * Industrial Informatics Group, FEE, Czech Technical University in Prague
 *
 * Based on work of Carlos Jenkins, Rostislav Lisovy, Jan Dolezal
 */

#ifndef __ETH_LWIP_H
#define __ETH_LWIP_H

#include "lwip/netif.h"
#include "rtems_lwip_conf.h"

#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * LwIP netif couldn't be added, it is likely that there was an error during initialization of the hardware.
 */
#define NETIF_ADD_ERR            -10 /* could be one of previous, except PHY_LINK_DOWN - currently */

/**
 * @brief   RTEMS lwIP initialization function.
 *
 * Call this method before using this module. This function also takes care of initializing
 * the lwIP stack by calling lwip_init.
 * @param   mac_addr            Can be set to NULL to use default mac address
 * @param   netif_status_cb     Callback function which will be called if the network
 *                              link status changes
 * @return
 *  - SUCCESS if initialization successful.\n
 *  - NETIF_ADD_ERR Error adding net interface.
 */
int8_t rtems_lwip_init(uint8_t *mac_addr, netif_status_callback_fn netif_status_cb);

/**
 * @brief   Access to the net interface instances
 */
struct netif *rtems_lwip_get_netif(uint32_t instance_number);

/**
 * @brief   Print information about the assigned DHCP address.
 */
void rtems_lwip_print_dhcp_info(void);

/**
 * Convert the given IP to its string representation
 * @param ip
 * @param ip_str
 */
void rtems_lwip_convert_ip_to_decimal_str(ip_addr_t ip, char *ip_str);

/**
 * Get the netif status by calling the lwIP  stats_display function
 * @param argc
 * @param arg
 * @return
 */
int rtems_lwip_get_netif_status_cmd(int argc, char *arg[]);

/**
 * Set the hardware (MAC) address
 * @param netif
 * @param mac_addr
 */
void rtems_lwip_set_hwaddr(struct netif *netif, uint8_t *mac_addr);

/**
 * Get the hardware (MAC) address
 * @param netif
 * @param mac_addr
 */
void rtems_lwip_get_hwaddr_str(struct netif *netif, uint8_t *mac_str);

/**
 * @brief   Determines a static IP address from the configuration files.
 * @param ip_addr
 * @param netmask
 * @param gw
 */
void rtems_lwip_determine_static_ipv4_address(ip4_addr_t* ip_addr, ip4_addr_t* netmask,
        ip4_addr_t* gw);

#ifdef __cplusplus
}
#endif

#endif /* __ETH_LWIP_H */
