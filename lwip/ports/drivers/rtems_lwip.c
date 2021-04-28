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

#include "rtems_lwip.h"
#include "rtems_lwip_conf.h"

#include "lwip/init.h"
#include "lwip/stats.h"
#include "lwip/dhcp.h"
#include "netif/ethernet.h"

#if LWIP_NETIF_API == 1
#include "lwip/netifapi.h"
#endif

#if NO_SYS == 0
#include "lwip/tcpip.h"
#endif

#include <stdio.h>

#define SUCCESS ERR_OK
#define FAILURE ERR_IF

/* Assigned in specific port */
extern netif_init_fn eth_lwip_init_fnc;

/* The lwIP network interface structure for the Ethernet EMAC. */
#ifndef MAX_EMAC_INSTANCE
#define MAX_EMAC_INSTANCE           1
#endif /*MAX_EMAC_INSTANCE*/

struct netif eth_lwip_netifs[MAX_EMAC_INSTANCE];

int8_t
rtems_lwip_init(uint8_t *mac_addr, netif_status_callback_fn netif_status_cb)
{
  unsigned int instance_number = 0;
  int8_t retval = SUCCESS;

#if NO_SYS == 1
  /* Initialize the lwIP stack */
  lwip_init();
#endif

  ip4_addr_t ip_addr;
  ip4_addr_t net_mask;
  ip4_addr_t gw_addr;
  struct netif *netif = &eth_lwip_netifs[instance_number];
  struct netif *netif_tmp;
  u8_t default_mac[MAC_ADDR_LEN] = ETH_MAC_ADDR;

  if (mac_addr == NULL)
    mac_addr = default_mac;             /* use default MAC */

  rtems_lwip_set_hwaddr(netif, mac_addr);

#if NO_SYS == 0
  tcpip_init(NULL, NULL);
#endif

  rtems_lwip_determine_static_ipv4_address(&ip_addr, &net_mask, &gw_addr);

  netif_input_fn netif_input_fnc = NULL;
#if NO_SYS == 1
  netif_input_fnc = &ethernet_input;
#else
  netif_input_fnc = &tcpip_input;
#endif

  netif_tmp = netif_add(netif, &ip_addr, &net_mask, &gw_addr,
                        NULL, eth_lwip_init_fnc, netif_input_fnc);

  if (netif_tmp == NULL)
    return NETIF_ADD_ERR;

  netif_set_default(netif);

#if LWIP_NETIF_API == 1
  netifapi_netif_set_up(netif);
#endif

#if LWIP_NETIF_LINK_CALLBACK
  if(netif_status_cb != NULL) {
    netif_status_cb(netif);
    netif_set_link_callback(netif, netif_status_cb);
  }
#endif

#if !STATIC_IP_ADDRESS && NO_SYS == 0 && LWIP_NETIF_API == 1
  netifapi_dhcp_start(netif);
#endif

  return retval;
}

void
rtems_lwip_print_dhcp_info(void)
{
  struct netif *netif = rtems_lwip_get_netif(0);

  if (dhcp_supplied_address(netif)) {
    printf("DHCP information: \n\r");
    printf("Address: %s\n\r", ip4addr_ntoa(netif_ip4_addr(netif)));
    printf("Netmask: %s\n\r", ip4addr_ntoa(netif_ip4_netmask(netif)));
    printf("Gateway: %s\n\r", ip4addr_ntoa(netif_ip4_gw(netif)));
  } else {
    printf("DHCP not bound\n\r");
  }
}

int
rtems_lwip_get_netif_status_cmd(int argc, char *arg[])
{
  stats_display();
  return 0;
}

struct netif *
rtems_lwip_get_netif(uint32_t instance_number)
{
  if (instance_number >= MAX_EMAC_INSTANCE) {
    return NULL;
  }
  return &eth_lwip_netifs[instance_number];
}

/*
* Function to set the MAC address to the interface
* @param   inst_num the instance number
*
* @note    mac_addr[0] is considered MSB
*/
void
rtems_lwip_set_hwaddr(struct netif *netif, uint8_t *mac_addr)
{
  int i;

  /* set MAC hardware address */
  for (i = 0; i < MAC_ADDR_LEN; i++) {
    netif->hwaddr[i] = mac_addr[i];
  }
  netif->hwaddr_len = MAC_ADDR_LEN;

#ifdef DEBUG
  uint8_t macStr[18];
  eth_lwip_get_hwaddr_str(netif, macStr);
  printf("Setting MAC... %s\r\n", macStr);
#endif
}

void
rtems_lwip_get_hwaddr_str(struct netif *netif, uint8_t *mac_str)
{
  uint8_t index, outindex = 0;
  char ch;

  for (index = 0; index < netif->hwaddr_len; index++) {
    if (index) {
      mac_str[outindex++] = ':';
    }
    ch = (netif->hwaddr[index] >> 4);
    mac_str[outindex++] = (ch < 10) ? (ch + '0') : (ch - 10 + 'A');
    ch = (netif->hwaddr[index] & 0xf);
    mac_str[outindex++] = (ch < 10) ? (ch + '0') : (ch - 10 + 'A');
  }
  mac_str[outindex] = 0;
}

void
rtems_lwip_convert_ip_to_decimal_str(ip_addr_t ip, char *ip_str)
{
  uint32_t addr;
 #if LWIP_IPV6
  addr = ip.u_addr.ip4.addr;
 #else
  addr = ip.addr;
 #endif

  snprintf((char *)ip_str, 16, "%lu.%lu.%lu.%lu",
           (addr >> 24), ((addr >> 16) & 0xff), ((addr >> 8) & 0xff), (addr & 0xff));
}

void rtems_lwip_determine_static_ipv4_address(ip4_addr_t* ip_addr, ip4_addr_t* netmask,
        ip4_addr_t* gw_addr) {
#if STATIC_IP_ADDRESS == 1

#if CALCULATE_ETH_IP_ADDR == 1
  IP_ADDR4(ip_addr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
#else
  ip_addr->addr = htonl(ETH_IP_ADDR);
#endif

#if CALCULATE_ETH_NETMASK == 1
  IP_ADDR4(net_mask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
#else
  net_mask->addr = htonl(ETH_NETMASK);
#endif

#if CALCULATE_GW_ADDRESS == 1
  IP_ADDR4(gw_addr, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#else
  gw_addr->addr = htonl(ETH_GW);
#endif

#else
  ip_addr_set_zero_ip4(ip_addr);
  ip_addr_set_zero_ip4(netmask);
  ip_addr_set_zero_ip4(gw_addr);
#endif /* STATIC_IP_ADDRESS == 0 */
}
