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

//#define DEBUG 1
#include "lwip/tcpip.h" /* includes - lwip/opt.h, lwip/api_msg.h, lwip/netifapi.h, lwip/pbuf.h, lwip/api.h, lwip/sys.h, lwip/timers.h, lwip/netif.h */
#include "lwip/stats.h"
#include "lwip/dhcp.h"
#include "lwip/netifapi.h"
#include "netif/etharp.h" /* includes - lwip/ip.h, lwip/netif.h, lwip/ip_addr.h, lwip/pbuf.h */
#include "eth_lwip_default.h"
#include "eth_lwip.h"
#include "tms570_netif.h"
#include <stdio.h>

/* The lwIP network interface structure for the Ethernet EMAC. */
#ifndef MAX_EMAC_INSTANCE
#define MAX_EMAC_INSTANCE           1
#endif /*MAX_EMAC_INSTANCE*/

#define SUCCESS ERR_OK
#define FAILURE ERR_IF

static struct netif eth_lwip_netifs[MAX_EMAC_INSTANCE];
static void eth_lwip_conv_IP_decimal_Str(ip_addr_t ip, uint8_t *ipStr);


void
eth_lwip_get_dhcp_info(void)
{
  struct netif *netif = eth_lwip_get_netif(0);

  if (dhcp_supplied_address(netif)) {
    uint8_t ipString[16]; // FIXME change the functions to use char
    eth_lwip_conv_IP_decimal_Str(netif->ip_addr, ipString);
    printf("Address: %s\n", ipString);
    eth_lwip_conv_IP_decimal_Str(netif->netmask, ipString);
    printf("Netmask: %s\n", ipString);
    eth_lwip_conv_IP_decimal_Str(netif->gw, ipString);
    printf("Gateway: %s\n", ipString);
  } else {
    printf("dhcp not bound\n");
  }
}

int8_t
eth_lwip_init(uint8_t *mac_addr)
{
  unsigned int instance_number = 0;
  int8_t retVal = SUCCESS;


  ip4_addr_t ip_addr;
  ip4_addr_t net_mask;
  ip4_addr_t gw_addr;
  struct netif *netif = &eth_lwip_netifs[instance_number];
  struct netif *netif_tmp;
  u8_t default_mac[MAC_ADDR_LEN] = ETH_MAC_ADDR;

  if (mac_addr == NULL)
    mac_addr = default_mac;             /* use default MAC */

  eth_lwip_set_hwaddr(netif, mac_addr);
  tcpip_init(NULL, NULL);

#if STATIC_IP_ADDRESS
  ip_addr.addr = htonl(ETH_IP_ADDR);
  net_mask.addr = htonl(ETH_NETMASK);
  gw_addr.addr = htonl(ETH_GW);
#else
  ip_addr.addr = 0;
  net_mask.addr = 0;
  gw_addr.addr = 0;
#endif

  netif_tmp = netif_add(netif, &ip_addr, &net_mask, &gw_addr,
                        NULL, ETH_LWIP_INIT_NETIF_FNC, tcpip_input);

  if (netif_tmp == NULL)
    return NETIF_ADD_ERR;

  netif_set_default(netif);
  netifapi_netif_set_up(netif);
#if !STATIC_IP_ADDRESS
  netifapi_dhcp_start(netif);
#endif

  return retVal;
}

int
eth_lwip_get_netif_status_cmd(int argc, char *arg[])
{
  stats_display();
  return 0;
}

struct netif *
eth_lwip_get_netif(uint32_t instance_number)
{
  if (instance_number >= MAX_EMAC_INSTANCE)
    return NULL;
  return &eth_lwip_netifs[instance_number];
}

static void
eth_lwip_conv_IP_decimal_Str(ip_addr_t ip, uint8_t *ipStr)
{
  uint32_t addr;
 #if LWIP_IPV6
  addr = ip.u_addr.ip4.addr;
 #else
  addr = ip.addr;
 #endif

  snprintf((char *)ipStr, 16, "%lu.%lu.%lu.%lu",
           (addr >> 24), ((addr >> 16) & 0xff), ((addr >> 8) & 0xff), (addr & 0xff));
}

/*
* Function to set the MAC address to the interface
* @param   inst_num the instance number
*
* @note    mac_addr[0] is considered MSB
*/
void
eth_lwip_set_hwaddr(struct netif *netif, uint8_t *mac_addr)
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
eth_lwip_get_hwaddr_str(struct netif *netif, uint8_t *macStr)
{
  uint8_t index, outindex = 0;
  char ch;

  for (index = 0; index < netif->hwaddr_len; index++) {
    if (index)
      macStr[outindex++] = ':';
    ch = (netif->hwaddr[index] >> 4);
    macStr[outindex++] = (ch < 10) ? (ch + '0') : (ch - 10 + 'A');
    ch = (netif->hwaddr[index] & 0xf);
    macStr[outindex++] = (ch < 10) ? (ch + '0') : (ch - 10 + 'A');
  }
  macStr[outindex] = 0;
}
