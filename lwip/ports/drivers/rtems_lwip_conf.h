#ifndef __ETH_LWIP_DEFAULT_H
#define __ETH_LWIP_DEFAULT_H

#include "lwipopts.h"
#include "lwip/etharp.h"

/* this MAC address is used when user put NULL on the right place when calling postInit function */
/**
 * Default MAC address for interface.
 */
#define MAC_ADDR_LEN              ETHARP_HWADDR_LEN

#ifndef ETH_MAC_ADDR
#define ETH_MAC_ADDR             { 0x12 /* Unicast, Locally administered */, 0x34, 0x56, 0x78, 0x9A, 0xBC }
#endif

/**
 * When static IP is configured in lwipopts.h, this IP address is used for interface.
 */
#ifndef ETH_IP_ADDR

#if defined(IP_ADDR0) && defined(IP_ADDR1) && defined(IP_ADDR2) && defined(IP_ADDR3)
#define CALCULATE_ETH_IP_ADDR
#else
#define ETH_IP_ADDR               0xC0A8F701 /* 192.168.247.1  */
#endif

#endif

/**
 * When static IP is configured in lwipopts.h, this NETMASK address is used for interface.
 */
#ifndef ETH_NETMASK

#if defined(NETMASK_ADDR0) && defined(NETMASK_ADDR1) && defined(NETMASK_ADDR2) && defined(NETMASK_ADDR3)
#define CALCULATE_ETH_NETMASK
#else
#define ETH_NETMASK               0xFFFFFF00 /* 255.255.255.0  */
#endif

#endif

/**
 * When static IP is configured in lwipopts.h, this Gateway address is used for interface.
 */
#ifndef ETH_GW

#if defined(GW_ADDR0) && defined(GW_ADDR1) && defined(GW_ADDR2) && defined(GW_ADDR3)
#define CALCULATE_GW_ADDRESS
#else
#define ETH_GW                    0xC0A8F7FE /* 192.168.247.254*/
#endif

#endif /* ETH_GW */

#endif /* __ETH_LWIP_DEFAULT_H */
