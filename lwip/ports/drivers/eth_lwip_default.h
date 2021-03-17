#ifndef __ETH_LWIP_DEFAULT_H
#define __ETH_LWIP_DEFAULT_H

/* #define DEBUG 1 */
/* #define STATIC_IP_ADDRESS 1 */

void tms570_eth_memp_avaible(int type);

#define ETH_LWIP_INIT_NETIF_FNC tms570_eth_init_netif
/*called from memp_free() when a memp pool was empty and an item is now available*/
#define LWIP_HOOK_MEMP_AVAILABLE tms570_eth_memp_avaible

/* this MAC address is used when user put NULL on the right place when calling postInit function */
/**
 * Default MAC address for interface.
 */
#define MAC_ADDR_LEN              ETHARP_HWADDR_LEN

#ifndef ETH_MAC_ADDR
#define ETH_MAC_ADDR             { 0x12 /* Unicast, Locally administered */, 0x34, 0x56, 0x78, 0x9A, 0xBC }
#endif

#if STATIC_IP_ADDRESS

/**
 * When static IP is configured in lwipopts.h, this IP address is used for interface.
 */
#ifndef ETH_IP_ADDR
#define ETH_IP_ADDR               0xC0A8F701 /* 192.168.247.1  */
#endif

/**
 * When static IP is configured in lwipopts.h, this NETMASK address is used for interface.
 */
#ifndef ETH_NETMASK
#define ETH_NETMASK               0xFFFFFF00 /* 255.255.255.0  */
#endif

/**
 * When static IP is configured in lwipopts.h, this Gateway address is used for interface.
 */
#ifndef ETH_GW
#define ETH_GW                    0xC0A8F7FE /* 192.168.247.254*/
#endif

#endif /* STATIC_IP_ADDRESS */

#endif /* __ETH_LWIP_DEFAULT_H */
