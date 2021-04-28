/* Includes ------------------------------------------------------------------*/
#include "app_dhcp.h"
#include "port_conf.h"
#include "ethernetif.h"
#include "stm32h7xx_hal.h"
#include "lwip/opt.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "rtems_lwip.h"
#include "rtems_lwip_conf.h"

#if NO_SYS == 0
#include "lwip/sys.h"
#endif
#include <sys/unistd.h>

#if LWIP_DHCP
#ifndef MAX_DHCP_TRIES
#define MAX_DHCP_TRIES  4
#endif /* MAX_DHCP_TRIES */


#if NO_SYS == 1
static uint32_t DHCPfineTimer = 0;

#endif /* NO_SYS == 1*/

static void dhcp_thread(void* argument);

LwipThreadArgs dhcp_args;
uint8_t DHCP_state = DHCP_START;

#if NO_SYS == 0
void rtems_lwip_start_dhcp_thread(uint32_t task_interval_ms,
    size_t task_stack, uint8_t task_priority) {
  dhcp_args.netif = rtems_lwip_get_netif(0);
  dhcp_args.task_interval_ms = task_interval_ms;
  sys_thread_new("DHCP", dhcp_thread, (void*) &dhcp_args, task_stack, task_priority);
}
#endif /* NO_SYS == 0 */

/**
 * @brief  This is the thread dedicated to DHCP handling
 * @param  argument: network interface
 * @retval None
 */
void dhcp_thread(void* argument)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp = NULL;
  LwipThreadArgs* args = (LwipThreadArgs*) argument;
  if(args == NULL) {
    printf("dhcp_thread: Passed arguments are invalid!\n\r");
    exit(1);
  }

  struct netif* netif = args->netif;
  if(netif == NULL) {
    printf("dhcp_thread: Passed netif is invalid!\n\r");
    exit(1);
  }

  if(args->task_interval_ms == 0) {
    printf("dhcp_thread: Invalid task interval!\n\r");
    exit(1);
  }

  for (;;) {
    switch (DHCP_state) {
    case DHCP_START: {
      ip_addr_set_zero_ip4(&netif->ip_addr);
      ip_addr_set_zero_ip4(&netif->netmask);
      ip_addr_set_zero_ip4(&netif->gw);
      DHCP_state = DHCP_WAIT_ADDRESS;

      dhcp_start(netif);
    }
    break;
    case DHCP_WAIT_ADDRESS: {
      if (dhcp_supplied_address(netif)) {
        DHCP_state = DHCP_ADDRESS_ASSIGNED;
#if RTEMS_LWIP_DHCP_PRINTOUT == 1
        printf("IP address assigned by a DHCP server: %s\n\r",
            ip4addr_ntoa(netif_ip4_addr(netif)));
#endif
      }
      else {
        dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

        /* DHCP timeout */
        if (dhcp->tries > MAX_DHCP_TRIES) {
          DHCP_state = DHCP_TIMEOUT;
#if RTEMS_LWIP_DHCP_PRINTOUT == 1
          uint8_t iptxt[20];
          sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
          printf("DHCP timeout\n\r");
          printf("Setting static IP address %s..\n\r", iptxt);
#endif
          /* Static address used */
          rtems_lwip_determine_static_ipv4_address(&ipaddr, &netmask, &gw);
          netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
        }
      }
    }
    break;
    case DHCP_LINK_DOWN: {
      DHCP_state = DHCP_OFF;
    }
    break;
    default: break;
    }

    usleep(args->task_interval_ms * 1000);
  }
}

#if NO_SYS == 1

void DHCP_Process(struct netif *netif);

/**
 * @brief  DHCP periodic check
 * @param  netif
 * @retval None
 */
void dhcp_periodic_handle(struct netif *netif)
{
  /* Fine DHCP periodic process every 500ms */
  if (HAL_GetTick() - DHCPfineTimer >= RTEMS_LWIP_DHCP_TASK_INTERVAL_MS) {
    DHCPfineTimer = HAL_GetTick();
    /* process DHCP state machine */
    DHCP_Process(netif);
  }
}

/**
 * @brief  DHCP_Process_Handle
 * @param  None
 * @retval None
 */
void DHCP_Process(struct netif *netif)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp = NULL;
  switch (DHCP_state) {
  case DHCP_START: {
    ip_addr_set_zero_ip4(&netif->ip_addr);
    ip_addr_set_zero_ip4(&netif->netmask);
    ip_addr_set_zero_ip4(&netif->gw);
    dhcp_start(netif);
    DHCP_state = DHCP_WAIT_ADDRESS;
  }
  break;

  case DHCP_WAIT_ADDRESS: {
    if (dhcp_supplied_address(netif)) {
      DHCP_state = DHCP_ADDRESS_ASSIGNED;
      printf("IP address assigned by a DHCP server: %s\n\r", ip4addr_ntoa(netif_ip4_addr(netif)));
    }
    else
    {
      dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

      /* DHCP timeout */
      if (dhcp->tries > MAX_DHCP_TRIES)
      {
        DHCP_state = DHCP_TIMEOUT;

        uint8_t iptxt[20];
        sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
        printf("DHCP timeout\n\r");
        printf("Setting static IP address %s..\n\r", iptxt);
        /* Static address used */
        rtems_lwip_determine_static_ipv4_address(&ipaddr, &netmask, &gw);
        netif_set_addr(netif, &ipaddr, &netmask, &gw);

      }
    }
  }
  break;
  case DHCP_LINK_DOWN: {
    /* Stop DHCP */
    //dhcp_release_and_stop(netif);
    DHCP_state = DHCP_OFF;
  }
  break;
  default: break;
  }
}

#endif /* NO_SYS == 1 */

void set_dhcp_state(uint8_t new_state)
{
  DHCP_state = new_state;
}

uint8_t get_dhcp_state()
{
  return DHCP_state;
}

#endif  /* LWIP_DHCP */

