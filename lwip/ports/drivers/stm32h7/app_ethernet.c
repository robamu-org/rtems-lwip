/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/app_ethernet.c 
  * @author  MCD Application Team
  * @brief   Ethernet specefic module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "rtems_lwip_conf.h"
#include "rtems_lwip.h"
#include "stm32h7/lan8742.h"
#include "stm32h7xx_hal.h"
#include "lwip/opt.h"
#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif

#if NO_SYS == 0
#include "lwip/sys.h"
#endif

#include "app_ethernet.h"
#include "app_dhcp.h"
#include "ethernetif.h"

/* Private typedef -----------------------------------------------------------*/
LwipThreadArgs linkArgs;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if NO_SYS == 1
uint32_t ethernet_link_timer = 0;
#endif

/* Private function prototypes -----------------------------------------------*/
#if LWIP_DHCP
void set_dhcp_state(uint8_t new_state);
#endif
static void ethernet_link_thread( void* argument );
lan8742_Object_t* get_lan_phy_handle();
ETH_HandleTypeDef* get_eth_handle();

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Notify the User about the nework interface config status 
  * @param  netif: the network interface
  * @retval None
  */
void ethernet_link_status_updated(struct netif *netif) 
{
  if (netif_is_up(netif)) {
#if LWIP_DHCP
    printf("Ethernet link up. Setting DHCP state..\n\r");
    /* Update DHCP state machine */
    set_dhcp_state(DHCP_START);
#endif /* LWIP_DHCP */
  }
  else {
#if LWIP_DHCP
    printf("Ethernet link down. Setting DHCP state..\n\r");
    /* Update DHCP state machine */
    set_dhcp_state(DHCP_LINK_DOWN);
#endif /* LWIP_DHCP */
  } 
}

#if LWIP_NETIF_LINK_CALLBACK == 1 && NO_SYS == 1
/**
  * @brief  Ethernet Link periodic check
  * @param  netif
  * @retval None
  */
void ethernet_link_periodic_handle(struct netif *netif)
{
  uint32_t time_now = HAL_GetTick();
  /* Ethernet Link every 100ms */
  if (time_now - ethernet_link_timer >= 100) {
    ethernet_link_timer = time_now;
    ethernet_link_check_state(netif);
  }
}
#endif

#if NO_SYS == 0

void rtems_lwip_start_link_thread(uint32_t task_interval_ms,
    size_t task_stack, uint8_t task_priority) {
  linkArgs.netif = rtems_lwip_get_netif(0);
  linkArgs.task_interval_ms = task_interval_ms;
  sys_thread_new("LINK", ethernet_link_thread, (void*) &linkArgs, task_stack, task_priority);
}

#endif

/**
  * @brief  Check the ETH link state and update netif accordingly.
  * @param  argument: netif
  * @retval None
  */
void ethernet_link_thread( void* argument )
{
  ETH_MACConfigTypeDef MACConf;
  int32_t PHYLinkState;
  uint32_t linkchanged = 0, speed = 0, duplex =0;
  LwipThreadArgs* args = (LwipThreadArgs*) argument;
  if(args == NULL) {
    printf("ethernet_link_thread: Passed arguments are invalid!\n\r");
    exit(1);
  }

  struct netif *netif = args->netif;
  if(netif == NULL) {
    printf("ethernet_link_thread: Passed netif is invalid!\n\r");
    exit(1);
  }

  if(args->task_interval_ms == 0) {
    printf("ethernet_link_thread: Invalid task interval!\n\r");
    exit(1);
  }

  lan8742_Object_t* lan = get_lan_phy_handle();
  ETH_HandleTypeDef* eth = get_eth_handle();
  for(;;) {

    PHYLinkState = LAN8742_GetLinkState(lan);

    if(netif_is_link_up(netif) && (PHYLinkState <= LAN8742_STATUS_LINK_DOWN)) {
      HAL_ETH_Stop_IT(eth);
      netif_set_down(netif);
      netif_set_link_down(netif);
    }
    else if(!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN)) {
      switch (PHYLinkState) {
      case LAN8742_STATUS_100MBITS_FULLDUPLEX: {
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        linkchanged = 1;
        break;
      }
      case LAN8742_STATUS_100MBITS_HALFDUPLEX: {
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        linkchanged = 1;
        break;
      }
      case LAN8742_STATUS_10MBITS_FULLDUPLEX: {
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        linkchanged = 1;
        break;
      }
      case LAN8742_STATUS_10MBITS_HALFDUPLEX: {
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        linkchanged = 1;
        break;
      }
      default:
        break;
      }

      if(linkchanged) {
        /* Get MAC Config MAC */
        HAL_ETH_GetMACConfig(eth, &MACConf);
        MACConf.DuplexMode = duplex;
        MACConf.Speed = speed;
        HAL_ETH_SetMACConfig(eth, &MACConf);
        HAL_ETH_Start_IT(eth);
        netif_set_up(netif);
        netif_set_link_up(netif);
      }
    }

    usleep(args->task_interval_ms * 1000);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
