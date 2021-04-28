/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Inc/app_ethernet.h 
  * @author  MCD Application Team
  * @brief   Header for app_ethernet.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_ETHERNET_H
#define __APP_ETHERNET_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lwip/netif.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ethernet_link_status_updated(struct netif *netif);

#if LWIP_NETIF_LINK_CALLBACK == 1 && NO_SYS == 1
/**
 * This function should be called in mainloop or in a dedicated thread and takes care of
 * the ethernet link handling.
 * @param netif
 */
void ethernet_link_periodic_handle(struct netif *netif);
#endif

/**
 * Start a ethernet link thread with the given parameters.
 * @param task_interval_ms
 * @param task_stack
 * @param task_priority
 */
void rtems_lwip_start_link_thread(uint32_t task_interval_ms,
    size_t task_stack, uint8_t task_priority);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ETHERNET_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

