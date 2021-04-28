/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Inc/ethernetif.h 
  * @author  MCD Application Team
  * @brief   Header for ethernetif.c module
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

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ETH_RX_BUFFER_SIZE                     (1536UL)

/* Exported types ------------------------------------------------------------*/
#ifdef __rtems__
typedef struct {
 struct netif* netif;
 uint32_t task_interval_ms;
} LwipThreadArgs;
#endif /* __rtems__ */

/* Exported functions ------------------------------------------------------- */
err_t ethernetif_init(struct netif *netif);      
void ethernetif_input(struct netif *netif);
void ethernet_link_check_state(struct netif *netif);

#ifdef __rtems__
/**
 * This function can be used to protect the TX lwIP buffer region (lwIP Heap).
 * @param base_addr
 * @param region_size
 */
void mpu_config_tx_buffers(uint32_t base_addr, size_t region_size);
#endif /* __rtems__ */

#ifdef __cplusplus
}
#endif

#endif /* __ETHERNETIF_H__ */
