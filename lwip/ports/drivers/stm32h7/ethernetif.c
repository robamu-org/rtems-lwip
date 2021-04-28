/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/ethernetif.c
  * @author  MCD Application Team
  * @brief   This file implements Ethernet network interface drivers for lwIP
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
#include "stm32h7xx_hal.h"
#include "port_conf.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/tcpip.h"
#include "ethernetif.h"

#include "lan8742.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DMA_DESCRIPTOR_ALIGNMENT                ( 0x20 )

/* Define those to better describe your network interface. */
#define IFNAME0 's'
#define IFNAME1 't'

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx buffers are allocated statically and passed directly to the LwIP stack,
          they will return back to ETH DMA after been processed by the stack.
        - Tx Buffers will be allocated from LwIP stack memory heap, 
          then passed to ETH HAL driver.

@Notes: 
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_RX_DESC_CNT in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_TX_DESC_CNT in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
  2.b. Rx Buffers must have the same size: ETH_RX_BUFFER_SIZE, this value must
       passed to ETH DMA in the init field (EthHandle.Init.RxBuffLen)
  2.c  The RX Ruffers addresses and sizes must be properly defined to be aligned
       to L1-CACHE line size (32 bytes).
*/

#ifdef __rtems__
netif_init_fn eth_lwip_init_fnc = &ethernetif_init;

/* Put into special RTEMS section and align correctly */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".bsp_nocache"), __aligned__(DMA_DESCRIPTOR_ALIGNMENT))); /* Ethernet Rx DMA Descriptors */
/* Put into special RTEMS section and align correctly */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]  __attribute__((section(".bsp_nocache"), __aligned__(DMA_DESCRIPTOR_ALIGNMENT)));  /* Ethernet Tx DMA Descriptors */
/* Ethernet Receive Buffers. Just place somewhere is BSS instead of explicitely placing it */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE] RTEMS_ALIGNED(DMA_DESCRIPTOR_ALIGNMENT);
#endif /* __rtems__ */

ETH_HandleTypeDef EthHandle;
ETH_TxPacketConfig TxConfig; 

lan8742_Object_t LAN8742;

#if NO_SYS == 0
sys_sem_t RxPktSemaphore; /* Semaphore to signal incoming packets */
#endif

/* Private function prototypes -----------------------------------------------*/
#if !STATIC_IP_ADDRESS && NO_SYS == 0 && LWIP_NETIF_API == 1
void set_dhcp_state(uint8_t new_state);
#endif

uint8_t determine_mpu_region_size_select(size_t lwip_heap_size);
static void low_level_init(struct netif *netif);
static err_t low_level_output(struct netif *netif, struct pbuf *p);
void ethernet_link_check_state(struct netif *netif);

void pbuf_free_custom(struct pbuf *p);

int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);


lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                               ETH_PHY_IO_DeInit,
                               ETH_PHY_IO_WriteReg,
                               ETH_PHY_IO_ReadReg,
                               ETH_PHY_IO_GetTick};

LWIP_MEMPOOL_DECLARE(RX_POOL, 10, sizeof(struct pbuf_custom), "Zero-copy RX PBUF pool");

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH) 
*******************************************************************************/

/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#ifdef __rtems__
#if RTEMS_LWIP_PROTECT_LWIP_HEAP_WITH_MPU == 1
  mpu_config_tx_buffers(LWIP_RAM_HEAP_POINTER, MEM_SIZE);
#endif /* RTEMS_LWIP_PROTECT_LWIP_HEAP_WITH_MPU == 1 */
#endif /* __rtems__ */

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

#if NO_SYS == 0
  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);
#endif

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;

  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* This is already done by the generic driver, so the internal state machine is set accordingly */
#if !STATIC_IP_ADDRESS && NO_SYS == 0 && LWIP_NETIF_API == 1
  set_dhcp_state(DHCP_WAIT_ADDRESS);
#endif

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
static void low_level_init(struct netif *netif)
{
#if NO_SYS == 0
  int32_t PHYLinkState;
  ETH_MACConfigTypeDef MACConf;
  uint32_t duplex, speed = 0;
#endif
  uint32_t idx = 0;
  uint8_t macaddress[6]= {ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2, ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5};
  
  EthHandle.Instance = ETH;  
  EthHandle.Init.MACAddr = macaddress;
  EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
  EthHandle.Init.RxDesc = DMARxDscrTab;
  EthHandle.Init.TxDesc = DMATxDscrTab;
  EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;
  
  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  HAL_ETH_Init(&EthHandle);
  
#if NO_SYS == 0
  /* Enable the Ethernet global Interrupt */
  HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
#endif

  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;
  
  /* set MAC hardware address */
  netif->hwaddr[0] =  ETH_MAC_ADDR0;
  netif->hwaddr[1] =  ETH_MAC_ADDR1;
  netif->hwaddr[2] =  ETH_MAC_ADDR2;
  netif->hwaddr[3] =  ETH_MAC_ADDR3;
  netif->hwaddr[4] =  ETH_MAC_ADDR4;
  netif->hwaddr[5] =  ETH_MAC_ADDR5;
  
  /* maximum transfer unit */
  netif->mtu = ETH_MAX_PAYLOAD;
  
  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
  
  for(idx = 0; idx < ETH_RX_DESC_CNT; idx ++)
  {
    HAL_ETH_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);
  }
  
  /* Initialize the RX POOL */
  LWIP_MEMPOOL_INIT(RX_POOL);
  
  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));  
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

#ifdef __rtems__
#if NO_SYS == 0
  /* create a binary semaphore used for informing ethernetif of frame reception */
  sys_sem_new(&RxPktSemaphore, 1);
  
  /* create the task that handles the ETH_MAC */
  sys_thread_new(
    "ETHM",
    (lwip_thread_fn) ethernetif_input,
    netif,
    RTEMS_LWIP_INTERFACE_THREAD_STACK_SIZE,
    RTEMS_LWIP_INTERFACE_THREAD_PRIORITY
  );
#endif /* NO_SYS == 0 */
#endif /* __rtems__ */
  
  /* Set PHY IO functions */
  LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
  
  /* Initialize the LAN8742 ETH PHY */
  LAN8742_Init(&LAN8742);
  
#if NO_SYS == 0
  PHYLinkState = LAN8742_GetLinkState(&LAN8742);
  
  /* Get link state */  
  if(PHYLinkState <= LAN8742_STATUS_LINK_DOWN)
  {
    netif_set_link_down(netif);
    netif_set_down(netif);
  }
  else 
  {
    switch (PHYLinkState)
    {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      break;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      break;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      break;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      break;
    default:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      break;      
    }
    
    /* Get MAC Config MAC */
    HAL_ETH_GetMACConfig(&EthHandle, &MACConf); 
    MACConf.DuplexMode = duplex;
    MACConf.Speed = speed;
    HAL_ETH_SetMACConfig(&EthHandle, &MACConf);
    HAL_ETH_Start_IT(&EthHandle);
#ifdef __rtems__
    /* Install the ETH IRQ for RTEMS */
    rtems_interrupt_handler_install(ETH_IRQn, NULL, RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) &HAL_ETH_IRQHandler, &EthHandle);
#endif
    netif_set_up(netif);
    netif_set_link_up(netif);
  }
#endif

  ethernet_link_check_state(netif);
}

/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become available since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  uint32_t i=0;
  struct pbuf *q;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
  
  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));
  
  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)	
      return ERR_IF;
    
    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;

#ifdef __rtems__
    /* For the socket API, a custom send buffer might be used which is not protected by the
    MPU. Therefore, clean DCache here as well */
#if RTEMS_LWIP_PROTECT_LWIP_HEAP_WITH_MPU == 0 || LWIP_SOCKET == 1
    /* Cleaning cache isn't necessary if the tx buffers are in a not-cacheable MPU region. */
    uint8_t *dataStart = q->payload;
    uint8_t *lineStart = (uint8_t *)((uint32_t)dataStart & ~31);
    SCB_CleanDCache_by_Addr((uint32_t *)lineStart, q->len + (dataStart - lineStart));
#endif
#endif

    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }
    
    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }

    i++;
  }

  TxConfig.Length = p->tot_len;
  TxConfig.TxBuffer = Txbuffer;

  HAL_ETH_Transmit(&EthHandle, &TxConfig, RTEMS_LWIP_ETH_DMA_TRANSMIT_TIMEOUT);
  
  return errval;
}

/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;
  ETH_BufferTypeDef RxBuff[ETH_RX_DESC_CNT];
  uint32_t framelength = 0, i = 0;;
  struct pbuf_custom* custom_pbuf;

  memset(RxBuff, 0 , ETH_RX_DESC_CNT*sizeof(ETH_BufferTypeDef));
  
  for(i = 0; i < ETH_RX_DESC_CNT -1; i++)
  {
    RxBuff[i].next=&RxBuff[i+1];
  }

#if NO_SYS == 0
  if(HAL_ETH_GetRxDataBuffer(&EthHandle, RxBuff) == HAL_OK) {
#else
  if (HAL_ETH_IsRxDataAvailable(&EthHandle)) {
#endif /* NO_SYS != 0 */

#if NO_SYS == 1
    HAL_ETH_GetRxDataBuffer(&EthHandle, RxBuff);
#endif /* NO_SYS == 1 */

    HAL_ETH_GetRxDataLength(&EthHandle, &framelength);

    /* Build Rx descriptor to be ready for next data reception */
    HAL_ETH_BuildRxDescriptors(&EthHandle);

    /* Invalidate data cache for ETH Rx Buffers */
    SCB_InvalidateDCache_by_Addr((uint32_t *)RxBuff->buffer, framelength);
    
    custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    if(custom_pbuf != NULL)
    {
      custom_pbuf->custom_free_function = pbuf_free_custom;

      p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf, RxBuff->buffer, framelength);
    }

  }
  
  return p;
}

#if NO_SYS == 0
/**
  * @brief This function is the ethernetif_input task, it is processed when a packet 
  * is ready to be read from the interface. It uses the function low_level_input() 
  * that should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input(struct netif *netif)
{
  struct pbuf *p;
  
  for( ;; )
  {
    if (sys_arch_sem_wait(&RxPktSemaphore, 0) != SYS_ARCH_TIMEOUT)
    {
      do
      {
        p = low_level_input( netif );
        if (p != NULL)
        {
          if (netif->input( p, netif) != ERR_OK )
          {
            pbuf_free(p);
          }
        }

      }while(p!=NULL);
    }
  }
}
#else
/**
  * @brief This function is the ethernetif_input task, it is processed when a packet
  * is ready to be read from the interface. It uses the function low_level_input()
  * that should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input(struct netif *netif)
{
  err_t err;
  struct pbuf *p;

  /* move received packet into a new pbuf */
  p = low_level_input(netif);

  /* no packet could be read, silently ignore this */
  if (p == NULL) return;

  /* entry point to the LwIP stack */
  err = netif->input(p, netif);

  if (err != ERR_OK)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }
}
#endif /* NO_SYS == 1 */

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
}


/*******************************************************************************
                       Ethernet MSP Routines
*******************************************************************************/
/**
  * @brief  Note: HAL_ETH_MspInit of RTEMS BSP is used.
  * @param  heth: ETH handle
  * @retval None
*/

#if NO_SYS == 0
/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  sys_sem_signal(&RxPktSemaphore);
}
#endif

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{  
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here 
  */
  
  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&EthHandle);
  
  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&EthHandle, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&EthHandle, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return HAL_GetTick();
}

void ethernet_link_check_state(struct netif *netif)
{
  ETH_MACConfigTypeDef MACConf;
  uint32_t PHYLinkState;
  uint32_t linkchanged = 0, speed = 0, duplex =0;

  PHYLinkState = LAN8742_GetLinkState(&LAN8742);

  if(netif_is_link_up(netif) && (PHYLinkState <= LAN8742_STATUS_LINK_DOWN))
  {
    HAL_ETH_Stop(&EthHandle);
    netif_set_down(netif);
    netif_set_link_down(netif);
  }
  else if(!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN))
  {
    switch (PHYLinkState)
    {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    default:
      break;
    }

    if(linkchanged)
    {
      /* Get MAC Config MAC */
      HAL_ETH_GetMACConfig(&EthHandle, &MACConf);
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      HAL_ETH_SetMACConfig(&EthHandle, &MACConf);
      HAL_ETH_Start(&EthHandle);
      netif_set_up(netif);
      netif_set_link_up(netif);
    }
  }
}

#ifdef __rtems__
#if NO_SYS == 1
/* Even for NO_SYS == 1 this needs to be implemented */
uint32_t
sys_now()
{
  /* Forward call to HAL function which already implements returning millisecond ticks */
  return HAL_GetTick();
}
#endif /* NO_SYS == 1 */

/* Configure the MPU for the lwIP heap. */
void mpu_config_tx_buffers(uint32_t base_addr, size_t region_size)
{
    MPU_Region_InitTypeDef MPU_InitStruct;
    uint8_t region_select = determine_mpu_region_size_select(region_size);

    /* Disable the MPU */
    HAL_MPU_Disable();

    /* Configure the MPU attributes as Normal Non Cacheable
       for LwIP RAM heap which contains the Tx buffers */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = base_addr;
    MPU_InitStruct.Size = region_select;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    SCB_CleanDCache_by_Addr((uint32_t*) base_addr, region_size);

    /* Enable the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

uint8_t determine_mpu_region_size_select(size_t lwip_heap_size)
{
    if(lwip_heap_size <= 1024) {
        return MPU_REGION_SIZE_1KB;
    }
    else if(lwip_heap_size <= 2 * 1024) {
        return MPU_REGION_SIZE_2KB;
    }
    else if(lwip_heap_size <= 4 * 1024) {
        return MPU_REGION_SIZE_4KB;
    }
    else if(lwip_heap_size <= 8 * 1024) {
        return MPU_REGION_SIZE_8KB;
    }
    else if(lwip_heap_size <= 16 * 1024) {
        return MPU_REGION_SIZE_16KB;
    }
    else if(lwip_heap_size <= 32 * 1024) {
        return MPU_REGION_SIZE_32KB;
    }
    else if(lwip_heap_size <= 64 * 1024) {
        return MPU_REGION_SIZE_64KB;
    }
    else if(lwip_heap_size <= 128 * 1024) {
        return MPU_REGION_SIZE_128KB;
    }
    else if(lwip_heap_size <= 256 * 1024) {
        return MPU_REGION_SIZE_256KB;
    }
    else if(lwip_heap_size <= 512 * 1024) {
        return MPU_REGION_SIZE_512KB;
    }
    else {
        /* There is only 1MB RAM available so a MPU protection of 1MB does not really make sense */
        return MPU_REGION_SIZE_512KB;
    }
}

lan8742_Object_t* get_lan_phy_handle() {
  return &LAN8742;
}

ETH_HandleTypeDef* get_eth_handle() {
  return &EthHandle;
}
#endif /* __rtems__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
