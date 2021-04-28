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

/* lwIP headers */
#include "lwip/init.h"
#if LWIP_VERSION_MAJOR >= 2
#include "lwip/timeouts.h"
#else /*LWIP_VERSION_MAJOR*/
#include "lwip/timers.h" /* for DHCP binding in NO_SYS mode */
#endif /*LWIP_VERSION_MAJOR*/
#include "lwip/sys.h" /* includes - lwip/opt.h, lwip/err.h, arch/sys_arch.h */
#include "lwip/tcpip.h" /* includes - lwip/opt.h, lwip/api_msg.h, lwip/netifapi.h, lwip/pbuf.h, lwip/api.h, lwip/sys.h, lwip/timers.h, lwip/netif.h */
#include "lwip/stats.h" /* includes - lwip/mem.h, lwip/memp.h, lwip/opt.h */
#include "lwip/snmp.h"
#include "netif/etharp.h" /* includes - lwip/ip.h, lwip/netif.h, lwip/ip_addr.h, lwip/pbuf.h */
#include <lwip/netifapi.h>
/* end - lwIP headers */

//--------moje
#include <strings.h>
#include <stdbool.h>
#include <stdlib.h>
#include <bsp/irq.h>
#include <bsp/tms570.h>
#include <bsp/tms570-pinmux.h>
#include "arch/cc.h"
#include "rtems_lwip.h"
#include "tms570_netif.h"
#include "ti_drv_emac.h"
#include "ti_drv_mdio.h"
#include "phy_dp83848h.h"
#include "tms570_emac.h"

netif_init_fn eth_lwip_init_fnc = &tms570_eth_init_netif;

#define LINK_SPEED_OF_YOUR_NETIF_IN_BPS 10000000

/* Number of EMAC Instances */

#define DEFAULT_PHY_ADDR            0x1
#define FIND_FIRST_PHY_ALIVE        1 /* or use default (phy_address: 1) */
#define NUM_OF_PHYs             32

/* Size of the Buffer descriptor defined by the EMAC in bytes */
#define SIZE_OF_DESC                16

/* Channel number used for for RX, TX, unicast, broadcast or damaged frames;
 * there are different channels for rx and tx operations (i.e. RXCH0 != TXCH0) */
#define CHANNEL                 0

/* take in account oversized frames */
#define MAX_TRANSFER_UNIT           1500

#ifndef TMS570_MMR_SELECT_GMII_SEL
  #define TMS570_MMR_SELECT_GMII_SEL TMS570_BALL_XX_GMII_SEL
#endif

#ifndef TMS570_BALL_K19_MII_RXCLK
  #define TMS570_BALL_K19_MII_RXCLK TMS570_BALL_K19_MII_RX_CLK
#endif

/* WARNING!
 * Be very carefull when setting this value. We have to keep in mind
 * that pbuf_alloc(..., PBUF_POOL) will usualy return a chain of PBUFs
 * pointing to the statically preallocated buffers (of the same size).
 * The problem is that if we ask to allocate 300 bytes whereby the size
 * of the statically preallocated PBUFs (PBUF_POOL_BUFSIZE) is 256, we
 * will get a chain containing two PBUFs -- one *reporting* its size to
 * be 256 bytes, the other one 44 bytes.
 * Everything seems to be just fine however during RX, after we call
 * netif->input(pbuf, netif) we have to newly allocate the PBUF(s) and
 * properly set the apropriate BDs. This will however work only if the
 * number of the freed BDs is the same as the number of the BDs newly
 * initialized. One possible situation when this may fail is when multiple
 * non-256 byte sized PBUFs will move near to each other, i.e. 3 BDs:
 * 256 B, 44 B, 44 B -- 344 bytes will be freed (3 BDs) but the new call
 * to pbuf_alloc(...) will return a chain comprising only two PBUFs
 * (256 B, 88 B).
 * This is the implementation limitation. The PBUF_LEN_MAX should therefore
 * be multiple of PBUF_POOL_BUFSIZE
 */
#define PBUF_LEN_MAX                (PBUF_POOL_BUFSIZE * 6)

/* Maximum number of PBUFs preallocated in the driver
 * init function to be used for the RX
 */
#define MAX_RX_PBUF_ALLOC           10
#define MIN_PKT_LEN             60

/* Define those to better describe the network interface. */
#define IFNAME0                 'e'
#define IFNAME1                 'n'

/* Time to wait for autonegotiation in ticks. */
#define TICKS_PHY_AUTONEG           4000

/* startup init indicator */
static bool initialized = false;

/*private?*/
#if !defined(__TI_COMPILER_VERSION__)
static
#endif /*__TI_COMPILER_VERSION__*/
SYS_IRQ_HANDLER_FNC(tms570_eth_irq);
static void tms570_eth_rx_pbuf_refill(struct tms570_netif_state *nf_state, int);
static void tms570_eth_rx_pbuf_refill_single(struct netif *);
static void tms570_eth_hw_set_RX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_rx_bd *new_head);
static void tms570_eth_hw_set_TX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_tx_bd *new_head);
static void tms570_eth_hw_set_hwaddr(struct tms570_netif_state *nf_state, uint8_t *mac_addr);
static void tms570_eth_process_irq_rx(void *arg);
static void tms570_eth_process_irq_tx(void *arg);
static void tms570_eth_process_irq_request(void *argument);
static void tms570_eth_process_irq(void *argument);
struct netif *tms570_eth_get_netif(uint32_t instance_number);
static err_t tms570_eth_send(struct netif *netif, struct pbuf *p);
static err_t tms570_eth_send_raw(struct netif *netif, struct pbuf *pbuf);
static err_t tms570_eth_init_hw(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_hw_post_init(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_interrupt(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_find_PHY(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_control_structures(struct netif *netif);
static void tms570_eth_init_netif_fill(struct netif *netif);
static void tms570_eth_init_buffer_descriptors(struct tms570_netif_state *nf_state);
static void tms570_eth_init_set_pinmux();
static void sys_arch_data_sync_barier();

/***** initializing functions **********************************************/


struct tms570_netif_state *
tms570_eth_init_state(void)
{
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)malloc(sizeof(struct tms570_netif_state));

  nf_state->emac_base = &TMS570_EMACM;
  nf_state->emac_ctrl_base = &TMS570_EMACC;
  nf_state->emac_ctrl_ram = EMAC_CTRL_RAM_BASE;
  nf_state->mdio_base = &TMS570_MDIO;
  nf_state->phy_addr = DEFAULT_PHY_ADDR;
#if !NO_SYS
  nf_state->waitTicksForPHYAneg = TICKS_PHY_AUTONEG;
#endif
  return nf_state;
}

static void
tms570_eth_init_netif_fill(struct netif *netif)
{
#if LWIP_NETIF_HOSTNAME
  netif->hostname = "tms570";
#endif

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  /* We directly use etharp_output() here to save a function call.
   * You can instead declare yo_SKIP_TO_HWur own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...)
   */
  netif->output = etharp_output;
  netif->linkoutput = tms570_eth_send;

  /* maximum transfer unit */
  netif->mtu = MAX_TRANSFER_UNIT;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
}

static err_t
tms570_eth_init_control_structures(struct netif *netif)
{
  err_t res;
  sys_thread_t tx_thread_id;
  struct tms570_netif_state *nf_state;

  nf_state = netif->state;

  res = sys_sem_new(&nf_state->intPend_sem, 0);
  if (res != ERR_OK) {
    sys_arch_printk("ERROR! semaphore creation error - 0x%08lx\n", (long)res);
  }
  tx_thread_id = sys_thread_new(0, tms570_eth_process_irq_request, netif, 1024, 3); //zkontrolovat priorita 0
  if (tx_thread_id == 0) {
    sys_arch_printk("ERROR! lwip interrupt thread not created");
    res = !ERR_OK;
  }
  return res;
}

err_t
tms570_eth_init_netif(struct netif *netif)
{
  err_t retVal;
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)netif->state;

  if (initialized)
    return ERR_IF;

  if (nf_state == NULL) {
    /* nf_state needs to be freed */
    if ((nf_state = tms570_eth_init_state()) == 0) {
      return ERR_IF;
    }
    netif->state = nf_state;
  }

  tms570_eth_init_netif_fill(netif);

  if ((retVal = tms570_eth_init_hw(nf_state)) != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_hw: %d", retVal);
    return retVal;
  }
  if ((retVal = tms570_eth_init_control_structures(netif)) != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_control_structures: %d", retVal);
    return retVal;
  }
  tms570_eth_init_buffer_descriptors(nf_state);
  tms570_eth_rx_pbuf_refill(nf_state, 0);
  tms570_eth_hw_set_hwaddr(nf_state, netif->hwaddr);
  tms570_eth_init_interrupt(nf_state);
  if ((retVal = tms570_eth_init_hw_post_init(nf_state)) != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_hw_post_init: %d", retVal);
    return retVal;
  }
  initialized = true;
#if TMS570_NETIF_DEBUG
  tms570_eth_debug_print_HDP(nf_state);
#endif
  return ERR_OK;
}

static err_t
tms570_eth_init_interrupt(struct tms570_netif_state *nf_state)
{
  int res;

  res = sys_request_irq(TMS570_IRQ_EMAC_TX, tms570_eth_irq,
                        0, "emac_tx", nf_state);
  if (res < 0) {
    sys_arch_printk("Failed to install tx handler\n");
    return ERR_IF;
  }

  res = sys_request_irq(TMS570_IRQ_EMAC_RX, tms570_eth_irq,
                        0, "emac_rx", nf_state);
  if (res < 0) {
    sys_arch_printk("Failed to install rx handler\n");
    return ERR_IF;
  }
  return ERR_OK;
}
static err_t
tms570_eth_init_find_PHY(struct tms570_netif_state *nf_state)
{
  uint8_t index;
  uint16_t regContent;
  uint32_t physAlive;

  MDIOPhyRegRead(nf_state->mdio_base, nf_state->phy_addr, PHY_BMSR, &regContent);
  physAlive = MDIOPhyAliveStatusGet(nf_state->mdio_base);
  /* Find first alive PHY -- or use default if alive */
  if (!(physAlive & (1 << nf_state->phy_addr))) {
    for (index = 0; index < NUM_OF_PHYs; index++) {
      if (physAlive & (1 << index)) {
        nf_state->phy_addr = index;
        break;
      } else {
        /*
         * Try to 'wake up' PHY on 'index' address by
         * reading random register, making MDIO set
         * alive bit for current PHY
         */
        MDIOPhyRegRead(nf_state->mdio_base, index,
                       PHY_BMCR, &regContent);

        /* Get updated register */
        physAlive = MDIOPhyAliveStatusGet(nf_state->mdio_base);
        if (physAlive & (1 << index)) {
          nf_state->phy_addr = index;
          break;
        }
      }
    }

    if (!physAlive) {             /* FIXME je to ok? */
      tms570_eth_debug_printf("no phy found, phys: %d\n", physAlive);
      return NO_PHY_ALIVE;
    }
  }
  return ERR_OK;
}

static void
tms570_eth_init_set_pinmux(void)
{
#if defined(__rtems__)
  TMS570_IOMM.KICK_REG0 = 0x83E70B13U;
  TMS570_IOMM.KICK_REG1 = 0x95A4F1E0U;

  tms570_bsp_pin_set_function(TMS570_BALL_V5_MDCLK, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_G3_MDIO, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_H19_MII_TXEN, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_E18_MII_TXD_3, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_R2_MII_TXD_2, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_J19_MII_TXD_1, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_J18_MII_TXD_0, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_D19_MII_TX_CLK, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_H18_MII_RXD_3, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_G19_MII_RXD_2, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_A14_MII_RXD_1, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_P1_MII_RXD_0, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_K19_MII_RXCLK, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_N19_MII_RX_ER, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_B11_MII_RX_DV, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_B4_MII_CRS, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_set_function(TMS570_BALL_F3_MII_COL, TMS570_PIN_FNC_AUTO);
  tms570_bsp_pin_clear_function(TMS570_MMR_SELECT_GMII_SEL, TMS570_PIN_FNC_AUTO);

  TMS570_IOMM.KICK_REG0 = 0;
  TMS570_IOMM.KICK_REG1 = 0;
#endif /*__rtems__*/
}

static err_t
tms570_eth_init_hw(struct tms570_netif_state *nf_state)
{
  err_t retVal;

  tms570_eth_init_set_pinmux();

  /* Initialize EMAC control module and EMAC module */
  EMACInit(nf_state->emac_ctrl_base, nf_state->emac_base);
  /* Initialize MDIO module (reset) */
  MDIOInit(nf_state->mdio_base, 0x0, 0x0);

  if ((retVal = tms570_eth_init_find_PHY(nf_state)) != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_find_PHY: %d", retVal);
    return retVal;
  }
  /*
   * Start autonegotiation and check on completion later or
   * when complete link register will be updated
   */
  PHY_auto_negotiate(nf_state->mdio_base, nf_state->phy_addr,
                     PHY_100BASETXDUPL_m | PHY_100BASETX_m |
                     PHY_10BASETDUPL_m | PHY_10BASET_m);
  tms570_eth_debug_printf("autoneg started -- check on cable if it's connected!\r\n");

  /*
   * TODO: you can implement init of receive flow control somewhere
   * here if desired - set RXBUFFERFLOWEN in MACCONTROL
   */

  /* Acknowledge EMAC control module RX, TX and MISC interrupts */
  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_RX);
  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_TX);
  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_MISC);

  /* Sets which channel will receive broadcasts */
  EMACRxBroadCastEnable(nf_state->emac_base, CHANNEL);

  /*
   * Sets channel where all frames will be copied to --
   * either with MAC address different from local device address,
   * either packets with error will be copied; appropriate error
   * will be set in the frame EOP buffer descriptor
   */
  EMACRxPromiscEnable(nf_state->emac_base, CHANNEL);

  /* Enable unicast */
  EMACRxUnicastSet(nf_state->emac_base, CHANNEL);

  /* Enable TX and RX interrupts in both EMAC module and EMAC control module */
  EMACTxIntPulseEnable(nf_state->emac_base, nf_state->emac_ctrl_base, 0, CHANNEL);
  EMACRxIntPulseEnable(nf_state->emac_base, nf_state->emac_ctrl_base, 0, CHANNEL);

  return ERR_OK;
}
static void
tms570_eth_hw_set_hwaddr(struct tms570_netif_state *nf_state, uint8_t *mac_addr)
{
  uint8_t mac[ETHARP_HWADDR_LEN];
  int i;

  for (i = 0; i < ETHARP_HWADDR_LEN; i++) {
    mac[i] = mac_addr[ETHARP_HWADDR_LEN - i - 1];
  }
  /* for flow control frames */
  EMACMACSrcAddrSet(nf_state->emac_base, mac);

  /*  Be sure to program all eight MAC address registers -
   *  whether the receive channel is to be enabled or not.
   */
  for (i = 0; i < 8; i++) {
    EMACMACAddrSet(nf_state->emac_base, i, mac, 0);
  }
}

static err_t
tms570_eth_init_hw_post_init(struct tms570_netif_state *nf_state)
{
  /* 0x3FFFFFFF is for 80MHz aproximately 13s */
  uint16_t regContent;

  /* wait for autonegotiation to be done or continue, when delay was reached */
  uint32_t timeToWake = nf_state->waitTicksForPHYAneg + sys_jiffies();

  while (PHY_is_done_auto_negotiate(nf_state->mdio_base, nf_state->phy_addr) == false &&
         timeToWake > sys_jiffies())
    sys_arch_delay(20);
  /* XXX: if init is not done at the startup,
   * but couple days later, this might cause troubles */

  if (PHY_is_done_auto_negotiate(nf_state->mdio_base, nf_state->phy_addr) != false)
    tms570_eth_debug_printf("autoneg finished\n");
  else
    tms570_eth_debug_printf("autoneg timeout\n");

  /* provide informations retrieved from autoneg to EMAC module */
  PHY_partner_ability_get(nf_state->mdio_base, nf_state->phy_addr, &regContent);
  if (regContent & (PHY_100BASETXDUPL_m | PHY_10BASETDUPL_m)) {
    EMACDuplexSet(nf_state->emac_base, 1);
    /* this is right place to implement transmit flow control if desired --
     * set TXFLOWEN in MACCONTROL
     */
  } else if (regContent & (PHY_100BASETX_m | PHY_10BASET_m)) {
    EMACDuplexSet(nf_state->emac_base, 0);
  } else {
    tms570_eth_debug_printf("Unknown duplex mode\r\n");
    return UNKN_DUPLEX_MODE;
  }

  /* enable hostpend interrupts in emac module */
  nf_state->emac_base->MACINTMASKSET |= TMS570_EMACM_MACINTMASKSET_HOSTMASK;

  /* enable hostpend interrupts in emac control module */
  nf_state->emac_ctrl_base->C0MISCEN |= TMS570_EMACC_C0MISCEN_HOSTPENDEN;

  EMACMIIEnable(nf_state->emac_base);
  EMACTxEnable(nf_state->emac_base);
  EMACRxEnable(nf_state->emac_base);

  return ERR_OK;
}
static void
tms570_eth_init_buffer_descriptors(struct tms570_netif_state *nf_state)
{
  uint32_t num_bd;
  volatile struct emac_tx_bd *curr_txbd;
  volatile struct emac_rx_bd *curr_rxbd;

  struct rxch *rxch;
  struct txch *txch;

  rxch = &(nf_state->rxch);
  txch = &(nf_state->txch);


  /*
   * Use the CPPI RAM to store RX/TX Buffer Descriptors (= BD).
   * 1/2 of CPPI RAM is used for TX BDs, another 1/2 for RX BDs.
   * All the TX BDs are 'owned' by the software. They are initialized
   * as a linked-list forming a ring. They are awaiting the application
   * to append pbuf payload to the them and correctly configure for
   * actual transmission.
   * Only such number of RX BDs is configured that the pbufs can be
   * allocated for (i.e. MAX_RX_PBUF_ALLOC). Pbufs are allocated from
   * the PBUF_POOL (thus the allocated pbufs might be chained).
   * Each payload part of a payload is them used to initialize single
   * RX BD. The RX BDs are then configured to be 'owned' bythe EMAC
   */

  /*
   * Initialize the Descriptor Memory For TX and RX
   * Only Channel 0 is supported for both TX and RX
   */
  txch->inactive_head = (volatile struct emac_tx_bd *)nf_state->emac_ctrl_ram;
  txch->inactive_tail = NULL;
  txch->active_head = NULL;
  txch->active_tail = NULL;

  /* Set the number of descriptors for the channel */
  /* take half of CPPI ram for TX BDs */

  num_bd = ((SIZE_EMAC_CTRL_RAM >> 1) / sizeof(struct emac_tx_bd))-1;
#if TMS570_NETIF_DEBUG
  tms570_eth_debug_printf("pocet bd %d\n", num_bd);
#endif

  curr_txbd = txch->inactive_head;

  /* Initialize all the TX buffer Descriptors */
  while (num_bd > 0) {
    curr_txbd->next = curr_txbd + 1;
    curr_txbd->flags_pktlen = 0;
    txch->inactive_tail = curr_txbd;
    curr_txbd = curr_txbd->next;
    num_bd--;
  }
  curr_txbd->next = NULL;

  /* Initialize the descriptors for the RX channel */
  rxch->inactive_head = ((volatile struct emac_rx_bd *)curr_txbd)+2;
  rxch->freed_pbuf_len = MAX_RX_PBUF_ALLOC*PBUF_LEN_MAX;
  rxch->inactive_head->flags_pktlen = EMAC_DSC_FLAG_OWNER;
  rxch->inactive_head->next = rxch->inactive_head + 1;
  curr_rxbd = rxch->inactive_head;

  num_bd = ((SIZE_EMAC_CTRL_RAM >> 1) / sizeof(struct emac_rx_bd))-1;

  while (num_bd > 0) {
    curr_rxbd = curr_rxbd->next;
    curr_rxbd->next = curr_rxbd + 1;
    curr_rxbd->flags_pktlen = EMAC_DSC_FLAG_OWNER;
    curr_rxbd->pbuf = NULL;
    num_bd--;
  }
  curr_rxbd->next = NULL;
  rxch->inactive_tail = curr_rxbd;
  rxch->active_tail = NULL;
  rxch->active_head = NULL;
#if TMS570_NETIF_DEBUG
  tms570_eth_debug_show_rx(nf_state);
  tms570_eth_debug_show_tx(nf_state);
#endif

}

/* send and receive functions / ISRs ******************************************/

static err_t
tms570_eth_send(struct netif *netif, struct pbuf *p)
{
  err_t retVal = ERR_OK;

  SYS_ARCH_DECL_PROTECT(lev);

  /**
   * This entire function must be protected to preserve
   * the integrity of the transmit pbuf queue.
   */
  SYS_ARCH_PROTECT(lev);
#if !SYS_LIGHTWEIGHT_PROT
  sys_prot_t prevProt = sys_arch_protect()
                        /*
                              uint32_t prevProt = (uint32_t) _get_CPSR() & 0x80;
                              _disable_IRQ();
                        */
#endif

  /**
   * Bump the reference count on the pbuf to prevent it from being
   * freed until we are done with it.
   */
  pbuf_ref(p);

  /* call the actual transmit function */
  retVal = tms570_eth_send_raw(netif, p);

  /* Return to prior interrupt state and return. */
  SYS_ARCH_UNPROTECT(lev);
#if !SYS_LIGHTWEIGHT_PROT
  sys_arch_unprotect(prevProt);
  /*
        if (!prevProt)
                _enable_IRQ();
  */
#endif

  return retVal;
}

/**
 * When called from tms570_eth_send(), the 'lev' lock is held
 */
static err_t
tms570_eth_send_raw(struct netif *netif, struct pbuf *pbuf)
{
  struct pbuf *q;
  struct txch *txch;
  struct tms570_netif_state *nf_state;
  unsigned int pktlen;
  unsigned int padlen = 0;
  volatile struct emac_tx_bd *curr_bd;
  volatile struct emac_tx_bd *packet_head;
  volatile struct emac_tx_bd *packet_tail;

  nf_state = (struct tms570_netif_state *)netif->state;
  txch = &(nf_state->txch);

  /* Get the first BD that is unused and will be used for TX */
  curr_bd = txch->inactive_head;
  if (curr_bd == NULL)
    goto error_out_of_descriptors;

  packet_head = curr_bd;
  packet_tail = curr_bd;

  /* adjust the packet length if less than minimum required */
  pktlen = pbuf->tot_len;
  if (pktlen < MIN_PKT_LEN) {
    padlen = MIN_PKT_LEN - pktlen;
    pktlen = MIN_PKT_LEN;
  }

  /* First 'part' of packet flags */
  curr_bd->flags_pktlen = pktlen | EMAC_DSC_FLAG_SOP |
                          EMAC_DSC_FLAG_OWNER;

  /* Copy pbuf information into TX BDs --
   * remember that the pbuf for a single packet might be chained!
   */
  for (q = pbuf; q != NULL; q = q->next) {
    if (curr_bd == NULL)
      goto error_out_of_descriptors;

    curr_bd->bufptr = (uint8_t *)(q->payload);
    curr_bd->bufoff_len = (q->len) & 0xFFFF;

    /* This is an extra field that is not par of the in-HW BD.
     * This is used when freeing the pbuf after the TX processing
     * is done in EMAC
     */
    curr_bd->pbuf = pbuf;
    packet_tail = curr_bd;
    curr_bd = curr_bd->next;
  }
  if (padlen) {
    if (curr_bd == NULL)
      goto error_out_of_descriptors;

    /* If the ETHERNET packet is smaller than 64 bytes, it has
     * to be padded. We need some data and do not want to leak
     * random memory. Reuse IP and possibly TCP/UDP header
     * of given frame as padding
     */
    curr_bd->bufptr = packet_head->bufptr;
    curr_bd->bufoff_len = padlen;
    curr_bd->pbuf = pbuf;
    packet_tail = curr_bd;
    curr_bd = curr_bd->next;
  }
  /* Indicate the end of the packet */
  packet_tail->next = NULL;
  packet_tail->flags_pktlen |= EMAC_DSC_FLAG_EOP;

  txch->inactive_head = curr_bd;
  if (curr_bd == NULL)
    txch->inactive_tail = curr_bd;

  sys_arch_data_sync_barier();

  if (txch->active_tail == NULL) {
    txch->active_head = packet_head;
    tms570_eth_hw_set_TX_HDP(nf_state, packet_head);
  } else {
    /* Chain the bd's. If the DMA engine already reached the
     * end of the chain, the EOQ will be set. In that case,
     * the HDP shall be written again.
     */
    txch->active_tail->next = packet_head;
    curr_bd = txch->active_tail;

    /* We were too slow and the EMAC already read the
     * 'pNext = NULL' of the former txch->active_tail. In this
     * case the transmission stopped and we need to write the
     * pointer to newly added BDs to the TX HDP
     */
    if (curr_bd->flags_pktlen & EMAC_DSC_FLAG_EOQ) {
      tms570_eth_hw_set_TX_HDP(nf_state, packet_head);
    }
  }
  txch->active_tail = packet_tail;

  return ERR_OK;

error_out_of_descriptors:
  pbuf_free(pbuf);
  return ERR_IF;
}

/* EMAC Packet Buffer Sizes and Placement */
#ifdef CONFIG_EMAC_PKT_FRAG_SIZE
  #define EMAC_FRAG_SIZE    CONFIG_EMAC_PKT_FRAG_SIZE
#else
  #define EMAC_FRAG_SIZE      1536
#endif

#ifdef CONFIG_EMAC_ETH_FRAME_SIZE
  #define EMAC_MAX_FLEN    CONFIG_EMAC_ETH_FRAME_SIZE
#else
  #define EMAC_MAX_FLEN       1536
#endif

#ifdef CONFIG_EMAC_NUM_RX_FRAGS
  #define EMAC_NUM_RX_FRAG    CONFIG_EMAC_NUM_RX_FRAGS
#else
  #define EMAC_NUM_RX_FRAG    4
#endif

#ifdef CONFIG_EMAC_NUM_TX_FRAGS
  #define EMAC_NUM_TX_FRAG    CONFIG_EMAC_NUM_TX_FRAGS
#else
  #define EMAC_NUM_TX_FRAG    2
#endif

static void
tms570_eth_process_irq_rx(void *arg)
{
  struct tms570_netif_state *nf_state;
  struct rxch *rxch;
  struct netif *netif = (struct netif *)arg;
  volatile struct emac_rx_bd *curr_bd;
  struct pbuf *pbuf;
  struct pbuf *q;

  nf_state = netif->state;
  rxch = &(nf_state->rxch);
  /* Get the bd which contains the earliest filled data */
  curr_bd = rxch->active_head;
  if (curr_bd == NULL) {
    tms570_eth_rx_pbuf_refill(nf_state, 0);
    return;
  }

  /* For each valid frame */
  while ((curr_bd->flags_pktlen & EMAC_DSC_FLAG_SOP) &&
         !(curr_bd->flags_pktlen & EMAC_DSC_FLAG_OWNER)) {
    unsigned int total_rx_len;
    unsigned int processed_rx_len = 0;
    int corrupt_fl = 0;

    sys_arch_data_sync_barier();

    pbuf = curr_bd->pbuf;
    total_rx_len = curr_bd->flags_pktlen & 0xFFFF;
    tms570_eth_debug_printf("recieve packet. L = %d ", total_rx_len);
    /* The received frame might be fragmented into muliple
     * pieces -- each one referenced by a separate BD.
     * To further process the data, we need to 'make' a
     * proper PBUF out of it -- that means linking each
     * buffer together, copy the length information form
     * the DB to PBUF, calculate the 'tot_len' etc.
     */
    for (;; ) {
      q = curr_bd->pbuf;
      /* Since this pbuf will be freed, we need to
       * keep track of its size to be able to
       * allocate it back again
       */
      rxch->freed_pbuf_len += q->len;
      tms570_eth_debug_printf("bd - %d ", tms570_eth_debug_get_BD_num(curr_bd, nf_state));
      tms570_eth_debug_printf("pbuf len - %d ", q->len);
      tms570_eth_debug_printf("A - 0x%08x ", q);
      /* This is the size of the "received data" not the PBUF */
      q->tot_len = total_rx_len - processed_rx_len;
      q->len = curr_bd->bufoff_len & 0xFFFF;

      if (curr_bd->flags_pktlen & EMAC_DSC_FLAG_EOP)
        break;
      /*
       * If we are executing here, it means this
       * packet is being split over multiple BDs
       */
      tms570_eth_debug_printf("MB");
      /* chain the pbufs since they belong
       * to the same packet
       */
      if (curr_bd->next == NULL) {
        corrupt_fl = 1;
        break;
      }
      curr_bd = curr_bd->next;
      q->next = curr_bd->pbuf;

      processed_rx_len += q->len;
    }
    tms570_eth_debug_printf("\n");
    /* Close the chain */
    q->next = NULL;
    if (rxch->inactive_tail == NULL) {
      rxch->inactive_head = rxch->active_head;
    } else {
      rxch->inactive_tail->next = rxch->active_head;
    }
    rxch->inactive_tail = curr_bd;
    rxch->active_head = curr_bd->next;
    if (curr_bd->next == NULL)
      rxch->active_tail = NULL;
    rxch->inactive_tail->next = NULL;


    LINK_STATS_INC(link.recv);

    /* Process the packet */
    /* ethernet_input((struct pbuf *)pbuf, netif) */
    if (!corrupt_fl)
      if (netif->input(pbuf, netif) != ERR_OK)
        corrupt_fl = 1;
    if (corrupt_fl) {
      LINK_STATS_INC(link.memerr);
      LINK_STATS_INC(link.drop);
      pbuf_free(pbuf);
    }

    /* Acknowledge that this packet is processed */
    EMACRxCPWrite(nf_state->emac_base, 0, (unsigned int)curr_bd);

    /* The earlier PBUF chain is freed from the upper layer.
     * So, we need to allocate a new pbuf chain and update
     * the descriptors with the PBUF info.
     * Care should be taken even if the allocation fails.
     */
    tms570_eth_rx_pbuf_refill(nf_state, 0);
    //tms570_eth_debug_print_rxch();
    curr_bd = rxch->active_head;
    if (curr_bd == NULL) {
      return;
    }
  }
}

static void
tms570_eth_process_irq_tx(void *arg)
{

  struct txch *txch;
  volatile struct emac_tx_bd *curr_bd;
  volatile struct emac_tx_bd *start_of_packet_bd;
  struct netif *netif = (struct netif *)arg;
  struct tms570_netif_state *nf_state;

  nf_state = netif->state;
  txch = &(nf_state->txch);

  start_of_packet_bd = txch->active_head;
  curr_bd = txch->active_head;

  /* Traverse the list of BDs used for transmission --
   * stop on the first unused
   */
  while ((curr_bd != NULL) && (curr_bd->flags_pktlen & EMAC_DSC_FLAG_SOP)) {
    /* Make sure the transmission is over */
    if (curr_bd->flags_pktlen & EMAC_DSC_FLAG_OWNER) {
      tms570_eth_debug_printf("TXthread ownership not transfered!!!!\n");
      break;
    }

    /* Find the last chunk of the packet */
    while (!(curr_bd->flags_pktlen & EMAC_DSC_FLAG_EOP))
      curr_bd = curr_bd->next;

    /* Remove flags for the transmitted BDs */
    start_of_packet_bd->flags_pktlen &= (~EMAC_DSC_FLAG_SOP);
    curr_bd->flags_pktlen &= (~EMAC_DSC_FLAG_EOP);

    /*
     * Return processed BDs to inactive list
     */
    if (txch->inactive_tail == NULL) {
      txch->inactive_head = start_of_packet_bd;
    } else {
      txch->inactive_tail->next = start_of_packet_bd;
    }
    txch->inactive_tail = curr_bd;

    /*
     * Remove BDs from active list
     */
    txch->active_head = curr_bd->next;
    if (curr_bd->next == NULL) {
      txch->active_tail = NULL;
    }

    /*
     * Insert null element at the end of the inactive list
                       */
    txch->inactive_tail->next = NULL;


    /* Ack the Interrupt in the EMAC peripheral */
    EMACTxCPWrite(nf_state->emac_base, CHANNEL,
                  (uint32_t)curr_bd);

    /* Free the corresponding pbuf
     * Sidenote: Each fragment of the single packet points
     * to the same pbuf
     */
    pbuf_free(start_of_packet_bd->pbuf);

    LINK_STATS_INC(link.xmit);

    /* Move to the next packet */
    start_of_packet_bd = txch->active_head;
    curr_bd = txch->active_head;
  }
}

static void
tms570_eth_process_irq(void *argument)
{
  struct netif *netif = (struct netif *)argument;
  struct tms570_netif_state *nf_state;
  uint32_t macints;

  nf_state = netif->state;

  if (nf_state == NULL)
    return;

  while (1) {
    macints = nf_state->emac_base->MACINVECTOR;
    if ((macints & 0xffff) == 0) {
      break;
    }
    if (macints & (0xff<<16)) { //TX interrupt
      tms570_eth_process_irq_tx(netif);
      EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_TX);
    }
    if (macints & (0xff<<0)) { //RX interrupt
      tms570_eth_process_irq_rx(netif);
      EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_RX);
    }
  }
  sys_arch_unmask_interrupt_source(TMS570_IRQ_EMAC_RX);
  sys_arch_unmask_interrupt_source(TMS570_IRQ_EMAC_TX);
}

void static
tms570_eth_process_irq_request(void *argument)
{
  struct netif *netif = (struct netif *)argument;
  struct tms570_netif_state *nf_state;

  nf_state = netif->state;

  for (;; ) {
    sys_arch_sem_wait(&nf_state->intPend_sem, 0);
    tcpip_callback((tcpip_callback_fn)tms570_eth_process_irq, netif);
  }
}

static void
tms570_eth_hw_set_RX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_rx_bd *new_head)
{
  /* Writes to RX HDP are allowed
                                                 * only when it is 0
                                                 */
  while (nf_state->emac_base->RXHDP[CHANNEL] != 0) {
    tms570_eth_debug_printf("HW -RX- is slacking!!!\n");
    sys_arch_delay(10);
  }
  tms570_eth_debug_printf("setting RX HDP");
  EMACRxHdrDescPtrWrite(
    nf_state->emac_base,
    (uint32_t)new_head,
    CHANNEL);
}
static void
tms570_eth_hw_set_TX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_tx_bd *new_head)
{
  /* Writes to RX HDP are allowed
                                                 * only when it is 0
                                                 */
  while (nf_state->emac_base->TXHDP[CHANNEL] != 0) {
    tms570_eth_debug_printf("HW -TX- is slacking!!!\n");
    sys_arch_delay(10);
  }
  tms570_eth_debug_printf("setting TX HDP");
  EMACTxHdrDescPtrWrite(
    nf_state->emac_base,
    (uint32_t)new_head,
    CHANNEL);
}

static void
tms570_eth_rx_pbuf_refill_single(struct netif *netif)
{
  tms570_eth_rx_pbuf_refill(netif->state, 1);
}

static void
tms570_eth_rx_pbuf_refill(struct tms570_netif_state *nf_state, int single_fl)
{
  struct rxch *rxch;
  volatile struct emac_rx_bd *curr_bd;
  volatile struct emac_rx_bd *curr_head;
  struct pbuf *new_pbuf;
  struct pbuf *q;
  uint32_t alloc_rq_bytes;

  rxch = &(nf_state->rxch);
  if (single_fl) {
    alloc_rq_bytes = PBUF_POOL_BUFSIZE;
  } else {
    alloc_rq_bytes = rxch->freed_pbuf_len;
  }

  for (; (rxch->freed_pbuf_len > 0) && (rxch->inactive_head != NULL); ) {
    //stats_display();

    curr_bd = rxch->inactive_head;
    curr_head = rxch->inactive_head;
    tms570_eth_debug_printf("attempt to allocate %d bytes from pbuf pool (RX)\n", alloc_rq_bytes);
    new_pbuf = pbuf_alloc(PBUF_RAW,
                          alloc_rq_bytes,
                          PBUF_POOL);
    if (new_pbuf == NULL) {
  #if defined(__GNUC__) && !defined(__TI_COMPILER_VERSION__)
      alloc_rq_bytes = (1 << (30-__builtin_clz(alloc_rq_bytes)));
  #else /*__GNUC__*/
      {
        int n = 0;
        while ((1 << n) < alloc_rq_bytes)
          n++;
        alloc_rq_bytes = 1 << (n - 1);
      }
  #endif /*__GNUC__*/
      if (alloc_rq_bytes <= PBUF_POOL_BUFSIZE) {
        tms570_eth_debug_printf("not enough memory\n");
        break;
      }
      alloc_rq_bytes = rxch->freed_pbuf_len > alloc_rq_bytes ?
                       alloc_rq_bytes : rxch->freed_pbuf_len;
      continue;
    } else {
      q = new_pbuf;
      for (;; ) {
        curr_bd->bufptr = (uint8_t *)q->payload;
        curr_bd->bufoff_len = q->len;
        curr_bd->flags_pktlen = EMAC_DSC_FLAG_OWNER;
        curr_bd->pbuf = q;
        rxch->freed_pbuf_len -= q->len;
        q = q->next;
        if (q == NULL)
          break;
        if (curr_bd->next == NULL) {
          rxch->inactive_tail = NULL;
          break;
        }
        curr_bd = curr_bd->next;
      }

      if (q != NULL)
        pbuf_free((struct pbuf *)q);
      /* Add the newly allocated BDs to the
       * end of the list
       */
      rxch->inactive_head = curr_bd->next;

      curr_bd->next = NULL;
      sys_arch_data_sync_barier();

      if (rxch->active_head == NULL) {
        rxch->active_head = curr_head;
        rxch->active_tail = curr_bd;
        tms570_eth_hw_set_RX_HDP(nf_state, rxch->active_head);
      } else {
        rxch->active_tail->next = curr_head;
        sys_arch_data_sync_barier();
        if ((rxch->active_tail->flags_pktlen & EMAC_DSC_FLAG_EOQ) != 0)
          tms570_eth_hw_set_RX_HDP(nf_state, rxch->active_head);
        rxch->active_tail = curr_bd;
      }
    }
  }
}

#if !defined(__TI_COMPILER_VERSION__)
static
#endif /*__TI_COMPILER_VERSION__*/
SYS_IRQ_HANDLER_FNC(tms570_eth_irq){
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)sys_irq_handler_get_context();

  sys_arch_mask_interrupt_source(TMS570_IRQ_EMAC_RX);
  sys_arch_mask_interrupt_source(TMS570_IRQ_EMAC_TX);
  if (nf_state != NULL)
    sys_sem_signal_from_ISR(&nf_state->intPend_sem);
}

void
tms570_eth_memp_avaible(int type)
{
  if (!initialized)
    return;
  if (type != MEMP_PBUF_POOL)
    return;
  netifapi_netif_common(rtems_lwip_get_netif(0), tms570_eth_rx_pbuf_refill_single, NULL);
}

void sys_arch_data_sync_barier() {
    _ARM_Data_synchronization_barrier();
}

#if TMS570_NETIF_DEBUG

void
tms570_eth_debug_print_info(struct netif *netif)
{
  struct tms570_netif_state *nf_state = netif->state;

  tms570_eth_debug_show_rx(nf_state);
  tms570_eth_debug_show_tx(nf_state);
  tms570_eth_debug_print_HDP(nf_state);
}

void
tms570_eth_debug_print_HDP(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("RX HDP = %d\n", tms570_eth_debug_get_BD_num(
                            (void *)nf_state->emac_base->RXHDP[0], nf_state));
  tms570_eth_debug_printf("TX HDP = %d\n", tms570_eth_debug_get_BD_num(
                            (void *)nf_state->emac_base->TXHDP[0], nf_state));
}

int
tms570_eth_debug_get_BD_num(volatile void *ptr, struct tms570_netif_state *nf_state)
{
  if (ptr == NULL)
    return -1;
  return ((uintptr_t)ptr-nf_state->emac_ctrl_ram)/sizeof(struct emac_rx_bd);
}

void
tms570_eth_debug_print_rxch(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("inactive_head = %d, inactive_tail = %d, active_head = %d, active_tail = %d, freed_pbuf_len = %d\n",
                          tms570_eth_debug_get_BD_num(nf_state->rxch.inactive_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->rxch.inactive_tail, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->rxch.active_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->rxch.active_tail, nf_state),
                          nf_state->rxch.freed_pbuf_len);
}
void
tms570_eth_debug_print_txch(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("inactive_head = %d, inactive_tail = %d, active_head = %d, active_tail = %d\n",
                          tms570_eth_debug_get_BD_num(nf_state->txch.inactive_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->txch.inactive_tail, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->txch.active_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->txch.active_tail, nf_state));
}
void
tms570_eth_debug_show_BD_chain_rx(volatile struct emac_rx_bd *curr_bd,
                                  struct tms570_netif_state *nf_state)
{
  int count = 0;

  while (curr_bd != NULL) {
    tms570_eth_debug_printf("%d ", tms570_eth_debug_get_BD_num(curr_bd, nf_state));
    curr_bd = curr_bd->next;
    count++;
  }
  tms570_eth_debug_printf(" count = %d\n", count);
}

void
tms570_eth_debug_show_BD_chain_tx(volatile struct emac_tx_bd *curr_bd,
                                  struct tms570_netif_state *nf_state)
{
  int count = 0;

  while (curr_bd != NULL) {
    tms570_eth_debug_printf("%d ", tms570_eth_debug_get_BD_num(curr_bd, nf_state));
    curr_bd = curr_bd->next;
    count++;
  }
  tms570_eth_debug_printf(" count = %d\n", count);
}

void
tms570_eth_debug_show_rx(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("!!!RX!!!\n");
  tms570_eth_debug_print_rxch(nf_state);
  tms570_eth_debug_printf("inactive chain\n");
  tms570_eth_debug_show_BD_chain_rx(nf_state->rxch.inactive_head, nf_state);
  tms570_eth_debug_printf("active chain\n");
  tms570_eth_debug_show_BD_chain_rx(nf_state->rxch.active_head, nf_state);
}
void
tms570_eth_debug_show_tx(struct tms570_netif_state *nf_state)
{

  tms570_eth_debug_printf("!!!TX!!!\n");
  tms570_eth_debug_print_txch(nf_state);
  tms570_eth_debug_printf("inactive chain\n");
  tms570_eth_debug_show_BD_chain_tx(nf_state->txch.inactive_head, nf_state);
  tms570_eth_debug_printf("active chain\n");
  tms570_eth_debug_show_BD_chain_tx(nf_state->txch.active_head, nf_state);
}
#endif /* if TMS570_NETIF_DEBUG */
