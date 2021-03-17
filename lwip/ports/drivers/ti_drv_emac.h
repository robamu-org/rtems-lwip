/*
* Copyright (C) 2009-2015 Texas Instruments Incorporated - www.ti.com
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/*****************************************************************************/
#include <stdint.h>
#include <bsp/tms570.h>
/*
** Macros which can be used as matchFilt  parameters to the API
** EMACMACAddrSet
*/

/*
** Macros which can be passed as eoiFlag to EMACRxThreshIntAckToClear API
*/
#define EMAC_INT_CORE0_RX_THRSH               (0x0u)
#define EMAC_INT_CORE1_RX_THRSH               (0x4u)
#define EMAC_INT_CORE2_RX_THRSH               (0x8u)

/*
** Macros which can be passed as eoiFlag to EMACRxIntAckToClear API
*/
#define EMAC_INT_CORE0_RX                     (0x1u)
#define EMAC_INT_CORE1_RX                     (0x5u)
#define EMAC_INT_CORE2_RX                     (0x9u)

/*
** Macros which can be passed as eoiFlag to EMACTxIntAckToClear API
*/
#define EMAC_INT_CORE0_TX                     (0x2u)
#define EMAC_INT_CORE1_TX                     (0x6u)
#define EMAC_INT_CORE2_TX                     (0xAu)

/*
** Macros which can be passed as eoiFlag to EMACMiscIntAckToClear API
** STATPEND, HOSTPEND, MDIO LINKINT0, MDIO USERINT0
*/
#define EMAC_INT_CORE0_MISC                   (0x3u)
#define EMAC_INT_CORE1_MISC                   (0x7u)
#define EMAC_INT_CORE2_MISC                   (0xBu)

/*****************************************************************************/
/**
 * \brief   Enables the TXPULSE Interrupt Generation.
 *
 * \param   emacBase      Base address of the EMAC Module registers.
 * \param   emacCtrlBase  Base address of the EMAC CONTROL module registers
 * \param   ctrlCore      Control core for which the interrupt to be enabled.
 * \param   channel       Channel number for which interrupt to be enabled
 *
 * \return  None
 *
 **/
static inline void
EMACTxIntPulseEnable(volatile tms570_emacm_t *emacBase, volatile tms570_emacc_t *emacCtrlBase,
                     unsigned int ctrlCore, unsigned int channel)
{
  emacBase->TXINTMASKSET |= (1 << channel);
  emacCtrlBase->C0TXEN |= (1 << channel);
}

/**
 * \brief   Disables the TXPULSE Interrupt Generation.
 *
 * \param   emacBase      Base address of the EMAC Module registers.
 * \param   emacCtrlBase  Base address of the EMAC CONTROL module registers
 * \param   ctrlCore      Control core for which the interrupt to be disabled.
 * \param   channel       Channel number for which interrupt to be disabled
 *
 * \return  None
 *
 **/
static inline void
EMACTxIntPulseDisable(volatile tms570_emacm_t *emacBase, volatile tms570_emacc_t *emacCtrlBase,
                      unsigned int ctrlCore, unsigned int channel)
{
  emacBase->TXINTMASKCLEAR |= (1 << channel);
  emacCtrlBase->C0TXEN &= ~(1 << channel);
}

/**
 * \brief   Enables the RXPULSE Interrupt Generation.
 *
 * \param   emacBase      Base address of the EMAC Module registers.
 * \param   emacCtrlBase  Base address of the EMAC CONTROL module registers
 * \param   ctrlCore      Control core for which the interrupt to be enabled.
 * \param   channel       Channel number for which interrupt to be enabled
 *
 * \return  None
 *
 **/
static inline void
EMACRxIntPulseEnable(volatile tms570_emacm_t *emacBase, volatile tms570_emacc_t *emacCtrlBase,
                     unsigned int ctrlCore, unsigned int channel)
{
  emacBase->RXINTMASKSET |= (1 << channel);
  emacCtrlBase->C0RXEN |= (1 << channel);
}

/**
 * \brief   This API enables the MII control block
 *
 * \param   emacBase     Base address of the EMAC Module registers.
 *
 * \return  None
 *
 **/
static inline void
EMACMIIEnable(volatile tms570_emacm_t *emacBase)
{
  emacBase->MACCONTROL |= TMS570_EMACM_MACCONTROL_GMIIEN;
}

/**
 * \brief   This API sets the duplex mode of operation(full/half) for MAC.
 *
 * \param   emacBase     Base address of the EMAC Module registers.
 * \param   duplexMode   duplex mode of operation.
 *          duplexMode can take the following values. \n
 *                nonZero - Full Duplex  \n
 *                0 - Half Duplex.
 *
 * \return  None
 *
 **/
static inline void
EMACDuplexSet(volatile tms570_emacm_t *emacBase, int duplexMode)
{
  if (duplexMode) {
    emacBase->MACCONTROL |= TMS570_EMACM_MACCONTROL_FULLDUPLEX;
  } else {
    emacBase->MACCONTROL &= ~TMS570_EMACM_MACCONTROL_FULLDUPLEX;
  }
}

/**
 * \brief   API to enable the transmit in the TX Control Register
 *          After the transmit is enabled, any write to TXHDP of
 *          a channel will start transmission
 *
 * \param   emacBase      Base Address of the EMAC Module Registers.
 *
 * \return  None
 *
 **/
static inline void
EMACTxEnable(volatile tms570_emacm_t *emacBase)
{
  emacBase->TXCONTROL = TMS570_EMACM_TXCONTROL_TXEN;
}

/**
 * \brief   API to enable the receive in the RX Control Register
 *          After the transmit is enabled, and write to RXHDP of
 *          a channel, the data can be received in the destination
 *          specified by the corresponding RX buffer descriptor.
 *
 * \param   emacBase      Base Address of the EMAC Module Registers.
 *
 * \return  None
 *
 **/
static inline void
EMACRxEnable(volatile tms570_emacm_t *emacBase)
{
  emacBase->RXCONTROL = TMS570_EMACM_RXCONTROL_RXEN;
}

/**
 * \brief   API to write the TX HDP register. If transmit is enabled,
 *          write to the TX HDP will immediately start transmission.
 *          The data will be taken from the buffer pointer of the TX buffer
 *          descriptor written to the TX HDP
 *
 * \param   emacBase      Base Address of the EMAC Module Registers.\n
 * \param   descHdr       Address of the TX buffer descriptor
 * \param   channel       Channel Number
 *
 * \return  None
 *
 **/
static inline void
EMACTxHdrDescPtrWrite(volatile tms570_emacm_t *emacBase, unsigned int descHdr,
                      unsigned int channel)
{
  emacBase->TXHDP[channel] = descHdr;
}

/**
 * \brief   API to write the RX HDP register. If receive is enabled,
 *          write to the RX HDP will enable data reception to point to
 *          the corresponding RX buffer descriptor's buffer pointer.
 *
 * \param   emacBase      Base Address of the EMAC Module Registers.\n
 * \param   descHdr       Address of the RX buffer descriptor
 * \param   channel       Channel Number
 *
 * \return  None
 *
 **/
static inline void
EMACRxHdrDescPtrWrite(volatile tms570_emacm_t *emacBase, unsigned int descHdr,
                      unsigned int channel)
{
  emacBase->RXHDP[channel] = descHdr;
}

/**
 * \brief   Sets the MAC Address in MACSRCADDR registers.
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   macAddr       Start address of a MAC address array.
 *                        The array[0] shall be the LSB of the MAC address
 *
 * \return  None
 *
 **/
static inline void
EMACMACSrcAddrSet(volatile tms570_emacm_t *emacBase, unsigned char *macAddr)
{
  emacBase->MACSRCADDRHI = macAddr[5] |(macAddr[4] << 8)
                           |(macAddr[3] << 16) |(macAddr[2] << 24);
  emacBase->MACSRCADDRLO = macAddr[1] | (macAddr[0] << 8);
}

/**
 * \brief   Sets the MAC Address in MACADDR registers.
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   channel       Channel Number
 * \param   matchFilt     Match or Filter
 * \param   macAddr       Start address of a MAC address array.
 *                        The array[0] shall be the LSB of the MAC address
 *          matchFilt can take the following values \n
 *          EMAC_MACADDR_NO_MATCH_NO_FILTER - Address is not used to match
 *                                             or filter incoming packet. \n
 *          EMAC_MACADDR_FILTER - Address is used to filter incoming packets \n
 *          EMAC_MACADDR_MATCH - Address is used to match incoming packets \n
 *
 * \return  None
 *
 **/
static inline void
EMACMACAddrSet(volatile tms570_emacm_t *emacBase, unsigned int channel,
               unsigned char *macAddr, unsigned int matchFilt)
{
  emacBase->MACINDEX = channel;

  emacBase->MACADDRHI = macAddr[5] |(macAddr[4] << 8)
                        |(macAddr[3] << 16) |(macAddr[2] << 24);
  emacBase->MACADDRLO = macAddr[1] | (macAddr[0] << 8)
                        | matchFilt | (channel << 16);
}

/**
 * \brief   Acknowledges an interrupt processed to the EMAC Control Core.
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   eoiFlag       Type of interrupt to acknowledge to the EMAC Control
 *                         module.
 *          eoiFlag can take the following values \n
 *             EMAC_INT_CORE0_TX - Core 0 TX Interrupt
 *             EMAC_INT_CORE1_TX - Core 1 TX Interrupt
 *             EMAC_INT_CORE2_TX - Core 2 TX Interrupt
 *             EMAC_INT_CORE0_RX - Core 0 RX Interrupt
 *             EMAC_INT_CORE1_RX - Core 1 RX Interrupt
 *             EMAC_INT_CORE2_RX - Core 2 RX Interrupt
 * \return  None
 *
 **/
static inline void
EMACCoreIntAck(volatile tms570_emacm_t *emacBase, unsigned int eoiFlag)
{
  /* Acknowledge the EMAC Control Core */
  emacBase->MACEOIVECTOR = eoiFlag;
}

/**
 * \brief   Writes the the RX Completion Pointer for a specific channel
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   channel       Channel Number.
 * \param   comPtr        Completion Pointer Value to be written
 *
 * \return  None
 *
 **/
static inline void
EMACRxCPWrite(volatile tms570_emacm_t *emacBase, unsigned int channel, unsigned int comPtr)
{
  emacBase->RXCP[channel] = comPtr;
}

/**
 * \brief   Acknowledges an interrupt processed to the EMAC module. After
 *          processing an interrupt, the last processed buffer descriptor is
 *          written to the completion pointer. Also this API acknowledges
 *          the EMAC Control Module that the RX interrupt is processed for
 *          a specified core
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   channel       Channel Number
 * \param   comPtr        Completion Pointer value. This shall be the buffer
 *                        descriptor address last processed.
 * \param   eoiFlag       Type of interrupt to acknowledge to the EMAC Control
                          module.
 *          eoiFlag can take the following values \n
 *             EMAC_INT_CORE0_RX - Core 0 RX Interrupt
 *             EMAC_INT_CORE1_RX - Core 1 RX Interrupt
 *             EMAC_INT_CORE2_RX - Core 2 RX Interrupt
 * \return  None
 *
 **/
static inline void
EMACRxIntAckToClear(volatile tms570_emacm_t *emacBase, unsigned int channel,
                    unsigned int comPtr, unsigned eoiFlag)
{
  emacBase->RXCP[channel] = comPtr;

  /* Acknowledge the EMAC Control Core */
  emacBase->MACEOIVECTOR = eoiFlag;
}

/**
 * \brief   Enables a specific channel to receive broadcast frames
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   channel       Channel Number.
 *
 * \return  None
 *
 **/
static inline void
EMACRxBroadCastEnable(volatile tms570_emacm_t *emacBase, unsigned int channel)
{
  emacBase->RXMBPENABLE &= ~TMS570_EMACM_RXMBPENABLE_RXBROADCH(-1);   //CHECK

  emacBase->RXMBPENABLE |=
    TMS570_EMACM_RXMBPENABLE_RXBROADEN |
    TMS570_EMACM_RXMBPENABLE_RXBROADCH(channel);
}

static inline void
EMACRxPromiscEnable(volatile tms570_emacm_t *emacBase, unsigned int channel)
{
  emacBase->RXMBPENABLE |= TMS570_EMACM_RXMBPENABLE_RXCAFEN | TMS570_EMACM_RXMBPENABLE_RXCEFEN;
  emacBase->RXMBPENABLE |= TMS570_EMACM_RXMBPENABLE_RXPROMCH(channel);
}
/**
 * \brief   Enables unicast for a specific channel
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   channel       Channel Number.
 *
 * \return  None
 *
 **/
static inline void
EMACRxUnicastSet(volatile tms570_emacm_t *emacBase, unsigned int channel)
{
  emacBase->RXUNICASTSET |= (1 << channel);
}


/**
 * \brief   Gets the interrupt vectors of EMAC, which are pending
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 *
 * \return  Vectors
 *
 **/
static inline unsigned int
EMACIntVectorGet(volatile tms570_emacm_t *emacBase)
{
  return (emacBase->MACINVECTOR);
}

/**
 * \brief   Writes the the TX Completion Pointer for a specific channel
 *
 * \param   emacBase      Base Address of the EMAC module registers.
 * \param   channel       Channel Number.
 * \param   comPtr        Completion Pointer Value to be written
 *
 * \return  None
 *
 **/
static inline void
EMACTxCPWrite(volatile tms570_emacm_t *emacBase, unsigned int channel, unsigned int comPtr)
{
  emacBase->TXCP[channel] = comPtr;
}

/**
 * \brief   This API Initializes the EMAC and EMAC Control modules. The
 *          EMAC Control module is reset, the CPPI RAM is cleared. also,
 *          all the interrupts are disabled. This API doesnot enable any
 *          interrupt or operation of the EMAC.
 *
 * \param   emacCtrlBase      Base Address of the EMAC Control module
 *                            registers.\n
 * \param   emacBase          Base address of the EMAC module registers
 *
 * \return  None
 *
 **/
static inline void
EMACInit(volatile tms570_emacc_t *emacCtrlBase, volatile tms570_emacm_t *emacBase)
{
  unsigned int cnt;

  /* Reset the EMAC Control Module. This clears the CPPI RAM also */
  emacCtrlBase->SOFTRESET = TMS570_EMACC_SOFTRESET_RESET;
  while (emacCtrlBase->SOFTRESET & TMS570_EMACC_SOFTRESET_RESET);

  /* Reset the EMAC Control Module. This clears the CPPI RAM also */
  emacBase->SOFTRESET = TMS570_EMACM_SOFTRESET_SOFTRESET;

  while (emacBase->SOFTRESET & TMS570_EMACM_SOFTRESET_SOFTRESET);

  emacBase->MACCONTROL = 0;
  emacBase->RXCONTROL = 0;
  emacBase->TXCONTROL = 0;

  /* Initialize all the header descriptor pointer registers */
  for (cnt =  0; cnt < sizeof(emacBase->RXHDP)/sizeof(emacBase->RXHDP[0]); cnt++) { //CHECK
    emacBase->RXHDP[cnt] = 0;
    emacBase->TXHDP[cnt] = 0;
    emacBase->RXCP[cnt] = 0;
    emacBase->TXCP[cnt] = 0;
    emacBase->RXFREEBUFFER[cnt] = 0xFF;
  }
  /* Clear the interrupt enable for all the channels */
  emacBase->TXINTMASKCLEAR = 0xFF;
  emacBase->RXINTMASKCLEAR = 0xFF;

  emacBase->MACHASH1 = 0;
  emacBase->MACHASH2 = 0;

  emacBase->RXBUFFEROFFSET = 0;
}
