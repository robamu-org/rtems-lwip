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

#include "ti_drv_mdio.h"
#include "phy_dp83848h.h"

#ifndef TRUE
/**
 * Boolean definition for TRUE
 */
#define TRUE 1
#endif


#ifndef FALSE
/**
 * Boolean definition for FALSE
 */
#define FALSE 0
#endif

void
PHY_reset(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr)
{
  volatile unsigned short regContent;

  MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_BMCR, PHY_RESET_m);
  while (MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMCR, &regContent) & PHY_RESET_m);
}

uint32_t
PHY_partner_ability_get(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, unsigned short *regContent)
{
  return (MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_ANLPAR, regContent));
}

uint32_t
PHY_start_auto_negotiate(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, unsigned short advVal)
{
  volatile unsigned short regContent = 0;

  /* Enable Auto Negotiation */
  if (MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMCR, &regContent) != TRUE) {
    return FALSE;
  }
  regContent |= PHY_AUTONEG_EN_m;
  MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_BMCR, regContent);       /* originally ...HY_BMCR, PHY_RESET_m | PHY_AUTONEG_EN_m); */

  /* Write Auto Negotiation capabilities */
  if (MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_ANAR, &regContent) != TRUE) {
    return FALSE;
  }
  regContent |= advVal;
  MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_ANAR, regContent);

  /* Start Auto Negotiation */
  MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMCR, &regContent);
  regContent |= PHY_AUTONEG_REST;
  MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_BMCR, regContent);

  return TRUE;   /* request to PHY through EMAC for autonegotiation established */
}

uint32_t
PHY_is_done_auto_negotiate(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr)
{
  volatile unsigned short regContent;

  if (MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMSR, &regContent) != TRUE) {
    return FALSE;
  }
  if ((regContent & PHY_A_NEG_COMPLETE_m) == 0)
    return FALSE;
  return TRUE;
}

uint32_t
PHY_auto_negotiate(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, unsigned short advVal)
{
  if (PHY_start_auto_negotiate(mdioBaseAddr, phyAddr, advVal) == FALSE)
    return FALSE;

  while (PHY_is_done_auto_negotiate(mdioBaseAddr, phyAddr) == FALSE);

  return TRUE;
}

uint32_t
PHY_link_status_get(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, volatile uint32_t retries)
{
  volatile unsigned short linkStatus;
  volatile uint32_t retVal = TRUE;

  while (retVal == TRUE) {
    /* Read the BSR of the PHY */
    MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMSR, &linkStatus);

    if (linkStatus & PHY_LINK_STATUS_m) {
      break;
    } else
    {
      (retries != 0) ? retries-- : (retVal = FALSE);
    }
  }

  return retVal;
}

uint32_t
PHY_RMII_mode_get(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr)
{
  volatile unsigned short regContent;

  /* Read the RBR of the PHY */
  MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_RBR, &regContent);
  return (regContent & PHY_RMII_MODE);
}

void
PHY_MII_mode_set(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, uint32_t mode)
{
  volatile unsigned short regContent;

  /* Read the RBR of the PHY */
  MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_RBR, &regContent);
  /* Write the RBR of the PHY */
  regContent &= 0x1f;
  regContent |= ( mode << 5 );
  MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_RBR, regContent);
}

void
PHY_Power_Down(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr)
{
  volatile unsigned short regContent;

  MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMCR, &regContent);
  MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_BMCR, regContent | PHY_POWERDOWN_m);
}

void
PHY_Power_Up(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr)
{
  volatile unsigned short regContent;

  MDIOPhyRegRead(mdioBaseAddr, phyAddr, PHY_BMCR, &regContent);
  MDIOPhyRegWrite(mdioBaseAddr, phyAddr, PHY_BMCR, regContent & ~PHY_POWERDOWN_m);
}
