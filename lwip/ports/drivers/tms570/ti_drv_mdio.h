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

#ifndef __MDIO_H__
#define __MDIO_H__

#include <bsp/tms570.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************/
/**
 * \brief   Reads a PHY register using MDIO.
 *
 * \param   baseAddr      Base Address of the MDIO Module Registers.
 * \param   phyAddr       PHY Adress.
 * \param   regNum        Register Number to be read.
 * \param   dataPtr       Pointer where the read value shall be written.
 *
 * \return  status of the read \n
 *          TRUE - read is successful.\n
 *          FALSE - read is not acknowledged properly.
 *
 **/

static inline uint32_t
MDIOPhyRegRead(volatile tms570_mdio_t *baseAddr, uint32_t phyAddr,
               uint32_t regNum, volatile unsigned short *dataPtr)
{
  /* Wait till transaction completion if any */
  while (baseAddr->USERACCESS0 & TMS570_MDIO_USERACCESS0_GO);

  baseAddr->USERACCESS0
    = (TMS570_MDIO_USERACCESS0_GO
       |TMS570_MDIO_USERACCESS0_REGADR(regNum)
       |TMS570_MDIO_USERACCESS0_PHYADR(phyAddr));

  /* wait for command completion */
  while (baseAddr->USERACCESS0 & TMS570_MDIO_USERACCESS0_GO);

  /* Store the data if the read is acknowledged */
  if (baseAddr->USERACCESS0 & TMS570_MDIO_USERACCESS0_ACK) {
    *dataPtr = (unsigned short)TMS570_MDIO_USERACCESS0_DATA_GET(baseAddr->USERACCESS0);
    return true;
  }

  return false;
}

/**
 * \brief   Writes a PHY register using MDIO.
 *
 * \param   baseAddr      Base Address of the MDIO Module Registers.
 * \param   phyAddr       PHY Adress.
 * \param   regNum        Register Number to be read.
 * \param   RegVal        Value to be written.
 *
 * \return  None
 *
 **/
static inline void
MDIOPhyRegWrite(volatile tms570_mdio_t *baseAddr, uint32_t phyAddr,
                uint32_t regNum, unsigned short RegVal)
{
  /* Wait till transaction completion if any */
  while (baseAddr->USERACCESS0 & TMS570_MDIO_USERACCESS0_GO);

  baseAddr->USERACCESS0 =
    (TMS570_MDIO_USERACCESS0_WRITE
     | TMS570_MDIO_USERACCESS0_GO
     |TMS570_MDIO_USERACCESS0_REGADR(regNum)
     |TMS570_MDIO_USERACCESS0_PHYADR(phyAddr)
     | RegVal);

  /* wait for command completion*/
  while (baseAddr->USERACCESS0 & TMS570_MDIO_USERACCESS0_GO);
}
/**
 * \brief   Reads the alive status of all PHY connected to this MDIO.
 *          The bit correponding to the PHY address will be set if the PHY
 *          is alive.
 *
 * \param   baseAddr      Base Address of the MDIO Module Registers.
 *
 * \return  MDIO alive register state
 *
 **/
static inline uint32_t
MDIOPhyAliveStatusGet(volatile tms570_mdio_t *baseAddr)
{
  return (baseAddr->ALIVE);
}

/**
 * \brief   Reads the link status of all PHY connected to this MDIO.
 *          The bit correponding to the PHY address will be set if the PHY
 *          link is active.
 *
 * \param   baseAddr      Base Address of the MDIO Module Registers.
 *
 * \return  MDIO link register state
 *
 **/
static inline uint32_t
MDIOPhyLinkStatusGet(volatile tms570_mdio_t *baseAddr)
{
  return (baseAddr->LINK);
}

/**
 * \brief   Initializes the MDIO peripheral. This enables the MDIO state
 *          machine, uses standard pre-amble and set the clock divider value.
 *
 * \param   baseAddr       Base Address of the MDIO Module Registers.
 * \param   mdioInputFreq  The clock input to the MDIO module
 * \param   mdioOutputFreq The clock output required on the MDIO bus
 * \return  None
 *
 **/
static inline void
MDIOInit(volatile tms570_mdio_t *baseAddr, uint32_t mdioInputFreq,
         uint32_t mdioOutputFreq)
{
  baseAddr->CONTROL = TMS570_MDIO_CONTROL_HIGHEST_USER_CHANNEL(1) |
                      TMS570_MDIO_CONTROL_ENABLE |
                      TMS570_MDIO_CONTROL_CLKDIV(0x60);
}

#ifdef __cplusplus
}
#endif
#endif /* __MDIO_H__ */
