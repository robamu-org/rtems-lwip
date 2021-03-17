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

#ifndef __DRV_PHY_H
#define __DRV_PHY_H

#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************* */
/*   PHY - register offset definition                */
/* ************************************************* */
#define PHY_BMCR                0x00u /* Basic Mode Control Register */ /*RW*/
#define PHY_BMSR                0x01u /* Basic Mode Status Register */ /*RO*/
#define PHY_IDR1                0x02u /* PHY Identifier Register #1 */ /*RO*/
#define PHY_IDR2                0x03u /* PHY Identifier Register #2 */ /*RO*/
#define PHY_ANAR                0x04u /* Auto-Negotiation Advertisement Register */ /*RW*/
#define PHY_ANLPAR              0x05u /* Auto-Negotiation Link Partner Ability Register */ /*RW*/
#define PHY_ANER                0x06u /* Auto-Negotiation Expansion Register */ /*RW*/
#define PHY_ANNPTR              0x07u /* Auto-Negotiation Next Page TX */ /*RW*/
/*Extended Registers*/
#define PHY_STS                 0x10u /* PHY Status Register */ /*RO*/
#define PHY_RBR                 0x17u /* RMII and Bypass Register */
#define PHY_LEDCR               0x18u /* LED Direct Control Register */ /*RW*/
#define PHY_PHYCR               0x19u /* PHY Control Register */ /*RW*/
/* #define PHY_PHYCR2				0x1C */ /* PHY Control Register 2 */ /*RW*/ /* in doc offset 0x1C marked as RESERVED */
#define PHY_EDCR                0x1Du /* Energy detect control register */ /*RW*/

/* PHY_BMCR - bit position definition */
#define PHY_RESET_m          (1 << 15)
#define PHY_LPBK_m           (1 << 14)
#define PHY_SPEED_m          (1 << 13)
#define PHY_AUTONEG_EN_m     (1 << 12) /* ! */
#define PHY_POWERDOWN_m      (1 << 11)
#define PHY_DUPLEX_m         (1 << 8)
#define PHY_AUTONEG_REST     (1 << 9)

/* PHY_BMSR - bit position definition */
#define PHY_A_NEG_COMPLETE_m (1 << 5)
#define PHY_A_NEG_ABLE_m     (1 << 3)
#define PHY_LINK_STATUS_m    (1 << 2)

/* PHY_ANAR & PHY_ANLPAR - bit position definition */
#define PHY_PAUSE_ASYM       (1 << 11)
#define PHY_PAUSE_SYM        (1 << 10)
#define PHY_100BASET4_m      (1 << 9)
#define PHY_100BASETXDUPL_m  (1 << 8)
#define PHY_100BASETX_m      (1 << 7)
#define PHY_10BASETDUPL_m    (1 << 6)
#define PHY_10BASET_m        (1 << 5)

/* RBR */
#define PHY_RMII_MODE        (1 << 5) /* Reduced Media Independent Interface mode */
#define PHY_MII_MODE         (0)      /* Standard Media Independent Interface mode */
#define PHY_RMII_REV1_0      (1 << 4)
#define PHY_RMII_REV1_2      (0)

/* ************************************************* */
/*   API - function prototypes                       */
/* ************************************************* */

/**
 * Resets PHY
 *
 * @param mdioBaseAddr    Base Address of MDIO Module Megisters.
 * @param phyAddr         PHY Address (0-31).
 *
 * @note                  Software driver code must wait 3 μs following a software
 *                        reset before allowing further serial MII operations with the DP83848H.
 *                        This must be implemented in the layer above.
 * @note                  Calling this function is blocking until PHY indicates the reset process is complete.
 */
void PHY_reset(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr);

/**
 * Reads link partner ability register from the PHY
 *
 * @param mdioBaseAddr    Base Address of MDIO Module Megisters.
 * @param phyAddr         PHY Address (0-31).
 * @param regContent      The partner abilities of the EMAC.
 *
 * @return TRUE if reading succesful, FALSE if reading failed
 */
uint32_t PHY_partner_ability_get(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, unsigned short *regContent);

/**
 * This function does Autonegotiates with the EMAC device connected
 * to the PHY. It will wait till the autonegotiation completes.
 *
 * @param mdioBaseAddr    Base Address of MDIO Module Megisters.
 * @param phyAddr         PHY Address (0-31).
 * @param advValue        Autonegotiation advertisement value \n
 *        advVal can take the following any OR combination of the values:
 *        PHY_100BASETXDUPL_m - full duplex capability for 100BaseTX
 *        PHY_100BASETX_m - half duplex capability for 100BaseTX
 *        PHY_10BASETDUPL_m - full duplex capability for 10BaseT
 *        PHY_10BASET_m - half duplex capability for 10BaseT
 *
 * @return TRUE if autonegotiation succesful, FALSE if autonegotiation failed
 *
 * @note  this function is blocking, waits till link is established
 */
uint32_t PHY_auto_negotiate(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, unsigned short advVal);

/**
 * This function starts autonegotiaon with the EMAC device connected
 * to the PHY. For examining, whether autonegotiation is done see
 * PHY_start_auto_negotiate function.
 *
 * @param mdioBaseAddr    Base Address of MDIO Module Megisters.
 * @param phyAddr         PHY Address (0-31).
 * @param advValue        Autonegotiation advertisement value \n
 *        advVal can take the following any OR combination of the values:
 *        PHY_100BASETXDUPL_m - full duplex capability for 100BaseTX
 *        PHY_100BASETX_m - half duplex capability for 100BaseTX
 *        PHY_10BASETDUPL_m - full duplex capability for 10BaseT
 *        PHY_10BASET_m - half duplex capability for 10BaseT
 *
 * @return TRUE if setting autonegotiation startup succesful,
 *         FALSE if starting autonegotiation failed
 */
uint32_t PHY_start_auto_negotiate(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, unsigned short advVal);

/**
 * This function examines, whether autonegotiation is done.
 * Must be called after PHY_start_auto_negotiate.
 *
 * @param mdioBaseAddr    Base Address of MDIO Module Megisters.
 * @param phyAddr         PHY Address (0-31).
 *
 * @return TRUE if autonegotiation succesfull and done, FALSE if autonegotiation failed.
 */
uint32_t PHY_is_done_auto_negotiate(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr);

/**
 * Reads the link status of the PHY.
 *
 * @param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * @param   phyAddr       PHY Address (0-31).
 * @param   retries       The number of retries before indicating down status.
 *
 * @return  link status after reading \n
 *          TRUE if link is up
 *          FALSE if link is down \n
 *
 * @note    This reads both the basic status register of the PHY and the
 *          link register of MDIO for double check
 **/
uint32_t PHY_link_status_get(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, volatile uint32_t retries);

/**
 * Fetches RMII/MII mode of PHY.
 *
 * @param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * @param   phyAddr       PHY Address (0-31).
 *
 * @return  TRUE if mode of PHY is set to RMII \
 *          FALSE if mode of PHY is set to MII
 */
uint32_t PHY_RMII_mode_get(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr);

/**
 * Sets RMII/MII mode of PHY.
 *
 * @param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * @param   phyAddr       PHY Address (0-31).
 * @param   mode          Mode to set. \
 *                        1 - RMII \
 *                        0 - MII
 */
void PHY_MII_mode_set(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr, uint32_t mode);

/**
 * Powers down the PHY.
 *
 * @param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * @param   phyAddr       PHY Address (0-31).
 */
void PHY_Power_Down(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr);


/**
 * Powers up the PHY.
 *
 * @param   mdioBaseAddr  Base Address of the MDIO Module Registers.
 * @param   phyAddr       PHY Address (0-31).
 */
void PHY_Power_Up(volatile tms570_mdio_t *mdioBaseAddr, uint32_t phyAddr);

#ifdef __cplusplus
}
#endif
#endif /* __DRV_PHY_H */
