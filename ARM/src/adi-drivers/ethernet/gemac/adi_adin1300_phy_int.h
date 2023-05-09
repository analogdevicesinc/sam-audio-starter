/*!
*********************************************************************************
 *
 * @file:    adi_adin1300_phy_int.h
 *
 * @brief:   ADIN1300 phy driver header
 *
 * @version: $Revision: 37938 $
 *
 * @date:    $Date: 2021-01-04 09:06:22 -0500 (Mon, 04 Jan 2021) $
 * ------------------------------------------------------------------------------
 *
 * Copyright (c) 2011-2022 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Modified versions of the software must be conspicuously marked as such.
 * - This software is licensed solely and exclusively for use with processors
 *   manufactured by or for Analog Devices, Inc.
 * - This software may not be combined or merged with other code in any manner
 *   that would cause the software to become subject to terms and conditions
 *   which differ from those listed here.
 * - Neither the name of Analog Devices, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 * - The use of this software may or may not infringe the patent rights of one
 *   or more patent holders.  This license does not release you from the
 *   requirement that you obtain separate licenses from these patent holders
 *   to use this software.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
 * PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/
#ifndef _ADI_PHYADIN1300_H_
#define _ADI_PHYADIN1300_H_

#if !defined (__ADSPSC589_FAMILY__)
#include <adi_types.h>
#else
#include <stdint.h>
#endif
#include "adi_gemac_int.h"
#include "adi_phy_int.h"

/*! Loop count to check PHY busy status */
#define PHY_LOOP_COUNT  (100000)

/*!
 * Macros to set GEMAC MII ADDR register values
 */
#define SET_PHYAD(x) ( ((x) & 0x1F) << 11 )  /*!< Set PHY address          */
#define SET_REGAD(x) ( ((x) & 0x1F) << 6  )  /*!< Set PHY register address */
#define SET_CR(x)    ( ((x) & 0x0F) << 2  )  /*!< Set PHY clock range      */

#define PHY1_ADDRESS     (1)                  /*!< PHY 1 address                   */
#define PHY2_ADDRESS     (2)                  /*!< PHY 2 address                   */
#define NUM_PHY_REGS     (0x20)               /*!< Number of registers             */

// ADIN1300 register
#define REG_PHY_MODECTL             (0)       /*!<   PHY Mode control register      */

 #define PHY_MODECTL_RESET          (0x8000)  /*!<RW Reset phy, self clearing       */
 #define PHY_MODECTL_LOOPBACK       (0x4000)  /*!<RW 1:enable loopback 0:disable    */
 #define PHY_MODECTL_SPEED_SELECT   (0x2000)  /*!<RW Speed, 2: 1000Mbps, 1:100Mbps, 0:10Mbps     */
 #define PHY_MODECTL_AUTO_NEGO_ENA  (0x1000)  /*!<RW 1: Auto-negotiation on, 0: off */
 #define PHY_MODECTL_POWER_DOWN     (0x0800)  /*!<RW 1: power down, 0: normal       */
 #define PHY_MODECTL_ISOLATE        (0x0400)  /*!<RW not supported                  */
 #define PHY_MODECTL_RESTART_A_NEGO (0x0200)  /*!<RW 1: restart Auto-negotiation, 0: normal */
 #define PHY_MODECTL_DUPLEX_MODE    (0x0100)  /*!<RW 1: full duplex, 0: half duplex */
 #define PHY_MODECTL_COLLISION_TEST (0x0080)  /*!<RO not supported                  */
 #define PHY_MODECTL_SPEED_SELECT1   (0x0040)  /*!<RW Speed,     */

// ADIN1300 register
#define REG_PHY_MODESTAT            (1)       /*!< Basic Status Register           */
 #define PHY_MODESTAT_100BASE_T4    (0x8000)  /*!< Status 100Base full T4          */
 #define PHY_MODESTAT_100BASE_FULL  (0x4000)  /*!< Status 100Base full duplex      */
 #define PHY_MODESTAT_100BASE_HALF  (0x2000)  /*!< Status 100Base half duplex      */
 #define PHY_MODESTAT_10BASE_FULL   (0x1000)  /*!< Status 10Base full duplex      */
 #define PHY_MODESTAT_10BASE_HALF   (0x0800)  /*!< Status 10Base half duplex      */
 #define PHY_MODESTAT_MF_PREAMBLE   (0x0400)  /*!< Status preamble                */
 #define PHY_MODESTAT_AUTO_NEGO_COM (0x0020)  /*!< Status Autonegotiation complete */
 #define PHY_MODESTAT_REMOTE_FAULT  (0x0010)  /*!< Remote fault detected          */
 #define PHY_MODESTAT_AUTO_NEGO_ABILITY (0x0008) /*!< Autoneotiation capable      */
 #define PHY_MODESTAT_LINK_STATUS   (0x0004)     /*!< Link status                 */
 #define PHY_MODESTAT_JABBER_DETECT (0x0002)     /*!< Jabber detected             */
 #define PHY_MODESTAT_EXT_CAPABILIT (0x0001)     /*!< Extended capabilities       */

// ADIN1300/7 register
#define REG_PHY_PHYID1              (2)       /*!< PHY Identifier High           */
#define REG_PHY_PHYID2              (3)       /*!< PHY Identifier Low            */

// ADIN1300 register
#define REG_PHY_ANAR                (4)       /*!< Auto-Negotiation Advertisement Register */
 #define PHY_ANAR_NEXT_PAGE         (0x8000)  /*!< Autonegotiation next page register */
 #define PHY_ANAR_REMOTE_FAULT      (0x2000)  /*!< Remote fault */
 #define PHY_ANAR_ASM_DIR           (0x0800)  /*!<Asymmetric PAUSE Support for Full Duplex Links */
 #define PHY_ANAR_PAUSE_OPERATION   (0x0400)  /*!< Supports Pause operations */
 #define PHY_ANAR_100BASE_T4        (0x0200)  /*!<TX 100BASE-T4 Support */
 #define PHY_ANAR_100BASE_FULL      (0x0100)  /*!< Autonegotiation adv 100 Base full */
 #define PHY_ANAR_100BASE_HALF      (0x0080)  /*!< Autonegotiation adv 100 Base half */
 #define PHY_ANAR_10BASE_FULL       (0x0040)  /*!< Autonegotiation adv 10 Base full  */
 #define PHY_ANAR_10BASE_HALF       (0x0020)  /*!< Autonegotiation adv 10 Base half  */

// ADIN1300 register
#define REG_PHY_ANLPAR              (5)       /*!< Auto-Negotiation Link Partner Ability Register */
 #define PHY_ANLPAR_NP              (0x8000)  /*!< Next page indication                           */
 #define PHY_ANLPAR_ACK             (0x4000)  /*!< Acknowledge                                    */
 #define PHY_ANLPAR_RF              (0x2000)  /*!< Remote Fault                                   */
 #define PHY_ANLPAR_ASM_DIR         (0x0800)  /*!< Asymmetric PAUSE Support for Full Duplex Links:*/
 #define PHY_ANLPAR_PAUSE_OPERATION (0x0400)  /*!< Pause operation is supported by remote MAC     */
 #define PHY_ANLPAR_100BASE_T4      (0x0200)  /*!< TX 100BASE-T4 Support                          */
 #define PHY_ANLPAR_100BASE_DUPLEX  (0x0100)  /*!< TX with full duplex                            */
 #define PHY_ANLPAR_100BASE_HALF    (0x0080)  /*!< TX with half duplex                            */
 #define PHY_ANLPAR_10BASE_DUPLEX   (0x0040)  /*!< 10Mbps with full duplex                        */
 #define PHY_ANLPAR_10BASE_HALF     (0x0020)  /*!<  10Mbps with half duplex                       */

//  ADIN1300 register
#define REG_PHY_ANER                 (6)       /*!< Auto_negotiation Expansion Register           */
 #define PHY_ANER_PARALLEL_DETECTION (0x0010)  /*!< fault dected by parallel detction logic       */
 #define PHY_ANER_LP_NEXT_PAGE       (0x0008)  /*!< link partner has next page ability            */
 #define PHY_ANER_NEXT_PAGE_ABLE     (0x0004)  /*!< local device has next page ability            */
 #define PHY_ANER_PAGE_RECEIVED      (0x0002)  /*!< new page received                             */
 #define PHY_ANER_LP_ANG_ABLE        (0x0001)  /*!< link partner has auto-negotiation ability     */

#define REG_PHY_1KTCR             (9)          /*!<  1000Base-T Control Register   */
#define PHY_1KTCR_1000BASET_FULL (0x0200)     /*!< Autonegotiation adv 1000 Base full  */

#define ADIN1300_OUI_VENDOR         0x0283
#define ADIN1300_OUT_MODEL          0xBC30

ADI_PHY_RESULT adin1300_phy_init (
                                 ADI_PHY_DEVICE* pPhyDevice,
                                 ADI_ETHER_HANDLE        const hDevice,
                                 ADI_PHY_DEV_INIT *      const pInitParams
                                 );
ADI_PHY_RESULT adin1300_phy_uninit (ADI_PHY_DEVICE* pPhyDevice);
ADI_PHY_RESULT adin1300_phy_get_status(ADI_PHY_DEVICE* pPhyDevice, uint32_t* const nStatus);

ADI_PHY_RESULT adin1300_phy_init (
                                 ADI_PHY_DEVICE* pPhyDevice,
                                 ADI_ETHER_HANDLE        const hDevice,
                                 ADI_PHY_DEV_INIT *      const pInitParams
                                 );
ADI_PHY_RESULT adin1300_phy_uninit (ADI_PHY_DEVICE* pPhyDevice);
ADI_PHY_RESULT adin1300_phy_get_status(ADI_PHY_DEVICE* pPhyDevice, uint32_t* const nStatus);

#define REG_PHY_LINK_AN_STATUS      (0x1A)     /*!<  Link & Autonegotiation Status Register   */
 #define PHY_LINK_STATUS_UP                 (0x0040)
 #define PHY_LINK_STATUS_FULL_DUPLEX        (0x0080)
 #define BITM_PHY_LINK_STATUS_SPEED         (0x0300)
 #define ENUM_PHY_LINK_STATUS_SPEED_1000    (0x0200)
 #define ENUM_PHY_LINK_STATUS_SPEED_100     (0x0100)
 #define ENUM_PHY_LINK_STATUS_SPEED_10      (0x0000)
 #define PHY_LINK_STATUS_AUTO_NEFO_COM      (0x1000)

// ADIN1300 interrupts
#define REG_PHY_INT_MASK            (0x18)
 #define PHY_HW_IRQ_EN              (0x0001)
 #define PHY_INT_MASK_SPEED         (0x0002)
 #define PHY_INT_MASK_AUTO_NEGO_COM (0x0100)
 #define PHY_INT_MASK_LINK          (0x0004)

#define REG_PHY_INT_READ_CLEAR      (0x19)
 #define PHY_HW_IRQ_PENDING         (0x0001)

#endif /* _ADI_PHYDP83867_H_ */
