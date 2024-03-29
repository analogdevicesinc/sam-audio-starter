/**
 * Copyright (c) 2021 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* ADI service includes */
#include <services/gpio/adi_gpio.h>
#include <services/int/adi_gic.h>
#include <services/tmr/adi_tmr.h>
#include <services/pwr/adi_pwr.h>

/* ADI processor includes */
#include <sruSC589.h>

/* Standard library includes */
#include <string.h>
#include <assert.h>

#ifdef FREE_RTOS
#include "FreeRTOS.h"
#endif

/* umm_malloc includes  */
#include "umm_malloc.h"
#include "umm_malloc_heaps.h"
#include "umm_malloc_cfg.h"

/* Simple driver includes */
#include "sport_simple.h"
#include "flash.h"
#include "mt25ql512.h"
#include "is25lp512.h"
#include "pcg_simple.h"

/* Simple service includes */
#include "syslog.h"
#include "a2b_to_sport_cfg.h"

/* Application includes */
#include "context.h"
#include "clocks.h"
#include "init.h"
#include "adau1761.h"
#include "codec_audio.h"
#include "spdif_audio.h"
#include "a2b_audio.h"
#include "util.h"
#include "sae_irq.h"
#include "flash_map.h"
#include "clock_domain.h"

/***********************************************************************
 * Audio Clock Initialization
 **********************************************************************/
/* DAI IE Bit definitions (not in any ADI header files) */
#define BITP_PADS0_DAI0_IE_PB06   (6)
#define BITP_PADS0_DAI0_IE_PB07   (7)
#define BITP_PADS0_DAI0_IE_PB08   (8)
#define BITP_PADS0_DAI0_IE_PB13   (13)

/*
 * The RED_BOARD is the very first engineering sample of the SAM
 * board.  This board did not route the audio MCLK to a DAI pin so
 * a jumper wire must be installed for this option to work with this
 * software.
 *
 * Enable this option in the makefile.
 *
 * Install the jumper wire between R48 (on the side opposite of the
 * clock generator) to P4 pin 11 (DAI0_PB13).
 */
#ifdef RED_BOARD
#define BITP_PADS0_DAI0_IE_MCLK BITP_PADS0_DAI0_IE_PB13
#define DAI0_MCLK_PIN 13
#else
#define BITP_PADS0_DAI0_IE_MCLK BITP_PADS0_DAI0_IE_PB06
#define DAI0_MCLK_PIN 6
#endif

#pragma pack(1)
typedef struct SI5356A_REG_DATA {
   uint8_t addr;
   uint8_t value;
} SI5356A_REG_DATA;
#pragma pack()

SI5356A_REG_DATA SI5356_CLK4_24567Mhz[] = {
    { 230,0x04},
    {  74,0x10},
    {  75,0xC2},
    {  76,0x2A},
    {  77,0x00},
    {  78,0x02},
    {  79,0x00},
    {  80,0x00},
    {  81,0x80},
    {  82,0x01},
    {  83,0x00},
    {  84,0x00},
    { 230,0x00}
};

/* This function sets CLK4/5 of U25 (the main audio @ MCLK DAI0_PIN06) to
 * 24.576MHz instead of it's default speed of 12.288MHz.
 */
void audio_mclk_24576_mhz(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT twiResult;
    sTWI *twi;
    uint8_t i;
    uint8_t len;

    /* Fixed clocks on SAM V2 */
    if (context->samVersion >= SAM_VERSION_2) {
        return;
    }

    twi = context->ethClkTwiHandle;

    len = sizeof(SI5356_CLK4_24567Mhz) / sizeof(SI5356A_REG_DATA);

    for (i = 0; i < len; i++) {
        twiResult = twi_write(twi, 0x70,
            (uint8_t *)&SI5356_CLK4_24567Mhz[i], sizeof(SI5356A_REG_DATA));
    }
}

#define AUDIO_CLK_DAI0_IE (          \
    (1 << BITP_PADS0_DAI0_IE_MCLK)   \
)

void disable_sport_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI0_IE &= ~(AUDIO_CLK_DAI0_IE);
}

void enable_sport_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI0_IE |= (AUDIO_CLK_DAI0_IE);
}

/***********************************************************************
 * System Clock Initialization
 **********************************************************************/
void system_clk_init(uint32_t *cclk)
{
    ADI_PWR_RESULT ePwrResult;
    uint32_t _cclk,sclk,sclk0,sclk1,dclk,oclk;

    /* Initialize ADI power service */
    ePwrResult = adi_pwr_Init(0, OSC_CLK);

    /* Set up core and system clocks to the values in clocks.h */
    ePwrResult = adi_pwr_SetFreq(0, CCLK, SYSCLK);
    ePwrResult = adi_pwr_SetClkDivideRegister(0, ADI_PWR_CLK_DIV_OSEL, OCLK_DIV);

    /* Query primary clocks from CGU0 for confirmation */
    ePwrResult = adi_pwr_GetCoreFreq(0, &_cclk);
    ePwrResult = adi_pwr_GetSystemFreq(0, &sclk, &sclk0, &sclk1);
    ePwrResult = adi_pwr_GetDDRClkFreq(0, &dclk);
    ePwrResult = adi_pwr_GetOutClkFreq(0, &oclk);

    /* Store the core clock frequency */
    if (cclk) {
        *cclk = _cclk;
    }

    /* CAN clock is derived from CDU0_CLKO4 (Pg. 4-3 of HRM)
     * Choose OCLK_0 (see oclk above)
     */
    ePwrResult = adi_pwr_ConfigCduInputClock(ADI_PWR_CDU_CLKIN_0, ADI_PWR_CDU_CLKOUT_4);
    ePwrResult = adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_4, true);

    /*
     * Take CGU1 out of bypass and enter full on.
     * Pg. 3-9 of ADSP-SC58x/ADSP-2158x SHARC+ Processor Hardware Reference
     */
    *pREG_CGU1_PLLCTL |= BITM_CGU_PLLCTL_PLLBPCL;
    while((*pREG_CGU1_STAT & 0xF) != 0x5);

    /*
     * ADSP-SC5xx EMAC0 require a clock of 125Mhz
     *
     * Set CGU1 to create a 250MHz core clock and 125MHz SYSCLK_1
     *
     * Divide SYSCLK_1 by 1 to derive 125MHz SCLK1_1 which can
     * be routed by the CDU via mux input 1 to EMAC0.
     *
     * Divide CCLK_1 by 5 to derive a 50MHz DCLK_1 which can be
     * routed by the CDU via mux input 3 to MSI0
     *
     * Be sure to update ALL/include/clocks.h if any of these
     * settings changes!
     *
     */
    ePwrResult = adi_pwr_Init(1, OSC_CLK);
    ePwrResult = adi_pwr_SetFreq(1, 250000000, 125000000);
    ePwrResult = adi_pwr_SetClkDivideRegister(1, ADI_PWR_CLK_DIV_S1SEL, 1);
    ePwrResult = adi_pwr_SetClkDivideRegister(1, ADI_PWR_CLK_DIV_DSEL, 5);

    ePwrResult = adi_pwr_GetCoreFreq(1, &_cclk);
    ePwrResult = adi_pwr_GetSystemFreq(1, &sclk, &sclk0, &sclk1);
    ePwrResult = adi_pwr_GetDDRClkFreq(1, &dclk);
    ePwrResult = adi_pwr_GetOutClkFreq(1, &oclk);

    /*
     * Set EMAC0 GigE/RGMII interface clock to SCLK1_1 and enable
     * Table 4-2, Pg. 4-3 of ADSP-SC58x/ADSP-2158x SHARC+ Processor Hardware Reference
     */
    ePwrResult = adi_pwr_ConfigCduInputClock(ADI_PWR_CDU_CLKIN_1, ADI_PWR_CDU_CLKOUT_7);
    ePwrResult = adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_7, true);

    /* MSI0 clock is derived from CDU0_CLKO9 (Pg. 4-3 of HRM)
     * Table 4-2, Pg. 4-3 of ADSP-SC58x/ADSP-2158x SHARC+ Processor Hardware Reference
     * Choose DCLK_1 (See dclk above)
     */
    ePwrResult = adi_pwr_ConfigCduInputClock(ADI_PWR_CDU_CLKIN_3, ADI_PWR_CDU_CLKOUT_9);
    ePwrResult = adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_9, true);

    /* SPDIF clock is derived from CDU0_CLKO5
     * Choose OCLK_0 (see oclk above)
     */
    ePwrResult = adi_pwr_ConfigCduInputClock(ADI_PWR_CDU_CLKIN_0, ADI_PWR_CDU_CLKOUT_5);
    ePwrResult = adi_pwr_EnableCduClockOutput(ADI_PWR_CDU_CLKOUT_5, true);
}

/***********************************************************************
 * GPIO / Pin MUX / SRU Initialization
 **********************************************************************/
/*
 * The port FER and MUX settings are detailed in:
 *    ADSP-SC582_583_584_587_589_ADSP-21583_584_587.pdf
 *
 */

/* EMAC0 GPIO FER bit positions (one bit per FER entry) */
#define EMAC0_MDIO_PORTA_FER          (1 << BITP_PORT_DATA_PX3)
#define EMAC0_MDC_PORTA_FER           (1 << BITP_PORT_DATA_PX2)
#define EMAC0_RXD0_PORTA_FER          (1 << BITP_PORT_DATA_PX4)
#define EMAC0_RXD1_PORTA_FER          (1 << BITP_PORT_DATA_PX5)
#define EMAC0_RXD2_PORTA_FER          (1 << BITP_PORT_DATA_PX8)
#define EMAC0_RXD3_PORTA_FER          (1 << BITP_PORT_DATA_PX9)
#define EMAC0_TXD0_PORTA_FER          (1 << BITP_PORT_DATA_PX0)
#define EMAC0_TXD1_PORTA_FER          (1 << BITP_PORT_DATA_PX1)
#define EMAC0_TXD2_PORTA_FER          (1 << BITP_PORT_DATA_PX12)
#define EMAC0_TXD3_PORTA_FER          (1 << BITP_PORT_DATA_PX13)
#define EMAC0_TXEN_PORTA_FER          (1 << BITP_PORT_DATA_PX10)
#define EMAC0_CRS_PORTA_FER           (1 << BITP_PORT_DATA_PX7)
#define EMAC0_RXCLK_REFCLK_PORTA_FER  (1 << BITP_PORT_DATA_PX6)
#define EMAC0_TXCLK_PORTA_FER         (1 << BITP_PORT_DATA_PX11)

/* EMAC GPIO MUX bit positions (two bits per MUX entry, all mux 0) */
#define EMAC0_MDIO_PORTA_MUX          (0 << (BITP_PORT_DATA_PX3 << 1))
#define EMAC0_MDC_PORTA_MUX           (0 << (BITP_PORT_DATA_PX2 << 1))
#define EMAC0_RXD0_PORTA_MUX          (0 << (BITP_PORT_DATA_PX4 << 1))
#define EMAC0_RXD1_PORTA_MUX          (0 << (BITP_PORT_DATA_PX5 << 1))
#define EMAC0_RXD2_PORTA_MUX          (0 << (BITP_PORT_DATA_PX8 << 1))
#define EMAC0_RXD3_PORTA_MUX          (0 << (BITP_PORT_DATA_PX9 << 1))
#define EMAC0_TXD0_PORTA_MUX          (0 << (BITP_PORT_DATA_PX0 << 1))
#define EMAC0_TXD1_PORTA_MUX          (0 << (BITP_PORT_DATA_PX1 << 1))
#define EMAC0_TXD2_PORTA_MUX          (0 << (BITP_PORT_DATA_PX12 << 1))
#define EMAC0_TXD3_PORTA_MUX          (0 << (BITP_PORT_DATA_PX13 << 1))
#define EMAC0_TXEN_PORTA_MUX          (0 << (BITP_PORT_DATA_PX10 << 1))
#define EMAC0_CRS_PORTA_MUX           (0 << (BITP_PORT_DATA_PX7 << 1))
#define EMAC0_RXCLK_REFCLK_PORTA_MUX  (0 << (BITP_PORT_DATA_PX6 << 1))
#define EMAC0_TXCLK_PORTA_MUX         (0 << (BITP_PORT_DATA_PX11 << 1))

/* SPI2 GPIO FER bit positions (one bit per FER entry) */
#define SPI2_CLK_PORTC_FER   (1 << BITP_PORT_DATA_PX1)
#define SPI2_MISO_PORTC_FER  (1 << BITP_PORT_DATA_PX2)
#define SPI2_MOSO_PORTC_FER  (1 << BITP_PORT_DATA_PX3)
#define SPI2_D2_PORTC_FER    (1 << BITP_PORT_DATA_PX4)
#define SPI2_D3_PORTC_FER    (1 << BITP_PORT_DATA_PX5)
#define SPI2_SEL_PORTC_FER   (1 << BITP_PORT_DATA_PX6)

/* SPI2 GPIO MUX bit positions (two bits per MUX entry) */
#define SPI2_CLK_PORTC_MUX   (0 << (BITP_PORT_DATA_PX1 << 1))
#define SPI2_MISO_PORTC_MUX  (0 << (BITP_PORT_DATA_PX2 << 1))
#define SPI2_MOSO_PORTC_MUX  (0 << (BITP_PORT_DATA_PX3 << 1))
#define SPI2_D2_PORTC_MUX    (0 << (BITP_PORT_DATA_PX4 << 1))
#define SPI2_D3_PORTC_MUX    (0 << (BITP_PORT_DATA_PX5 << 1))
#define SPI2_SEL_PORTC_MUX   (0 << (BITP_PORT_DATA_PX6 << 1))

/* UART0 GPIO FER bit positions */
#define UART0_TX_PORTC_FER   (1 << BITP_PORT_DATA_PX13)
#define UART0_RX_PORTC_FER   (1 << BITP_PORT_DATA_PX14)
#define UART0_RTS_PORTC_FER  (1 << BITP_PORT_DATA_PX15)
#define UART0_CTS_PORTD_FER  (1 << BITP_PORT_DATA_PX0)

/* UART0 GPIO MUX bit positions (two bits per MUX entry) */
#define UART0_TX_PORTC_MUX   (0 << (BITP_PORT_DATA_PX13 << 1))
#define UART0_RX_PORTC_MUX   (0 << (BITP_PORT_DATA_PX14 << 1))
#define UART0_RTS_PORTC_MUX  (0 << (BITP_PORT_DATA_PX15 << 1))
#define UART0_CTS_PORTD_MUX  (0 << (BITP_PORT_DATA_PX0  << 1))

/* MSI0 GPIO FER bit positions (one bit per FER entry) */
#define MSI0_D0_PORTF_FER   (1 << BITP_PORT_DATA_PX2)
#define MSI0_D1_PORTF_FER   (1 << BITP_PORT_DATA_PX3)
#define MSI0_D2_PORTF_FER   (1 << BITP_PORT_DATA_PX4)
#define MSI0_D3_PORTF_FER   (1 << BITP_PORT_DATA_PX5)
#define MSI0_CMD_PORTF_FER  (1 << BITP_PORT_DATA_PX10)
#define MSI0_CLK_PORTF_FER  (1 << BITP_PORT_DATA_PX11)
#define MSI0_CD_PORTF_FER   (1 << BITP_PORT_DATA_PX12)

/* MSI0 GPIO MUX bit positions (two bits per MUX entry, all mux 0) */
#define MSI0_D0_PORTF_MUX   (0 << (BITP_PORT_DATA_PX2 << 1))
#define MSI0_D1_PORTF_MUX   (0 << (BITP_PORT_DATA_PX3 << 1))
#define MSI0_D2_PORTF_MUX   (0 << (BITP_PORT_DATA_PX4 << 1))
#define MSI0_D3_PORTF_MUX   (0 << (BITP_PORT_DATA_PX5 << 1))
#define MSI0_CMD_PORTF_MUX  (0 << (BITP_PORT_DATA_PX10 << 1))
#define MSI0_CLK_PORTF_MUX  (0 << (BITP_PORT_DATA_PX11 << 1))
#define MSI0_CD_PORTF_MUX   (0 << (BITP_PORT_DATA_PX12 << 1))

void gpio_init(void)
{
    static uint8_t gpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE];
    uint32_t numCallbacks;

    ADI_GPIO_RESULT  result;

    /* Configure ETH0 Alternate Function GPIO */
    *pREG_PORTA_FER |= (
        EMAC0_MDIO_PORTA_FER |
        EMAC0_MDC_PORTA_FER |
        EMAC0_RXD0_PORTA_FER |
        EMAC0_RXD1_PORTA_FER |
        EMAC0_TXD0_PORTA_FER |
        EMAC0_TXD1_PORTA_FER |
        EMAC0_RXD2_PORTA_FER |
        EMAC0_RXD3_PORTA_FER |
        EMAC0_TXD2_PORTA_FER |
        EMAC0_TXD3_PORTA_FER |
        EMAC0_TXCLK_PORTA_FER |
        EMAC0_TXEN_PORTA_FER |
        EMAC0_CRS_PORTA_FER |
        EMAC0_RXCLK_REFCLK_PORTA_FER
    );

    *pREG_PORTA_MUX |= (
        EMAC0_MDIO_PORTA_MUX |
        EMAC0_MDC_PORTA_MUX |
        EMAC0_RXD0_PORTA_MUX |
        EMAC0_RXD1_PORTA_MUX |
        EMAC0_TXD0_PORTA_MUX |
        EMAC0_TXD1_PORTA_MUX |
        EMAC0_RXD2_PORTA_MUX |
        EMAC0_RXD3_PORTA_MUX |
        EMAC0_TXD2_PORTA_MUX |
        EMAC0_TXD3_PORTA_MUX |
        EMAC0_TXCLK_PORTA_MUX |
        EMAC0_TXEN_PORTA_MUX |
        EMAC0_CRS_PORTA_MUX |
        EMAC0_RXCLK_REFCLK_PORTA_MUX
    );

    /* Configure SPI2 Alternate Function GPIO */
    *pREG_PORTC_FER |= (
        SPI2_CLK_PORTC_FER |
        SPI2_MISO_PORTC_FER |
        SPI2_MOSO_PORTC_FER |
        SPI2_D2_PORTC_FER |
        SPI2_D3_PORTC_FER |
        SPI2_SEL_PORTC_FER
    );
    *pREG_PORTC_MUX |= (
        SPI2_CLK_PORTC_MUX |
        SPI2_MISO_PORTC_MUX |
        SPI2_MOSO_PORTC_MUX |
        SPI2_D2_PORTC_MUX |
        SPI2_D3_PORTC_MUX |
        SPI2_SEL_PORTC_MUX
    );

    /* Configure UART0 Alternate Function GPIO */
    *pREG_PORTC_FER |= (
        UART0_TX_PORTC_FER |
        UART0_RX_PORTC_FER |
        UART0_RTS_PORTC_FER
    );
    *pREG_PORTC_MUX |= (
        UART0_TX_PORTC_MUX |
        UART0_RX_PORTC_MUX |
        UART0_RTS_PORTC_MUX
    );
    *pREG_PORTD_FER |= (
        UART0_CTS_PORTD_FER
    );
    *pREG_PORTD_MUX |= (
        UART0_CTS_PORTD_MUX
    );

    /* Configure MSI0 Alternate Function GPIO */
    *pREG_PORTF_FER |= (
        MSI0_D0_PORTF_FER |
        MSI0_D1_PORTF_FER |
        MSI0_D2_PORTF_FER |
        MSI0_D3_PORTF_FER |
        MSI0_CMD_PORTF_FER |
        MSI0_CLK_PORTF_FER |
        MSI0_CD_PORTF_FER
    );
    *pREG_PORTF_MUX |= (
        MSI0_D0_PORTF_MUX |
        MSI0_D1_PORTF_MUX |
        MSI0_D2_PORTF_MUX |
        MSI0_D3_PORTF_MUX |
        MSI0_CMD_PORTF_MUX |
        MSI0_CLK_PORTF_MUX |
        MSI0_CD_PORTF_MUX
    );


    result = adi_gpio_Init(gpioMemory, sizeof(gpioMemory), &numCallbacks);

    /* LEDs 10, 11, 12 */
    result = adi_gpio_SetDirection(
        ADI_GPIO_PORT_D,
        ADI_GPIO_PIN_1 | ADI_GPIO_PIN_2 | ADI_GPIO_PIN_3,
        ADI_GPIO_DIRECTION_OUTPUT
    );

    result = adi_gpio_Clear (
        ADI_GPIO_PORT_D,
        ADI_GPIO_PIN_1 | ADI_GPIO_PIN_2 | ADI_GPIO_PIN_3
    );

    /* Init Pushbuttons */
    result = adi_gpio_SetDirection(
        ADI_GPIO_PORT_F,
        ADI_GPIO_PIN_0 | ADI_GPIO_PIN_1,
        ADI_GPIO_DIRECTION_INPUT
    );

    /* Init A2B IO7 as an Input */
    result = adi_gpio_SetDirection(
        ADI_GPIO_PORT_F,
        ADI_GPIO_PIN_13,
        ADI_GPIO_DIRECTION_INPUT
    );

    /* Init Ethernet reset pin (leave in reset)*/
    result = adi_gpio_Clear (
        ADI_GPIO_PORT_B,
        ADI_GPIO_PIN_7
    );
    result = adi_gpio_SetDirection(
        ADI_GPIO_PORT_B,
        ADI_GPIO_PIN_7,
        ADI_GPIO_DIRECTION_OUTPUT
    );

    /* PADS0 DAI0/1 Port Input Enable Control Register */
    *pREG_PADS0_DAI0_IE = (unsigned int)0x001FFFFE;
    *pREG_PADS0_DAI1_IE = (unsigned int)0x001FFFFE;

    // ADAU1761 I2C Address pins set low (DAI0_5 and PORTB.6)
    SRU(HIGH, DAI0_PBEN05_I);         // configure DAI0_P05 as an output
    SRU(LOW, DAI0_PB05_I);            // set 1761 I2C address pins low
#ifdef RED_BOARD
    SRU(HIGH, DAI0_PBEN06_I);         // configure DAI0_P06 as an output
    SRU(LOW, DAI0_PB06_I);            // set 1761 I2C address pins low
#else
    result = adi_gpio_SetDirection(
        ADI_GPIO_PORT_B,
        ADI_GPIO_PIN_6,
        ADI_GPIO_DIRECTION_OUTPUT
    );
    result = adi_gpio_Clear(
        ADI_GPIO_PORT_B,
        ADI_GPIO_PIN_6
    );
#endif
}

/*
 * This macro is used to set the interrupt priority.  Interrupts of a
 * higher priority (lower number) will nest with interrupts of a lower
 * priority (higher number).
 *
 * Priority can range from 0 (highest) to 15 (lowest)
 *
 * Currently only USB interrupts are elevated, all others are lower.
 *
 */
#define INTERRUPT_PRIO(x) \
    ((configMAX_API_CALL_INTERRUPT_PRIORITY + x) << portPRIORITY_SHIFT)

/***********************************************************************
 * GIC Initialization
 **********************************************************************/
void gic_init(void)
{
    ADI_GIC_RESULT  result;

    result = adi_gic_Init();

#ifdef FREE_RTOS
    uint32_t saeIrq = sae_getInterruptID();
    /*
     * Setup peripheral interrupt priorities:
     *   Details: FreeRTOSv9.0.0/portable/GCC/ARM_CA9/port.c (line 574)
     *
     * All registered system interrupts can be identified by setting a breakpoint in
     * adi_rtl_register_dispatched_handler().
     *
     * Interrupts that need to call FreeRTOS functions, or that must be suspended during
     * critical section processing, must be registered with the required interrupt priority.
     *
     * If you end up in vAssertCalled() from vPortValidateInterruptPriority() then
     * the offending interrupt must be added here.  The interrupt ID (iid) can be found
     * by looking backwards in the call stack in vApplicationIRQHandler().
     *
     */
    adi_gic_SetBinaryPoint(ADI_GIC_CORE_0, 0);
    adi_gic_SetIntPriority(INTR_SPI0_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPI1_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPI2_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_TWI0_DATA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_TWI1_DATA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_TWI2_DATA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_UART0_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_UART1_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_UART2_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT0_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT0_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT1_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT1_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT2_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT2_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_MSI0_STAT, INTERRUPT_PRIO(1));

    /* TMR0, INTR_USB0_DATA is used for UAC2.0 */
    adi_gic_SetIntPriority(INTR_TIMER0_TMR0, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_USB0_DATA, INTERRUPT_PRIO(0));
    adi_gic_SetIntPriority(INTR_USB0_STAT, INTERRUPT_PRIO(0));

    /* SHARC Audio Engine IRQ */
    adi_gic_SetIntPriority(saeIrq, INTERRUPT_PRIO(1));

    /* HADC0 interrupt */
    adi_gic_SetIntPriority(INTR_HADC0_EVT, INTERRUPT_PRIO(1));

    /* MSI0 interrupt */
    adi_gic_SetIntPriority(INTR_MSI0_STAT, INTERRUPT_PRIO(1));

    /* WARNING: The ADI FreeRTOS port uses TMR7 as the tick timer which must be configured as the lowest
     *          priority interrupt.  If you're using the stock ADI v9.0.0 or v10.0.1 port, be sure to
     *          enable the line below.  This countermeasure has been applied to the reusable module
     *          FreeRTOS v10.2.1.
     *
     *    The SysTick handler needs to run at the lowest priority.  This is because the critical section
     *    within the handler itself assumes it is running at the lowest priority, so saves time by not
     *    saving the old priority mask and then restoring the previous priority mask.
     */
    //adi_gic_SetIntPriority(INTR_TIMER0_TMR7, 30 << portPRIORITY_SHIFT);
#endif
}

/***********************************************************************
 * libc heap initialization
 **********************************************************************/
#ifndef STD_C_HEAP_SIZE
#define STD_C_HEAP_SIZE (1024 * 1024)
#endif
uint8_t __adi_heap_object[STD_C_HEAP_SIZE] __attribute__ ((section (".heap")));

/***********************************************************************
 * libc stack initialization
 **********************************************************************/
uint8_t __adi_stack_sys_object[1024] __attribute__ ((section (".stack_sys")));
uint8_t __adi_stack_sup_object[1024] __attribute__ ((section (".stack_sup")));
uint8_t __adi_stack_fiq_object[1024] __attribute__ ((section (".stack_fiq")));
uint8_t __adi_stack_irq_object[1024] __attribute__ ((section (".stack_irq")));
uint8_t __adi_stack_abort_object[1024] __attribute__ ((section (".stack_abort")));
uint8_t __adi_stack_undef_object[1024] __attribute__ ((section (".stack_undef")));

/***********************************************************************
 * UMM_MALLOC heap initialization
 **********************************************************************/
__attribute__ ((section(".heap")))
    static uint8_t umm_sdram_heap[UMM_SDRAM_HEAP_SIZE];

__attribute__ ((section(".l3_uncached_data")))
    static uint8_t umm_sdram_uncached_heap[UMM_SDRAM_UNCACHED_HEAP_SIZE];

__attribute__ ((section(".l2_uncached_data")))
    static uint8_t umm_l2_uncached_heap[UMM_L2_UNCACHED_HEAP_SIZE];

__attribute__ ((section(".l2_cached_data")))
    static uint8_t umm_l2_cached_heap[UMM_L2_CACHED_HEAP_SIZE];

void heap_init(void)
{
    /* Initialize the cached L3 SDRAM heap (default heap). */
    umm_init(UMM_SDRAM_HEAP, umm_sdram_heap, UMM_SDRAM_HEAP_SIZE);

    /* Initialize the un-cached L3 SDRAM heap. */
    umm_init(UMM_SDRAM_UNCACHED_HEAP, umm_sdram_uncached_heap,
        UMM_SDRAM_UNCACHED_HEAP_SIZE);

    /* Initialize the L2 uncached heap. */
    umm_init(UMM_L2_UNCACHED_HEAP, umm_l2_uncached_heap,
        UMM_L2_UNCACHED_HEAP_SIZE);

    /* Initialize the L2 cached heap. */
    umm_init(UMM_L2_CACHED_HEAP, umm_l2_cached_heap, UMM_L2_CACHED_HEAP_SIZE);
}

/***********************************************************************
 * SPI Flash initialization
 **********************************************************************/
void flash_init(APP_CONTEXT *context)
{
    SPI_SIMPLE_RESULT spiResult;

    /* Open a SPI handle to SPI2 */
    spiResult = spi_open(SPI2, &context->spi2Handle);

    /* Open a SPI2 device handle for the flash */
    spiResult = spi_openDevice(context->spi2Handle, &context->spiFlashHandle);

    /* Configure the flash device handle */
    spiResult = spi_setClock(context->spiFlashHandle, 1);
    spiResult = spi_setMode(context->spiFlashHandle, SPI_MODE_3);
    spiResult = spi_setFastMode(context->spiFlashHandle, true);
    spiResult = spi_setLsbFirst(context->spiFlashHandle, false);
    spiResult = spi_setSlaveSelect(context->spiFlashHandle, SPI_SSEL_1);

    /* Try to open the Micron flash driver (HW Rev 1.4 and below) first.
     * If that doesn't succeeed, open the ISSI flash (HW Rev 1.5 and above).
     */
    context->flashHandle = mt25q_open(context->spiFlashHandle);
    if (context->flashHandle == NULL) {
        context->flashHandle = is25lp_open(context->spiFlashHandle);
    }
}

/***********************************************************************
 * CGU Timestamp init
 **********************************************************************/
void cgu_ts_init(void)
{
    /* Configure the CGU timestamp counter.  See clocks.h for more detail. */
    *pREG_CGU0_TSCTL =
        ( 1 << BITP_CGU_TSCTL_EN ) |
        ( CGU_TS_DIV << BITP_CGU_TSCTL_TSDIV );
}

/***********************************************************************
 * This function allocates audio buffers in L3 cached memory and
 * initializes a single SPORT using the simple SPORT driver.
 **********************************************************************/
static sSPORT *single_sport_init(SPORT_SIMPLE_PORT sport,
    SPORT_SIMPLE_CONFIG *cfg, SPORT_SIMPLE_AUDIO_CALLBACK cb,
    void **pingPongPtrs, unsigned *pingPongLen, void *usrPtr,
    bool cached, SPORT_SIMPLE_RESULT *result)
{
    sSPORT *sportHandle;
    SPORT_SIMPLE_RESULT sportResult;
    uint32_t dataBufferSize;
    int i;

    /* Open a handle to the SPORT */
    sportResult = sport_open(sport, &sportHandle);
    if (sportResult != SPORT_SIMPLE_SUCCESS) {
        if (result) {
            *result = sportResult;
        }
        return(NULL);
    }

    /* Copy application callback info */
    cfg->callBack = cb;
    cfg->usrPtr = usrPtr;

    /* Allocate audio buffers if not already allocated */
    dataBufferSize = sport_buffer_size(cfg);
    for (i = 0; i < 2; i++) {
        if (!cfg->dataBuffers[i]) {
            cfg->dataBuffers[i] = umm_malloc_heap_aligned(
                UMM_SDRAM_HEAP, dataBufferSize, sizeof(uint32_t));
            memset(cfg->dataBuffers[i], 0, dataBufferSize);
        }
    }
    cfg->dataBuffersCached = cached;
    cfg->syncDMA = true;

    /* Configure the SPORT */
    sportResult = sport_configure(sportHandle, cfg);

    /* Save ping pong data pointers */
    if (pingPongPtrs) {
        pingPongPtrs[0] = cfg->dataBuffers[0];
        pingPongPtrs[1] = cfg->dataBuffers[1];
    }
    if (pingPongLen) {
        *pingPongLen = dataBufferSize;
    }
    if (result) {
        *result = sportResult;
    }

    return(sportHandle);
}

/***********************************************************************
 * Simple SPORT driver 8-ch TDM settings
 **********************************************************************/
SPORT_SIMPLE_CONFIG cfgTDM8x1 = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_MASTER,
    .dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN,
    .bitClkOptions = SPORT_SIMPLE_CLK_FALLING,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_EARLY,
    .tdmSlots = SPORT_SIMPLE_TDM_8,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY,
    .frames = SYSTEM_BLOCK_SIZE,
};

/***********************************************************************
 * PCGA generates 12.288 MHz TDM8 BCLK from 24.576 MCLK/BCLK
 **********************************************************************/
void adau1761_cfg_pcg(void)
{
    /* Configure static PCG A parameters */
    PCG_SIMPLE_CONFIG pcg_a = {
        .pcg = PCG_A,                    // PCG A
        .clk_src = PCG_SRC_DAI_PIN,      // Sourced from DAI
        .clk_in_dai_pin = DAI0_MCLK_PIN, // Sourced from DAI MCLK pin
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure the PCG BCLK depending on the cfgTDM8x1 SPORT config */
    pcg_a.bitclk_div =
        SYSTEM_MCLK_RATE / (cfgTDM8x1.wordSize * cfgTDM8x1.tdmSlots * SYSTEM_SAMPLE_RATE);
    assert(pcg_a.bitclk_div > 0);

    /* This sets everything up */
    pcg_open(&pcg_a);
    pcg_enable(PCG_A, true);
}

void debug_signal_init(void)
{
#if 0
    /* Enable this code to route the I2S signals to DAI pins on the expansion
     * header for debugging.
     */
    SRU(HIGH,  DAI0_PBEN14_I);
    SRU(HIGH,  DAI0_PBEN15_I);
    SRU(HIGH,  DAI0_PBEN16_I);

    // Route select signals to higher DAI pins on expansion header for debug purposes
    SRU(PCG0_CLKB_O, DAI0_PB14_I);
    SRU(SPT0_AFS_O, DAI0_PB15_I);
    SRU(SPT2_AFS_O, DAI0_PB16_I);
#endif
}

/***********************************************************************
 * ADAU1761 CODEC / SPORT0 / SRU initialization (TDM8 clock slave)
 **********************************************************************/
static void sru_config_sharc_sam_adau1761_slave(void)
{
    SRU(HIGH, DAI0_PBEN01_I);        // ADAU1761 DAC data is an output
    SRU(LOW,  DAI0_PBEN02_I);        // ADAU1761 ADC data is an input
    SRU(HIGH, DAI0_PBEN03_I);        // ADAU1761 CLK is an output
    SRU(HIGH, DAI0_PBEN04_I);        // ADAU1761 FS is an output

    SRU(PCG0_CLKA_O, SPT0_ACLK_I);   // PCG-A BCLK to SPORT0A BCLK
    SRU(PCG0_CLKA_O, SPT0_BCLK_I);   // PCG-A BCLK to SPORT0A BCLK
    SRU(PCG0_CLKA_O, DAI0_PB03_I);   // PCG-A BCLK to ADAU1761

    SRU(SPT0_AFS_O, DAI0_PB04_I);    // SPORT0A FS to ADAU1761
    SRU(SPT0_AFS_O, SPT0_BFS_I);     // SPORT0A FS to SPORT0B

    SRU(DAI0_PB02_O, SPT0_BD0_I);     // ADAU1761 ADC pin to SPORT0B input
    SRU(SPT0_AD0_O,  DAI0_PB01_I);    // SPORT0A output to ADAU1761 DAC pin
}

#define SAM_ADAU1761_I2C_ADDR  (0x38)

void adau1761_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    unsigned len;

    /* SPORT0A: CODEC data out */
    sportCfg = cfgTDM8x1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->codecAudioOut, sizeof(sportCfg.dataBuffers));
    context->codecSportOutHandle = single_sport_init(
        SPORT0A, &sportCfg, codecAudioOut,
        NULL, &len, context, false, NULL
    );
    assert(context->codecAudioOutLen == len);


    /* SPORT0B: CODEC data in */
    sportCfg = cfgTDM8x1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->codecAudioIn, sizeof(sportCfg.dataBuffers));
    context->codecSportInHandle = single_sport_init(
        SPORT0B, &sportCfg, codecAudioIn,
        NULL, &len, context, false, NULL
    );
    assert(context->codecAudioInLen == len);

    /* Start SPORT0A/B */
    sportResult = sport_start(context->codecSportOutHandle, true);
    sportResult = sport_start(context->codecSportInHandle, true);
}

void adau1761_sport_deinit(APP_CONTEXT *context)
{
    if (context->codecSportOutHandle) {
        sport_close(&context->codecSportOutHandle);
    }
    if (context->codecSportInHandle) {
        sport_close(&context->codecSportInHandle);
    }
}

void adau1761_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    sru_config_sharc_sam_adau1761_slave();

    /* Configure the 1761 bit clock PCG */
    adau1761_cfg_pcg();

    /* Initialize the CODEC */
    init_adau1761(context->adau1761TwiHandle, SAM_ADAU1761_I2C_ADDR);

    /* Initialize the CODEC SPORTs */
    adau1761_sport_init(context);

}

/**************************************************************************
 * SPDIF Init
 *************************************************************************/
SPORT_SIMPLE_CONFIG cfgI2Sx1 = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_MASTER,
    .dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN,
    .bitClkOptions = SPORT_SIMPLE_CLK_FALLING,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_INV | SPORT_SIMPLE_FS_OPTION_EARLY |
                 SPORT_SIMPLE_FS_OPTION_50,
    .tdmSlots = SPORT_SIMPLE_TDM_2,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY,
    .frames = SYSTEM_BLOCK_SIZE,
};

/* PCGB generates 3.072 MHz I2S BCLK from 24.576 MCLK/BCLK */
void spdif_cfg_pcg(void)
{
    /* Configure static PCG A parameters */
    PCG_SIMPLE_CONFIG pcg_b = {
        .pcg = PCG_B,                    // PCG B
        .clk_src = PCG_SRC_DAI_PIN,      // Sourced from DAI
        .clk_in_dai_pin = DAI0_MCLK_PIN, // Sourced from DAI pin 6
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure the PCG BCLK depending on the cfgI2Sx1 SPORT config */
    pcg_b.bitclk_div =
        SYSTEM_MCLK_RATE / (cfgI2Sx1.wordSize * cfgI2Sx1.tdmSlots * SYSTEM_SAMPLE_RATE);
    assert(pcg_b.bitclk_div > 0);

    /* This sets everything up */
    pcg_open(&pcg_b);
    pcg_enable(PCG_B, true);
}

/*
 * WARNING: The SPDIF HFCLK is derived from the TDM8 clock used
 *          for the 1761 CODEC coming from PCG0_CLKA_O
 *
 * WARNING: The SPDIF BCLK is derived from the PCG0_CLKB_O
 *
 */
static void spdif_sru_config(void)
{
    // Assign SPDIF I/O pins
    SRU(HIGH, DAI0_PBEN20_I);       // SPDIF TX is an output
    SRU(LOW,  DAI0_PBEN19_I);       // SPDIF RX is an input

    // Connect I/O pins to SPDIF module
    SRU(DAI0_PB19_O, SPDIF0_RX_I);  // route DAI0_PB19 to SPDIF RX
    SRU(SPDIF0_TX_O, DAI0_PB20_I);  // route SPDIF TX to DAI0_PB20

    // Connect 64Fs BCLK to SPORT2A/B
    SRU(PCG0_CLKB_O, SPT2_ACLK_I);     // route PCG 64fs BCLK signal to SPORT2A BCLK
    SRU(PCG0_CLKB_O, SPT2_BCLK_I);     // route PCG 64fs BCLK signal to SPORT2B BCLK

    // Connect SPDIF RX to SRC 0 "IP" side
    SRU(SPDIF0_RX_CLK_O, SRC0_CLK_IP_I);     // route SPDIF RX BCLK to SRC IP BCLK
    SRU(SPDIF0_RX_FS_O,  SRC0_FS_IP_I);      // route SPDIF RX FS to SRC IP FS
    SRU(SPDIF0_RX_DAT_O, SRC0_DAT_IP_I);     // route SPDIF RX Data to SRC IP Data

    // Connect SPORT2B to SRC 0 "OP" side
    SRU(PCG0_CLKB_O,    SRC0_CLK_OP_I);     // route PCG 64fs BCLK signal to SRC OP BCLK
    SRU(SPT2_BFS_O,     SRC0_FS_OP_I);      // route PCG FS signal to SRC OP FS
    SRU(SRC0_DAT_OP_O,  SPT2_BD0_I);        // route SRC0 OP Data output to SPORT 2B data

    // Connect 256Fs MCLK to SPDIF TX
    SRU(PCG0_CLKA_O, SPDIF0_TX_HFCLK_I); // route PCGA_CLK to SPDIF TX HFCLK

    // Connect SPORT2A to SPDIF TX
    SRU(PCG0_CLKB_O, SPDIF0_TX_CLK_I);    // route 64fs BCLK signal to SPDIF TX BCLK
    SRU(SPT2_AFS_O,  SPDIF0_TX_FS_I);     // route SPORT2A FS signal to SPDIF TX FS
    SRU(SPT2_AD0_O,  SPDIF0_TX_DAT_I);    // SPT2A AD0 output to SPDIF TX data pin
}

void spdif_sport_deinit(APP_CONTEXT *context)
{
    if (context->spdifSportOutHandle) {
        sport_close(&context->spdifSportOutHandle);
    }
    if (context->spdifSportInHandle) {
        sport_close(&context->spdifSportInHandle);
    }
}

void spdif_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    unsigned len;

    /* SPORT2A: SPDIF data out */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->spdifAudioOut, sizeof(sportCfg.dataBuffers));
    context->spdifSportOutHandle = single_sport_init(
        SPORT2A, &sportCfg, spdifAudioOut,
        NULL, &len, context, false, NULL
    );
    assert(context->spdifAudioOutLen == len);


    /* SPORT2B: SPDIF data in */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->spdifAudioIn, sizeof(sportCfg.dataBuffers));
    context->spdifSportInHandle = single_sport_init(
        SPORT2B, &sportCfg, spdifAudioIn,
        NULL, &len, context, false, NULL
    );
    assert(context->spdifAudioInLen == len);

    /* Start SPORT0A/B */
    sportResult = sport_start(context->spdifSportOutHandle, true);
    sportResult = sport_start(context->spdifSportInHandle, true);
}

void spdif_asrc_init(void)
{
    // Configure and enable SRC 0/1
    *pREG_ASRC0_CTL01 =
        BITM_ASRC_CTL01_EN0 |                // Enable SRC0
        (0x1 << BITP_ASRC_CTL01_SMODEIN0) |  // Input mode = I2S
        (0x1 << BITP_ASRC_CTL01_SMODEOUT0) | // Output mode = I2S
        0;

    // Configure and enable SPDIF RX
    *pREG_SPDIF0_RX_CTL =
        BITM_SPDIF_RX_CTL_EN |          // Enable the SPDIF RX
        BITM_SPDIF_RX_CTL_FASTLOCK |    // Enable SPDIF Fastlock (see HRM 32-15)
        BITM_SPDIF_RX_CTL_RSTRTAUDIO |
        0;

    // Configure SPDIF Transmitter in auto mode
    *pREG_SPDIF0_TX_CTL =
        (0x1 << BITP_SPDIF_TX_CTL_SMODEIN) |  // I2S Mode
        BITM_SPDIF_TX_CTL_AUTO |             // Standalone mode
        0;

    // Enable SPDIF transmitter
    *pREG_SPDIF0_TX_CTL |=
        BITM_SPDIF_TX_CTL_EN |         // Enable SPDIF TX
        0;
}

void spdif_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    spdif_sru_config();

    /* Initialize the SPDIF HFCLK PCG */
    spdif_cfg_pcg();

    /* Initialize the SPDIF and ASRC modules */
    spdif_asrc_init();

    /* Initialize the SPORTs */
    spdif_sport_init(context);
}

/***********************************************************************
 * AD2425 / SPORT1 / SRU initialization
 **********************************************************************/

static void ad2425_disconnect_master_clocks(void)
{
    // Set A2B BCLK LOW
    SRU(LOW, DAI0_PB07_I);
    // Set A2B FS LOW
    SRU(LOW, DAI0_PB08_I);
}

static void ad2425_connect_master_clocks(void)
{
    // Route BCLK to A2B BCLK
#ifdef RED_BOARD
    SRU(DAI0_PB13_O, DAI0_PB07_I);
#else
    SRU(DAI0_PB06_O, DAI0_PB07_I);
#endif
    // Route FS to A2B SYNC
    SRU(SPT1_AFS_O, DAI0_PB08_I);
}

static void ad2425_disconnect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE &= ~(
        BITP_PADS0_DAI0_IE_PB07 | BITP_PADS0_DAI0_IE_PB08
    );
}

static void ad2425_connect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE |= (
        BITP_PADS0_DAI0_IE_PB07 | BITP_PADS0_DAI0_IE_PB08
    );
}

/**
 *
 * A2B Master Mode Configuration:
 *    - MCLK/BCLK to SPORT1B/A2B Transceiver
 *    - SPORT1A FS to SPORT1B/A2B Transceiver
 *
 * NOTE: This function does not connect the A2B transceiver FS and BCLK.
 *       That happens in ad2425_connect_clocks().
 *
 */
void sru_config_a2b_master(void)
{
    // Set up pins for AD2425W (A2B)
    SRU(HIGH,  DAI0_PBEN07_I);        // pin for A2B BCLK is an output (to A2B bus)
    SRU(HIGH,  DAI0_PBEN08_I);        // pin for A2B FS is an output (to A2B bus)
    SRU(LOW,   DAI0_PBEN09_I);        // DTX0 is always an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN10_I);        // DTX1 is always an input (from A2B bus)
    SRU(HIGH,  DAI0_PBEN11_I);        // DRX0 is always an output (to A2B bus)
    SRU(HIGH,  DAI0_PBEN12_I);        // DRX1 is always an output (to A2B bus)

    // BCLK/MCLK to SPORTA/B CLK */
#ifdef RED_BOARD
    SRU(DAI0_PB13_O, SPT1_ACLK_I);     // route MCLK/BCLK to SPORT1A
    SRU(DAI0_PB13_O, SPT1_BCLK_I);     // route MCLK/BCLK to SPORT1B
#else
    SRU(DAI0_PB06_O, SPT1_ACLK_I);     // route MCLK/BCLK to SPORT1A
    SRU(DAI0_PB06_O, SPT1_BCLK_I);     // route MCLK/BCLK to SPORT1B
#endif

    // SPORT1A FS to SPORT1B FS */
    SRU(SPT1_AFS_O, SPT1_BFS_I);      // route SPORT1A FS to SPORT1B

    // Connect A2B data signals to SPORT1
    SRU(SPT1_AD0_O, DAI0_PB11_I);     // route SPORT1A data TX primary to A2B DRX0
    SRU(SPT1_AD1_O, DAI0_PB12_I);     // route SPORT1A data TX secondary to A2B DRX0
    SRU(DAI0_PB09_O, SPT1_BD0_I);     // route A2B DTX0 to SPORT1B data RX primary
    SRU(DAI0_PB10_O, SPT1_BD1_I);     // route A2B DTX1 to SPORT1B data RX secondary
}

/**
 *
 * A2B Slave Mode Configuration:
 *    - A2B BCLK to SPORT1B
 *    - A2B FS to SPORT1B
 *
 */
void sru_config_a2b_slave(void)
{
    // Set up pins for AD2425W (A2B)
    SRU(LOW,   DAI0_PBEN07_I);        // pin for A2B BCLK is an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN08_I);        // pin for A2B FS is an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN09_I);        // DTX0 is always an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN10_I);        // DTX1 is always an input (from A2B bus)
    SRU(HIGH,  DAI0_PBEN11_I);        // DRX0 is always an output (to A2B bus)
    SRU(HIGH,  DAI0_PBEN12_I);        // DRX1 is always an output (to A2B bus)

    // A2B BCLK SPORTA/B CLK */
    SRU(DAI0_PB07_O, SPT1_ACLK_I);     // route A2B BCLK to SPORT1A
    SRU(DAI0_PB07_O, SPT1_BCLK_I);     // route A2B BCLK to SPORT1B

    // A2B BCLK SPORTA/B CLK */
    SRU(DAI0_PB08_O, SPT1_AFS_I);     // route A2B FS to SPORT1A
    SRU(DAI0_PB08_O, SPT1_BFS_I);     // route A2B FS to SPORT1B

    // Connect A2B data signals to SPORT1
    SRU(SPT1_AD0_O, DAI0_PB11_I);     // route SPORT1A data TX primary to A2B DRX0
    SRU(SPT1_AD1_O, DAI0_PB12_I);     // route SPORT1A data TX secondary to A2B DRX0
    SRU(DAI0_PB09_O, SPT1_BD0_I);     // route A2B DTX0 to SPORT1B data RX primary
    SRU(DAI0_PB10_O, SPT1_BD1_I);     // route A2B DTX1 to SPORT1B data RX secondary
}

#define AD242X_CONTROL             0x12u
#define AD242X_CONTROL_SOFTRST     0x04u
#define AD242X_CONTROL_MSTR        0x80u

/* Soft reset a single transceiver */
bool ad2425_restart(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT result;
    uint8_t wBuf[2];

    wBuf[0] = AD242X_CONTROL;
    wBuf[1] = AD242X_CONTROL_SOFTRST;
    if (context->a2bmode == A2B_BUS_MODE_MAIN) {
        wBuf[1] |= AD242X_CONTROL_MSTR;
    }

    result = twi_write(context->ad2425TwiHandle, AD2425W_SAM_I2C_ADDR,
        wBuf, sizeof(wBuf));

    delay(10);

    return(result == TWI_SIMPLE_SUCCESS);
}

void ad2425_reset(APP_CONTEXT *context)
{
#if 0
    // Idle A2B SYNC pin for at least 1mS to reset transceiver */
    ad2425_disconnect_master_clocks();
    delay(2);
    ad2425_connect_master_clocks();
    delay(2);
    UNUSED(context);
#else
    ad2425_restart(context);
#endif
}

void sportCfg2ipcMsg(SPORT_SIMPLE_CONFIG *sportCfg, unsigned dataLen, IPC_MSG *msg)
{
    msg->audio.wordSize = sportCfg->wordSize / 8;
    msg->audio.numChannels = dataLen / (sportCfg->frames * msg->audio.wordSize);
}

bool ad2425_sport_init(APP_CONTEXT *context,
    bool master, CLOCK_DOMAIN clockDomain, uint8_t I2SGCFG, uint8_t I2SCFG,
    bool verbose)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    bool sportCfgOk;
    IPC_MSG *msg;
    bool rxtx;
    int i;

    /* Calculate the SPORT0A TX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = false;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG,
        &sportCfg, verbose, A2B_TO_SPORT_CFG_XCVR_AD242x);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
    if (master) {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    } else {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    }
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->a2bAudioOut, sizeof(sportCfg.dataBuffers));
    context->a2bSportOutHandle = single_sport_init(
        SPORT1A, &sportCfg, a2bAudioOut,
        NULL, &context->a2bAudioOutLen, context, false, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        for (i = 0; i < 2; i++) {
            msg = (IPC_MSG *)sae_getMsgBufferPayload(context->a2bMsgOut[i]);
            sportCfg2ipcMsg(&sportCfg, context->a2bAudioOutLen, msg);
        }
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_OUT);
        sportResult = sport_start(context->a2bSportOutHandle, true);
    } else {
        if (context->a2bSportOutHandle) {
            sport_close(&context->a2bSportOutHandle);
        }
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_OUT);
    }

    /* Calculate the SPORT0B RX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = true;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG,
        &sportCfg, verbose, A2B_TO_SPORT_CFG_XCVR_AD242x);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->a2bAudioIn, sizeof(sportCfg.dataBuffers));
    context->a2bSportInHandle = single_sport_init(
        SPORT1B, &sportCfg, a2bAudioIn,
        NULL, &context->a2bAudioInLen, context, false, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        for (i = 0; i < 2; i++) {
            msg = (IPC_MSG *)sae_getMsgBufferPayload(context->a2bMsgIn[i]);
            sportCfg2ipcMsg(&sportCfg, context->a2bAudioInLen, msg);
        }
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_IN);
        sportResult = sport_start(context->a2bSportInHandle, true);
    } else {
        if (context->a2bSportInHandle) {
            sport_close(&context->a2bSportInHandle);
        }
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_IN);
    }

abort:
    return(sportCfgOk);
}

bool ad2425_sport_deinit(APP_CONTEXT *context)
{
    if (context->a2bSportOutHandle) {
        sport_close(&context->a2bSportOutHandle);
    }
    if (context->a2bSportInHandle) {
        sport_close(&context->a2bSportInHandle);
    }
    return(true);
}

bool ad2425_init_master(APP_CONTEXT *context)
{
    bool ok;

    sru_config_a2b_master();

    ok = ad2425_sport_init(context, true, CLOCK_DOMAIN_SYSTEM,
        SYSTEM_I2SGCFG, SYSTEM_I2SCFG, false);
    if (ok) {
        context->a2bmode = A2B_BUS_MODE_MAIN;
        context->a2bSlaveActive = false;
        ad2425_connect_master_clocks();
    }

    return(ok);
}

bool ad2425_init_slave(APP_CONTEXT *context)
{
    sru_config_a2b_slave();

    context->a2bmode = A2B_BUS_MODE_SUB;

    /*
     * Disconnect A2B from all clock domains.  IN and OUT will be re-attached
     * to the A2B domain during discovery when/if the TX and RX serializers
     * are enabled.
     */
    clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_IN);
    clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_OUT);

    return(true);
}

bool ad2425_set_mode(APP_CONTEXT *context, A2B_BUS_MODE mode)
{
    if (mode == context->a2bmode) {
        return(true);
    }

    ad2425_sport_deinit(context);

    if (mode == A2B_BUS_MODE_SUB) {
        ad2425_init_slave(context);
    } else {
        adau1761_sport_deinit(context);
        spdif_sport_deinit(context);
        disable_sport_mclk(context);
        adau1761_sport_init(context);
        spdif_sport_init(context);
        ad2425_init_master(context);
        enable_sport_mclk(context);
    }

    ad2425_restart(context);

    return(true);
}

bool ad2425_sport_start(APP_CONTEXT *context, uint8_t I2SGCFG, uint8_t I2SCFG)
{
    bool ok;
    ok = ad2425_sport_init(context, false, CLOCK_DOMAIN_A2B,
        I2SGCFG, I2SCFG, true);
    ad2425_connect_slave_clocks();
    return(ok);
}

bool ad2425_sport_stop(APP_CONTEXT *context)
{
    bool ok;
    ok = ad2425_sport_deinit(context);
    ad2425_disconnect_slave_clocks();
    return(ok);
}

int sam_hw_version(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT twiResult;
    sTWI *twiHandle;
    int version;

    /* Probe for the SiLabs clock generator */
    twiHandle = context->ethClkTwiHandle;
    twiResult = twi_write(twiHandle, 0x70, NULL, 0);
    if (twiResult == TWI_SIMPLE_SUCCESS) {
        version = SAM_VERSION_1;
    } else {
        version = SAM_VERSION_2;
    }

    return(version);
}

void system_reset(APP_CONTEXT *context)
{
    /*
     * The ISSI flash must be put back to 3 byte address mode to boot.
     * The Micron flash never leaves 3 byte mode.
     */
    is25lp_close(context->flashHandle);
    taskENTER_CRITICAL();
    *pREG_RCU0_CTL = BITM_RCU_CTL_SYSRST | BITM_RCU_CTL_RSTOUTASRT;
    while(1);
}

/***********************************************************************
 * SHARC Audio Engine (SAE) Audio IPC buffer configuration
 **********************************************************************/
/*
 * allocateIpcAudioMsg()
 *
 * Allocates an IPC_MSG_AUDIO Audio message and saves the data payload
 * pointer.
 *
 */
static SAE_MSG_BUFFER *allocateIpcAudioMsg(APP_CONTEXT *context,
    uint16_t size, uint8_t streamID, uint8_t numChannels, uint8_t wordSize,
    void **audioPtr)
{
    SAE_CONTEXT *saeContext = context->saeContext;
    SAE_MSG_BUFFER *msgBuffer;
    IPC_MSG *msg;
    uint16_t msgSize;

    /* Create an IPC message large enough to hold an IPC_MSG_AUDIO struct
     * with the data payload.
     */
    msgSize = sizeof(*msg) + size;

    /* Allocate a message buffer and initialize both the USB_IPC_SRC_MSG's
     * 'msgBuffer' and 'msg' members.
     */
    msgBuffer = sae_createMsgBuffer(saeContext, msgSize, (void **)&msg);
    assert(msgBuffer);

    /* Set fixed 'IPC_MSG_AUDIO' parameters */
    msg->type = IPC_TYPE_AUDIO;
    msg->audio.streamID = streamID;
    msg->audio.numChannels = numChannels;
    msg->audio.wordSize = wordSize;
    msg->audio.numFrames = size / (numChannels * wordSize);
    if (audioPtr) {
        *audioPtr = msg->audio.data;
    }

    return(msgBuffer);
}

/*
 * sae_buffer_init()
 *
 * Allocates and configures all of the SAE message/audio ping/pong
 * buffers between the ARM and SHARC0 and SHARC1.  Audio DMA buffers
 * are sent by reference from the ARM to the SHARCs
 *
 * These buffers can be referenced and used locally through the
 * context->xxxAudioIn/Out[] ping/pong buffers and sent/received via
 * the IPC message buffers context->xxxMsgIn/Out[].
 *
 */
void sae_buffer_init(APP_CONTEXT *context)
{
    APP_CFG *cfg = &context->cfg;
    int i;

    /* Allocate and initialize audio IPC ping/pong message buffers */
    for (i = 0; i < 2; i++) {

        /* CODEC Audio In */
        context->codecAudioInLen =
            CODEC_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->codecMsgIn[i] = allocateIpcAudioMsg(
            context, context->codecAudioInLen,
            IPC_STREAMID_CODEC_IN, CODEC_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->codecAudioIn[i]
        );
        memset(context->codecAudioIn[i], 0, context->codecAudioInLen);

        /* CODEC Audio Out */
        context->codecAudioOutLen =
            CODEC_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->codecMsgOut[i] = allocateIpcAudioMsg(
            context, context->codecAudioOutLen,
            IPC_STREAMID_CODEC_OUT, CODEC_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->codecAudioOut[i]
        );
        memset(context->codecAudioOut[i], 0, context->codecAudioOutLen);

        /* SPDIF Audio In */
        context->spdifAudioInLen =
            SPDIF_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->spdifMsgIn[i] = allocateIpcAudioMsg(
            context, context->spdifAudioInLen,
            IPC_STREAMID_SPDIF_IN, SPDIF_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->spdifAudioIn[i]
        );
        memset(context->spdifAudioIn[i], 0, context->spdifAudioInLen);

        /* SPDIF Audio Out */
        context->spdifAudioOutLen =
            SPDIF_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->spdifMsgOut[i] = allocateIpcAudioMsg(
            context, context->spdifAudioOutLen,
            IPC_STREAMID_SPDIF_OUT, SPDIF_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->spdifAudioOut[i]
        );
        memset(context->spdifAudioOut[i], 0, context->spdifAudioOutLen);

        /* A2B Audio In */
        context->a2bAudioInLen =
            A2B_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->a2bMsgIn[i] = allocateIpcAudioMsg(
            context, context->a2bAudioInLen,
            IPC_STREAMID_A2B_IN, A2B_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->a2bAudioIn[i]
        );
        memset(context->a2bAudioIn[i], 0, context->a2bAudioInLen);

        /* A2B Audio Out */
        context->a2bAudioOutLen =
            A2B_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->a2bMsgOut[i] = allocateIpcAudioMsg(
            context, context->a2bAudioOutLen,
            IPC_STREAMID_A2B_OUT, A2B_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->a2bAudioOut[i]
        );
        memset(context->a2bAudioOut[i], 0, context->a2bAudioOutLen);

        /* Only need one buffer for the rest of these (no ping/pong) */
        if (i == 0) {
            /* USB Audio Rx */
            context->usbAudioRxLen =
                cfg->usbOutChannels * cfg->usbWordSize * SYSTEM_BLOCK_SIZE;
            context->usbMsgRx[i] = allocateIpcAudioMsg(
                context, context->usbAudioRxLen,
                IPC_STREAMID_USB_RX, cfg->usbOutChannels, cfg->usbWordSize,
                &context->usbAudioRx[i]
            );
            memset(context->usbAudioRx[i], 0, context->usbAudioRxLen);

            /* USB Audio Tx */
            context->usbAudioTxLen =
                cfg->usbInChannels * cfg->usbWordSize * SYSTEM_BLOCK_SIZE;
            context->usbMsgTx[i] = allocateIpcAudioMsg(
                context, context->usbAudioTxLen,
                IPC_STREAMID_USB_TX, cfg->usbInChannels, cfg->usbWordSize,
                &context->usbAudioTx[i]
            );
            memset(context->usbAudioTx[i], 0, context->usbAudioTxLen);

            /* WAV Audio Src */
            context->wavAudioSrcLen =
                WAV_MAX_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
            context->wavMsgSrc[i] = allocateIpcAudioMsg(
                context, context->wavAudioSrcLen,
                IPC_STREAM_ID_WAV_SRC, WAV_MAX_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
                &context->wavAudioSrc[i]
            );
            memset(context->wavAudioSrc[i], 0, context->wavAudioSrcLen);

            /* WAV Audio Sink */
            context->wavAudioSinkLen =
                WAV_MAX_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
            context->wavMsgSink[i] = allocateIpcAudioMsg(
                context, context->wavAudioSinkLen,
                IPC_STREAM_ID_WAV_SINK, WAV_MAX_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
                &context->wavAudioSink[i]
            );
            memset(context->wavAudioSink[i], 0, context->wavAudioSinkLen);

        }

    }
}

/*
 * audio_routing_init()
 *
 * Allocates and configures the audio routing array message for use
 * with the SAE.
 *
 */
void audio_routing_init(APP_CONTEXT *context)
{
    SAE_CONTEXT *saeContext = context->saeContext;
    IPC_MSG *msg;
    unsigned msgSize;

    /* Create an IPC message large enough to hold the routing table */
    msgSize = sizeof(*msg) +
        (MAX_AUDIO_ROUTES - 1) * sizeof(ROUTE_INFO);

    /* Allocate a message buffer */
    context->routingMsgBuffer = sae_createMsgBuffer(
        saeContext, msgSize, (void **)&context->routingMsg
    );
    assert(context->routingMsgBuffer);

    /* Initialize the message and routing table */
    memset(context->routingMsg, 0 , msgSize);
    context->routingMsg->type = IPC_TYPE_AUDIO_ROUTING;
    context->routingMsg->routes.numRoutes = MAX_AUDIO_ROUTES;
}
