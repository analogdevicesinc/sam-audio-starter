/**
 * Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* Standard library includes */
#include <string.h>
#include <limits.h>
#include <assert.h>
#include <stdbool.h>

/* ADI service includes */
#include <services/gpio/adi_gpio.h>
#include <services/int/adi_gic.h>
#include <services/tmr/adi_tmr.h>
#include <services/pwr/adi_pwr.h>

/* ADI processor includes */
#include <sruSC589.h>

/* Kernel includes */
#include "FreeRTOS.h"
#include "semphr.h"

/* Simple driver includes */
#include "uart_simple.h"
#include "uart_stdio.h"
#include "twi_simple.h"
#include "spi_simple.h"
#include "sport_simple.h"
#include "pcg_simple.h"
#include "flash.h"
#include "is25lp512.h"
#include "mt25ql512.h"

/* Simple service includes */
#include "adau1761.h"
#include "syslog.h"
#include "a2b_to_sport_cfg.h"
#include "sae.h"

/* OSS service includes */
#include "umm_malloc.h"
#include "umm_malloc_heaps.h"

/* Application includes */
#include "context.h"
#include "clocks.h"
#include "init.h"
#include "util.h"
#include "codec_audio.h"
#include "spdif_audio.h"
#include "a2b_audio.h"
#include "route.h"
#include "clock_domain.h"
#include "si3536.h"

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

    /* Make sure they match clocks.h */
    assert(_cclk == CCLK);
    assert(sclk == SYSCLK);
    assert(sclk0 == SCLK0);
    assert(oclk == OCLK);

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

/* DAI IE Bit definitions (not in any ADI header files) */
#define BITP_PADS0_DAI0_IE_PB06   (6)
#define BITP_PADS0_DAI0_IE_PB07   (7)
#define BITP_PADS0_DAI0_IE_PB08   (8)
#define BITP_PADS0_DAI0_IE_PB13   (13)

#define BITP_PADS0_DAI0_IE_MCLK BITP_PADS0_DAI0_IE_PB06
#define DAI0_MCLK_PIN 6

/*
 * WARNING: Order must match the GPIO_PIN_ID enum in gpio_pins.h!
 */
GPIO_CONFIG gpioPins[GPIO_PIN_MAX] = {
    { ADI_GPIO_PORT_F, ADI_GPIO_PIN_0,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_PB1
    { ADI_GPIO_PORT_F, ADI_GPIO_PIN_1,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_PB2
    { ADI_GPIO_PORT_D, ADI_GPIO_PIN_1,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_LED10
    { ADI_GPIO_PORT_D, ADI_GPIO_PIN_2,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_LED11
    { ADI_GPIO_PORT_D, ADI_GPIO_PIN_3,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_LED12
    { ADI_GPIO_PORT_F, ADI_GPIO_PIN_7,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_A2B_IRQ
    { ADI_GPIO_PORT_F, ADI_GPIO_PIN_8,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_A2B_IO1
    { ADI_GPIO_PORT_F, ADI_GPIO_PIN_9,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_A2B_IO2
    { ADI_GPIO_PORT_F, ADI_GPIO_PIN_13, ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_A2B_IO7
    { ADI_GPIO_PORT_B, ADI_GPIO_PIN_6,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_1761_ADDR1
    { ADI_GPIO_PORT_B, ADI_GPIO_PIN_7,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_ETH0_RESET
#if 0
    { ADI_GPIO_PORT_E, ADI_GPIO_PIN_14,  ADI_GPIO_DIRECTION_OUTPUT, 0 }, // SIGMA Pin 5 (MISO/Debug)
    { ADI_GPIO_PORT_E, ADI_GPIO_PIN_13,  ADI_GPIO_DIRECTION_OUTPUT, 0 }, // SIGMA Pin 7 (CLK/Debug)
#endif
};

bool gpio_get_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;
    uint32_t value;

    pin = &pinConfig[pinId];

    result = adi_gpio_GetData(pin->port, &value);

    pin->state = value & pin->pinNum;

    return(pin->state);
}

void gpio_set_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId, bool state)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;

    pin = &pinConfig[pinId];

    if (state) {
        result = adi_gpio_Set(pin->port, pin->pinNum);
    } else {
        result = adi_gpio_Clear(pin->port, pin->pinNum);
    }

    pin->state = state;
}

void gpio_toggle_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;

    pin = &pinConfig[pinId];

    result = adi_gpio_Toggle(pin->port, pin->pinNum);

    gpio_get_pin(pinConfig, pinId);
}

void gpio_init_pins(GPIO_CONFIG *pinConfig, int numPins)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;
    int i;

    for (i = 0; i < GPIO_PIN_MAX; i++) {

        /* Select the pin */
        pin = &pinConfig[i];

        /* Set the initial pin state */
        if (pin->state) {
            result = adi_gpio_Set(pin->port, pin->pinNum);
        } else {
            result = adi_gpio_Clear(pin->port, pin->pinNum);
        }

        /* Set the direction */
        result = adi_gpio_SetDirection(pin->port, pin->pinNum, pin->dir);

        /* Set the flags */
        if (pin->flags & GPIO_FLAGS_PUE) {
            //adi_pads_PortPullup(pin->port, pin->pinNum, true);
        }
    }
}

void gpio_init(void)
{
    static uint8_t gpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE * 16];
    uint32_t numCallbacks;
    ADI_GPIO_RESULT result;

    /* Init the GPIO system service */
    result = adi_gpio_Init(gpioMemory, sizeof(gpioMemory), &numCallbacks);


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

    /* Configure EMAC0 Alternate Function GPIO */
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


    /* Configure straight GPIO */
    gpio_init_pins(gpioPins, GPIO_PIN_MAX);

    /* PADS0 DAI0/1 Port Input Enable Control Register */
    *pREG_PADS0_DAI0_IE = BITM_PADS_DAI0_IE_VALUE;
    *pREG_PADS0_DAI1_IE = BITM_PADS_DAI1_IE_VALUE;

    /* ADAU1761 I2C Address pins set low (DAI0_5 and PORTB.6) */
    SRU(HIGH, DAI0_PBEN05_I); SRU(LOW, DAI0_PB05_I);
    gpio_set_pin(gpioPins, GPIO_PIN_1761_ADDR1, 0);
}

/*
 * This macro is used to set the interrupt priority.  Interrupts of a
 * higher priority (lower number) will nest with interrupts of a lower
 * priority (higher number).
 *
 * Priority passed to INTERRUPT_PRIO can range from 0 (highest)
 * until the macro returns (portLOWEST_USABLE_INTERRUPT_PRIORITY - 1).
 * portLOWEST_USABLE_INTERRUPT_PRIORITY is reserved for the timer tick
 * interrupt.
 *
 * USB must be higher priority than SPORTs so that USB audio is not
 * starved during periods of high audio processing load.
 *
 * TWI must be higher than SPORTs so that TWI peripherals can be serviced
 * quickly to react to TWI transaction state changes during high audio
 * processing loads.
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
    adi_gic_SetBinaryPoint(ADI_GIC_CORE_0, 0);
}

/* Highest priority */
uint32_t GIC_IRQ_PRIO_0[] = {
    /* USB */
    INTR_USB0_DATA,
    INTR_USB0_STAT,
    INTR_TIMER0_TMR0,
    0
};

/* Second highest priority */
uint32_t GIC_IRQ_PRIO_1[] = {
    /* TWI */
    INTR_TWI0_DATA,
    INTR_TWI1_DATA,
    INTR_TWI2_DATA,
    0
};

/* Third highest priority */
uint32_t GIC_IRQ_PRIO_2[] = {
    /* SAE */
    INTR_TRU0_SLV3,

    /* SPI */
    INTR_SPI0_STAT,
    INTR_SPI1_STAT,
    INTR_SPI2_STAT,

    /* UART */
    INTR_UART0_STAT,
    INTR_UART1_STAT,
    INTR_UART2_STAT,

    /* SPORTs */
    INTR_SPORT0_A_DMA,
    INTR_SPORT0_B_DMA,
    INTR_SPORT1_A_DMA,
    INTR_SPORT1_B_DMA,
    INTR_SPORT2_A_DMA,
    INTR_SPORT2_B_DMA,

    /* MSI0 */
    INTR_MSI0_STAT,

    /* EMAC0 */
    INTR_EMAC0_STAT,
    INTR_PINT4_BLOCK,

    /* HADC0 */
    INTR_HADC0_EVT,

    /* A2B IRQ */
    INTR_PINT5_BLOCK,
    0
};

void gic_set_irq_prio(void)
{
    ADI_GIC_RESULT  result;
    unsigned idx;

    idx = 0;
    while (GIC_IRQ_PRIO_0[idx]) {
        result = adi_gic_SetIntPriority(GIC_IRQ_PRIO_0[idx], INTERRUPT_PRIO(0));
        idx++;
    }

    idx = 0;
    while (GIC_IRQ_PRIO_1[idx]) {
        result = adi_gic_SetIntPriority(GIC_IRQ_PRIO_1[idx], INTERRUPT_PRIO(1));
        idx++;
    }

    idx = 0;
    while (GIC_IRQ_PRIO_2[idx]) {
        result = adi_gic_SetIntPriority(GIC_IRQ_PRIO_2[idx], INTERRUPT_PRIO(2));
        idx++;
    }
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

void umm_heap_init(void)
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
 * This function allocates audio buffers in L2 cached memory and
 * initializes a single SPORT using the simple SPORT driver.
 *
 * NOTE: OUT buffers are generally marked as not cached since outbound
 *       buffers are flushed in process_audio().  This keeps the driver
 *       from prematurely (and redundantly) flushing the buffer when the
 *       SPORT callback returns.
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

static void single_sport_deinit(sSPORT **sportHandle,
    void **pingPongPtrs, unsigned *pingPongLen)
{
    if (sportHandle && *sportHandle) {
        sport_close(sportHandle);
    }
    if (pingPongPtrs) {
        if (pingPongPtrs[0]) {
            umm_free_heap_aligned(UMM_SDRAM_HEAP, pingPongPtrs[0]);
            pingPongPtrs[0] = NULL;
        }
        if (pingPongPtrs[1]) {
            umm_free_heap_aligned(UMM_SDRAM_HEAP, pingPongPtrs[1]);
            pingPongPtrs[1] = NULL;
        }
    }
    if (pingPongLen) {
        *pingPongLen = 0;
    }
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

void disable_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI0_IE &= ~(1 << BITP_PADS0_DAI0_IE_MCLK);
}

void enable_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI0_IE |= (1 << BITP_PADS0_DAI0_IE_MCLK);
}

static void sru_config_mclk(APP_CONTEXT *context)
{
    SRU(LOW, DAI0_PBEN08_I);
}

static void pcg_init_dai0_tdm8_bclk(void)
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

void mclk_init(APP_CONTEXT *context)
{
    sru_config_mclk(context);
    pcg_init_dai0_tdm8_bclk();
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

    /* SPORT0A: CODEC data out */
    sportCfg = cfgTDM8x1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    memcpy(sportCfg.dataBuffers, context->codecAudioOut, sizeof(sportCfg.dataBuffers));
    context->codecSportOutHandle = single_sport_init(
        SPORT0A, &sportCfg, codecAudioOut,
        context->codecAudioOut, &context->codecAudioOutLen, context, false, NULL
    );
    if (context->codecSportOutHandle) {
        sportResult = sport_start(context->codecSportOutHandle, true);
    }


    /* SPORT0B: CODEC data in */
    sportCfg = cfgTDM8x1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    memcpy(sportCfg.dataBuffers, context->codecAudioIn, sizeof(sportCfg.dataBuffers));
    context->codecSportInHandle = single_sport_init(
        SPORT0B, &sportCfg, codecAudioIn,
        context->codecAudioIn, &context->codecAudioInLen, context, true, NULL
    );
    if (context->codecSportInHandle) {
        sportResult = sport_start(context->codecSportInHandle, true);
    }
}

void adau1761_sport_deinit(APP_CONTEXT *context)
{
    if (context->codecSportOutHandle) {
        single_sport_deinit(
            &context->codecSportOutHandle, context->codecAudioOut, &context->codecAudioOutLen
        );
    }
    if (context->codecSportInHandle) {
        single_sport_deinit(
            &context->codecSportInHandle, context->codecAudioIn, &context->codecAudioInLen
        );
    }
}

void adau1761_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    sru_config_sharc_sam_adau1761_slave();

    /* Configure the TDM8 bit clock PCG */
    pcg_init_dai0_tdm8_bclk();

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

/* PCGA generates 12.288 MHz CLK from 24.576 MCLK/BCLK */
static void spdif_cfg_hfclk(void)
{
/*
 * The SPDIF HFCLK is derived from the TDM8 clock used
 * for the 1761 CODEC coming from PCG0_CLKA_O
 *
 */
}

/* PCGB generates 3.072 MHz I2S BCLK from 24.576 MCLK/BCLK */
static void spdif_cfg_bclk(void)
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
 * The SPDIF HFCLK is derived from PCG0_CLKA_O
 * The SPDIF BCLK is derived from PCG0_CLKB_O
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

static void spdif_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;

    /* SPORT2A: SPDIF data out */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    memcpy(sportCfg.dataBuffers, context->spdifAudioOut, sizeof(sportCfg.dataBuffers));
    context->spdifSportOutHandle = single_sport_init(
        SPORT2A, &sportCfg, spdifAudioOut,
        context->spdifAudioOut, &context->spdifAudioOutLen, context, false, NULL
    );
    if (context->spdifSportOutHandle) {
        sportResult = sport_start(context->spdifSportOutHandle, true);
    }

    /* SPORT2B: SPDIF data in */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    memcpy(sportCfg.dataBuffers, context->spdifAudioIn, sizeof(sportCfg.dataBuffers));
    context->spdifSportInHandle = single_sport_init(
        SPORT2B, &sportCfg, spdifAudioIn,
        context->spdifAudioIn, &context->spdifAudioInLen, context, true, NULL
    );
    if (context->spdifSportInHandle) {
        sportResult = sport_start(context->spdifSportInHandle, true);
    }
}

static void spdif_sport_deinit(APP_CONTEXT *context)
{
    if (context->spdifSportOutHandle) {
        single_sport_deinit(
            &context->spdifSportOutHandle, context->spdifAudioOut, &context->spdifAudioOutLen
        );
        clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_SPDIF_OUT);
    }
    if (context->spdifSportInHandle) {
        single_sport_deinit(
            &context->spdifSportInHandle, context->spdifAudioIn, &context->spdifAudioInLen
        );
        clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_SPDIF_IN);
    }
}

static void spdif_asrc_init(void)
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

    /* Initialize the SPDIF BCLK and HFCLK PCGs */
    spdif_cfg_bclk();
    spdif_cfg_hfclk();

    /* Initialize the SPDIF and ASRC modules */
    spdif_asrc_init();

    /* Initialize the SPORTs */
    spdif_sport_init(context);
}

/***********************************************************************
 * A2B Generic functions
 **********************************************************************/
#include "a2b_irq.h"
#include "a2b_to_sport_cfg.h"

/***********************************************************************
 * A2B (Main/Sub) / SPORT1 / SRU / IRQ initialization
 **********************************************************************/
void a2b_pint_init(APP_CONTEXT *context)
{
    ADI_GPIO_RESULT result;
    uint32_t pint5pins = 0x00000000;

    /* Don't wire in interrupts if no A2B */
    if (!context->a2bPresent) {
        return;
    }

    /*
     * This code must match the values in a2b_irq.h
     * WARNING: Ethernet PHY interrupt uses PINT4
     */
    assert(A2B1_PINT_IRQ == ADI_GPIO_PIN_INTERRUPT_5);
    assert(A2B1_PINT_PIN == ADI_GPIO_INT_PIN_7);

    /*
     * A2B: Port F.7 -> PINT5 Byte 0.7
     */
    pint5pins |= ADI_GPIO_INT_PIN_7;

    /* Map Pins PF.0-PF.7 to PINT5 Byte 0 */
    result = adi_gpio_PinInterruptAssignment(
        ADI_GPIO_PIN_INTERRUPT_5,
        ADI_GPIO_PIN_ASSIGN_BYTE_0,
        ADI_GPIO_PIN_ASSIGN_PFL_PINT5
    );
    result = adi_gpio_SetPinIntEdgeSense(
        ADI_GPIO_PIN_INTERRUPT_5,
        pint5pins,
        ADI_GPIO_SENSE_RISING_EDGE
    );
    result = adi_gpio_RegisterCallback(
        ADI_GPIO_PIN_INTERRUPT_5,
        pint5pins,
        a2b_irq,
        context
    );
    result = adi_gpio_EnablePinInterruptMask(
        ADI_GPIO_PIN_INTERRUPT_5,
        pint5pins,
        true
    );
}

/**
 *
 * A2B Master Mode Configuration:
 *    - MCLK/BCLK to SPORT1B/A2B Transceiver
 *    - SPORT1A FS to SPORT1B/A2B Transceiver
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
    SRU(DAI0_PB06_O, DAI0_PB07_I);     // route MCLK/BCLK to A2B
    SRU(DAI0_PB06_O, SPT1_ACLK_I);     // route MCLK/BCLK to SPORT1A
    SRU(DAI0_PB06_O, SPT1_BCLK_I);     // route MCLK/BCLK to SPORT1B

    // SPORT1A FS to SPORT1B FS */
    SRU(SPT1_AFS_O, DAI0_PB08_I);    // route SPORT1A FS to A2B
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

#define AD242X_PRODUCT             0x03u

A2B_TO_SPORT_CFG_XCVR a2b_product(sTWI *twi, uint8_t twiAddr)
{
    TWI_SIMPLE_RESULT result;
    A2B_TO_SPORT_CFG_XCVR xcvr;
    uint8_t wBuf[1];
    uint8_t rBuf[1];

    wBuf[0] = AD242X_PRODUCT;

    result = twi_writeRead(twi, twiAddr, wBuf, sizeof(wBuf),
        rBuf, sizeof(rBuf));

    if (result != TWI_SIMPLE_SUCCESS) {
        return(A2B_TO_SPORT_CFG_XCVR_UNKNOWN);
    }

    switch (rBuf[0] & 0xF0) {
        case 0x10:
            xcvr = A2B_TO_SPORT_CFG_XCVR_AD241x;
            break;
        case 0x20:
            xcvr = A2B_TO_SPORT_CFG_XCVR_AD242x;
            break;
        case 0x30:
            xcvr = A2B_TO_SPORT_CFG_XCVR_AD243x;
            break;
        default:
            xcvr = A2B_TO_SPORT_CFG_XCVR_UNKNOWN;
            break;
    }

    return(xcvr);
}

#define AD242X_CONTROL             0x12u
#define AD242X_CONTROL_SOFTRST     0x04u
#define AD242X_CONTROL_MSTR        0x80u

/* Soft reset a single transceiver */
bool a2b_restart(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT result;
    uint8_t wBuf[2];

    wBuf[0] = AD242X_CONTROL;
    wBuf[1] = AD242X_CONTROL_SOFTRST;
    if (context->a2bmode != A2B_BUS_MODE_SUB) {
        wBuf[1] |= AD242X_CONTROL_MSTR;
    }

    result = twi_write(context->a2bTwiHandle, context->cfg.a2bI2CAddr,
        wBuf, sizeof(wBuf));

    delay(10);

    context->a2bxcvr = a2b_product(context->a2bTwiHandle, context->cfg.a2bI2CAddr);
    if (context->a2bxcvr == A2B_TO_SPORT_CFG_XCVR_UNKNOWN) {
        result = TWI_SIMPLE_ERROR;
    }

    return(result == TWI_SIMPLE_SUCCESS);
}

void a2b_reset(APP_CONTEXT *context)
{
    a2b_restart(context);
}

bool a2b_sport_init(APP_CONTEXT *context,
    bool master, CLOCK_DOMAIN clockDomain, uint8_t I2SGCFG, uint8_t I2SCFG,
    bool verbose)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    bool sportCfgOk;
    bool rxtx;

    /* Calculate the SPORT1 TX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = false;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG, &sportCfg, verbose, context->a2bxcvr);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;

    /* Configure SPORT1 Tx */
    if (master) {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    } else {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    }
    memcpy(sportCfg.dataBuffers, context->a2bAudioOut, sizeof(sportCfg.dataBuffers));
    context->a2bSportOutHandle = single_sport_init(
        SPORT1A, &sportCfg, a2bAudioOut,
        context->a2bAudioOut, &context->a2bAudioOutLen, context, false, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        context->a2bOutChannels = context->a2bAudioOutLen / (sportCfg.frames * (sportCfg.wordSize/8));
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_OUT);
        sportResult = sport_start(context->a2bSportOutHandle, true);
    } else {
        if (context->a2bSportOutHandle) {
            sport_close(&context->a2bSportOutHandle);
        }
        context->a2bOutChannels = 0;
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_OUT);
    }

    /* Calculate the SPORT0B RX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = true;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG, &sportCfg, verbose, context->a2bxcvr);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;

    /* Configure SPORT1 Rx */
    memcpy(sportCfg.dataBuffers, context->a2bAudioIn, sizeof(sportCfg.dataBuffers));
    context->a2bSportInHandle = single_sport_init(
        SPORT1B, &sportCfg, a2bAudioIn,
        context->a2bAudioIn, &context->a2bAudioInLen, context, true, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        context->a2bInChannels = context->a2bAudioInLen / (sportCfg.frames * (sportCfg.wordSize/8));
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_IN);
        sportResult = sport_start(context->a2bSportInHandle, true);
    } else {
        if (context->a2bSportInHandle) {
            sport_close(&context->a2bSportInHandle);
        }
        context->a2bInChannels = 0;
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_IN);
    }

abort:
    return(sportCfgOk);
}

bool a2b_sport_deinit(APP_CONTEXT *context)
{
    single_sport_deinit(
        &context->a2bSportOutHandle, context->a2bAudioOut, &context->a2bAudioOutLen
    );
    clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_A2B_OUT);
    single_sport_deinit(
        &context->a2bSportInHandle, context->a2bAudioIn, &context->a2bAudioInLen
    );
    clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_A2B_IN);
    return(true);
}

bool a2b_master_init(APP_CONTEXT *context)
{
    bool ok;

    /* Don't enable if no A2B */
    if (!context->a2bPresent) {
        return(false);
    }

    sru_config_a2b_master();

    ok = a2b_sport_init(context, true, CLOCK_DOMAIN_SYSTEM,
        SYSTEM_I2SGCFG, SYSTEM_I2SCFG, false);
    if (ok) {
        context->a2bmode = A2B_BUS_MODE_MAIN;
        context->a2bSlaveActive = false;
    }

    return(ok);
}

static void a2b_disconnect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE &= ~(
        BITP_PADS0_DAI0_IE_PB07 | BITP_PADS0_DAI0_IE_PB08
    );
}

static void a2b_connect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE |= (
        BITP_PADS0_DAI0_IE_PB07 | BITP_PADS0_DAI0_IE_PB08
    );
}

bool a2b_init_slave(APP_CONTEXT *context)
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

bool a2b_set_mode(APP_CONTEXT *context, A2B_BUS_MODE mode)
{
    if (mode == context->a2bmode) {
        return(true);
    }

    a2b_sport_deinit(context);

    if (mode == A2B_BUS_MODE_SUB) {
        a2b_init_slave(context);
    } else {
        adau1761_sport_deinit(context);
        spdif_sport_deinit(context);
        disable_mclk(context);
        adau1761_sport_init(context);
        spdif_sport_init(context);
        a2b_master_init(context);
        enable_mclk(context);
    }

    a2b_restart(context);

    return(true);
}

bool a2b_sport_start(APP_CONTEXT *context, uint8_t I2SGCFG, uint8_t I2SCFG)
{
    bool ok;
    ok = a2b_sport_init(context, false, CLOCK_DOMAIN_A2B,
        I2SGCFG, I2SCFG, true);
    a2b_connect_slave_clocks();
    return(ok);
}

bool a2b_sport_stop(APP_CONTEXT *context)
{
    bool ok;
    ok = a2b_sport_deinit(context);
    a2b_disconnect_slave_clocks();
    return(ok);
}

/**********************************************************************
 * Ethernet initialization
 **********************************************************************/
/* This function resets the Ethernet phy */
static void eth_emac0_init(APP_CONTEXT *context)
{
    /* Reset PHY */
    gpio_set_pin(gpioPins, GPIO_PIN_ETH0_RESET, 0);
    delay(10);
    gpio_set_pin(gpioPins, GPIO_PIN_ETH0_RESET, 1);
    delay(10);

    /* Select RGMII for EMAC0 */
    *pREG_PADS0_PCFG0 &= ~(BITM_PADS_PCFG0_EMACPHYISEL);
    *pREG_PADS0_PCFG0 |= (1 << BITP_PADS_PCFG0_EMACPHYISEL);

    /* Bring EMAC0 out of reset */
    *pREG_PADS0_PCFG0 |= BITM_PADS_PCFG0_EMACRESET;

    /* SPU Settings for EMAC0 */
    if (*pREG_SPU0_SECURECHK == 0xFFFFFFFF) {
        *pREG_SPU0_SECUREP55 = 0x03;
    }
}

void eth_hardware_init(APP_CONTEXT *context)
{
    ether_clk_25_mhz(context);
    eth_emac0_init(context);
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
 * buffers between the ARM and SHARC0 and SHARC1.
 *
 * The audio data payload can be referenced and used locally on the ARM
 * through context->xxxAudioIn/Out[] ping/pong buffers.  The audio can
 * be sent/received via the IPC message buffers context->xxxMsgIn/Out[].
 *
 */
void sae_buffer_init(APP_CONTEXT *context)
{
    int i;

    /* Allocate and initialize audio IPC ping/pong message buffers */
    for (i = 0; i < 2; i++) {

        /* SHARC0 Audio In (SHARC0 -> ARM) */
        context->sharc0AudioInLen =
            SHARC0_AUDIO_IN_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->sharc0MsgIn[i] = allocateIpcAudioMsg(
            context, context->sharc0AudioInLen,
            IPC_STREAMID_SHARC0_IN, SHARC0_AUDIO_IN_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->sharc0AudioIn[i]
        );
        memset(context->sharc0AudioIn[i], 0, context->sharc0AudioInLen);

        /* SHARC0 Audio Out (ARM -> SHARC0 */
        context->sharc0AudioOutLen =
            SHARC0_AUDIO_OUT_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->sharc0MsgOut[i] = allocateIpcAudioMsg(
            context, context->sharc0AudioOutLen,
            IPC_STREAMID_SHARC0_OUT, SHARC0_AUDIO_OUT_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->sharc0AudioOut[i]
        );
        memset(context->sharc0AudioOut[i], 0, context->sharc0AudioOutLen);

        /* SHARC1 Audio In (SHARC0 -> ARM) */
        context->sharc1AudioInLen =
            SHARC1_AUDIO_IN_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->sharc1MsgIn[i] = allocateIpcAudioMsg(
            context, context->sharc1AudioInLen,
            IPC_STREAMID_SHARC1_IN, SHARC1_AUDIO_IN_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->sharc1AudioIn[i]
        );
        memset(context->sharc1AudioIn[i], 0, context->sharc1AudioInLen);

        /* SHARC1 Audio Out (ARM -> SHARC1 */
        context->sharc1AudioOutLen =
            SHARC1_AUDIO_OUT_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->sharc1MsgOut[i] = allocateIpcAudioMsg(
            context, context->sharc1AudioOutLen,
            IPC_STREAMID_SHARC1_OUT, SHARC1_AUDIO_OUT_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->sharc1AudioOut[i]
        );
        memset(context->sharc1AudioOut[i], 0, context->sharc1AudioOutLen);
    }
}

/*
 * audio_routing_init()
 */
void audio_routing_init(APP_CONTEXT *context)
{
    context->routingTable = calloc(MAX_AUDIO_ROUTES, sizeof(ROUTE_INFO));
}

/**********************************************************************
 * SAM HW probe
 **********************************************************************/
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

/**********************************************************************
 * System Reset
 **********************************************************************/
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
