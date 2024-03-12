/**
 * @file
 *
 * @ingroup arm_stm32h7
 *
 * @brief STM32H7 SPI bus initialization and API Support.
 *
 * Based on bsps/m68k/gen68360/spi/m360_softc_ptr->spi.c
 */

/*
 * Copyright (c) 2018 Pierre-Louis Garnier <garnie_a@epita.fr>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */
#include <bsp.h>
#include <errno.h>
#include <rtems/bspIo.h>
#include <rtems/error.h>
#include <dev/spi/spi.h>
#include <bsp/stm32h7_spi.h>
#include <stdio.h>
#include <stdlib.h>

// #define DEBUG
// #define TRACE

#define EVENT_TXEMPTY RTEMS_EVENT_0
#define EVENT_RXFULL RTEMS_EVENT_1

static void setpins(const uint32_t spi_number)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (spi_number == 1)
    {
        /* USER CODE BEGIN SPI1_MspInit 0 */

        /* USER CODE END SPI1_MspInit 0 */
        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            return;
        }

        /* SPI1 clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PD7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

/* Initialize the driver
 *
 * Returns: o = ok or error code
 */
rtems_status_code stm32h7_spi_init(
    spi_bus *bh,
    const uint8_t spi_number)
{
    stm32h7_spi_softc_t *softc_ptr = &(((stm32h7_spi_desc_t *)(bh))->softc);

    rtems_status_code rc = RTEMS_SUCCESSFUL;

#if defined(DEBUG)
    printk("stm32h7_spi_init called...\r\n");
#endif
    if (spi_number == 1)
    {
        setpins(1);
        softc_ptr->spi.Instance = SPI1;
        softc_ptr->spi.Init.Mode = SPI_MODE_MASTER;
        softc_ptr->spi.Init.Direction = SPI_DIRECTION_2LINES;
        softc_ptr->spi.Init.DataSize = SPI_DATASIZE_4BIT;
        softc_ptr->spi.Init.CLKPolarity = SPI_POLARITY_LOW;
        softc_ptr->spi.Init.CLKPhase = SPI_PHASE_1EDGE;
        softc_ptr->spi.Init.NSS = SPI_NSS_SOFT;
        softc_ptr->spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        softc_ptr->spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
        softc_ptr->spi.Init.TIMode = SPI_TIMODE_DISABLE;
        softc_ptr->spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        softc_ptr->spi.Init.CRCPolynomial = 0x0;
        softc_ptr->spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
        softc_ptr->spi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
        softc_ptr->spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
        softc_ptr->spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        softc_ptr->spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        softc_ptr->spi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
        softc_ptr->spi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
        softc_ptr->spi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
        softc_ptr->spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
        softc_ptr->spi.Init.IOSwap = SPI_IO_SWAP_DISABLE;
        if (HAL_SPI_Init(&softc_ptr->spi) != HAL_OK)
        {
            return -1;
        }
    }

    if (softc_ptr->activate_irq)
    {
        // Setup interrupt
        rc = rtems_interrupt_handler_install(
            softc_ptr->irq,
            NULL,
            RTEMS_INTERRUPT_UNIQUE,
            (rtems_interrupt_handler)stm32h7_spi_irq_handler,
            softc_ptr);
    }

#if defined(DEBUG)
    printk("stm32h7_spi_init done\r\n");
#endif

    if (rc == RTEMS_SUCCESSFUL)
    {
        softc_ptr->initialized = TRUE;
    }

    return rc;
}

static int stm32h7_spi_read_write_bytes(
    spi_bus *bh,                 /* bus specifier structure        */
    const spi_ioc_transfer *cmd, /* buffer to send                 */
    uint32_t len                 /* number of bytes to send        */
)
{
    stm32h7_spi_softc_t *softc_ptr = &(((stm32h7_spi_desc_t *)(bh))->softc);

    rtems_status_code sc;
    rtems_event_set received_events;

#if defined(TRACE)
    printk("stm32h7_spi_read_write_bytes called...\r\n");
#endif

    softc_ptr->task_id = rtems_task_self();

    if (HAL_SPI_TransmitReceive(&softc_ptr->spi, cmd->tx_buf, cmd->rx_buf, cmd->len, 1000) != HAL_OK)
    {
        return -1;
    }

#if defined(TRACE)
    printk("stm32h7_spi_read_write_bytes done\r\n");
#endif

    return len;
}

/*
 * Receive some bytes from SPI device
 *
 * Returns: number of bytes received or (negative) error code
 */
int stm32h7_spi_read_bytes(
    spi_bus *bh,        /* bus specifier structure        */
    unsigned char *buf, /* buffer to store bytes          */
    uint32_t len        /* number of bytes to receive     */
)
{
    // FIXME
#if defined(DEBUG)
    printk("stm32h7_spi_read_bytes called...\r\n");
#endif

    int n = HAL_SPI_Receive(&(((stm32h7_spi_desc_t *)(bh))->softc).spi, buf, len, 1000);

#if defined(DEBUG)
    printk("stm32h7_spi_read_bytes done\r\n");
#endif

    return n;
}

/*
 * Send some bytes to SPI device
 *
 * Returns: number of bytes sent or (negative) error code
 */
int stm32h7_spi_write_bytes(
    spi_bus *bh,        /* bus specifier structure        */
    unsigned char *buf, /* buffer to send                 */
    uint32_t len        /* number of bytes to send        */
)
{
#if defined(DEBUG)
    printk("stm32h7_spi_write_bytes called...\r\n");
#endif

    int n = HAL_SPI_Transmit(&(((stm32h7_spi_desc_t *)(bh))->softc).spi, buf, len, 1000);

#if defined(DEBUG)
    printk("stm32h7_spi_write_bytes done\r\n");
#endif

    return n;
}

/*
 * Interrupt handler
 *
 * Returns: rtems_status_code
 */
rtems_status_code stm32h7_spi_irq_handler(
    rtems_vector_number vector,
    void *arg)
{
    stm32h7_spi_softc_t *softc_ptr = (stm32h7_spi_softc_t *)arg;

#if defined(DEBUG)
    printk("stm32h7_spi_irq_handler called...\r\n");
#endif

    // FIXME
    return RTEMS_SUCCESSFUL;
}

/*
 * Perform selected ioctl function for SPI
 *
 * Returns: rtems_status_code
 */
int stm32h7_spi_ioctl(
    spi_bus *bh,         /* bus specifier structure        */
    ioctl_command_t cmd, /* ioctl command code             */
    void *arg            /* additional argument array      */
)
{
    int ret_val = -1;

    stm32h7_spi_softc_t *softc_ptr = &(((stm32h7_spi_desc_t *)(bh))->softc);
#if defined(DEBUG)
    printk("stm32h7_spi_ioctl called...\r\n");
#endif

    switch (cmd)
    {
    default:
    {
#if defined(TRACE)
        printk("cmd == RTEMS_SPI_IOCTL_READ_WRITE\r\n");
#endif
        softc_ptr->task_id = rtems_task_self();

        spi_ioc_transfer *cmd = (spi_ioc_transfer *)arg;

        if (HAL_SPI_TransmitReceive(&softc_ptr->spi, cmd->tx_buf, cmd->rx_buf, cmd->len, 1000) != HAL_OK)
        {
            return -1;
        }

        ret_val = cmd->len;

        break;
    }
    }

#if defined(DEBUG)
    printk("stm32h7_spi_ioctl done\r\n");
#endif

    return ret_val;
}

/*=========================================================================*\
| Board-specific adaptation functions                                       |
\*=========================================================================*/

/*
 * Destroy the driver
 *
 * Returns: o = ok or error code
 */
void stm32h7_spi_destroy(
    spi_bus *bh /* bus specifier structure        */
)
{
    stm32h7_spi_softc_t *softc_ptr = &(((stm32h7_spi_desc_t *)(bh))->softc);

    rtems_status_code rc = RTEMS_SUCCESSFUL;

#if defined(DEBUG)
    printk("stm32h7_spi_destroy called...\r\n");
#endif

    if (softc_ptr->activate_irq)
    {
        // Remove interrupt handler
        rc = rtems_interrupt_handler_remove(softc_ptr->irq, NULL, NULL);
    }

    spi_bus_destroy(bh);

#if defined(DEBUG)
    printk("stm32h7_spi_destroy done\r\n");
#endif
}

/*=========================================================================*\
| list of handlers                                                          |
\*=========================================================================*/

spi_bus bsp_spi_ops = {
    transfer : stm32h7_spi_read_write_bytes,
    ioctl : stm32h7_spi_ioctl,
    destroy : stm32h7_spi_destroy
};

static stm32h7_spi_desc_t bsp_spi_bus_desc = {
    bus_desc : &bsp_spi_ops,
    {
        initialized : FALSE,
    }
};

/*=========================================================================*\
| initialization                                                            |
\*=========================================================================*/

/*
 * Register SPI bus and devices
 *
 * Returns: Bus number or error code
 */
rtems_status_code bsp_register_spi(
    const char *bus_path,
    uint8_t activate_irq,
    rtems_vector_number irq)
{
    int ret_code;
    int spi_busno;

    stm32h7_spi_softc_t *softc_ptr = &bsp_spi_bus_desc.softc;

    if (softc_ptr->initialized)
    {
        printk("ERROR: Only one SPI bus at a time is supported\n");
        return -RTEMS_RESOURCE_IN_USE;
    }

    softc_ptr->activate_irq = activate_irq;
    softc_ptr->irq = irq;

    /*
     * register SPI bus
     */
    ret_code = spi_bus_register((bsp_spi_bus_desc.bus_desc), bus_path);
    if (ret_code < 0)
    {
        return -ret_code;
    }
    spi_busno = ret_code;

#if IS_AM335X
    // TODO: register board devices
#endif

    return spi_busno;
}
