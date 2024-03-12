/**
 * @file
 *
 * @ingroup arm_stm32h7
 *
 * @brief STM32H7 CAN bus initialization and API Support.
 */

/*
 * Copyright (c) 2023 Nicola di Gruttola Giardino <nicoladgg@protonmail.com> <nicola.digruttolagiardino@teamdiana.it>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <rtems/bspIo.h>
#include <stdio.h>
#include <bsp/stm32h7_fdcan.h>
#include <rtems/irq-extension.h>
#include <rtems/score/assert.h>
#include <ofw/ofw.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <stdarg.h>

typedef struct stm32h7_fdcan_bus_
{
    can_bus base;
    rtems_id task_id;
    rtems_vector_number irq;
    fdcan_driver_entry fdcan;
    fdcan_acceptance_filter filters[FDCAN_MAX_FILTERS];
    can_msg *buffer;
    can_freq freq;
    int error;
} stm32h7_fdcan_bus;

#define TRANSFER_TIMEOUT_COUNT 100
#define FIFO_THRESHOLD 5
#define min(l, r) ((l) < (r) ? (l) : (r))
#if 0
#define debug_print(fmt, args...) printk("stm32h7-fdcan: " fmt, ##args)
#else
#define debug_print(fmt, args...)
#endif
/*
 * Here we assume the number of i2c nodes
 * will be less than 100.
 */
#define PATH_LEN (strlen("/dev/fdcan-xx") + 1)

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

static inline void setpins(const uint32_t can_number)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (can_number == FDCAN1_periph)
    {
        /* FDCAN1 clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
        {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**FDCAN1 GPIO Configuration
        PD0     ------> FDCAN1_RX
        PD1     ------> FDCAN1_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
    else if (can_number == FDCAN2_periph)
    {
        /* FDCAN2 clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
        {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**FDCAN2 GPIO Configuration
        PB5     ------> FDCAN2_RX
        PB6     ------> FDCAN2_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

static rtems_status_code create_can_message(
    fdcan_tx_message *msg,
    const can_msg message)
{
    rtems_status_code sc = RTEMS_INVALID_NUMBER;

    if (message.dlc <= 8)
    {
        msg->header.Identifier = message.id;
        if (message.ide == 1)
        {
            msg->header.IdType = FDCAN_EXTENDED_ID;
        }
        else
        {
            msg->header.IdType = FDCAN_STANDARD_ID;
        }
        if (message.rtr == 1)
        {
            msg->header.TxFrameType = FDCAN_REMOTE_FRAME;
        }
        else
        {
            msg->header.TxFrameType = FDCAN_DATA_FRAME;
        }
        msg->header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        msg->header.BitRateSwitch = FDCAN_BRS_OFF;
        msg->header.FDFormat = FDCAN_CLASSIC_CAN;
        msg->header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        msg->header.MessageMarker = 0;
        msg->header.DataLength = message.dlc << 16;
        memcpy(msg->data, message.data, message.dlc);
        sc = RTEMS_SUCCESSFUL;
    }

    return sc;
}

static rtems_status_code read_can_message(
    const fdcan_rx_message msg,
    can_msg *message)
{
    rtems_status_code sc = RTEMS_INVALID_NUMBER;

    message->id = msg.header.Identifier;
    if (msg.header.IdType == FDCAN_EXTENDED_ID)
    {
        message->ide = 1;
    }
    else
    {
        message->ide = 0;
    }
    if (msg.header.RxFrameType == FDCAN_REMOTE_FRAME)
    {
        message->rtr = 1;
    }
    else
    {
        message->rtr = 0;
    }
    message->dlc = msg.header.DataLength >> 16;
    memcpy(message->data, msg.data, message->dlc);
    sc = RTEMS_SUCCESSFUL;

    return sc;
}

static int stm32h7_fdcan_set_freq(can_bus *base, can_freq freq)
{
    stm32h7_fdcan_bus *bus = (stm32h7_fdcan_bus *)base;
    rtems_status_code sc = RTEMS_INVALID_NUMBER;
    bus->base.freq.prescaler = freq.prescaler;
    bus->base.freq.sjw = freq.sjw;
    bus->base.freq.ts1 = freq.ts1;
    bus->base.freq.ts2 = freq.ts2;
    bus->fdcan.Init.NominalPrescaler = freq.prescaler;
    bus->fdcan.Init.NominalSyncJumpWidth = freq.sjw;
    bus->fdcan.Init.NominalTimeSeg1 = freq.ts1;
    bus->fdcan.Init.NominalTimeSeg2 = freq.ts2;
    HAL_FDCAN_Stop(&bus->fdcan);
    if (HAL_FDCAN_Init(&bus->fdcan) != HAL_OK)
    {
        printf("FDCAN init failed\n");
        sc = RTEMS_IO_ERROR;
    }
    else
    {
        if (HAL_FDCAN_Start(&bus->fdcan) != HAL_OK)
        {
            printf("FDCAN init failed\n");
            sc = RTEMS_IO_ERROR;
        }
        printf("FDCAN initialized\n");
        sc = RTEMS_SUCCESSFUL;
    }
    return 0;
}

static int stm32h7_fdcan_reset(can_bus *bus)
{

    int err;

    err = stm32h7_fdcan_set_freq(bus, bus->freq);
    if (err)
    {
        return err;
    }

    return 0;
}

/* Return true if done. */
static bool stm32h7_fdcan_read_intr(stm32h7_fdcan_bus *bus)
{

    return false;
}

static int stm32h7_fdcan_init(can_bus *base, can_freq freq, int busno)
{
    stm32h7_fdcan_bus *bus = (stm32h7_fdcan_bus *)base;
    rtems_status_code sc = RTEMS_INVALID_NUMBER;
    printf("stm32h7_fdcan_init %d\n", busno);
    if (busno < FDCAN_COUNT)
    {
        setpins(busno);
        bus->fdcan.Instance = FDCAN1;
        bus->fdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        bus->fdcan.Init.Mode = FDCAN_MODE_NORMAL;
        bus->fdcan.Init.AutoRetransmission = ENABLE;
        bus->fdcan.Init.TransmitPause = DISABLE;
        bus->fdcan.Init.ProtocolException = DISABLE;
        bus->fdcan.Init.NominalPrescaler = 2;
        bus->fdcan.Init.NominalSyncJumpWidth = 4;
        bus->fdcan.Init.NominalTimeSeg1 = 34;
        bus->fdcan.Init.NominalTimeSeg2 = 5;
        bus->fdcan.Init.DataPrescaler = 1;
        bus->fdcan.Init.DataSyncJumpWidth = 1;
        bus->fdcan.Init.DataTimeSeg1 = 1;
        bus->fdcan.Init.DataTimeSeg2 = 1;
        bus->fdcan.Init.MessageRAMOffset = 0;
        bus->fdcan.Init.StdFiltersNbr = 10;
        bus->fdcan.Init.ExtFiltersNbr = 10;
        bus->fdcan.Init.RxFifo0ElmtsNbr = 20;
        bus->fdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
        bus->fdcan.Init.RxFifo1ElmtsNbr = 20;
        bus->fdcan.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
        bus->fdcan.Init.RxBuffersNbr = 0;
        bus->fdcan.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
        bus->fdcan.Init.TxEventsNbr = 0;
        bus->fdcan.Init.TxBuffersNbr = 0;
        bus->fdcan.Init.TxFifoQueueElmtsNbr = 20;
        bus->fdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
        bus->fdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
        if (busno == FDCAN2_periph)
        {
            bus->fdcan.Init.MessageRAMOffset = 1280;
            bus->fdcan.Instance = FDCAN2;
        }
        if (HAL_FDCAN_Init(&bus->fdcan) != HAL_OK)
        {
            printf("FDCAN init failed\n");
            sc = RTEMS_IO_ERROR;
        }
        else
        {
            if (HAL_FDCAN_Start(&bus->fdcan) != HAL_OK)
            {
                printf("FDCAN init failed\n");
                sc = RTEMS_IO_ERROR;
            }
            printf("FDCAN initialized\n");
            sc = RTEMS_SUCCESSFUL;
        }
    }

    return sc;
}

static void stm32h7_fdcan_interrupt(void *arg)
{
    stm32h7_fdcan_bus *bus = arg;

    if (bus->buffer == NULL)
    {
        debug_print("Buffer is NULL\n");
        bus->error = EINVAL;
    }

    if (bus->buffer == NULL || stm32h7_fdcan_read_intr(bus))
    {
        rtems_status_code sc;
        sc = rtems_event_transient_send(bus->task_id);
        _Assert(sc == RTEMS_SUCCESSFUL);
        (void)sc; /* suppress warning in case of no assert */
    }
}

static int stm32h7_fdcan_read(
    can_bus *base,
    can_msg *msg,
    can_fifo fifo)
{
    stm32h7_fdcan_bus *bus = (stm32h7_fdcan_bus *)base;
    rtems_status_code sc;
    fdcan_rx_message rx_message;

    sc = RTEMS_IO_ERROR;

    bus->task_id = rtems_task_self();

    if (HAL_FDCAN_GetRxFifoFillLevel(&bus->fdcan, fifo) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(&bus->fdcan, fifo, &rx_message.header, rx_message.data) == HAL_OK)
        {
            sc = RTEMS_SUCCESSFUL;
        }
        read_can_message(rx_message, msg);
    }
    return sc;
}

static int stm32h7_fdcan_write(
    can_bus *base,
    can_msg *msg)
{
    stm32h7_fdcan_bus *bus = (stm32h7_fdcan_bus *)base;
    rtems_status_code sc;
    fdcan_tx_message tx_message;
    int i;

    sc = RTEMS_IO_ERROR;

    bus->task_id = rtems_task_self();

    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef ec;
    create_can_message(&tx_message, *msg);
    i = HAL_FDCAN_AddMessageToTxFifoQ(&bus->fdcan, &tx_message.header, tx_message.data);
    if (i == HAL_OK)
    {
        if (!HAL_FDCAN_GetTxFifoFreeLevel(&bus->fdcan))
        {
            HAL_FDCAN_Stop(&bus->fdcan);
            HAL_FDCAN_Start(&bus->fdcan);
        }
        sc = RTEMS_SUCCESSFUL;
    }

    return sc;
}

static int stm32h7_fdcan_add_filter(can_bus *base, can_filter filter)
{
    rtems_status_code sc = RTEMS_IO_ERROR;
    stm32h7_fdcan_bus *bus = (stm32h7_fdcan_bus *)base;

    if (filter.filter_index >= FDCAN_MAX_FILTERS)
    {
        return sc;
    }

    if (filter.is_extended == 1)
    {
        bus->filters[filter.filter_index].IdType = FDCAN_EXTENDED_ID;
    }
    else
    {
        bus->filters[filter.filter_index].IdType = FDCAN_STANDARD_ID;
    }
    bus->filters[filter.filter_index].FilterIndex = filter.filter_index;
    bus->filters[filter.filter_index].FilterType = filter.filter_type;
    if (filter.filter_fifo == 0)
    {
        bus->filters[filter.filter_index].FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    }
    else
    {
        bus->filters[filter.filter_index].FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    }
    bus->filters[filter.filter_index].FilterID1 = filter.filter_id1;
    bus->filters[filter.filter_index].FilterID2 = filter.filter_id2;
    if (HAL_FDCAN_ConfigFilter(&bus->fdcan, &bus->filters[filter.filter_index]) == HAL_OK)
    {
        sc = RTEMS_SUCCESSFUL;
    }

    HAL_FDCAN_ConfigGlobalFilter(&bus->fdcan, FDCAN_REJECT /* 11bit frame */, FDCAN_REJECT /* 29bit frame */, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    return sc;
}

static void stm32h7_fdcan_destroy(can_bus *base)
{
    stm32h7_fdcan_bus *bus = (stm32h7_fdcan_bus *)base;
    if (bus->fdcan.Instance == FDCAN1)
    {
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
        {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN1 GPIO Configuration
        PD0     ------> FDCAN1_RX
        PD1     ------> FDCAN1_TX
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
    }
    else if (bus->fdcan.Instance == FDCAN2)
    {
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
        {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN2 GPIO Configuration
        PB5     ------> FDCAN2_RX
        PB6     ------> FDCAN2_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);
    }

#if FDCAN_IRQ_ENABLE
    sc = rtems_interrupt_handler_remove(bus->irq, stm32h7_fdcan_interrupt, bus);
    _Assert(sc == RTEMS_SUCCESSFUL);
    (void)sc;
#endif
    can_bus_destroy_and_free(&bus->base);
}

int stm32h7_fdcan_bus_register(
    uint8_t busno,
    can_freq freq,
    const char *bus_path)
{
    stm32h7_fdcan_bus *bus;
    int err;

    bus = (stm32h7_fdcan_bus *)can_bus_alloc_and_init(sizeof(*bus));

    if (bus == NULL)
    {
        return -1;
    }

    err = stm32h7_fdcan_init(&bus->base, freq, busno);
    if (err != 0)
    {
        printf("can: invalid register base\n");
        (*bus->base.destroy)(&bus->base);
        rtems_set_errno_and_return_minus_one(err);
    }

#if FDCAN_IRQ_ENABLE
    if (bus->fdcan.Instance == FDCAN1)
    {
        bus->irq = FDCAN1_IT0_IRQn;
    }
    else
    {
        bus->irq = FDCAN2_IT0_IRQn;
    }
    sc = rtems_interrupt_handler_install(
        bus->irq,
        "STM32_FDCAN",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler)stm32h7_fdcan_interrupt,
        bus);

    if (sc != RTEMS_SUCCESSFUL)
    {
        (*bus->base.destroy)(&bus->base);
        rtems_set_errno_and_return_minus_one(EIO);
    }
#endif

    bus->base.can_write = stm32h7_fdcan_write;
    bus->base.can_read = stm32h7_fdcan_read;
    bus->base.set_freq = stm32h7_fdcan_set_freq;
    bus->base.add_filter = stm32h7_fdcan_add_filter;
    bus->base.destroy = stm32h7_fdcan_destroy;

    return can_bus_register(&bus->base, bus_path);
}
