/**
 * @file
 *
 * @ingroup arm_stm32h7
 *
 * @brief CANBus support API.
 */

/*
 * Copyright (c) 2023 Nicola di Gruttola Giardino <nicoladgg@protonmail.com> <nicola.digruttolagiardino@teamdiana.it>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef STM32H7_FDCAN_H
#define STM32H7_FDCAN_H

#include <rtems.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_fdcan.h>
#include <bsp.h>
#include <bsp/can-drv.h>
#include <ofw/ofw.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define STM32_FDCAN_0_BUS_PATH "/dev/fdcan-0"
#define STM32_FDCAN_1_BUS_PATH "/dev/fdcan-1"

#define FDCAN_MAX_FILTERS 28

    typedef enum
    {
        FDCAN1_periph,
        FDCAN2_periph,
        FDCAN_COUNT
    } stm32h7_fdcan_id_t;

    typedef FDCAN_HandleTypeDef fdcan_driver_entry;
    typedef FDCAN_FilterTypeDef fdcan_acceptance_filter;
    typedef uint32_t fdcan_rx_fifo;

    typedef struct
    {
        FDCAN_TxHeaderTypeDef header;
        uint8_t data[8];
    } fdcan_tx_message;

    typedef struct
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
    } fdcan_rx_message;

    int stm32h7_fdcan_bus_register(
        uint8_t busno,
        can_freq freq,
        const char *bus_path);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_I2C_H */
