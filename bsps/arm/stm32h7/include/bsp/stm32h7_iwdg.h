/**
 * @file
 *
 * @ingroup arm_stm32h7
 *
 * @brief IWDG support API.
 */

/*
 * Copyright (c) 2023 Nicola di Gruttola Giardino <nicoladgg@protonmail.com> <nicola.digruttolagiardino@teamdiana.it>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef STM32H7_IWDG_H
#define STM32H7_IWDG_H

#include <rtems.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_iwdg.h>
#include <bsp.h>
#include <bsp/iwdg.h>
#include <ofw/ofw.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define STM32_IWDG_IFACE_PATH "/dev/iwdg-1"

    typedef IWDG_HandleTypeDef iwdg_iface_entry;

    int stm32h7_iwdg_iface_register(
        uint16_t prescaler_value,
        uint16_t window_value,
        uint16_t reload_value,
        const char *iface_path);
        

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_I2C_H */
