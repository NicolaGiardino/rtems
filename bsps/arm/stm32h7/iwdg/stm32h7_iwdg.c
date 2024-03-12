/**
 * @file
 *
 * @ingroup arm_stm32h7
 *
 * @brief STM32H7 IWDG support API.
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
#include <bsp/stm32h7_iwdg.h>
#include <rtems/irq-extension.h>
#include <rtems/score/assert.h>
#include <ofw/ofw.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <stdarg.h>

typedef struct stm32h7_iwdg_iface_t
{
    iwdg_iface base;
    rtems_id task_id;
    rtems_vector_number irq;
    iwdg_iface_entry hiwdg;
    int error;
} stm32h7_iwdg_iface;

#define min(l, r) ((l) < (r) ? (l) : (r))
#if 0
#define debug_print(fmt, args...) printk("stm32h7-iwdg: " fmt, ##args)
#else
#define debug_print(fmt, args...)
#endif

#define PATH_LEN (strlen("/dev/iwdg-1") + 1)

static int stm32h7_iwdg_init(iwdg_iface *base)
{
    stm32h7_iwdg_iface *iface = (stm32h7_iwdg_iface *)base;
    rtems_status_code sc = RTEMS_INVALID_NUMBER;
    printf("iwdg_init\n");
    iface->hiwdg.Instance = IWDG1;
    switch(iface->base.prescaler_value)
    {
        case 4:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
            break;
        case 8:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
            break;
        case 16:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
            break;
        case 32:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
            break;
        case 64:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
            break;
        case 128:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
            break;
        case 256:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
            break;
        default:
            return -1;
    }
    iface->hiwdg.Init.Window = iface->base.window_value;
    iface->hiwdg.Init.Reload = iface->base.reload_value;
    if (HAL_IWDG_Init(&iface->hiwdg) != HAL_OK)
    {
        sc = RTEMS_INVALID_NUMBER;
    }
    else
    {
        sc = RTEMS_SUCCESSFUL;
    }
    return sc;
}

static int stm32h7_iwdg_set_prescaler(
    iwdg_iface *base)
{
    stm32h7_iwdg_iface *iface = (stm32h7_iwdg_iface *)base;
    rtems_status_code sc = RTEMS_INVALID_NUMBER;
    iface->task_id = rtems_task_self();
    printf("iwdg_set_prescaler %d\n", iface->base.prescaler_value);
    switch(iface->base.prescaler_value)
    {
        case 4:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
            break;
        case 8:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
            break;
        case 16:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
            break;
        case 32:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
            break;
        case 64:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
            break;
        case 128:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
            break;
        case 256:
            iface->hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
            break;
        default:
            return -1;
    }
    if (HAL_IWDG_Init(&iface->hiwdg) != HAL_OK)
    {
        sc = RTEMS_INVALID_NUMBER;
    }
    else
    {
        sc = RTEMS_SUCCESSFUL;
    }
    return sc;
}

static int stm32h7_iwdg_set_window(
    iwdg_iface *base)
{
    stm32h7_iwdg_iface *iface = (stm32h7_iwdg_iface *)base;
    rtems_status_code sc = RTEMS_INVALID_NUMBER;
    iface->task_id = rtems_task_self();
    printf("iwdg_set_window %d\n", iface->base.window_value);
    iface->hiwdg.Init.Window = iface->base.window_value;
    if (HAL_IWDG_Init(&iface->hiwdg) != HAL_OK)
    {
        sc = RTEMS_INVALID_NUMBER;
    }
    else
    {
        sc = RTEMS_SUCCESSFUL;
    }
    return sc;
}

static int stm32h7_iwdg_set_reload(
    iwdg_iface *base)
{
    stm32h7_iwdg_iface *iface = (stm32h7_iwdg_iface *)base;
    rtems_status_code sc = RTEMS_INVALID_NUMBER;
    iface->task_id = rtems_task_self();
    printf("iwdg_set_reload %d\n", iface->base.reload_value);
    iface->hiwdg.Init.Reload = iface->base.reload_value;
    if (HAL_IWDG_Init(&iface->hiwdg) != HAL_OK)
    {
        sc = RTEMS_INVALID_NUMBER;
    }
    else
    {
        sc = RTEMS_SUCCESSFUL;
    }
    return sc;
}

static int stm32h7_iwdg_reload(iwdg_iface *base)
{
    stm32h7_iwdg_iface *iface = (stm32h7_iwdg_iface *)base;
    rtems_status_code sc = RTEMS_INVALID_NUMBER;
    iface->task_id = rtems_task_self();
    if (HAL_IWDG_Refresh(&iface->hiwdg) != HAL_OK)
    {
        sc = RTEMS_INVALID_NUMBER;
    }
    else
    {
        sc = RTEMS_SUCCESSFUL;
    }
    return sc;
}

static void stm32h7_iwdg_destroy(iwdg_iface *base)
{
    stm32h7_iwdg_iface *iface = (stm32h7_iwdg_iface *)base;
    printf("iwdg_destroy is not possible\n");
    iwdg_iface_destroy_and_free(&iface->base);
}

int stm32h7_iwdg_iface_register(
    uint16_t prescaler_value,
    uint16_t window_value,
    uint16_t reload_value,
    const char *iface_path)
{
    stm32h7_iwdg_iface *iface;
    int err;

    iface = (stm32h7_iwdg_iface *)iwdg_iface_alloc_and_init(sizeof(*iface));

    if (iface == NULL)
    {
        return -1;
    }

    iface->base.prescaler_value = prescaler_value;
    iface->base.window_value = window_value;
    iface->base.reload_value = reload_value;
    err = stm32h7_iwdg_init(&iface->base);
    if (err != 0)
    {
        printf("iwdg: invalid register base\n");
        (*iface->base.destroy)(&iface->base);
        rtems_set_errno_and_return_minus_one(err);
    }

    iface->base.set_prescaler = stm32h7_iwdg_set_prescaler;
    iface->base.set_window = stm32h7_iwdg_set_window;
    iface->base.set_reload = stm32h7_iwdg_set_reload;
    iface->base.setup = stm32h7_iwdg_init;
    iface->base.reload = stm32h7_iwdg_reload;
    iface->base.destroy = stm32h7_iwdg_destroy;

    return iwdg_iface_register(&iface->base, iface_path);
}
