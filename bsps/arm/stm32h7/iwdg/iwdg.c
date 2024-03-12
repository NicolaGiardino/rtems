/**
 * @file
 *
 * @brief Internal Watchdog (iwdg) interface Implementation
 *
 * @ingroup iwdgiface
 */

/*
 * Copyright (C) 2023 Nicola di Gruttola Giardino <nicoladgg@protonmail.com> <nicola.digruttolagiardino@teamdiana.it>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <bsp/iwdg.h>

#include <rtems/imfs.h>

#include <stdlib.h>
#include <string.h>

static void iwdg_iface_obtain(iwdg_iface *iface)
{
    rtems_recursive_mutex_lock(&iface->mutex);
}

static void iwdg_iface_release(iwdg_iface *iface)
{
    rtems_recursive_mutex_unlock(&iface->mutex);
}

static ssize_t iwdg_iface_read(
    rtems_libio_t *iop,
    void *buffer,
    size_t count)
{
    iwdg_iface *iface = IMFS_generic_get_context_by_iop(iop);
    int err = 0;

    iwdg_iface_obtain(iface);

    if (err == 0)
    {
        iwdg_iface_release(iface);
        return 0;
    }
    else
    {
        rtems_set_errno_and_return_minus_one(-err);
        iwdg_iface_release(iface);
    }
}

static ssize_t iwdg_iface_write(
    rtems_libio_t *iop,
    const void *buffer,
    size_t count)
{
    iwdg_iface *iface = IMFS_generic_get_context_by_iop(iop);
    int err = 0;

    iwdg_iface_obtain(iface);
    iwdg_iface_release(iface);

    if (err == 0)
    {
        return 0;
    }
    else
    {
        rtems_set_errno_and_return_minus_one(-err);
    }
}

static int iwdg_iface_ioctl(
    rtems_libio_t *iop,
    ioctl_command_t command,
    void *arg)
{
    iwdg_iface *iface = IMFS_generic_get_context_by_iop(iop);
    int err;
    uint32_t previous;

    iwdg_iface_obtain(iface);

    switch (command)
    {
    case IWDG_IFACE_OBTAIN:
        iwdg_iface_obtain(iface);
        err = 0;
        break;
    case IWDG_IFACE_RELEASE:
        iwdg_iface_release(iface);
        err = 0;
        break;
    case IWDG_IFACE_SET_PRESCALER:
        previous = iface->prescaler_value;
        iface->prescaler_value = *(uint16_t *)arg;
        err = (*iface->set_prescaler)(iface);
        break;
    case IWDG_IFACE_GET_PRESCALER:
        *(uint16_t *)arg = iface->prescaler_value;
        err = 0;
        break;
    case IWDG_IFACE_SET_WINDOW:
        previous = iface->window_value;
        iface->window_value = *(uint16_t *)arg;
        err = (*iface->set_window)(iface);
        break;
    case IWDG_IFACE_GET_WINDOW:
        *(uint16_t *)arg = iface->window_value;
        err = 0;
        break;
    case IWDG_IFACE_SET_RELOAD:
        previous = iface->reload_value;
        iface->reload_value = *(uint16_t *)arg;
        err = (*iface->set_reload)(iface);
        break;
    case IWDG_IFACE_GET_RELOAD:
        *(uint16_t *)arg = iface->reload_value;
        err = 0;
        break;
    case IWDG_IFACE_RELOAD:
        err = (*iface->reload)(iface);
        break;
    default:
        err = -EINVAL;
    }

    iwdg_iface_release(iface);

    if (err == 0)
    {
        return 0;
    }
    else
    {
        rtems_set_errno_and_return_minus_one(-err);
    }
}

static const rtems_filesystem_file_handlers_r iwdg_iface_handler = {
    .open_h = rtems_filesystem_default_open,
    .close_h = rtems_filesystem_default_close,
    .read_h = iwdg_iface_read,
    .write_h = iwdg_iface_write,
    .ioctl_h = iwdg_iface_ioctl,
    .lseek_h = rtems_filesystem_default_lseek,
    .fstat_h = IMFS_stat,
    .ftruncate_h = rtems_filesystem_default_ftruncate,
    .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
    .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
    .fcntl_h = rtems_filesystem_default_fcntl,
    .kqfilter_h = rtems_filesystem_default_kqfilter,
    .mmap_h = rtems_filesystem_default_mmap,
    .poll_h = rtems_filesystem_default_poll,
    .readv_h = rtems_filesystem_default_readv,
    .writev_h = rtems_filesystem_default_writev};

static void iwdg_iface_node_destroy(IMFS_jnode_t *node)
{
    iwdg_iface *iface;

    iface = IMFS_generic_get_context_by_node(node);
    (*iface->destroy)(iface);

    IMFS_node_destroy_default(node);
}

static const IMFS_node_control iwdg_iface_node_control = IMFS_GENERIC_INITIALIZER(
    &iwdg_iface_handler,
    IMFS_node_initialize_generic,
    iwdg_iface_node_destroy);

int iwdg_iface_register(
    iwdg_iface *iface,
    const char *iface_path)
{
    int rv;

    rv = IMFS_make_generic_node(
        iface_path,
        S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
        &iwdg_iface_node_control,
        iface);
    if (rv != 0)
    {
        (*iface->destroy)(iface);
    }

    return rv;
}

static int set_prescaler_default(
    iwdg_iface *iface)
{
    (void)iface;

    return -EIO;
}

static int set_window_default(
    iwdg_iface *iface)
{
    (void)iface;

    return -EIO;
}

static int set_reload_default(
    iwdg_iface *iface)
{
    (void)iface;

    return -EIO;
}

static int iwdg_iface_setup_default(
    iwdg_iface *iface)
{
    (void)iface;

    return -EIO;
}

static int iwdg_iface_reload_default(
    iwdg_iface *iface)
{
    (void)iface;

    return -EIO;
}

static int iwdg_iface_do_init(
    iwdg_iface *iface,
    void (*destroy)(iwdg_iface *iface))
{
    rtems_recursive_mutex_init(&iface->mutex, "iwdg iface");
    iface->set_prescaler = set_prescaler_default;
    iface->set_window = set_window_default;
    iface->set_reload = set_reload_default;
    iface->setup = iwdg_iface_setup_default;
    iface->reload = iwdg_iface_reload_default;
    iface->destroy = destroy;
    iface->ioctl = NULL;
    iface->prescaler_value = 0;
    iface->window_value = 0;
    iface->reload_value = 0;

    return 0;
}

void iwdg_iface_destroy(iwdg_iface *iface)
{
    rtems_recursive_mutex_destroy(&iface->mutex);
}

void iwdg_iface_destroy_and_free(iwdg_iface *iface)
{
    iwdg_iface_destroy(iface);
    free(iface);
}

int iwdg_iface_init(iwdg_iface *iface)
{
    memset(iface, 0, sizeof(*iface));

    return iwdg_iface_do_init(iface, iwdg_iface_destroy);
}

iwdg_iface *iwdg_iface_alloc_and_init(size_t size)
{
    iwdg_iface *iface = NULL;

    if (size >= sizeof(*iface))
    {
        iface = calloc(1, size);
        if (iface != NULL)
        {
            int rv;

            rv = iwdg_iface_do_init(iface, iwdg_iface_destroy_and_free);
            if (rv != 0)
            {
                return NULL;
            }
        }
    }

    return iface;
}
