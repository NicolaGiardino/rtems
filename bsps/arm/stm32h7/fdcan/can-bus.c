/**
 * @file
 *
 * @brief Controller Area Network (CAN) Bus Implementation
 *
 * @ingroup CANBus
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

#include <bsp/can-drv.h>

#include <rtems/imfs.h>

#include <stdlib.h>
#include <string.h>

int can_bus_try_obtain(can_bus *bus)
{
    return rtems_recursive_mutex_try_lock(&bus->mutex);
}

void can_bus_obtain(can_bus *bus)
{
    rtems_recursive_mutex_lock(&bus->mutex);
}

void can_bus_release(can_bus *bus)
{
    rtems_recursive_mutex_unlock(&bus->mutex);
}

int can_bus_do_write(can_bus *bus, can_msg *msg, uint32_t flags)
{
    int err;

    if ((flags & CAN_BUS_NOBLOCK) != 0)
    {
        if (can_bus_try_obtain(bus) != 0)
        {
            return -EAGAIN;
        }
    }
    else
    {
        can_bus_obtain(bus);
    }
    err = (*bus->can_write)(bus, msg);
    can_bus_release(bus);

    return err;
}

int can_bus_do_read(can_bus *bus, can_msg *msg, can_fifo fifo, uint32_t flags)
{
    int err;

    if ((flags & CAN_BUS_NOBLOCK) != 0)
    {
        if (can_bus_try_obtain(bus) != 0)
        {
            return -EAGAIN;
        }
    }
    else
    {
        can_bus_obtain(bus);
    }
    err = (*bus->can_read)(bus, msg, fifo);
    can_bus_release(bus);

    return err;
}

static ssize_t can_bus_read(
    rtems_libio_t *iop,
    void *buffer,
    size_t count)
{
    can_bus *bus = IMFS_generic_get_context_by_iop(iop);
    can_msg *msg;
    msg = buffer;
    can_fifo *fifo;
    fifo = (buffer + sizeof(can_msg));
    int err;
    unsigned flags = 0;

    if (rtems_libio_iop_is_no_delay(iop))
    {
        flags |= CAN_BUS_NOBLOCK;
    }

    err = can_bus_do_read(bus, msg, *fifo, flags);
    if (err == 0)
    {
        return -1;
    }
    else
    {
        rtems_set_errno_and_return_minus_one(-err);
    }
}

static ssize_t can_bus_write(
    rtems_libio_t *iop,
    const void *buffer,
    size_t count)
{
    can_bus *bus = IMFS_generic_get_context_by_iop(iop);
    can_msg *msg;
    msg = RTEMS_DECONST(void *, buffer);
    int err;
    unsigned flags = 0;

    if (rtems_libio_iop_is_no_delay(iop))
    {
        flags |= CAN_BUS_NOBLOCK;
    }

    err = can_bus_do_write(bus, msg, flags);
    if (err == 0)
    {
        return -1;
    }
    else
    {
        rtems_set_errno_and_return_minus_one(-err);
    }
}

static int can_bus_ioctl(
    rtems_libio_t *iop,
    ioctl_command_t command,
    void *arg)
{
    can_bus *bus = IMFS_generic_get_context_by_iop(iop);
    can_rdwr_ioctl_data *rdwr;
    can_msg *msg;
    can_freq *freq;
    int err;
    unsigned flags = 0;

    switch (command)
    {
    case CAN_BUS_WR:
        msg = arg;
        if (rtems_libio_iop_is_no_delay(iop))
        {
            flags |= CAN_BUS_NOBLOCK;
        }
        err = can_bus_do_write(bus, msg, flags);
        break;
    case CAN_BUS_RD:
        rdwr = arg;
        if (rtems_libio_iop_is_no_delay(iop))
        {
            flags |= CAN_BUS_NOBLOCK;
        }
        err = can_bus_do_read(bus, &rdwr->msg, rdwr->fifo, flags);
        break;
    case CAN_BUS_OBTAIN:
        can_bus_obtain(bus);
        err = 0;
        break;
    case CAN_BUS_RELEASE:
        can_bus_release(bus);
        err = 0;
        break;
    case CAN_BUS_GET_CONTROL:
        *(can_bus **)arg = bus;
        err = 0;
        break;
    case CAN_BUS_SET_FREQ:
        freq = arg;
        can_bus_obtain(bus);
        err = (*bus->set_freq)(bus, *freq);
        can_bus_release(bus);
        break;
    case CAN_BUS_ADD_FILTER:
        can_filter *filter;
        filter = arg;
        can_bus_obtain(bus);
        err = (*bus->add_filter)(bus, *filter);
        can_bus_release(bus);
        break;
    default:
        err = -ENOTTY;
        break;
    }

    if (err == 0)
    {
        return 0;
    }
    else
    {
        rtems_set_errno_and_return_minus_one(-err);
    }
}

static const rtems_filesystem_file_handlers_r can_bus_handler = {
    .open_h = rtems_filesystem_default_open,
    .close_h = rtems_filesystem_default_close,
    .read_h = can_bus_read,
    .write_h = can_bus_write,
    .ioctl_h = can_bus_ioctl,
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

static void can_bus_node_destroy(IMFS_jnode_t *node)
{
    can_bus *bus;

    bus = IMFS_generic_get_context_by_node(node);
    (*bus->destroy)(bus);

    IMFS_node_destroy_default(node);
}

static const IMFS_node_control can_bus_node_control = IMFS_GENERIC_INITIALIZER(
    &can_bus_handler,
    IMFS_node_initialize_generic,
    can_bus_node_destroy);

int can_bus_register(
    can_bus *bus,
    const char *bus_path)
{
    int rv;

    rv = IMFS_make_generic_node(
        bus_path,
        S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
        &can_bus_node_control,
        bus);
    if (rv != 0)
    {
        (*bus->destroy)(bus);
    }

    return rv;
}

static int can_bus_do_write_default(can_bus *bus, can_msg *msg)
{
    (void)bus;
    (void)msg;

    return -EIO;
}

static int can_bus_do_read_default(can_bus *bus, can_msg *msg, can_fifo fifo)
{
    (void)bus;
    (void)msg;
    (void)fifo;

    return -EIO;
}

static int can_bus_set_freq_default(can_bus *bus, can_freq freq)
{
    (void)bus;
    (void)freq;

    return -EIO;
}

static int can_bus_add_filter_default(can_bus *bus, can_filter filter)
{
    (void)bus;
    (void)filter;

    return -EIO;
}

static int can_bus_do_init(
    can_bus *bus,
    void (*destroy)(can_bus *bus))
{
    rtems_recursive_mutex_init(&bus->mutex, "CAN-Bus");
    bus->can_write = can_bus_do_write_default;
    bus->can_read = can_bus_do_read_default;
    bus->set_freq = can_bus_set_freq_default;
    bus->add_filter = can_bus_add_filter_default;
    bus->destroy = destroy;

    return 0;
}

void can_bus_destroy(can_bus *bus)
{
    rtems_recursive_mutex_destroy(&bus->mutex);
}

void can_bus_destroy_and_free(can_bus *bus)
{
    can_bus_destroy(bus);
    free(bus);
}

int can_bus_init(can_bus *bus)
{
    memset(bus, 0, sizeof(*bus));

    return can_bus_do_init(bus, can_bus_destroy);
}

can_bus *can_bus_alloc_and_init(size_t size)
{
    can_bus *bus = NULL;

    if (size >= sizeof(*bus))
    {
        bus = calloc(1, size);
        if (bus != NULL)
        {
            int rv;

            rv = can_bus_do_init(bus, can_bus_destroy_and_free);
            if (rv != 0)
            {
                return NULL;
            }
        }
    }

    return bus;
}
