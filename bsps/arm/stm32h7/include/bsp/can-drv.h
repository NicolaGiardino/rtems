/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @brief Controller Area Network (CAN) Bus Driver API
 *
 * @ingroup CANBus
 */

/*
 * Copyright (C) 2023 Nicola di Gruttola Giardino <nicoladgg@protonmail.com> <nicola.digruttolagiardino@teamdiana.it>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DEV_CAN_CAN_H
#define _DEV_CAN_CAN_H

#include <rtems.h>
#include <rtems/seterr.h>
#include <rtems/thread.h>

#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

    typedef struct can_msg_
    {
        uint32_t id;
        uint8_t data[8];
        uint8_t dlc;
        uint8_t rtr;
        uint8_t ide;
    } can_msg;

    typedef struct can_freq_
    {
        uint32_t prescaler;
        uint32_t sjw;
        uint32_t ts1;
        uint32_t ts2;
    } can_freq;

    typedef struct can_filter_
    {
        uint32_t filter_index;
        uint32_t is_extended;
        uint32_t filter_type;
        uint32_t filter_fifo;
        uint32_t filter_id1;
        uint32_t filter_id2;
    } can_filter;

    typedef uint8_t can_fifo;
    /**
     * @brief Argument type for I2C_RDWR IO control call.
     */
    struct can_rdwr_ioctl_data
    {
        can_msg msg;
        uint32_t fifo;
    };

    typedef struct can_bus can_bus;

    typedef struct can_dev can_dev;

    typedef struct can_rdwr_ioctl_data can_rdwr_ioctl_data;

/**
 * @defgroup CAN Inter-Integrated Circuit (CAN) Driver
 *
 * @ingroup RTEMSDeviceDrivers
 *
 * @brief Inter-Integrated Circuit (CAN) bus and device driver support.
 *
 * @{
 */

/**
 * @defgroup CANBus CAN Bus Driver
 *
 * @ingroup CAN
 *
 * @{
 */

/**
 * @name CAN IO Control Commands
 *
 * @{
 */
#define CAN_BUS_RD 0x700

#define CAN_BUS_WR 0x701

/**
 * @brief Obtains the bus.
 *
 * This command has no argument.
 */
#define CAN_BUS_OBTAIN 0x800

/**
 * @brief Releases the bus.
 *
 * This command has no argument.
 */
#define CAN_BUS_RELEASE 0x801

/**
 * @brief Gets the bus control.
 *
 * The argument type is a pointer to can_bus pointer.
 */
#define CAN_BUS_GET_CONTROL 0x802

/**
 * @brief Sets the bus clock in Hz.
 *
 * The argument type is unsigned long.
 */
#define CAN_BUS_SET_FREQ 0x803

#define CAN_BUS_ADD_FILTER 0x804

/** @} */

/**
 * @brief Default CAN bus clock in Hz.
 */
#define CAN_BUS_CLOCK_DEFAULT 500000

    /**
     * @brief CAN bus control.
     */
    struct can_bus
    {
        /**
         * @brief Write CAN messages.
         *
         * @param[in] bus The bus control.
         * @param[in] msgs The messages to transfer.
         * @param[in] msg_count The count of messages to transfer.  It must be
         * positive.
         *
         * @retval 0 Successful operation.
         * @retval negative Negative error number in case of an error.
         */
        int (*can_write)(can_bus *bus, can_msg *msg);

        /**
         * @brief Read CAN messages.
         *
         * @param[in] bus The bus control.
         * @param[in] msgs The messages to transfer.
         * @param[in] msg_count The count of messages to transfer.  It must be
         * positive.
         *
         * @retval 0 Successful operation.
         * @retval negative Negative error number in case of an error.
         */
        int (*can_read)(can_bus *bus, can_msg *msg, can_fifo fifo);

        /**
         * @brief Sets the bus frequency.
         *
         * @param[in] bus The bus control.
         * @param[in] clock The desired bus clock in Hz.
         *
         * @retval 0 Successful operation.
         * @retval negative Negative error number in case of an error.
         */
        int (*set_freq)(can_bus *bus, can_freq freq);

        /**
         * @brief Sets acceptance filters for a given FIFO.
         *
         * @param[in] bus The bus control.
         *
         * @retval 0 Successful operation.
         * @retval negative Negative error number in case of an error.
         */
        int (*add_filter)(can_bus *bus, can_filter filter);

        /**
         * @brief Destroys the bus.
         *
         * @param[in] bus The bus control.
         */
        void (*destroy)(can_bus *bus);

        /**
         * @brief Mutex to protect the bus access.
         */
        rtems_recursive_mutex mutex;

        /**
         * @brief The bus frequency.
         */
        can_freq freq;
    };

    /**
     * @brief Initializes a bus control.
     *
     * After a sucessful initialization the bus control must be destroyed via
     * can_bus_destroy().  A registered bus control will be automatically destroyed
     * in case the device file is unlinked.  Make sure to call can_bus_destroy() in
     * a custom destruction handler.
     *
     * @param[in] bus The bus control.
     *
     * @retval 0 Successful operation.
     * @retval -1 An error occurred.  The errno is set to indicate the error.
     *
     * @see can_bus_register()
     */
    int can_bus_init(can_bus *bus);

    /**
     * @brief Allocates a bus control from the heap and initializes it.
     *
     * After a sucessful allocation and initialization the bus control must be
     * destroyed via can_bus_destroy_and_free().  A registered bus control will be
     * automatically destroyed in case the device file is unlinked.  Make sure to
     * call can_bus_destroy_and_free() in a custom destruction handler.
     *
     * @param[in] size The size of the bus control.  This enables the addition of
     * bus controller specific data to the base bus control.  The bus control is
     * zero initialized.
     *
     * @retval non-NULL The new bus control.
     * @retval NULL An error occurred.  The errno is set to indicate the error.
     *
     * @see can_bus_register()
     */
    can_bus *can_bus_alloc_and_init(size_t size);

    /**
     * @brief Destroys a bus control.
     *
     * @param[in] bus The bus control.
     */
    void can_bus_destroy(can_bus *bus);

    /**
     * @brief Destroys a bus control and frees its memory.
     *
     * @param[in] bus The bus control.
     */
    void can_bus_destroy_and_free(can_bus *bus);

    /**
     * @brief Registers a bus control.
     *
     * This function claims ownership of the bus control regardless if the
     * registration is successful or not.
     *
     * @param[in] bus The bus control.
     * @param[in] bus_path The path to the bus device file.
     *
     * @retval 0 Successful operation.
     * @retval -1 An error occurred.  The errno is set to indicate the error.
     */
    int can_bus_register(
        can_bus *bus,
        const char *bus_path);

    /**
     * @brief Try to obtain the bus.
     *
     * @param[in] bus The bus control.
     *
     * @retval 0 Successful operation.
     * @retval EBUSY if mutex is already locked.
     */
    int can_bus_try_obtain(can_bus *bus);

    /**
     * @brief Obtains the bus.
     *
     * @param[in] bus The bus control.
     */
    void can_bus_obtain(can_bus *bus);

    /**
     * @brief Releases the bus.
     *
     * @param[in] bus The bus control.
     */
    void can_bus_release(can_bus *bus);

    /**
     * @brief Transfers CAN messages.
     *
     * The bus is obtained before the transfer and released afterwards. This is the
     * same like calling @ref can_bus_do_transfer with flags set to 0.
     *
     * @param[in] bus The bus control.
     * @param[in] msgs The messages to transfer.
     * @param[in] msg_count The count of messages to transfer.  It must be
     * positive.
     *
     * @retval 0 Successful operation.
     * @retval negative Negative error number in case of an error.
     */
    int can_bus_do_write(can_bus *bus, can_msg *msg, uint32_t flags);

    /**
     * @brief Reads CAN messages.
     *
     * The bus is obtained before the transfer and released afterwards. This is the
     * same like calling @ref can_bus_do_transfer with flags set to 0.
     *
     * @param[in] bus The bus control.
     * @param[in] msgs The messages to transfer.
     * @param[in] msg_count The count of messages to transfer.  It must be
     * positive.
     *
     * @retval 0 Successful operation.
     * @retval negative Negative error number in case of an error.
     */
    int can_bus_do_read(can_bus *bus, can_msg *msg, can_fifo fifo, uint32_t flags);

/**
 * @brief CAN bus transfer flag to indicate that the task should not block if
 * the bus is busy on a new transfer.
 */
#define CAN_BUS_NOBLOCK (1u << 0)

/** @} */

/**
 * @defgroup CANDevice CAN Device Driver
 *
 * @ingroup CAN
 *
 * @{
 */

/**
 * @brief Base number for device IO control commands.
 */
#define CAN_DEV_IO_CONTROL 0x900

    /**
     * @brief CAN slave device control.
     */
    struct can_dev
    {
        /**
         * @brief Reads from the device.
         *
         * @retval non-negative Bytes transferred from device.
         * @retval negative Negative error number in case of an error.
         */
        ssize_t (*read)(can_dev *dev, void *buf, size_t n, off_t offset);

        /**
         * @brief Writes to the device.
         *
         * @retval non-negative Bytes transferred to device.
         * @retval negative Negative error number in case of an error.
         */
        ssize_t (*write)(can_dev *dev, const void *buf, size_t n, off_t offset);

        /**
         * @brief Device IO control.
         *
         * @retval 0 Successful operation.
         * @retval negative Negative error number in case of an error.
         */
        int (*ioctl)(can_dev *dev, ioctl_command_t command, void *arg);

        /**
         * @brief Gets the file size.
         */
        off_t (*get_size)(can_dev *dev);

        /**
         * @brief Gets the file block size.
         */
        blksize_t (*get_block_size)(can_dev *dev);

        /**
         * @brief Destroys the device.
         */
        void (*destroy)(can_dev *dev);

        /**
         * @brief The bus control.
         */
        can_bus *bus;

        /**
         * @brief The device address.
         */
        uint16_t address;

        /**
         * @brief File descriptor of the bus.
         *
         * This prevents destruction of the bus since we hold a reference to it with
         * this.
         */
        int bus_fd;
    };

    /**
     * @brief Initializes a device control.
     *
     * After a sucessful initialization the device control must be destroyed via
     * can_dev_destroy().  A registered device control will be automatically
     * destroyed in case the device file is unlinked.  Make sure to call
     * can_dev_destroy_and_free() in a custom destruction handler.
     *
     * @param[in] device The device control.
     * @param[in] bus_path The path to the bus device file.
     * @param[in] address The address of the device.
     *
     * @retval 0 Successful operation.
     * @retval -1 An error occurred.  The errno is set to indicate the error.
     *
     * @see can_dev_register()
     */
    int can_dev_init(can_dev *dev, const char *bus_path, uint16_t address);

    /**
     * @brief Allocates a device control from the heap and initializes it.
     *
     * After a sucessful allocation and initialization the device control must be
     * destroyed via can_dev_destroy_and_free().  A registered device control will
     * be automatically destroyed in case the device file is unlinked.  Make sure
     * to call can_dev_destroy_and_free() in a custom destruction handler.
     *
     * @param[in] size The size of the device control.  This enables the addition
     * of device specific data to the base device control.  The device control is
     * zero initialized.
     * @param[in] bus_path The path to the bus device file.
     * @param[in] address The address of the device.
     *
     * @retval non-NULL The new device control.
     * @retval NULL An error occurred.  The errno is set to indicate the error.
     *
     * @see can_dev_register()
     */
    can_dev *can_dev_alloc_and_init(
        size_t size,
        const char *bus_path,
        uint16_t address);

    /**
     * @brief Destroys a device control.
     *
     * @param[in] dev The device control.
     */
    void can_dev_destroy(can_dev *dev);

    /**
     * @brief Destroys a device control and frees its memory.
     *
     * @param[in] dev The device control.
     */
    void can_dev_destroy_and_free(can_dev *dev);

    /**
     * @brief Registers a device control.
     *
     * This function claims ownership of the device control regardless if the
     * registration is successful or not.
     *
     * @param[in] dev The dev control.
     * @param[in] dev_path The path to the device file of the device.
     *
     * @retval 0 Successful operation.
     * @retval -1 An error occurred.  The errno is set to indicate the error.
     */
    int can_dev_register(
        can_dev *dev,
        const char *dev_path);

    /** @} */ /* end of can device driver */

    /** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_CAN_CAN_H */
