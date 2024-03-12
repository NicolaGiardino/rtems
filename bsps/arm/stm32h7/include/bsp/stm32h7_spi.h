/**
 * @file
 *
 * @ingroup arm_stm32h7
 *
 * @brief SPI support API.
 *
 * Based on bsps/m68k/gen68360/spi/m360_spi.h
 */

/*
 * Copyright (c) 2023 Nicola di Gruttola Giardino
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_stm32h7_SPI_H
#define LIBBSP_ARM_stm32h7_SPI_H

#include <bsp.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_spi.h>
#include <dev/spi/spi.h>
#include <rtems/irq.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define STM32H7_SPI_TIMEOUT 1000

#define STM32H7_SPI_0_BUS_PATH "/dev/spi-0"
#define STM32H7_SPI_1_BUS_PATH "/dev/spi-1"

    typedef struct stm32h7_SPI_BufferDescriptor_
    {
        unsigned short status;
        unsigned short length;
        volatile void *buffer;
    } stm32h7_SPI_BufferDescriptor_t;

    typedef struct stm32h7_spi_softc
    {
        int initialized;
        rtems_id task_id;
        SPI_HandleTypeDef spi;
        uint8_t activate_irq;
        rtems_vector_number irq;
    } stm32h7_spi_softc_t;

    typedef struct
    {
        spi_bus *bus_desc;
        stm32h7_spi_softc_t softc;
    } stm32h7_spi_desc_t;

    /*
     * Initialize the driver
     *
     * Returns: o = ok or error code
     */
    rtems_status_code stm32h7_spi_init(
        spi_bus *bh,
        const uint8_t spi_number
    );

    /*
     * Receive some bytes from SPI device
     *
     * Returns: number of bytes received or (negative) error code
     */
    int stm32h7_spi_read_bytes(
        spi_bus *bh, /* bus specifier structure        */
        unsigned char *buf,     /* buffer to store bytes          */
        uint32_t len                 /* number of bytes to receive     */
    );

    /*
     * Send some bytes to SPI device
     *
     * Returns: number of bytes sent or (negative) error code
     */
    int stm32h7_spi_write_bytes(
        spi_bus *bh, /* bus specifier structure        */
        unsigned char *buf,     /* buffer to send                 */
        uint32_t len                 /* number of bytes to send        */
    );


    /*
     * Perform selected ioctl function for SPI
     *
     * Returns: rtems_status_code
     */
    int stm32h7_spi_ioctl(
        spi_bus *bh,         /* bus specifier structure        */
        ioctl_command_t cmd, /* ioctl command code             */
        void *arg            /* additional argument array      */
    );

    /*
     * IRQ handler for SPI
     * Returns: rtems_status_code
     */
    rtems_status_code stm32h7_spi_irq_handler(
        rtems_vector_number vector,
        void *arg
    );

    /*
     * Delete SPI bus and devices
     *
     * Returns: Bus number or error code
     */
    void stm32h7_spi_destroy(
        spi_bus *bh /* bus specifier structure        */
    );

    /*
     * Register SPI bus and devices
     *
     * Returns: Bus number or error code
     */
    rtems_status_code bsp_register_spi(
        const char *bus_path,
        uint8_t activate_irq,
        rtems_vector_number irq);

    static inline rtems_status_code stm32h7_register_spi_0(void)
    {
        return bsp_register_spi(
            STM32H7_SPI_0_BUS_PATH,
            0,
            0);
    }

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_stm32h7_SPI_H */
