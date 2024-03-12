/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @brief Internal Watchdog (iwdg) Interface Driver API
 *
 * @ingroup iwdg
 */

/*
 * Copyright (C) 2023 Nicola di Gruttola Giardino <nicoladgg@protonmail.com>
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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR ifaceINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DEV_iwdg_iwdg_H
#define _DEV_iwdg_iwdg_H

#include <rtems.h>
#include <rtems/seterr.h>
#include <rtems/thread.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

	typedef struct iwdg_ioc_transfer iwdg_ioc_transfer;

	typedef struct iwdg_iface iwdg_iface;

/**                                                                \
 * @defgroup iwdg Serial Peripheral Interface (iwdg) Driver        \
 *                                                                 \
 * @ingroup RTEMSDeviceDrivers                                     \
 *                                                                 \
 * @brief Serial Peripheral Interface (iwdg) iface driver support. \
 *                                                                 \
 * @{                                                              \
 */                                                                \
#define IWDG_IOC_MAGIC 'i'

/**
 * @brief Obtains the iface.
 *
 * This command has no argument.
 */
#define IWDG_IFACE_OBTAIN _IO(IWDG_IOC_MAGIC, 13)

/**
 * @brief Releases the iface.
 *
 * This command has no argument.
 */
#define IWDG_IFACE_RELEASE _IO(IWDG_IOC_MAGIC, 23)

/*
 * @brief Set the prescaler.
 *
 * This command has an argument of type uint16_t.
 */
#define IWDG_IFACE_SET_PRESCALER _IOW(IWDG_IOC_MAGIC, 33, uint16_t)

/*
 * @brief Get the prescaler.
 *
 * This command has an argument of type uint16_t.
 * The prescaler is returned in the argument.
 */
#define IWDG_IFACE_GET_PRESCALER _IOR(IWDG_IOC_MAGIC, 43, uint16_t)

/*
 * @brief Set the window.
 *
 * This command has an argument of type uint16_t.
 */
#define IWDG_IFACE_SET_WINDOW _IOW(IWDG_IOC_MAGIC, 53, uint16_t)

/*
 * @brief Get the window.
 *
 * This command has an argument of type uint16_t.
 * The window is returned in the argument.
 */
#define IWDG_IFACE_GET_WINDOW _IOR(IWDG_IOC_MAGIC, 63, uint16_t)

/*
 * @brief Set the reload.
 *
 * This command has an argument of type uint16_t.
 */
#define IWDG_IFACE_SET_RELOAD _IOW(IWDG_IOC_MAGIC, 73, uint16_t)

/*
 * @brief Get the reload.
 *
 * This command has an argument of type uint16_t.
 * The reload is returned in the argument.
 */
#define IWDG_IFACE_GET_RELOAD _IOR(IWDG_IOC_MAGIC, 83, uint16_t)

/*
 * @brief Reload the watchdog.
 *
 * This command has no argument.
 */
#define IWDG_IFACE_RELOAD _IO(IWDG_IOC_MAGIC, 93)

	/**
	 * @brief iwdg iface control.
	 */
	struct iwdg_iface
	{
		/**
		 * @brief
		 *
		 * @param[in] iface The iface control.
		 *
		 * @retval 0 Successful operation.
		 * @retval negative Negative error number in case of an error.
		 */
		int (*setup)(iwdg_iface *iface);

		/**
		 * @brief Destroys the iface.
		 *
		 * @param[in] iface The iface control.
		 */
		void (*destroy)(iwdg_iface *iface);

		/**
		 * @brief Set the prescaler.
		 *
		 * @param[in] iface The iface control.
		 */
		int (*set_prescaler)(iwdg_iface *iface);

		/**
		 * @brief Set the window.
		 *
		 * @param[in] iface The iface control.
		 */
		int (*set_window)(iwdg_iface *iface);

		/**
		 * @brief Set the reload.
		 *
		 * @param[in] iface The iface control.
		 */
		int (*set_reload)(iwdg_iface *iface);

		/*
		 * @brief Reload the watchdog.
		 *
		 * @param[in] iface The iface control.
		 */
		int (*reload)(iwdg_iface *iface);

		/**
		 * @brief Mutex to protect the iface access.
		 */
		rtems_recursive_mutex mutex;

		uint16_t prescaler_value;

		uint16_t window_value;

		uint16_t reload_value;

		/**
		 * @brief Driver specific ioctl.
		 *
		 * @param[in] iface The iface control.
		 */
		int (*ioctl)(iwdg_iface *iface, ioctl_command_t command, void *arg);
	};

	/**
	 * @brief Initializes a iface control.
	 *
	 * After a sucessful initialization the iface control must be destroyed via
	 * iwdg_iface_destroy().  A registered iface control will be automatically destroyed
	 * in case the device file is unlinked.  Make sure to call iwdg_iface_destroy() in
	 * a custom destruction handler.
	 *
	 * @param[in] iface The iface control.
	 *
	 * @retval 0 Successful operation.
	 * @retval -1 An error occurred.  The errno is set to indicate the error.
	 *
	 * @see iwdg_iface_register()
	 */
	int iwdg_iface_init(iwdg_iface *iface);

	/**
	 * @brief Allocates a iface control from the heap and initializes it.
	 *
	 * After a sucessful allocation and initialization the iface control must be
	 * destroyed via iwdg_iface_destroy_and_free().  A registered iface control will be
	 * automatically destroyed in case the device file is unlinked.  Make sure to
	 * call iwdg_iface_destroy_and_free() in a custom destruction handler.
	 *
	 * @param[in] size The size of the iface control.  This enables the addition of
	 * iface controller specific data to the base iface control.  The iface control is
	 * zero initialized.
	 *
	 * @retval non-NULL The new iface control.
	 * @retval NULL An error occurred.  The errno is set to indicate the error.
	 *
	 * @see iwdg_iface_register()
	 */
	iwdg_iface *iwdg_iface_alloc_and_init(size_t size);

	/**
	 * @brief Destroys a iface control.
	 *
	 * @param[in] iface The iface control.
	 */
	void iwdg_iface_destroy(iwdg_iface *iface);

	/**
	 * @brief Destroys a iface control and frees its memory.
	 *
	 * @param[in] iface The iface control.
	 */
	void iwdg_iface_destroy_and_free(iwdg_iface *iface);

	/**
	 * @brief Registers a iface control.
	 *
	 * This function claims ownership of the iface control regardless if the
	 * registration is successful or not.
	 *
	 * @param[in] iface The iface control.
	 * @param[in] iface_path The path to the iface device file.
	 *
	 * @retval 0 Successful operation.
	 * @retval -1 An error occurred.  The errno is set to indicate the error.
	 */
	int iwdg_iface_register(
		iwdg_iface *iface,
		const char *iface_path);

	/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_iwdg_iwdg_H */
