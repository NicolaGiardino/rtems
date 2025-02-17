/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (C) 2020 embedded brains GmbH & Co. KG
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stm32h7/hal.h>

// const RCC_ClkInitTypeDef stm32h7_config_clocks = {
//   .ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
//     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
//     | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1,
//   .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
//   .SYSCLKDivider = RCC_SYSCLK_DIV1,
//   .AHBCLKDivider = RCC_HCLK_DIV2,
//   .APB3CLKDivider = RCC_APB3_DIV2,
//   .APB1CLKDivider = RCC_APB1_DIV2,
//   .APB2CLKDivider = RCC_APB2_DIV2,
//   .APB4CLKDivider = RCC_APB4_DIV2
// };

const RCC_ClkInitTypeDef stm32h7_config_clocks = {
    .ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1,
    .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
    .SYSCLKDivider = RCC_SYSCLK_DIV1,
    .AHBCLKDivider = RCC_HCLK_DIV2,
    .APB3CLKDivider = RCC_APB3_DIV2,
    .APB1CLKDivider = RCC_APB1_DIV4,
    .APB2CLKDivider = RCC_APB2_DIV8,
    .APB4CLKDivider = RCC_APB4_DIV2,
};
