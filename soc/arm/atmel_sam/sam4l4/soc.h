/*
 * Copyright (c) 2018 Madani Lainani
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ATMEL_SAMD_SOC_H_
#define _ATMEL_SAMD_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr.h>

#if defined(CONFIG_SOC_PART_NUMBER_SAM4LC4C)
#include <sam4lc4c.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

/** Processor Clock (HCLK) Frequency */
#define SOC_ATMEL_SAM4L_HCLK_FREQ_HZ CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC

/** Master Clock (MCK) Frequency */
#define SOC_ATMEL_SAM4L_MCK_FREQ_HZ SOC_ATMEL_SAM4L_HCLK_FREQ_HZ

#endif /* _ATMEL_SAMD_SOC_H_ */
