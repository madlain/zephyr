/*
 * Copyright (c) 2018 Madani Lainani
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAM4L4 MCU series initialization code
 */

#include <arch/cpu.h>
#include <cortex_m/exc.h>
#include <device.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>

static int atmel_sam4l_init(struct device *arg)
{
	u32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	_ClearFaults();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(atmel_sam4l_init, PRE_KERNEL_1, 0);
