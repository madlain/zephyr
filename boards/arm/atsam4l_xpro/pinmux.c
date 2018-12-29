/*
 * Copyright (c) 2018 Madani Lainani.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <pinmux.h>

static int board_pinmux_init(struct device *dev)
{
	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
