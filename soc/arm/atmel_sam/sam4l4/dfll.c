/**
 * \file
 *
 * \brief Chip-specific DFLL implementation
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <dfll.h>
#include <tpaste.h>

#include <soc.h>

#define cpu_irq_save() irq_lock()
#define cpu_irq_restore(flags) irq_unlock(flags)

typedef uint32_t irqflags_t;

#if 1
#define dfll_write_reg(name, value)		\
	do { \
		while (!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY));                 \
		irqflags_t ATPASTE2(dfll_flags, __LINE__) = cpu_irq_save();          \
		SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAUL)                          \
			| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_##name - (uint32_t)SCIF);\
		SCIF->reg.SCIF_##name = (value);                                          \
		cpu_irq_restore(ATPASTE2(dfll_flags, __LINE__));					\
	} while (0)
#else
	do { \
		while (!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY));                 \
		irqflags_t _CONCAT(dfll_flags, __LINE__) = cpu_irq_save();          \
		SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAUL)                          \
			| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_##name - (uint32_t)SCIF);\
		SCIF->reg.SCIF_##name = (value);                                          \
		cpu_irq_restore(ATPASTE2(dfll_flags, __LINE__));                     \
	} while (0)
#endif


void dfll_enable_open_loop(const struct dfll_config *cfg, uint32_t dfll_id)
{
	irqflags_t flags;

	ARG_UNUSED(dfll_id);

	/* First, enable the DFLL, then configure it */
	flags = cpu_irq_save();
	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAUL)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_DFLL0CONF - (uint32_t)SCIF);
	SCIF->reg.SCIF_DFLL0CONF = SCIF_DFLL0CONF_EN;
	cpu_irq_restore(flags);
	dfll_write_reg(DFLL0CONF, cfg->conf | SCIF_DFLL0CONF_EN);
	dfll_write_reg(DFLL0MUL, cfg->mul);
	dfll_write_reg(DFLL0VAL, cfg->val);
	dfll_write_reg(DFLL0SSG, cfg->ssg);
}

void dfll_disable_open_loop(uint32_t dfll_id)
{
	ARG_UNUSED(dfll_id);

	/* First, disable the DFLL. */
	// Do a sync before reading a dfll conf register
	SCIF->reg.SCIF_DFLL0SYNC = SCIF_DFLL0SYNC_SYNC;
	while (!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY));

	uint32_t conf = SCIF->reg.SCIF_DFLL0CONF;
	conf &= ~SCIF_DFLL0CONF_EN;
	dfll_write_reg(DFLL0CONF, conf);

	/* Finally, stop the reference clock */
	genclk_disable(0);
}

void dfll_enable_closed_loop(const struct dfll_config *cfg, uint32_t dfll_id)
{
	irqflags_t flags;

	ARG_UNUSED(dfll_id);

	/* Enable the reference clock */
	genclk_enable(&cfg->ref_cfg, 0);

	/*
	 * Enable the DFLL first, but don't wait for the DFLL0RDY bit
	 * because if the DFLL has been disabled before, the DFLL0RDY
	 * bit stays cleared until it is re-enabled.
	 */
	flags = cpu_irq_save();
	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAUL)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_DFLL0CONF - (uint32_t)SCIF);
	SCIF->reg.SCIF_DFLL0CONF = SCIF_DFLL0CONF_EN;
	cpu_irq_restore(flags);

	/*
	 * Then, configure the DFLL, taking care to wait for the
	 * DFLL0RDY bit before every step.
	 */
	dfll_write_reg(DFLL0STEP, cfg->step);
	dfll_write_reg(DFLL0MUL, cfg->mul);
	dfll_write_reg(DFLL0SSG, cfg->ssg);
	dfll_write_reg(DFLL0CONF, cfg->conf | SCIF_DFLL0CONF_EN);
}

void dfll_disable_closed_loop(uint32_t dfll_id)
{
	ARG_UNUSED(dfll_id);

	/* First, disable the DFLL. */
	// Do a sync before reading a dfll conf register
	SCIF->reg.SCIF_DFLL0SYNC = SCIF_DFLL0SYNC_SYNC;
	while (!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY));

	uint32_t conf = SCIF->reg.SCIF_DFLL0CONF;
	conf &= ~SCIF_DFLL0CONF_EN;
	dfll_write_reg(DFLL0CONF, conf);

	/* Finally, stop the reference clock */
	genclk_disable(0);
}

void dfll_enable_config_defaults(uint32_t dfll_id)
{
#ifdef CONFIG_DFLL0_SOURCE
	struct dfll_config dfllcfg;
#endif
	static bool open_loop_done = false;

	if((SCIF->reg.SCIF_DFLL0CONF & SCIF_DFLL0CONF_MODE) &&
	   (SCIF->reg.SCIF_DFLL0CONF & SCIF_DFLL0CONF_EN)) {
		// Closed-loop mode
		if (dfll_is_fine_locked(dfll_id)) {
			return; // DFLL already running
		}
	}
	if (open_loop_done == true) {
		return;
	}

	switch (dfll_id) {
#ifdef CONFIG_DFLL0_SOURCE
	case 0:
		dfll_enable_source(CONFIG_DFLL0_SOURCE);
		dfll_config_init_closed_loop_mode(&dfllcfg,
			CONFIG_DFLL0_SOURCE,
			CONFIG_DFLL0_DIV,
			CONFIG_DFLL0_MUL);

		dfll_enable_closed_loop(&dfllcfg, dfll_id);
		while (!dfll_is_fine_locked(dfll_id));
		break;
#endif

	default:
		assert(false);
		break;
	}
}
