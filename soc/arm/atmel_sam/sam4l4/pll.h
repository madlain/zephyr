/**
 * \file
 *
 * \brief PLL management
 *
 * Copyright (c) 2010-2018 Microchip Technology Inc. and its subsidiaries.
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
#ifndef CLK_PLL_H_INCLUDED
#define CLK_PLL_H_INCLUDED

#include <soc.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCIF0_PLL_VCO_RANGE1_MAX_FREQ   240000000
#define SCIF_PLL0_VCO_RANGE1_MIN_FREQ   160000000
#define SCIF_PLL0_VCO_RANGE0_MAX_FREQ   180000000
#define SCIF_PLL0_VCO_RANGE0_MIN_FREQ    80000000

/**
 * \weakgroup pll_group
 * @{
 */

#define PLL_MAX_STARTUP_CYCLES    (SCIF_PLL_PLLCOUNT_Msk >> SCIF_PLL_PLLCOUNT_Pos)
#define NR_PLLS                   1

/**
 * \brief Number of milliseconds to wait for PLL lock
 */
#define PLL_TIMEOUT_MS \
	div_ceil(1000 * (PLL_MAX_STARTUP_CYCLES * 2), OSC_RCSYS_MIN_HZ)

/**
 * \note The PLL must run at twice this frequency internally, but the
 * output frequency may be divided by two by setting the PLLOPT[1] bit.
 */
#define PLL_MIN_HZ                40000000
#define PLL_MAX_HZ                240000000

//! \name Chip-specific PLL options
//@{
//! VCO frequency range is 160-240 MHz (80-180 MHz if unset).
#define PLL_OPT_VCO_RANGE_HIGH    0
//! Divide output frequency by two
#define PLL_OPT_OUTPUT_DIV        1
//! Disable wide-bandwidth mode
#define PLL_OPT_WBM_DISABLE       2
//! Number of PLL options
#define PLL_NR_OPTIONS            3
//! The threshold above which to set the #PLL_OPT_VCO_RANGE_HIGH option
#define PLL_VCO_LOW_THRESHOLD     ((SCIF_PLL0_VCO_RANGE1_MIN_FREQ \
	+ SCIF_PLL0_VCO_RANGE0_MAX_FREQ) / 2)
//@}

#ifndef __ASSEMBLY__

#include <osc.h>

enum pll_source {
	PLL_SRC_OSC0            = 0,    //!< Oscillator 0
	PLL_SRC_GCLK9           = 1,    //!< Generic Clock 9
	PLL_NR_SOURCES,                 //!< Number of PLL sources
};

struct pll_config {
	uint32_t ctrl;
};

#define pll_get_default_rate(pll_id)                \
	((osc_get_rate(CONFIG_PLL ## pll_id ## _SOURCE) \
	* CONFIG_PLL ## pll_id ## _MUL)                 \
	/ CONFIG_PLL ## pll_id ## _DIV)

static inline void pll_config_set_option(struct pll_config *cfg,
		uint32_t option)
{
	assert(option < PLL_NR_OPTIONS);

	cfg->ctrl |= 1U << (SCIF_PLL_PLLOPT_Pos + option);
}

static inline void pll_config_clear_option(struct pll_config *cfg,
		uint32_t option)
{
	assert(option < PLL_NR_OPTIONS);

	cfg->ctrl &= ~(1U << (SCIF_PLL_PLLOPT_Pos + option));
}

/**
 * The PLL options #PLL_OPT_VCO_RANGE_HIGH and #PLL_OPT_OUTPUT_DIV will
 * be set automatically based on the calculated target frequency.
 */
static inline void pll_config_init(struct pll_config *cfg,
		enum pll_source src, uint32_t divide, uint32_t mul)
{
#define MUL_MIN    2
#define MUL_MAX    16
#define DIV_MIN    0
#define DIV_MAX    15

	uint32_t vco_hz;

	assert(src < PLL_NR_SOURCES);
	assert(divide != 0);

	/* Calculate internal VCO frequency */
	vco_hz = osc_get_rate(src) * mul;
	vco_hz /= divide;
	assert(vco_hz >= PLL_MIN_HZ);
	assert(vco_hz <= PLL_MAX_HZ);

	cfg->ctrl = 0;

	/* Bring the internal VCO frequency up to the minimum value */
	if ((vco_hz < PLL_MIN_HZ * 2) && (mul <= 8)) {
		mul *= 2;
		vco_hz *= 2;
		pll_config_set_option(cfg, PLL_OPT_OUTPUT_DIV);
	}

	/* Set VCO frequency range according to calculated value */
	if (vco_hz >= PLL_VCO_LOW_THRESHOLD) {
		pll_config_set_option(cfg, PLL_OPT_VCO_RANGE_HIGH);
	}

	assert(mul > MUL_MIN && mul <= MUL_MAX);
	assert(divide > DIV_MIN && divide <= DIV_MAX);

	cfg->ctrl |= ((mul - 1) << SCIF_PLL_PLLMUL_Pos)
			| (divide << SCIF_PLL_PLLDIV_Pos)
			| (PLL_MAX_STARTUP_CYCLES << SCIF_PLL_PLLCOUNT_Pos)
			| (src << SCIF_PLL_PLLOSC_Pos);
}

#define pll_config_defaults(cfg, pll_id) \
	pll_config_init(cfg,                 \
		CONFIG_PLL ## pll_id ## _SOURCE, \
		CONFIG_PLL ## pll_id ## _DIV,    \
		CONFIG_PLL ## pll_id ## _MUL)

static inline void pll_config_read(struct pll_config *cfg, uint32_t pll_id)
{
	assert(pll_id < NR_PLLS);

	cfg->ctrl = SCIF->reg.SCIF_PLL[pll_id].reg.SCIF_PLL;
}

extern void pll_config_write(const struct pll_config *cfg, uint32_t pll_id);
extern void pll_enable(const struct pll_config *cfg, uint32_t pll_id);
extern void pll_disable(uint32_t pll_id);

static inline bool pll_is_locked(uint32_t pll_id)
{
	assert(pll_id < NR_PLLS);
	return !!(SCIF->reg.SCIF_PCLKSR & (1U << (6 + pll_id)));
}

static inline void pll_enable_source(enum pll_source src)
{
	switch (src) {
	case PLL_SRC_OSC0:
		if (!osc_is_ready(OSC_ID_OSC0)) {
			osc_enable(OSC_ID_OSC0);
			osc_wait_ready(OSC_ID_OSC0);
		}
		break;
#ifdef CONFIG_GCLK9_SOURCE
	case PLL_SRC_GCLK9:
		SCIF->reg.SCIF_GCCTRL[9].SCIF_GCCTRL =
			SCIF_GCCTRL_OSCSEL(CONFIG_GCLK9_SOURCE) |
			SCIF_GCCTRL_CEN;
		break;
#endif
	default:
		assert(false);
		break;
	}
}

static inline void pll_enable_config_defaults(uint32_t pll_id)
{
	struct pll_config pllcfg;

	if (pll_is_locked(pll_id)) {
		return; // Pll already running
	}

	switch (pll_id) {
#ifdef CONFIG_PLL0_SOURCE
	case 0:
		pll_enable_source(CONFIG_PLL0_SOURCE);
		pll_config_init(&pllcfg,
				CONFIG_PLL0_SOURCE,
				CONFIG_PLL0_DIV,
				CONFIG_PLL0_MUL);
		break;

#endif
	default:
		assert(false);
		break;
	}
	pll_enable(&pllcfg, pll_id);
	while (!pll_is_locked(pll_id));
}

#endif /* __ASSEMBLY__ */

//! @}

#ifdef __cplusplus
}
#endif

/**
 * \ingroup clk_group
 * \defgroup pll_group PLL Management
 *
 * This group contains functions and definitions related to configuring
 * and enabling/disabling on-chip PLLs. A PLL will take an input signal
 * (the \em source), optionally divide the frequency by a configurable
 * \em divider, and then multiply the frequency by a configurable \em
 * multiplier.
 *
 * Some devices don't support input dividers; specifying any other
 * divisor than 1 on these devices will result in an assertion failure.
 * Other devices may have various restrictions to the frequency range of
 * the input and output signals.
 *
 * \par Example: Setting up PLL0 with default parameters
 *
 * The following example shows how to configure and enable PLL0 using
 * the default parameters specified using the configuration symbols
 * listed above.
 * \code
	pll_enable_config_defaults(0); \endcode
 *
 * To configure, enable PLL0 using the default parameters and to disable
 * a specific feature like Wide Bandwidth Mode (a UC3A3-specific
 * PLL option.), you can use this initialization process.
 * \code
	struct pll_config pllcfg;
	if (pll_is_locked(pll_id)) {
		return; // Pll already running
	}
	pll_enable_source(CONFIG_PLL0_SOURCE);
	pll_config_defaults(&pllcfg, 0);
	pll_config_set_option(&pllcfg, PLL_OPT_WBM_DISABLE);
	pll_enable(&pllcfg, 0);
	pll_wait_for_lock(0); \endcode
 *
 * When the last function call returns, PLL0 is ready to be used as the
 * main system clock source.
 *
 * \section pll_group_config Configuration Symbols
 *
 * Each PLL has a set of default parameters determined by the following
 * configuration symbols in the application's configuration file:
 *   - \b CONFIG_PLLn_SOURCE: The default clock source connected to the
 *     input of PLL \a n. Must be one of the values defined by the
 *     #pll_source enum.
 *   - \b CONFIG_PLLn_MUL: The default multiplier (loop divider) of PLL
 *     \a n.
 *   - \b CONFIG_PLLn_DIV: The default input divider of PLL \a n.
 *
 * These configuration symbols determine the result of calling
 * pll_config_defaults() and pll_get_default_rate().
 *
 * @{
 */

//! \name Chip-specific PLL characteristics
//@{
/**
 * \def PLL_MAX_STARTUP_CYCLES
 * \brief Maximum PLL startup time in number of slow clock cycles
 */
/**
 * \def NR_PLLS
 * \brief Number of on-chip PLLs
 */

/**
 * \def PLL_MIN_HZ
 * \brief Minimum frequency that the PLL can generate
 */
/**
 * \def PLL_MAX_HZ
 * \brief Maximum frequency that the PLL can generate
 */
/**
 * \def PLL_NR_OPTIONS
 * \brief Number of PLL option bits
 */
//@}

/**
 * \enum pll_source
 * \brief PLL clock source
 */

//! \name PLL configuration
//@{

/**
 * \struct pll_config
 * \brief Hardware-specific representation of PLL configuration.
 *
 * This structure contains one or more device-specific values
 * representing the current PLL configuration. The contents of this
 * structure is typically different from platform to platform, and the
 * user should not access any fields except through the PLL
 * configuration API.
 */

/**
 * \fn void pll_config_init(struct pll_config *cfg,
 *              enum pll_source src, unsigned int div, unsigned int mul)
 * \brief Initialize PLL configuration from standard parameters.
 *
 * \note This function may be defined inline because it is assumed to be
 * called very few times, and usually with constant parameters. Inlining
 * it will in such cases reduce the code size significantly.
 *
 * \param cfg The PLL configuration to be initialized.
 * \param src The oscillator to be used as input to the PLL.
 * \param div PLL input divider.
 * \param mul PLL loop divider (i.e. multiplier).
 *
 * \return A configuration which will make the PLL run at
 * (\a mul / \a div) times the frequency of \a src
 */
/**
 * \def pll_config_defaults(cfg, pll_id)
 * \brief Initialize PLL configuration using default parameters.
 *
 * After this function returns, \a cfg will contain a configuration
 * which will make the PLL run at (CONFIG_PLLx_MUL / CONFIG_PLLx_DIV)
 * times the frequency of CONFIG_PLLx_SOURCE.
 *
 * \param cfg The PLL configuration to be initialized.
 * \param pll_id Use defaults for this PLL.
 */
/**
 * \def pll_get_default_rate(pll_id)
 * \brief Get the default rate in Hz of \a pll_id
 */
/**
 * \fn void pll_config_set_option(struct pll_config *cfg,
 *              unsigned int option)
 * \brief Set the PLL option bit \a option in the configuration \a cfg.
 *
 * \param cfg The PLL configuration to be changed.
 * \param option The PLL option bit to be set.
 */
/**
 * \fn void pll_config_clear_option(struct pll_config *cfg,
 *              unsigned int option)
 * \brief Clear the PLL option bit \a option in the configuration \a cfg.
 *
 * \param cfg The PLL configuration to be changed.
 * \param option The PLL option bit to be cleared.
 */
/**
 * \fn void pll_config_read(struct pll_config *cfg, unsigned int pll_id)
 * \brief Read the currently active configuration of \a pll_id.
 *
 * \param cfg The configuration object into which to store the currently
 * active configuration.
 * \param pll_id The ID of the PLL to be accessed.
 */
/**
 * \fn void pll_config_write(const struct pll_config *cfg,
 *              unsigned int pll_id)
 * \brief Activate the configuration \a cfg on \a pll_id
 *
 * \param cfg The configuration object representing the PLL
 * configuration to be activated.
 * \param pll_id The ID of the PLL to be updated.
 */

//@}

//! \name Interaction with the PLL hardware
//@{
/**
 * \fn void pll_enable(const struct pll_config *cfg,
 *              unsigned int pll_id)
 * \brief Activate the configuration \a cfg and enable PLL \a pll_id.
 *
 * \param cfg The PLL configuration to be activated.
 * \param pll_id The ID of the PLL to be enabled.
 */
/**
 * \fn void pll_disable(unsigned int pll_id)
 * \brief Disable the PLL identified by \a pll_id.
 *
 * After this function is called, the PLL identified by \a pll_id will
 * be disabled. The PLL configuration stored in hardware may be affected
 * by this, so if the caller needs to restore the same configuration
 * later, it should either do a pll_config_read() before disabling the
 * PLL, or remember the last configuration written to the PLL.
 *
 * \param pll_id The ID of the PLL to be disabled.
 */
/**
 * \fn bool pll_is_locked(unsigned int pll_id)
 * \brief Determine whether the PLL is locked or not.
 *
 * \param pll_id The ID of the PLL to check.
 *
 * \retval true The PLL is locked and ready to use as a clock source
 * \retval false The PLL is not yet locked, or has not been enabled.
 */
/**
 * \fn void pll_enable_source(enum pll_source src)
 * \brief Enable the source of the pll.
 * The source is enabled, if the source is not already running.
 *
 * \param src The ID of the PLL source to enable.
 */
/**
 * \fn void pll_enable_config_defaults(unsigned int pll_id)
 * \brief Enable the pll with the default configuration.
 * PLL is enabled, if the PLL is not already locked.
 *
 * \param pll_id The ID of the PLL to enable.
 */

/**
 * \brief Wait for PLL \a pll_id to become locked
 *
 * \todo Use a timeout to avoid waiting forever and hanging the system
 *
 * \param pll_id The ID of the PLL to wait for.
 *
 * \retval STATUS_OK The PLL is now locked.
 * \retval ERR_TIMEOUT Timed out waiting for PLL to become locked.
 */
static inline int pll_wait_for_lock(unsigned int pll_id)
{
	assert(pll_id < NR_PLLS);

	while (!pll_is_locked(pll_id)) {
		/* Do nothing */
	}

	return 0;
}

//@}
//! @}

#endif /* CLK_PLL_H_INCLUDED */
