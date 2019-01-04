/**
 * \file
 *
 * \brief DFLL management
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
#ifndef CLK_DFLL_H_INCLUDED
#define CLK_DFLL_H_INCLUDED

#include <soc.h>

#include "conf_clock.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \weakgroup dfll_group
 * @{
 */

#define NR_DFLLS          1
#define DFLL_MIN_HZ       20000000UL
#define DFLL_MAX_HZ       150000000UL
#define DFLL_MIN_KHZ      (DFLL_MIN_HZ / 1000)
#define DFLL_MAX_KHZ      (DFLL_MAX_HZ / 1000)

#define DFLL_COARSE_MAX   (255)
#define DFLL_FINE_MAX     (511)
#define DFLL_FINE_HALF    (255)

#define DFLL_CALIB_VALUE  (0x0B)

#define DFLL_RANGE0       (0)
#define DFLL_RANGE1       (1)
#define DFLL_RANGE2       (2)
#define DFLL_RANGE3       (3)
#define DFLL_MAX_RANGE1   (110000000)
#define DFLL_MAX_RANGE2   (55000000)
#define DFLL_MAX_RANGE3   (30000000)

#ifndef __ASSEMBLY__

#include <genclk.h>
#include <osc.h>

typedef enum genclk_source dfll_refclk_t;

struct dfll_config {
	struct genclk_config    ref_cfg;        //!< Reference clock
	uint32_t                conf;           //!< DFLLnCONF
	uint32_t                mul;            //!< DFLLnMUL
	uint32_t                step;           //!< DFLLnSTEP
	uint32_t                ssg;            //!< DFLLnSSG
	uint32_t                val;            //!< DFLLnVAL
	uint8_t                 freq_range;     //!< Frequency Range
};

static inline void dfll_config_set_max_step(struct dfll_config *cfg,
		uint16_t coarse, uint16_t fine)
{
	cfg->step = (SCIF_DFLL0STEP_CSTEP(coarse)
			| SCIF_DFLL0STEP_FSTEP(fine));
}

static inline void dfll_priv_set_frequency_range(struct dfll_config *cfg,
		uint32_t freq)
{
	if (freq < DFLL_MAX_RANGE3){
		cfg->freq_range = DFLL_RANGE3;
	}
	else if (freq < DFLL_MAX_RANGE2){
		cfg->freq_range = DFLL_RANGE2;
	}
	else if (freq < DFLL_MAX_RANGE1){
		cfg->freq_range = DFLL_RANGE1;
	}
	else {
		cfg->freq_range = DFLL_RANGE0;
	}
	cfg->conf &= ~SCIF_DFLL0CONF_RANGE_Msk;
	cfg->conf |=SCIF_DFLL0CONF_RANGE(cfg->freq_range);
}

static inline void dfll_config_init_open_loop_mode(struct dfll_config *cfg)
{
	genclk_config_defaults(&cfg->ref_cfg, 0);
	// Do a sync before reading a dfll conf register
	SCIF->reg.SCIF_DFLL0SYNC = SCIF_DFLL0SYNC_SYNC;
	while (!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY));
	cfg->conf = SCIF->reg.SCIF_DFLL0CONF;
	// Select Open Loop Mode
	cfg->conf &= ~SCIF_DFLL0CONF_MODE;
	// Clear DFLL Frequency Range
	cfg->freq_range = 0;
	cfg->conf &= ~SCIF_DFLL0CONF_RANGE_Msk;
	cfg->conf |= SCIF_DFLL0CONF_RANGE(cfg->freq_range);

	cfg->mul = 0;
	cfg->step = 0;
	cfg->ssg = 0;
	cfg->val = 0;
}

#ifdef CONFIG_DFLL0_FREQ
static inline void dfll_config_init_closed_loop_mode(struct dfll_config *cfg,
		dfll_refclk_t refclk, uint16_t divide, uint16_t mul)
{
	/*
	 * Set up generic clock source with specified reference clock
	 * and divider.
	 */
	genclk_config_defaults(&cfg->ref_cfg, 0);
	genclk_config_set_source(&cfg->ref_cfg, refclk);
	genclk_config_set_divider(&cfg->ref_cfg, divide);

	// Do a sync before reading a dfll conf register
	SCIF->reg.SCIF_DFLL0SYNC = SCIF_DFLL0SYNC_SYNC;
	while (!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY));
	cfg->conf = SCIF->reg.SCIF_DFLL0CONF;
	// Select Closed Loop Mode
	cfg->conf |= SCIF_DFLL0CONF_MODE;
	// Write DFLL Frequency Range
	dfll_priv_set_frequency_range(cfg, CONFIG_DFLL0_FREQ);

	cfg->mul = mul;
	cfg->val = 0;
	/*
	 * Initial step length of 4. If this is set too high, the DFLL
	 * may fail to lock.
	 */
	dfll_config_set_max_step(cfg, 4, 4);
	cfg->ssg = 0;
}
#endif

static inline uint32_t dfll_priv_get_source_hz(dfll_refclk_t src)
{
	/*
	 * Only handle the cases that actually make sense as a DFLL
	 * source. The DFLL itself is obviously not one of those cases.
	 */
	switch (src) {
	case GENCLK_SRC_RCSYS:
		return OSC_RCSYS_NOMINAL_HZ;

#ifdef BOARD_OSC32_HZ
	case GENCLK_SRC_OSC32K:
		return BOARD_OSC32_HZ;
#endif

#ifdef BOARD_OSC0_HZ
	case GENCLK_SRC_OSC0:
		return BOARD_OSC0_HZ;
#endif

	case GENCLK_SRC_RC80M:
		return OSC_RC80M_NOMINAL_HZ;

	case GENCLK_SRC_RC32K:
		return OSC_RC32K_NOMINAL_HZ;

	default:
		/* unhandled_case(src) */
		return 0;
	}
}

#define dfll_config_defaults(cfg, dfll_id)                                     \
	dfll_config_init_closed_loop_mode(cfg,                                 \
			CONFIG_DFLL##dfll_id##_SOURCE,                         \
			CONFIG_DFLL##dfll_id##_DIV,                            \
			CONFIG_DFLL##dfll_id##_MUL)

#define dfll_get_default_rate(dfll_id)                                         \
	((dfll_priv_get_source_hz(CONFIG_DFLL##dfll_id##_SOURCE)               \
			* CONFIG_DFLL##dfll_id##_MUL)                          \
		/ CONFIG_DFLL##dfll_id##_DIV)

static inline void dfll_config_set_initial_tuning(struct dfll_config *cfg,
		uint16_t coarse, uint16_t fine)
{
	cfg->val = (SCIF_DFLL0VAL_COARSE(coarse)
			| SCIF_DFLL0VAL_FINE(fine));
}

/**
 * \brief Tune the DFLL configuration for a specific target frequency
 *
 * This will set the initial coarse and fine DFLL tuning to match the
 * given target frequency. In open loop mode, this will cause the DFLL
 * to run close to the specified frequency, though it may not match
 * exactly. In closed loop mode, the DFLL will automatically tune itself
 * to the target frequency regardless of the initial tuning, but this
 * function may be used to set a starting point close to the desired
 * frequency in order to reduce the startup time.
 *
 * \par Calculating the DFLL frequency
 *
 * \f{eqnarray*}{
	f_{DFLL} &=& \left[f_{min} + \left(f_{max} - f_{min}\right)
		\frac{\mathrm{COARSE}}{\mathrm{COARSE}_{max}}\right]
		\left(1 + x \frac{\mathrm{FINE}
			- \mathrm{FINE}_{half}}{\mathrm{FINE}_{max}}\right)
		= f_{coarse} \left(1 + x
		\frac{\mathrm{FINE}
			- \mathrm{FINE}_{half}}{\mathrm{FINE}_{max}}\right) \\
	\mathrm{COARSE} &=& \frac{\left(f_{DFLL} - f_{min}\right)}
		{f_{max} - f_{min}} \mathrm{COARSE}_{max} \\
	f_{coarse} &=& f_{min} + \left(f_{max} - f_{min}\right)
		\frac{\mathrm{COARSE}}{\mathrm{COARSE}_{max}} \\
	\mathrm{FINE} &=& \left(10 \frac{f_{DFLL} - f_{coarse}}
		{f_{coarse}} + \mathrm{FINE}_{half}\right) / 4
   \f}
 *
 * \param cfg The DFLL configuration to be tuned.
 * \param target_hz Target frequency in Hz.
 */
static inline void dfll_config_tune_for_target_hz(struct dfll_config *cfg,
		uint32_t target_hz)
{
	uint32_t target_khz;
	uint32_t coarse_khz;
	uint32_t delta_khz;
	uint32_t coarse;
	uint32_t fine;

	target_khz = target_hz / 1000;
	coarse = ((target_khz - DFLL_MIN_KHZ) * DFLL_COARSE_MAX)
			/ (DFLL_MAX_KHZ - DFLL_MIN_KHZ);
	coarse_khz = DFLL_MIN_KHZ + (((DFLL_MAX_KHZ - DFLL_MIN_KHZ)
			/ DFLL_COARSE_MAX) * coarse);
	delta_khz = target_khz - coarse_khz;
	fine = (((delta_khz * DFLL_FINE_MAX) * 2) / coarse_khz) * 5;
	fine += DFLL_FINE_HALF;
	fine /= 4;

	dfll_config_set_initial_tuning(cfg, coarse, fine);
	dfll_priv_set_frequency_range(cfg, target_hz);
}

static inline void dfll_config_enable_ssg(struct dfll_config *cfg,
		uint16_t amplitude, uint16_t step_size)
{
	cfg->ssg = (SCIF_DFLL0SSG_EN
			| SCIF_DFLL0SSG_AMPLITUDE(amplitude)
			| SCIF_DFLL0SSG_STEPSIZE(step_size));
}

static inline void dfll_config_disable_ssg(struct dfll_config *cfg)
{
	cfg->ssg = 0;
}

extern void dfll_enable_open_loop(const struct dfll_config *cfg,
		uint32_t dfll_id);
extern void dfll_disable_open_loop(uint32_t dfll_id);
extern void dfll_enable_closed_loop(const struct dfll_config *cfg,
		uint32_t dfll_id);
extern void dfll_disable_closed_loop(uint32_t dfll_id);
#ifndef CHIP_GENCLK_H_INCLUDED
// This function already has a prototype in genclk.h.
extern void dfll_enable_config_defaults(uint32_t dfll_id);
#endif

static inline bool dfll_is_coarse_locked(uint32_t dfll_id)
{
	ARG_UNUSED(dfll_id);
	return !!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0LOCKC);
}

static inline bool dfll_is_fine_locked(uint32_t dfll_id)
{
	ARG_UNUSED(dfll_id);
	return !!(SCIF->reg.SCIF_PCLKSR & SCIF_PCLKSR_DFLL0LOCKF);
}

static inline bool dfll_is_accurate_locked(uint32_t dfll_id)
{
	ARG_UNUSED(dfll_id);

	return (dfll_is_coarse_locked(dfll_id) &&
			dfll_is_fine_locked(dfll_id));
}

static inline void dfll_enable_source(dfll_refclk_t src)
{
	switch (src) {
	case GENCLK_SRC_RCSYS:
		/* Nothing to do */
		break;

#ifdef BOARD_OSC32_HZ
	case GENCLK_SRC_OSC32K:
		if (!osc_is_ready(OSC_ID_OSC32)) {
			osc_enable(OSC_ID_OSC32);
			osc_wait_ready(OSC_ID_OSC32);
		}
		break;
#endif

#ifdef BOARD_OSC0_HZ
	case GENCLK_SRC_OSC0:
		if (!osc_is_ready(OSC_ID_OSC0)) {
			osc_enable(OSC_ID_OSC0);
			osc_wait_ready(OSC_ID_OSC0);
		}
		break;
#endif

	case GENCLK_SRC_RC80M:
		if (!osc_is_ready(OSC_ID_RC80M)) {
			osc_enable(OSC_ID_RC80M);
			osc_wait_ready(OSC_ID_RC80M);
		}
		break;

	case GENCLK_SRC_RC32K:
		if (!osc_is_ready(OSC_ID_RC32K)) {
			osc_enable(OSC_ID_RC32K);
			osc_wait_ready(OSC_ID_RC32K);
		}
		break;

	default:
		assert(false);
		break;
	}
}

#endif /* __ASSEMBLY__ */

//! @}

#ifdef __cplusplus
}
#endif

/**
 * \ingroup clk_group
 * \defgroup dfll_group DFLL Management
 *
 * A Digital Frequency Locked Loop can be used to generate a highly
 * accurate frequency from a slower-running reference clock, in much the
 * same way as a PLL. DFLLs typically have shorter startup times and
 * less jitter. They can also be used in open-loop mode to generate a
 * less accurate frequency without the use of a reference clock.
 *
 * There may be significant variations between platforms in the support
 * for certain features.
 *
 * \par Example: Setting up DFLL0 with default parameters and dithering enabled
 *
 * The following example shows how to configure and enable DFLL0 in
 * closed-loop mode using the default parameters specified through
 * configuration symbols.
 * \code
	dfll_enable_config_defaults(0); \endcode
 *
 * To configure and enable DFLL0 in closed-loop mode using the default
 * parameters and to enable specific feature like dithering for better accuracy,
 * you can use this initialization process.
 * \code
	struct dfll_config dfllcfg;

	dfll_enable_source(CONFIG_DFLL0_SOURCE);
	dfll_config_defaults(&dfllcfg, 0);
	dfll_config_enable_dithering(&dfllcfg);
	dfll_enable(&dfllcfg, 0);
	dfll_wait_for_accurate_lock(0); \endcode
 *
 * When the last function call returns, DFLL0 is running at a frequency
 * which matches the default configuration as accurately as possible.
 * Any additional alterations to the default configuration can be added
 * at the same place as the call to dfll_config_enable_dithering(), but
 * note that the DFLL will never achieve "accurate" lock if dithering is
 * disabled.
 *
 * @{
 */

//! \name Chip-specific DFLL characteristics
//@{
/**
 * \def NR_DFLLS
 * \brief Number of on-chip DFLLs.
 */
/**
 * \def DFLL_MIN_HZ
 * \brief Minimum frequency that the DFLL can generate.
 */
/**
 * \def DFLL_MAX_HZ
 * \brief Maximum frequency that the DFLL can generate.
 */
//@}

/**
 * \typedef dfll_refclk_t
 * \brief Type used for identifying a reference clock source for the DFLL.
 */

//! \name DFLL Configuration
//@{

/**
 * \struct dfll_config
 * \brief Hardware-specific representation of DFLL configuration.
 *
 * This structure contains one or more device-specific values
 * representing the current DFLL configuration. The contents of this
 * structure is typically different from platform to platform, and the
 * user should not access any fields except through the DFLL
 * configuration API.
 */

/**
 * \fn void dfll_config_init_open_loop_mode(struct dfll_config *cfg)
 * \brief Configure the DFLL configuration \a cfg for open-loop mode.
 *
 * \param cfg The DFLL configuration to be initialized.
 */
/**
 * \fn void dfll_config_init_closed_loop_mode(struct dfll_config *cfg,
 *              dfll_refclk_t refclk, uint16_t div, uint16_t mul)
 * \brief Configure the DFLL configuration \a cfg for closed-loop mode.
 *
 * \param cfg The DFLL configuration to be initialized.
 * \param refclk The reference clock source.
 * \param div Reference clock divider.
 * \param mul Multiplier (integer part only).
 */
/**
 * \def dfll_config_defaults(cfg, dfll_id)
 * \brief Initialize DFLL configuration using default parameters.
 *
 * After this function returns, \a cfg will contain a configuration
 * which will make the DFLL run at (CONFIG_DFLLx_MUL / CONFIG_DFLLx_DIV)
 * times the frequency of CONFIG_DFLLx_SOURCE. The default configuration
 * will always use closed-loop mode with no fractional multiplier.
 *
 * \param cfg The DFLL configuration to be initialized.
 * \param dfll_id Use defaults for this DFLL.
 */
/**
 * \def dfll_get_default_rate(dfll_id)
 * \brief Return the default rate in Hz of \a dfll_id.
 */

/**
 * \fn void dfll_config_set_fractional_multiplier(struct dfll_config *cfg,
 *              uint16_t mul_i, uint16_t mul_f)
 * \brief Set a fractional multiplier.
 *
 * This function has no effect in open-loop mode, and is only available
 * on devices which support fractional multipliers.
 *
 * The fractional part of the multiplier is assumed to be 16 bits. The
 * low-level driver will make sure to shift this value to match the
 * hardware if necessary.
 *
 * \param cfg The DFLL configuration to be modified.
 * \param mul_i Integer part of multiplier.
 * \param mul_f Fractional part of multiplier.
 */
/**
 * \fn void dfll_config_enable_dithering(struct dfll_config *cfg)
 * \brief Enable dithering for more accurate frequency generation.
 *
 * The fine LSB input to the VCO is dithered to achieve fractional
 * approximation to the correct multiplication ratio.
 *
 * \param cfg The DFLL configuration to be modified.
 */
/**
 * \fn void dfll_config_disable_dithering(struct dfll_config *cfg)
 * \brief Disable dithering.
 *
 * \see dfll_config_enable_dithering()
 *
 * \param cfg The DFLL configuration to be modified.
 */
/**
 * \fn void dfll_config_set_initial_tuning(struct dfll_config *cfg,
 *              uint16_t coarse, uint16_t fine)
 * \brief Set initial VCO tuning.
 *
 * In open loop mode, this will determine the frequency of the output.
 *
 * In closed loop mode, this will provide an initial estimate of the VCO
 * tuning. While the DFLL will automatically adjust these values to
 * match the desired output frequency, careful selection of initial
 * values might reduce the time to achieve coarse and fine lock.
 *
 * \param cfg The DFLL configuration to be modified.
 * \param coarse Coarse tuning of the frequency generator.
 * \param fine Fine tuning of the frequency generator.
 */
/**
 * \fn void dfll_config_set_max_step(struct dfll_config *cfg,
 *              uint16_t coarse, uint16_t fine)
 * \brief Set the maximum VCO tuning step size.
 *
 * This function has no effect in open-loop mode.
 *
 * By default, both of these values are set to 50% of their respective
 * maximums.  It is not recommended to set the values any higher than
 * this, but setting them lower might reduce the frequency overshoot at
 * the expense of longer time to achieve coarse and/or fine lock.
 *
 * \param cfg The DFLL configuration to be modified
 * \param coarse The maximum step size of the coarse VCO tuning.
 * \param fine The maximum step size of the fine VCO tuning.
 */
/**
 * \fn void dfll_config_enable_ssg(struct dfll_config *cfg,
 *              uint16_t amplitude, uint16_t step_size)
 * \brief Enable Spread Spectrum Generator.
 *
 * \param cfg The DFLL configuration to be modified.
 * \param amplitude The amplitude of the spread spectrum.
 * \param step_size The step size of the spread spectrum.
 */
/**
 * \fn void dfll_config_disable_ssg(struct dfll_config *cfg)
 * \brief Disable Spread Spectrum Generator.
 *
 * \param cfg The DFLL configuration to be modified.
 */
//@}

//! \name Interaction with the DFLL hardware
//@{
/**
 * \fn void dfll_enable_open_loop(const struct dfll_config *cfg,
 *              unsigned int dfll_id)
 * \brief Activate the configuration \a cfg and enable DFLL \a dfll_id
 * in open-loop mode.
 *
 * \pre The configuration in \a cfg must represent an open-loop
 * configuration.
 *
 * \param cfg The configuration to be activated.
 * \param dfll_id The ID of the DFLL to be enabled.
 */
/**
 * \fn void dfll_enable_closed_loop(const struct dfll_config *cfg,
 *              unsigned int dfll_id)
 * \brief Activate the configuration \a cfg and enable DFLL \a dfll_id
 * in closed-loop mode.
 *
 * \pre The configuration in \a cfg must represent a closed-loop
 * configuration.
 *
 * \param cfg The configuration to be activated.
 * \param dfll_id The ID of the DFLL to be enabled.
 */
/**
 * \fn void dfll_disable_open_loop(unsigned int dfll_id)
 * \brief Disable the DFLL identified by \a dfll_id.
 *
 * \pre The DFLL must have been enabled in open loop mode.
 *
 * \param dfll_id The ID of the DFLL to be disabled.
 */
/**
 * \fn void dfll_disable_closed_loop(unsigned int dfll_id)
 * \brief Disable the DFLL identified by \a dfll_id.
 *
 * \pre The DFLL must have been enabled in closed loop mode.
 *
 * \param dfll_id The ID of the DFLL to be disabled.
 */
/**
 * \fn bool dfll_is_coarse_locked(unsigned int dfll_id)
 * \brief Determine whether or not a DFLL has achieved coarse lock.
 *
 * \param dfll_id The ID of the DFLL to check.
 *
 * \retval true The DFLL has determined the final value of the coarse
 * VCO tuning value.
 * \retval false The DFLL has not yet determined the coarse VCO tuning
 * value, or has not been enabled.
 */
/**
 * \fn bool dfll_is_fine_locked(unsigned int dfll_id)
 * \brief Determine whether or not a DFLL has achieved fine lock.
 *
 * \param dfll_id The ID of the DFLL to check.
 *
 * \retval true The DFLL has determined the final value of the fine VCO
 * tuning value.
 * \retval false The DFLL has not yet determined the fine VCO tuning
 * value, or has not been enabled.
 */
/**
 * \fn bool dfll_is_accurate_locked(unsigned int dfll_id)
 * \brief Determine whether or not a DFLL has achieved accurate lock.
 *
 * \param dfll_id The ID of the DFLL to check.
 *
 * \retval true The DFLL has determined the final dithering duty cycle.
 * \retval false The DFLL has not yet determined the dithering duty
 * cycle, or has not been enabled with dithering enabled.
 */
/**
 * \fn void dfll_enable_source(enum dfll_refclk_t src)
 * \brief Enable the source of the dfll.
 * The source is enabled, if the source is not already running.
 *
 * \param dfll_source src The ID of the DFLL source to enable.
 */
/**
 * \fn void dfll_enable_config_defaults(unsigned int dfll_id)
 * \brief Enable the dfll with the default configuration.
 * DFLL is enabled, if the DFLL is not already locked.
 *
 * \param dfll_id The ID of the DFLL to enable.
 */

/**
 * \brief Wait for the DFLL identified by \a dfll_id to achieve coarse
 * lock.
 *
 * \param dfll_id The ID of the DFLL to wait for.
 *
 * \retval STATUS_OK The DFLL has achieved coarse lock.
 * \retval ERR_TIMEOUT Timed out waiting for lock.
 */
static inline int dfll_wait_for_coarse_lock(unsigned int dfll_id)
{
	/* TODO: Add timeout mechanism */
	while (!dfll_is_coarse_locked(dfll_id)) {
		/* Do nothing */
	}

	return 0;
}

/**
 * \brief Wait for the DFLL identified by \a dfll_id to achieve fine
 * lock.
 *
 * \param dfll_id The ID of the DFLL to wait for.
 *
 * \retval STATUS_OK The DFLL has achieved fine lock.
 * \retval ERR_TIMEOUT Timed out waiting for lock.
 */
static inline int dfll_wait_for_fine_lock(unsigned int dfll_id)
{
	/* TODO: Add timeout mechanism */
	while (!dfll_is_fine_locked(dfll_id)) {
		/* Do nothing */
	}

	return 0;
}

/**
 * \brief Wait for the DFLL identified by \a dfll_id to achieve accurate
 * lock.
 *
 * \param dfll_id The ID of the DFLL to wait for.
 *
 * \retval STATUS_OK The DFLL has achieved accurate lock.
 * \retval ERR_TIMEOUT Timed out waiting for lock.
 */
static inline int dfll_wait_for_accurate_lock(unsigned int dfll_id)
{
	/* TODO: Add timeout mechanism */
	while (!dfll_is_accurate_locked(dfll_id)) {
		/* Do nothing */
	}

	return 0;
}

//@}
//! @}

#endif /* CLK_DFLL_H_INCLUDED */
