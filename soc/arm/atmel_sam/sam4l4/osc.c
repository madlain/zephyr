/**
 * \file
 *
 * \brief Chip-specific oscillator management functions
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
#include <osc.h>

#include <irq.h>

#define cpu_irq_save() irq_lock()
#define cpu_irq_restore(flags) irq_unlock(flags)

typedef u32_t irqflags_t;

#ifdef BOARD_OSC0_HZ
void osc_priv_enable_osc0(void)
{
	irqflags_t flags;

	flags = cpu_irq_save();
	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_OSCCTRL0 - (uint32_t)SCIF);
	SCIF->reg.SCIF_OSCCTRL0 =
			OSC0_STARTUP_VALUE
# if BOARD_OSC0_IS_XTAL == true
			| OSC0_GAIN_VALUE
#endif
			| OSC0_MODE_VALUE
			| SCIF_OSCCTRL0_OSCEN;
	cpu_irq_restore(flags);
}

void osc_priv_disable_osc0(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	temp = SCIF->reg.SCIF_OSCCTRL0;
	temp &= ~SCIF_OSCCTRL0_OSCEN;
	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_OSCCTRL0 - (uint32_t)SCIF);
	SCIF->reg.SCIF_OSCCTRL0 = temp;
	cpu_irq_restore(flags);
}
#endif /* BOARD_OSC0_HZ */

#ifdef BOARD_OSC32_HZ
void osc_priv_enable_osc32(void)
{
	irqflags_t flags;

	flags = cpu_irq_save();
	BSCIF->reg.BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
		| BSCIF_UNLOCK_ADDR((uint32_t)&BSCIF->reg.BSCIF_OSCCTRL32 - (uint32_t)BSCIF);
	BSCIF->reg.BSCIF_OSCCTRL32 =
			OSC32_STARTUP_VALUE
			| BOARD_OSC32_SELCURR
			| OSC32_MODE_VALUE
			| BSCIF_OSCCTRL32_EN1K
			| BSCIF_OSCCTRL32_EN32K
			| BSCIF_OSCCTRL32_OSC32EN;
	cpu_irq_restore(flags);
}

void osc_priv_disable_osc32(void)
{
	irqflags_t flags;

	flags = cpu_irq_save();
	BSCIF->reg.BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
		| BSCIF_UNLOCK_ADDR((uint32_t)&BSCIF->reg.BSCIF_OSCCTRL32 - (uint32_t)BSCIF);
	BSCIF->reg.BSCIF_OSCCTRL32 &= ~BSCIF_OSCCTRL32_OSC32EN;
	// Wait until OSC32 RDY flag is cleared.
	while (BSCIF->reg.BSCIF_PCLKSR & BSCIF_PCLKSR_OSC32RDY);
	cpu_irq_restore(flags);
}
#endif /* BOARD_OSC32_HZ */

void osc_priv_enable_rc32k(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	temp = BSCIF->reg.BSCIF_RC32KCR;
	BSCIF->reg.BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
		| BSCIF_UNLOCK_ADDR((uint32_t)&BSCIF->reg.BSCIF_RC32KCR - (uint32_t)BSCIF);
	BSCIF->reg.BSCIF_RC32KCR = temp | BSCIF_RC32KCR_EN32K | BSCIF_RC32KCR_EN;
	cpu_irq_restore(flags);
}

void osc_priv_disable_rc32k(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	temp = BSCIF->reg.BSCIF_RC32KCR;
	temp &= ~BSCIF_RC32KCR_EN;
	BSCIF->reg.BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
		| BSCIF_UNLOCK_ADDR((uint32_t)&BSCIF->reg.BSCIF_RC32KCR - (uint32_t)BSCIF);
	BSCIF->reg.BSCIF_RC32KCR = temp;
	cpu_irq_restore(flags);
}

void osc_priv_enable_rc1m(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	temp = BSCIF->reg.BSCIF_RC1MCR;
	BSCIF->reg.BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
		| BSCIF_UNLOCK_ADDR((uint32_t)&BSCIF->reg.BSCIF_RC1MCR - (uint32_t)BSCIF);
	BSCIF->reg.BSCIF_RC1MCR = temp | BSCIF_RC1MCR_CLKOE;
	cpu_irq_restore(flags);
}

void osc_priv_disable_rc1m(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	temp = BSCIF->reg.BSCIF_RC1MCR;
	temp &= ~BSCIF_RC1MCR_CLKOE;
	BSCIF->reg.BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
		| BSCIF_UNLOCK_ADDR((uint32_t)&BSCIF->reg.BSCIF_RC1MCR - (uint32_t)BSCIF);
	BSCIF->reg.BSCIF_RC1MCR = temp;
	cpu_irq_restore(flags);
}

void osc_priv_enable_rc80m(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	temp = SCIF->reg.SCIF_RC80MCR;
	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_RC80MCR - (uint32_t)SCIF);
	SCIF->reg.SCIF_RC80MCR = temp | SCIF_RC80MCR_EN;
	cpu_irq_restore(flags);
}

void osc_priv_disable_rc80m(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	temp = SCIF->reg.SCIF_RC80MCR;
	temp &= ~SCIF_RC80MCR_EN ;
	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_RC80MCR - (uint32_t)SCIF);
	SCIF->reg.SCIF_RC80MCR = temp;
	cpu_irq_restore(flags);
}

void osc_priv_enable_rcfast(void)
{
	irqflags_t flags;
	uint32_t temp;

	flags = cpu_irq_save();
	// Let FCD and calibration value by default
	temp = SCIF->reg.SCIF_RCFASTCFG;
	// Clear previous FRANGE value
	temp &= ~SCIF_RCFASTCFG_FRANGE_Msk;

	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_RCFASTCFG - (uint32_t)SCIF);
	SCIF->reg.SCIF_RCFASTCFG = temp | SCIF_RCFASTCFG_EN
		| SCIF_RCFASTCFG_FRANGE(CONFIG_RCFAST_FRANGE);
	cpu_irq_restore(flags);
}

void osc_priv_disable_rcfast(void)
{
	irqflags_t flags;
	uint32_t temp;
	flags = cpu_irq_save();
	// Let FCD and calibration value by default
	temp = SCIF->reg.SCIF_RCFASTCFG;
	// Clear previous FRANGE value
	temp &= ~SCIF_RCFASTCFG_FRANGE_Msk;
	// Disalbe RCFAST
	temp &= ~SCIF_RCFASTCFG_EN;
	SCIF->reg.SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
		| SCIF_UNLOCK_ADDR((uint32_t)&SCIF->reg.SCIF_RCFASTCFG - (uint32_t)SCIF);
	SCIF->reg.SCIF_RCFASTCFG = temp;
	cpu_irq_restore(flags);
}

