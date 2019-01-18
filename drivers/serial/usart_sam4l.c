/*
 * Copyright (c) 2019 Madani Lainani
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <soc.h>
#include <uart.h>

/* The write protect key value. */
#ifndef US_WPMR_WPKEY_PASSWD
#define US_WPMR_WPKEY_PASSWD    US_WPMR_WPKEY(0x555341U)
#endif

#ifndef US_WPMR_WPKEY_PASSWD
#  define US_WPMR_WPKEY_PASSWD US_WPMR_WPKEY(US_WPKEY_VALUE)
#endif

/* The CD value scope programmed in MR register. */
#define MIN_CD_VALUE                  0x01
#define MIN_CD_VALUE_SPI              0x04
#define MAX_CD_VALUE                  US_BRGR_CD_Msk

/* The receiver sampling divide of baudrate clock. */
#define HIGH_FRQ_SAMPLE_DIV           16
#define LOW_FRQ_SAMPLE_DIV            8

/* Input parameters when initializing RS232 and similar modes. */
typedef struct {
	/* Set baud rate of the USART (unused in slave modes). */
	uint32_t baudrate;

	/*
	 * Number of bits, which should be one of the following: US_MR_CHRL_5_BIT,
	 * US_MR_CHRL_6_BIT, US_MR_CHRL_7_BIT, US_MR_CHRL_8_BIT or
	 * US_MR_MODE9.
	 */
	uint32_t char_length;

	/*
	 * Parity type, which should be one of the following: US_MR_PAR_EVEN,
	 * US_MR_PAR_ODD, US_MR_PAR_SPACE, US_MR_PAR_MARK, US_MR_PAR_NO
	 * or US_MR_PAR_MULTIDROP.
	 */
	uint32_t parity_type;

	/*
	 * Number of stop bits between two characters: US_MR_NBSTOP_1_BIT,
	 * US_MR_NBSTOP_1_5_BIT, US_MR_NBSTOP_2_BIT.
	 * \note US_MR_NBSTOP_1_5_BIT is supported in asynchronous modes only.
	 */
	uint32_t stop_bits;

	/*
	 * Run the channel in test mode, which should be one of following:
	 * US_MR_CHMODE_NORMAL, US_MR_CHMODE_AUTOMATIC,
	 * US_MR_CHMODE_LOCAL_LOOPBACK, US_MR_CHMODE_REMOTE_LOOPBACK.
	 */
	uint32_t channel_mode;

	/* Filter of IrDA mode, useless in other modes. */
	uint32_t irda_filter;
} sam_usart_opt_t;

struct usart_sam4l_dev_cfg {
	Usart *usart;
};

struct usart_sam4l_dev_data {
};

extern void sysclk_enable_peripheral_clock(const volatile void *module);
extern uint32_t sysclk_get_peripheral_bus_hz(const volatile void *module);

#define DEV_CFG(dev) \
	((const struct usart_sam4l_dev_cfg *const)(dev)->config->config_info)

#define DEV_DATA(dev) ((struct usart_sam4l_dev_data * const)(dev)->driver_data)

/**
 * \brief Disable write protect of USART registers.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_disable_writeprotect(Usart *p_usart)
{
	p_usart->reg.US_WPMR = US_WPMR_WPKEY_PASSWD;
}

/**
 * \brief Immediately stop and disable USART receiver.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_reset_rx(Usart *p_usart)
{
	/* Reset Receiver */
	p_usart->reg.US_CR = US_CR_RSTRX | US_CR_RXDIS;
}

/**
 * \brief Immediately stop and disable USART transmitter.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_reset_tx(Usart *p_usart)
{
	/* Reset transmitter */
	p_usart->reg.US_CR = US_CR_RSTTX | US_CR_TXDIS;
}

/**
 * \brief Reset status bits (PARE, OVER, MANERR, UNRE and PXBRK in US_CSR).
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_reset_status(Usart *p_usart)
{
	p_usart->reg.US_CR = US_CR_RSTSTA;
}

/**
 * \brief Drive the pin RTS to 1.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_drive_RTS_pin_high(Usart *p_usart)
{
	p_usart->reg.US_CR = US_CR_RTSDIS;
}

void usart_drive_DTR_pin_high(Usart *p_usart)
{
	p_usart->reg.US_CR = US_CR_DTRDIS;
}

/**
 * \brief Reset the USART and disable TX and RX.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_reset(Usart *p_usart)
{
	/* Disable the Write Protect. */
	usart_disable_writeprotect(p_usart);

	/* Reset registers that could cause unpredictable behavior after reset. */
	p_usart->reg.US_MR = 0;
	p_usart->reg.US_RTOR = 0;
	p_usart->reg.US_TTGR = 0;

	/* Disable TX and RX. */
	usart_reset_tx(p_usart);
	usart_reset_rx(p_usart);
	/* Reset status bits. */
	usart_reset_status(p_usart);
	/* Turn off RTS and DTR if exist. */
	usart_drive_RTS_pin_high(p_usart);
#if (SAM3S || SAM4S || SAM3U || SAM4L || SAM4E)
	usart_drive_DTR_pin_high(p_usart);
#endif
}

/**
 * \brief Calculate a clock divider(CD) and a fractional part (FP) for the
 * USART asynchronous modes to generate a baudrate as close as possible to
 * the baudrate set point.
 *
 * \note Baud rate calculation: Baudrate = ul_mck/(Over * (CD + FP/8))
 * (Over being 16 or 8). The maximal oversampling is selected if it allows to
 * generate a baudrate close to the set point.
 *
 * \param p_usart Pointer to a USART instance.
 * \param baudrate Baud rate set point.
 * \param ul_mck USART module input clock frequency.
 *
 * \retval 0 Baud rate is successfully initialized.
 * \retval 1 Baud rate set point is out of range for the given input clock
 * frequency.
 */
uint32_t usart_set_async_baudrate(Usart *p_usart,
		uint32_t baudrate, uint32_t ul_mck)
{
	uint32_t over;
	uint32_t cd_fp;
	uint32_t cd;
	uint32_t fp;

	/* Calculate the receiver sampling divide of baudrate clock. */
	if (ul_mck >= HIGH_FRQ_SAMPLE_DIV * baudrate) {
		over = HIGH_FRQ_SAMPLE_DIV;
	} else {
		over = LOW_FRQ_SAMPLE_DIV;
	}

	/* Calculate clock divider according to the fraction calculated formula. */
	cd_fp = (8 * ul_mck + (over * baudrate) / 2) / (over * baudrate);
	cd = cd_fp >> 3;
	fp = cd_fp & 0x07;
	if (cd < MIN_CD_VALUE || cd > MAX_CD_VALUE) {
		return 1;
	}

	/* Configure the OVER bit in MR register. */
	if (over == 8) {
		p_usart->reg.US_MR |= US_MR_OVER;
	}

	/* Configure the baudrate generate register. */
	p_usart->reg.US_BRGR = (cd << US_BRGR_CD_Pos) | (fp << US_BRGR_FP_Pos);

	return 0;
}

/**
 * \brief Configure USART to work in RS232 mode.
 *
 * \note By default, the transmitter and receiver aren't enabled.
 *
 * \param p_usart Pointer to a USART instance.
 * \param p_usart_opt Pointer to sam_usart_opt_t instance.
 * \param ul_mck USART module input clock frequency.
 *
 * \retval 0 on success.
 * \retval 1 on failure.
 */
uint32_t usart_init_rs232(Usart *p_usart,
			  const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck)
{
	static uint32_t ul_reg_val;

	/* Reset the USART and shut down TX and RX. */
	usart_reset(p_usart);

	ul_reg_val = 0;
	/* Check whether the input values are legal. */
	if (!p_usart_opt || usart_set_async_baudrate(p_usart,
			p_usart_opt->baudrate, ul_mck)) {
		return 1;
	}

	/* Configure the USART option. */
	ul_reg_val |= p_usart_opt->char_length | p_usart_opt->parity_type |
			p_usart_opt->channel_mode | p_usart_opt->stop_bits;

	/* Configure the USART mode as normal mode. */
	ul_reg_val |= US_MR_USART_MODE_NORMAL;

	p_usart->reg.US_MR |= ul_reg_val;

	return 0;
}

/**
 * \brief Enable USART transmitter.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_enable_tx(Usart *p_usart)
{
	p_usart->reg.US_CR = US_CR_TXEN;
}

/**
 * \brief Enable USART receiver.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_enable_rx(Usart *p_usart)
{
	p_usart->reg.US_CR = US_CR_RXEN;
}

static int usart_sam4l_init(struct device *dev)
{
	Usart *usart = DEV_CFG(dev)->usart;
	sam_usart_opt_t usart_settings;

	sysclk_enable_peripheral_clock(usart);

	/* Configure USART */
	usart_init_rs232(usart, &usart_settings,
			 sysclk_get_peripheral_bus_hz(usart));

	/* Enable the receiver and transmitter. */
	usart_enable_tx(usart);
	usart_enable_rx(usart);

	return 0;
}

static int usart_sam4l_poll_in(struct device *dev, unsigned char *p_char)
{
	return 0;
}

static void usart_sam4l_poll_out(struct device *dev, unsigned char out_char)
{
}

static const struct uart_driver_api usart_sam4l_driver_api = {
	.poll_in = usart_sam4l_poll_in,
	.poll_out = usart_sam4l_poll_out,
};

#define USART_SAM4L_CONFIG_DEFN(n)					       \
static const struct usart_sam4l_dev_cfg usart_sam4l_config_##n = {	       \
}									       \

#define USART_SAM4L_DEVICE_INIT(n)					       \
static struct usart_sam4l_dev_data usart_sam4l_data_##n;		       \
USART_SAM4L_CONFIG_DEFN(n);						       \
DEVICE_AND_API_INIT(usart_sam4l_##n, DT_SAM4L_USART##n##_LABEL,		       \
                    usart_sam4l_init, &usart_sam4l_data_##n,		       \
                    &usart_sam4l_config_##n, PRE_KERNEL_1,		       \
                    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			       \
                    &usart_sam4l_driver_api);				       \

#if DT_SAM4L_USART0_BASE_ADDRESS
USART_SAM4L_DEVICE_INIT(0)
#endif

#if DT_SAM4L_USART1_BASE_ADDRESS
USART_SAM4L_DEVICE_INIT(1)
#endif

#if DT_SAM4L_USART2_BASE_ADDRESS
USART_SAM4L_DEVICE_INIT(2)
#endif

#if DT_SAM4L_USART3_BASE_ADDRESS
USART_SAM4L_DEVICE_INIT(3)
#endif
