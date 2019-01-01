/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS	DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define DT_GPIO_SAM_PORTA_BASE_ADDRESS	DT_ATMEL_SAM_GPIO_400E1000_BASE_ADDRESS
#define DT_GPIO_SAM_PORTA_IRQ		DT_ATMEL_SAM_GPIO_400E1000_IRQ_0
#define DT_GPIO_SAM_PORTA_IRQ_PRIO	DT_ATMEL_SAM_GPIO_400E1000_IRQ_0_PRIORITY
#define DT_GPIO_SAM_PORTA_LABEL		DT_ATMEL_SAM_GPIO_400E1000_LABEL
#define DT_GPIO_SAM_PORTA_PERIPHERAL_ID	DT_ATMEL_SAM_GPIO_400E1000_PERIPHERAL_ID

#define DT_GPIO_SAM_PORTB_BASE_ADDRESS	DT_ATMEL_SAM_GPIO_400E1200_BASE_ADDRESS
#define DT_GPIO_SAM_PORTB_IRQ		DT_ATMEL_SAM_GPIO_400E1200_IRQ_0
#define DT_GPIO_SAM_PORTB_IRQ_PRIO	DT_ATMEL_SAM_GPIO_400E1200_IRQ_0_PRIORITY
#define DT_GPIO_SAM_PORTB_LABEL		DT_ATMEL_SAM_GPIO_400E1200_LABEL
#define DT_GPIO_SAM_PORTB_PERIPHERAL_ID	DT_ATMEL_SAM_GPIO_400E1200_PERIPHERAL_ID

#define DT_GPIO_SAM_PORTC_BASE_ADDRESS	DT_ATMEL_SAM_GPIO_400E1400_BASE_ADDRESS
#define DT_GPIO_SAM_PORTC_IRQ		DT_ATMEL_SAM_GPIO_400E1400_IRQ_0
#define DT_GPIO_SAM_PORTC_IRQ_PRIO	DT_ATMEL_SAM_GPIO_400E1400_IRQ_0_PRIORITY
#define DT_GPIO_SAM_PORTC_LABEL		DT_ATMEL_SAM_GPIO_400E1400_LABEL
#define DT_GPIO_SAM_PORTC_PERIPHERAL_ID	DT_ATMEL_SAM_GPIO_400E1400_PERIPHERAL_ID

#define DT_USART_SAM_PORT_1_NAME	DT_ATMEL_SAM_USART_40028000_LABEL
#define DT_USART_SAM_PORT_1_BAUD_RATE	DT_ATMEL_SAM_USART_40028000_CURRENT_SPEED

/* End of SoC Level DTS fixup file */
