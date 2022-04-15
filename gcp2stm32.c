
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#define delay(cycles)       		for (int i = 0; i < cycles; i++) __asm__("nop")
#define delay_ms(milliseconds)      for (int i = 0; i < (milliseconds*6000); i++) __asm__("nop")

/* USART TX on PA9, USART RX on PA10 */
#define USART_PORT  GPIOA 
#define USART_TXPIN GPIO9
#define USART_RXPIN GPIO10
#define USART_DATARATE  2000000

/* LED */
#define LED_PORT    GPIOC
#define LED_PIN     GPIO13

static void clock_gpio_setup(void)
{
    /* Set up for external clock with PLL */
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSE_12MHZ_PLL_64MHZ]);

    /* Fix divider since clock is actually 8MHz */
    rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSE,2,32,2,2,2);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

static void usart_setup(void)
{
    /* Setup GPIO pins for USART transmit. */
    gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TXPIN);

    /* Setup USART TX pin as alternate function. */
    gpio_set_af(USART_PORT, GPIO_AF0, USART_TXPIN);

    /* Enable clocks for USART. */
    rcc_periph_clock_enable(RCC_USART1);

    /* Setup USART parameters. */
    usart_set_baudrate(USART1, USART_DATARATE);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);
}

static void usart_print_string(char *str)
{
    uint8_t c;

    while ((c = *str++) != 0) 
        usart_send_blocking(USART1, c);
}

int main(void)
{
    char tempstr[]="\nThis is a test!!!\n";

    /* Wait for power to settle down */
    delay_ms(250);

    /* Set up clocks and GPIO */
    clock_gpio_setup();

    /* Set up USART */
    usart_setup();
	
    while(1)
	{
    	gpio_toggle(LED_PORT, LED_PIN);	/* LED on/off */
    	delay_ms(250);

        usart_send_blocking(USART1, 'X'); /* USART1: Send byte. */
        usart_print_string((char*)tempstr);
	}

	return 0;
}
