
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/assert.h>

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

volatile unsigned int adc_buffer0[1000];
volatile unsigned int adc_buffer1[1000];

volatile unsigned int ledcounter=0;
volatile unsigned int whiteport=0;

/* My clock setup structure */
/* 64MHz using 8MHz External Xtal */
/* AHB,APB = 64MHz, 2 Wait States */
const struct rcc_clock_scale my_clock_config = {
    .sysclock_source = RCC_PLL,
    .pll_source = RCC_PLLCFGR_PLLSRC_HSE,
    .pll_div = RCC_PLLCFGR_PLLM_DIV(2),
    .pll_mul = RCC_PLLCFGR_PLLN_MUL(32),
    .pllp_div = RCC_PLLCFGR_PLLP_DIV(2),
    .pllq_div = RCC_PLLCFGR_PLLQ_DIV(2),
    .pllr_div = RCC_PLLCFGR_PLLR_DIV(2),
    .hpre = RCC_CFGR_HPRE_NODIV,
    .ppre = RCC_CFGR_PPRE_NODIV,
    .flash_waitstates = FLASH_ACR_LATENCY_2WS,
    .voltage_scale = PWR_SCALE1,
    .ahb_frequency = 64000000,
    .apb_frequency = 64000000
};

/* Timer 2 ISR */
/* Every .1mS */
void tim2_isr(void)
{
    whiteport=gpio_port_read(GPIOB);

    if(ledcounter++>500)    
    {
        if(whiteport & 0x0100)
            gpio_set(LED_PORT,LED_PIN);
        else
            gpio_clear(LED_PORT,LED_PIN);
            //gpio_toggle(LED_PORT, LED_PIN); /* LED on/off */
        ledcounter=0;
    }
    /* Clear Timer 2 Interrupt Flag */
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
}

static void clock_gpio_setup(void)
{
    rcc_clock_setup( &my_clock_config ) ;
    
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9 | \
                     GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

static void adc_setup(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO6);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7);

    rcc_periph_clock_enable(RCC_ADC);

    /* Make sure the ADC doesn't run during config. */
    adc_power_off(ADC1);

    /* We configure everything for one single conversion. */
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    /* We want to read the temperature sensor, so we have to enable it. */
    adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_039DOT5);
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);

    adc_power_on(ADC1);
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

static void nvic_setup(void)
{
    nvic_enable_irq(NVIC_TIM2_IRQ);         /* Enable Timer 2 IRQ */
    nvic_set_priority(NVIC_TIM2_IRQ, 1);    /* Set Timer 2 Int Priority */
    rcc_periph_clock_enable(RCC_TIM2);      /* Enable clock to Timer 2 */

    /* Peripheral Clock = 64MHz */
    /* Set to 10KHz = 0.1mS */
    TIM_CNT(TIM2) = 0;      /* Timer 2 Start Value */
    TIM_PSC(TIM2) = 64;     /* Timer 2 Prescaler */
    TIM_ARR(TIM2) = 100;    /* Timer 2 Max Value */
    
    TIM_DIER(TIM2) |= TIM_DIER_UIE; /* Enable Timer 2 Interrupt */
    TIM_CR1(TIM2) |= TIM_CR1_CEN;   /* Start Timer 2 */ 
}

static void usart_print_string(char *str)
{
    uint8_t c;

    while ((c = *str++) != 0) 
        usart_send_blocking(USART1, c);
}

int main(void)
{
    int m=0;
    char tempstr[]="\nThis is a test!!!\n";

    adc_buffer0[203]=6;
    adc_buffer1[203]=6;

    /* Wait for power to settle down */
    delay_ms(50);

    /* Set up clocks and GPIO */
    clock_gpio_setup();

    /* Set up USART */
    usart_setup();

    /* Set up ADC */
    adc_setup();
	
    nvic_setup();

    while(1)
	{
    	delay_ms(1000);

        if(m>=10)
            m=0;

        usart_send_blocking(USART1, m++ + 0x30); /* USART1: Send byte. */
        usart_print_string((char*)tempstr);
 	}

	return 0;
}
