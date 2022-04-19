/********************************
* GCP 2.0 STM32G030 Firmware    *
* April 2022                    *
* Chris K Cockrum               *
* https://accuforge.com         *
********************************/

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/common/adc_common_v2.h>

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

/* Sync Pin */
#define SYNC_PORT   GPIOA
#define SYNC_PIN    GPIO11

/* I2C LM75A */
#define TEMP_PORT   GPIOB
#define TEMP_SDA    GPIO0
#define TEMP_SCL    GPIO1

/* I2C Stuff */
#define SCL0    gpio_clear(TEMP_PORT,TEMP_SCL)
#define SCL1    gpio_set(TEMP_PORT,TEMP_SCL)
#define SDA0    gpio_clear(TEMP_PORT,TEMP_SDA)
#define SDA1    gpio_set(TEMP_PORT,TEMP_SDA)
#define CLK_DELAY   15
#define SEND_ZERO   SCL0;SDA0;CLK_DELAY;SCL1;CLK_DELAY;
#define SEND_ONE    SCL0;SDA1;CLK_DELAY;SCL1;CLK_DELAY;
#define SEND_CLK    SCL0;CLK_DELAY;SCL1;CLK_DELAY;

unsigned int templimit=0;
/* ADC Stuff */
uint8_t channel_array[8] = { 0,1,2,3,4,5,6,7 };

unsigned int adc_index=0;
unsigned int adc_int_counter=0;

/* Table for Hex Digits */
const char nibble_to_hex[] = {  '0', '1', '2', '3', '4', \
                                '5', '6', '7', '8', '9', \
                                'A', 'B', 'C', 'D', 'E', \
                                'F'  };

/* Table for converting 3-bit binary fraction to BCD fraction */
const unsigned int bin_to_eighths[8][3] = { \
        {'0','0','0'} ,
        {'1','2','5'} ,
        {'2','5','0'} ,
        {'3','7','5'} ,
        {'5','0','0'} ,
        {'6','2','5'} ,
        {'7','5','0'} ,
        {'8','7','5'} ,
         };

/* Current Temperature */
unsigned int current_temp=0;

/* ADC Buffers */
volatile unsigned char adc_buffer0[256];
volatile unsigned char adc_buffer1[256];

volatile unsigned int ledcounter=0;
volatile unsigned int whiteport=0;
volatile unsigned int whitebyte=0;
volatile unsigned int alternate_toggle=0;

volatile unsigned int adc_values[8]={0,0,0,0,0,0,0,0};

volatile unsigned int adc_min[8]={0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff };
volatile unsigned int  adc_max[8]={0,0,0,0,0,0,0,0 };
volatile unsigned int  adc_count[8]={0,0,0,0,0,0,0,0 };


volatile unsigned int pps_trigger=0;

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

unsigned int i2c_send_byte(unsigned int sendbyte)
{
    unsigned int tempbyte;
    tempbyte=sendbyte;

    gpio_mode_setup(TEMP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TEMP_SDA);
    delay(CLK_DELAY);
    SCL0;
    delay(CLK_DELAY);

    for(int k=0;k<8;k++)
    {
        if(tempbyte & 0x80)
        {
            SDA1;
            delay(CLK_DELAY);
            SCL1;
            delay(CLK_DELAY);
            SCL0;
            delay(CLK_DELAY);
        }
        else
        {
            SDA0;
            delay(CLK_DELAY);
            SCL1;
            delay(CLK_DELAY);
            SCL0;
            delay(CLK_DELAY);
        }
        tempbyte<<=1;
    }

    gpio_mode_setup(TEMP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TEMP_SDA);
    SCL1;
    delay(CLK_DELAY);
    tempbyte=gpio_port_read(TEMP_PORT);
    SCL0;
    delay(CLK_DELAY);
    
    return (tempbyte & 1);
}

unsigned int i2c_receive_byte(void)
{
    unsigned int tempbyte;
    tempbyte=0;

    gpio_mode_setup(TEMP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TEMP_SDA);
    for(int k=0;k<8;k++)
    {

        tempbyte<<=1;
        delay(CLK_DELAY);
        SCL1;delay(CLK_DELAY);
        if((gpio_port_read(TEMP_PORT) & 0x1)==0x1)
            tempbyte|=1;
        else
            tempbyte|=0;
        SCL0;

        delay(CLK_DELAY);
    }

    /* Send ACK */
    SDA0;
    gpio_mode_setup(TEMP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TEMP_SDA);
    SCL1;
    delay(CLK_DELAY);
    SCL0;
    delay(CLK_DELAY);
    
    return tempbyte;
}

void read_temp(void)
{
    volatile unsigned int tempvar=0;

    /* Set SCL, SDA to outputs */
    SCL1;SDA1;
    delay(CLK_DELAY);;
    gpio_mode_setup(TEMP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TEMP_SCL | TEMP_SDA);
    SDA0;
    
    i2c_send_byte(145);
    tempvar=i2c_receive_byte();
    tempvar<<=8;
    tempvar|=i2c_receive_byte();
    
    delay(CLK_DELAY);
    SDA0;delay(CLK_DELAY);;
    SCL1;delay(CLK_DELAY);;
    SDA1;delay(CLK_DELAY);;
    /* Set SCL, SDA to float */
    gpio_mode_setup(TEMP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TEMP_SCL | TEMP_SDA);

    /* Write read temperature to global variable */
    current_temp=tempvar >> 6;
}

/* Timer 2 ISR */
/* Every .1mS */
void tim2_isr(void)
{
    volatile unsigned int tempadc=0;

    /* Read bits from port B */
    /* Step 1 Output */
    whiteport=gpio_port_read(GPIOB);

    /* XOR Bits */
    /* Step 2 */
    whiteport ^=(whiteport>>1);

    /* Copy XOR'd bits into whitebyte in the correct order */
    /* Low Nibble = 0bDCBA */
    whitebyte=  ((whiteport & 0x4000) >> 12) |  /* C */
                ((whiteport & 0x1000) >> 12) |  /* A */
                ((whiteport & 0x0400) >> 9)  |  /* B */
                ((whiteport & 0x0100) >> 5);    /* D */

    /* Step 3 is implied since the data was read at a 10,000 sps rate */

    /* Step 4 */
    if (alternate_toggle^=1)
        whitebyte^=0xf;

    /* Start Next Conversion */
    //if(templimit++<100000)
    {
        for(int b=0;b<8;b++)
        {
            if(adc_values[b]>adc_max[b])   
                adc_max[b]=adc_values[b];
            if(adc_values[b]<adc_min[b])   
                adc_min[b]=adc_values[b];
            adc_count[b]+=adc_values[b];

        }
        adc_index=0;
        adc_start_conversion_regular(ADC1);
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
    rcc_periph_clock_enable(RCC_TIM2);      
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_DMA1);

    /* Set up Pin Modes */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SYNC_PIN);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, \
        GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 |  GPIO5 |  GPIO6 |  GPIO7 );
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9 | \
                     GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TXPIN);
    gpio_mode_setup(TEMP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TEMP_SCL | TEMP_SDA);

    /* Setup USART TX pin as alternate function. */
    gpio_set_af(USART_PORT, GPIO_AF1, USART_TXPIN);

    gpio_mode_setup(SYNC_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SYNC_PIN);
}

static void adc_setup(void)
{
    unsigned int temp=0;
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    nvic_set_priority(NVIC_ADC_COMP_IRQ, 2);  
    
    adc_power_off(ADC1);
    rcc_periph_reset_pulse(RST_ADC);
    adc_enable_eoc_interrupt(ADC1);
    adc_set_clk_source(ADC1, ADC_CFGR2_CKMODE_PCLK_DIV2);
    //adc_set_clk_prescale(ADC1, ADC_CCR_PRESC_DIV2);
    
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_single_conversion_mode(ADC1);
    ADC_CFGR2(ADC1) = (ADC_CFGR2(ADC1) & ~0x3FB) | 0x021;
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_160DOT5);
    adc_set_regular_sequence(ADC1, 8, channel_array);
    adc_power_on(ADC1);

    adc_calibrate_async(ADC1);
    delay_ms(10);

    adc_start_conversion_regular(ADC1);
}

static void usart_setup(void)
{

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

static void timer2_setup(void)
{
    nvic_enable_irq(NVIC_TIM2_IRQ);         /* Enable Timer 2 IRQ */
    nvic_set_priority(NVIC_TIM2_IRQ, 1);    /* Set Timer 2 Int Priority */

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

/* Handle Sync Pulse */
void exti4_15_isr(void)
{
    exti_reset_request(EXTI11);
    pps_trigger=1;
}

/* Set up interrupt and exti for Sync Pulse */
static void sync_setup(void)
{

    nvic_enable_irq(NVIC_EXTI4_15_IRQ);      
    nvic_set_priority(NVIC_EXTI4_15_IRQ, 2);  

    /* Configure the EXTI subsystem. */
    exti_select_source(EXTI11, GPIOA);
    exti_set_trigger(EXTI11, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI11);

}

void adc_comp_isr(void)
{
    adc_values[adc_index++]=adc_read_regular(ADC1);
    adc_index&=0x7;
    adc_int_counter++;
}


unsigned int binary_to_bcd(unsigned int inbinary)
{
    unsigned int outbcd, binarytemp;
    outbcd=0;
    binarytemp=inbinary;

    for(int k=0;k<8;k++)
    {
        if((outbcd & 0xf00)>= 0x500)
            outbcd+=0x300;
        if((outbcd & 0x0f0)>= 0x050)
            outbcd+=0x030;
        if((outbcd & 0x00f)>= 0x005)
            outbcd+=0x003;


        outbcd<<=1;
        if(binarytemp & 0x80)
            outbcd|=1;
        binarytemp<<=1;

    }
    return outbcd;
}



int main(void)
{
    int m=0,k=0;
    int signed_temp=0;
    unsigned int bcdtemp=0;
    unsigned int fractemp=0;
    adc_buffer0[203]=6;
    adc_buffer1[203]=6;
    unsigned int tempval[8];

unsigned int minimum=0xffff, maximum=0;

    /* Wait for power to settle down */
    delay_ms(50);

    clock_gpio_setup();
    adc_setup();
    timer2_setup();
    usart_setup();
    sync_setup();

    while(1)
	{  
        /* If we got the 1pps trigger */
        if(pps_trigger)
        {
        
            pps_trigger=0;

    	    gpio_toggle(LED_PORT,LED_PIN);

            /* Read Temperature */
            read_temp();

            /* Send counting pattern */
            if(m>=10)
                m=0;
            signed_temp=current_temp;
            usart_send_blocking(USART1, m++ + 0x30); /* USART1: Send byte. */
            usart_send_blocking(USART1,':');


            /* Check sign and do twos compliment if needed */
            if(signed_temp & 0x800)
            {
                usart_send_blocking(USART1,'-');
                signed_temp=(signed_temp ^ 0xFFF)+1;            
            }

            bcdtemp=binary_to_bcd(signed_temp >>2);
            fractemp=signed_temp & 0x7;

            //adc_int_counter=adc_values[k++];
            k&=7;
            usart_send_blocking(USART1,((bcdtemp>>8)&0xf) + 0x30);
            usart_send_blocking(USART1,((bcdtemp>>4)&0xf) + 0x30);
            usart_send_blocking(USART1,((bcdtemp)&0xf) + 0x30);
            usart_send_blocking(USART1,'.');
            usart_send_blocking(USART1,bin_to_eighths[fractemp][0]);
            usart_send_blocking(USART1,bin_to_eighths[fractemp][1]);
            usart_send_blocking(USART1,bin_to_eighths[fractemp][2]);
            usart_send_blocking(USART1,'C');

            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>28)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>24)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>20)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>16)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>12)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter)&0xf]);

            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[0]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[0]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[0]&0xf]);
            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[1]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[1]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[1]&0xf]);
            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[2]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[2]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[2]&0xf]);
            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[3]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[3]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[3]&0xf]);

            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[4]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[4]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[4]&0xf]);
            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[5]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[5]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[5]&0xf]);
            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[6]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[6]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[6]&0xf]);
            usart_send_blocking(USART1,' ');            
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[7]>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_values[7]>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[adc_values[7]&0xf]);

            usart_send_blocking(USART1,':');    
            usart_send_blocking(USART1,':');    


            for(int j = 0 ;j<8;j++)
            {
                tempval[j]=adc_count[j];
                adc_count[j]=0;
            }

            for(int j = 0 ;j<8;j++)
            {

                tempval[j]=tempval[j] / 10000;
                usart_send_blocking(USART1,nibble_to_hex[(tempval[j]>>12)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(tempval[j]>>8)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(tempval[j]>>4)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(tempval[j])&0xf]);
                usart_send_blocking(USART1,' ');       
                if(adc_int_counter>0x20000)
                {
                    if(tempval[j]>maximum)
                        maximum=tempval[j];
                    if(tempval[j]<minimum)
                        minimum=tempval[j];
                }
            }

                usart_send_blocking(USART1,'H'); 
                usart_send_blocking(USART1,nibble_to_hex[(maximum>>12)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(maximum>>8)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(maximum>>4)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(maximum)&0xf]);
                usart_send_blocking(USART1,' '); 
                
                usart_send_blocking(USART1,'L'); 
                usart_send_blocking(USART1,nibble_to_hex[(minimum>>12)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(minimum>>8)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(minimum>>4)&0xf]);
                usart_send_blocking(USART1,nibble_to_hex[(minimum)&0xf]);
            usart_send_blocking(USART1,'\n'); 
        }
 	}

	return 0;
}
