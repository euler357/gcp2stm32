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

/* For Interrupt vs sample number testing */
unsigned int templimit=0;

/* Median Calculation Stuff */
#define MEDIAN_RANGE    4096
#define FREQ_WINDOW     512
#define MEDIAN_MAX (MEDIAN_RANGE+FREQ_WINDOW)/2-1
#define MEDIAN_MIN (MEDIAN_RANGE/2)-(FREQ_WINDOW/2)

unsigned int medianValue0 =     2048;
unsigned int medianValue1 =     2048;
unsigned int previousMedian0 =  2048;
unsigned int previousMedian1 =  2048;

/* Frequency arrays for Median Calculation */
uint16_t stream0_freq[FREQ_WINDOW];
uint16_t stream1_freq[FREQ_WINDOW];
unsigned int alt1_count=0;
unsigned int alt2_count=0;

unsigned int alt1_count_out=0;
unsigned int alt2_count_out=0;

/* Welford Variance Variables */
float tempfloat ,Svar[8];
unsigned int variance,meanval,M[8], Mnext[8];

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
volatile unsigned int ledcounter=0;
volatile unsigned int whiteport=0;
volatile unsigned int whitebyte=0;
volatile unsigned int alternate_toggle=0;

volatile unsigned int adc_values[8]={0,0,0,0,0,0,0,0};
volatile unsigned int adc_tempval[8]={0,0,0,0,0,0,0,0};

volatile unsigned int adc_min[8]={0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff };
volatile unsigned int adc_max[8]={0,0,0,0,0,0,0,0 };

volatile unsigned int adc_count[8]={0,0,0,0,0,0,0,0 };
volatile unsigned int adc_count_samples=0;
volatile unsigned int white_count2[8]={0,0,0,0,0,0,0,0};
volatile unsigned int white_count3[4]={0,0,0,0};
volatile unsigned int white_count4[4]={0,0,0,0};

volatile unsigned int adc_count_out[8]={0,0,0,0,0,0,0,0 };
volatile unsigned int adc_count_samples_out=0;
volatile unsigned int white_count2_out[8]={0,0,0,0,0,0,0,0};
volatile unsigned int white_count3_out[4]={0,0,0,0};
volatile unsigned int white_count4_out[4]={0,0,0,0};

volatile unsigned int isrtemp0=0;
volatile unsigned int isrtemp1=0;
volatile unsigned int sampleCounter=0;

#define SAMPLE_BUFFER_SIZE      256
volatile unsigned int sampleBufferInPtr=0;
volatile unsigned int sampleBufferOutPtr=0;
volatile uint16_t samplebuffer0[SAMPLE_BUFFER_SIZE];
volatile uint16_t samplebuffer1[SAMPLE_BUFFER_SIZE];
volatile uint16_t samplebuffer2[SAMPLE_BUFFER_SIZE];
volatile uint16_t samplebuffer3[SAMPLE_BUFFER_SIZE];
volatile uint16_t samplebuffer4[SAMPLE_BUFFER_SIZE];
volatile uint16_t samplebuffer5[SAMPLE_BUFFER_SIZE];
volatile uint16_t samplebuffer6[SAMPLE_BUFFER_SIZE];
volatile uint16_t samplebuffer7[SAMPLE_BUFFER_SIZE];
volatile uint8_t whitebuffer[SAMPLE_BUFFER_SIZE];

volatile unsigned int endSampleNumber=10100;
volatile unsigned int endSampleNumberPrev=10100;
volatile unsigned int processedSampleCount=0;
volatile unsigned int processingDone=0;

/* My clock setup structure */
/* 64MHz using 8MHz External Xtal */
/* AHB,APB = 64MHz, 2 Wait States */
const struct rcc_clock_scale my_clock_config = {
    .sysclock_source = RCC_PLL,
    .pll_source = RCC_PLLCFGR_PLLSRC_HSE,
    .pll_div = RCC_PLLCFGR_PLLM_DIV(2),
    .pll_mul = RCC_PLLCFGR_PLLN_MUL(32),
    .pllp_div = RCC_PLLCFGR_PLLP_DIV(2),
    .pllq_div = RCC_PLLCFGR_PLLQ_DIV(2), /* Only for STM32G0Bxxx */
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
    /* Clear Timer 2 Interrupt Flag */
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
    
    /* Zero adc index and start conversion */     
    adc_index=0;
    adc_start_conversion_regular(ADC1);
    
    sampleCounter++;
 
    /* Increment input pointer and save to circular buffer */
    sampleBufferInPtr=(sampleBufferInPtr+1)&0xff;
    whitebuffer[sampleBufferInPtr]=(gpio_port_read(GPIOB) >> 8);

    samplebuffer0[sampleBufferInPtr]=adc_tempval[0];
    samplebuffer1[sampleBufferInPtr]=adc_tempval[1];
    samplebuffer2[sampleBufferInPtr]=adc_tempval[2];
    samplebuffer3[sampleBufferInPtr]=adc_tempval[3];
    samplebuffer4[sampleBufferInPtr]=adc_tempval[4];
    samplebuffer5[sampleBufferInPtr]=adc_tempval[5];
    samplebuffer6[sampleBufferInPtr]=adc_tempval[6];
    samplebuffer7[sampleBufferInPtr]=adc_tempval[7];

}

static void process_samples(void)
{
    unsigned int b;

    /* If the pointers are not equal */
    while(sampleBufferInPtr != sampleBufferOutPtr)
    {
        whiteport=whitebuffer[sampleBufferOutPtr];
        adc_values[0]=samplebuffer0[sampleBufferOutPtr];
        adc_values[1]=samplebuffer1[sampleBufferOutPtr];
        adc_values[2]=samplebuffer2[sampleBufferOutPtr];
        adc_values[3]=samplebuffer3[sampleBufferOutPtr];
        adc_values[4]=samplebuffer4[sampleBufferOutPtr];
        adc_values[5]=samplebuffer5[sampleBufferOutPtr];
        adc_values[6]=samplebuffer6[sampleBufferOutPtr];
        adc_values[7]=samplebuffer7[sampleBufferOutPtr];

        sampleBufferOutPtr=(sampleBufferOutPtr+1)&0xff;

        if(processedSampleCount++<200)
            {
                /* Count ones */
                /* Data Step 2 */
                if(whiteport & 0x20) white_count2[0]++;
                if(whiteport & 0x10) white_count2[1]++;
                if(whiteport & 0x08) white_count2[2]++;
                if(whiteport & 0x04) white_count2[3]++;
                if(whiteport & 0x80) white_count2[4]++;
                if(whiteport & 0x40) white_count2[5]++;
                if(whiteport & 0x02) white_count2[6]++;
                if(whiteport & 0x01) white_count2[7]++;

                /* XOR Bits */
                /* Step 2 */
                whiteport ^=(whiteport>>1);

                /* Step 3 is implied since the data was read at a 10,000 sps rate */

                /* Count ones */
                /* Data Step 3 */
                if(whiteport & 0x10) white_count3[0]++;  /* A */
                if(whiteport & 0x04) white_count3[1]++;  /* B */
                if(whiteport & 0x40) white_count3[2]++;  /* C */
                if(whiteport & 0x01) white_count3[3]++;  /* D */

                if(adc_values[0]>previousMedian0)
                    alt1_count++;
                if(adc_values[1]>previousMedian1)
                    alt2_count++;
            }
            else /* Over 200 */
            {
                isrtemp0=adc_values[0];
                isrtemp1=adc_values[1];
                /* If the value is outside of the window */
                if(isrtemp0>MEDIAN_MAX)
                    isrtemp0=FREQ_WINDOW-1;
                else
                {
                    if(isrtemp0<MEDIAN_MIN)
                        isrtemp0=0;
                    else
                        isrtemp0-=MEDIAN_MIN;
                }

                /* If the value is outside of the window */
                if(isrtemp1>MEDIAN_MAX)
                    isrtemp1=FREQ_WINDOW-1;
                else
                {
                    if(isrtemp1<MEDIAN_MIN)
                        isrtemp1=0;
                    else
                        isrtemp1-=MEDIAN_MIN;
                }

                stream0_freq[isrtemp0]++;
                stream1_freq[isrtemp1]++;
            }
#if 1
            /* Mean, Min, Max */
            for(int b=0;b<8;b++)
            {
                adc_count[b]+=adc_values[b];
                adc_count_samples++;

        #if 1
                if(adc_values[b]>adc_max[b])   
                    adc_max[b]=adc_values[b];
                if(adc_values[b]<adc_min[b])   
                    adc_min[b]=adc_values[b];
        #endif
            }
#endif
    gpio_clear(LED_PORT,LED_PIN);
        b=processedSampleCount&0x7;//for(int b=0;b<8;b++)
        {
            tempfloat=adc_values[b];
            if(processedSampleCount == 1)
            {
                M[b]=tempfloat;
            }
            else
            {
                Mnext[b]= M[b] + ( (tempfloat-M[b]) / processedSampleCount );
                Svar[b] =    Svar[b] + ( (tempfloat-M[b]) * (tempfloat- Mnext[b]) );
                M[b]=Mnext[b];
            }
        }
    gpio_set(LED_PORT,LED_PIN);
    }


    if(processedSampleCount>=endSampleNumber)
    {
        /* Copy Stored Stuff to Temp Variables */
        /* and clear working variables */
        for(int n=0;n<8;n++)
        {
            adc_count_out[n]=adc_count[n];
            adc_count[n]=0;
            white_count2_out[n]=white_count2[n];
            white_count2[n]=0;
        }
        for(int n=0;n<4;n++)
        {
            white_count3_out[n]=white_count3[n];
            white_count3[n]=0;
            white_count4_out[n]=white_count4[n];
            white_count4[n]=0;
        }
        adc_count_samples_out=adc_count_samples;
        alt1_count_out=alt1_count;
        alt2_count_out=alt2_count;
        
        processingDone=1;
        endSampleNumberPrev=endSampleNumber;
        endSampleNumber=10100;
        processedSampleCount=0;
        adc_count_samples=0;
        alt1_count=0;
        alt2_count=0;
        variance=((unsigned int)Svar[0])/1249;;
        meanval=(unsigned int)M[0];
        
        M[0]=0;Svar[0]=0;
        previousMedian0 = medianValue0;
        previousMedian1 = medianValue1;
    }
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
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    nvic_set_priority(NVIC_ADC_COMP_IRQ, 2);
    
    adc_power_off(ADC1);
    rcc_periph_reset_pulse(RST_ADC);
    adc_enable_eoc_interrupt(ADC1);
    adc_set_clk_source(ADC1, ADC_CFGR2_CKMODE_PCLK_DIV4); /* 32MHz ADC Clock */
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_single_conversion_mode(ADC1);
    ADC_CFGR2(ADC1) = (ADC_CFGR2(ADC1) & ~0x3FB) | 0x021;
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_019DOT5);
    adc_set_regular_sequence(ADC1, 8, channel_array);
    adc_power_on(ADC1);

    adc_calibrate_async(ADC1);
    delay_ms(500);

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
    TIM_PSC(TIM2) = 3;      /* Prescale div4 to 16 MHz */
    TIM_CNT(TIM2) = 0;      /* Timer 2 Start Value */
    TIM_ARR(TIM2) = 1599;    /* Timer 2 Max Value */
    TIM_DIER(TIM2) |= TIM_DIER_UIE; /* Enable Timer 2 Interrupt */
    TIM_CR1(TIM2) |= TIM_CR1_CEN;   /* Start Timer 2 */ 
}

void usart_print_string(char *str)
{
    uint8_t c;

    while ((c = *str++) != 0) 
        usart_send_blocking(USART1, c);
}

void usart_print_hex(unsigned int inValue)
{    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>28)&0xf]);   
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>24)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>20)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>16)&0xf]);  
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>12)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>8)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>4)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[inValue&0xf]);    
    usart_send_blocking(USART1,' ');
}
void usart_print_2bytes(unsigned int inValue)
{    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>12)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>8)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>4)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[inValue&0xf]);    
    usart_send_blocking(USART1,' ');
}
void usart_print_byte(unsigned int inValue)
{    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>4)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[inValue&0xf]);    
    usart_send_blocking(USART1,' ');
}

void usart_print_long(int64_t inValue)
{    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>60)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>56)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>52)&0xf]);  
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>48)&0xf]);   
    usart_send_blocking(USART1,':');
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>44)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>40)&0xf]);  
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>36)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>32)&0xf]);    
    usart_send_blocking(USART1,':');
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>28)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>24)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>20)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>16)&0xf]);  
    usart_send_blocking(USART1,':');
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>12)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>8)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[(inValue>>4)&0xf]);    
    usart_send_blocking(USART1, nibble_to_hex[inValue&0xf]);    
    usart_send_blocking(USART1,' ');
}

/* Handle Sync Pulse */
void exti4_15_isr(void)
{
    exti_reset_request(EXTI11);

    endSampleNumber=sampleCounter;
    sampleCounter=0;
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
    adc_tempval[adc_index++]=adc_read_regular(ADC1);
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
    unsigned int m=0,k=0,temp=0;
    int signed_temp=0;
    unsigned int bcdtemp=0;
    unsigned int fractemp=0;
    unsigned int tempval[8];

    unsigned int rawmean[8];

    unsigned int minimum=0xffff, maximum=0;

    /* Reset Median Frequency Arrays */
    for(int k=0;k<(MEDIAN_MAX-MEDIAN_MIN);k++)
    {
        stream0_freq[k]=0;
        stream1_freq[k]=0;
    }

    /* Wait for power to settle down */
    delay_ms(50);

    clock_gpio_setup();
    adc_setup();
    timer2_setup();
    usart_setup();
    sync_setup();

    while(1)
	{  
        process_samples();
        /* If we got the 1pps trigger */
        if(processingDone)
        {
            processingDone=0;

    	    //gpio_clear(LED_PORT,LED_PIN);

            /* 8 Mean Values over a second */
            /* 12-bit value per channel */
            for(int n;n<8;n++)
                rawmean[n]=adc_count_out[n]/sampleCounter;

            /* Read Temperature */
            read_temp();

            usart_print_string("=================\n Temp   ");     
            signed_temp=current_temp;

            /* Check sign and do twos compliment if needed */
            if(signed_temp & 0x800)
            {
                usart_send_blocking(USART1,'-');
                signed_temp=(signed_temp ^ 0xFFF)+1;            
            }

            bcdtemp=binary_to_bcd(signed_temp >>2);
            fractemp=signed_temp & 0x7;

            k&=7;
            usart_send_blocking(USART1,((bcdtemp>>8)&0xf) + 0x30);
            usart_send_blocking(USART1,((bcdtemp>>4)&0xf) + 0x30);
            usart_send_blocking(USART1,((bcdtemp)&0xf) + 0x30);
            usart_send_blocking(USART1,'.');
            usart_send_blocking(USART1,bin_to_eighths[fractemp][0]);
            usart_send_blocking(USART1,bin_to_eighths[fractemp][1]);
            usart_send_blocking(USART1,bin_to_eighths[fractemp][2]);
            usart_print_string("C ");    
            usart_print_string("\n Count  "); 
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>28)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>24)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>20)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>16)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>12)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>8)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter>>4)&0xf]);
            usart_send_blocking(USART1,nibble_to_hex[(adc_int_counter)&0xf]);
            

            usart_print_string("\n Samp   "); 
            usart_print_hex(endSampleNumberPrev);

#if 0 /* Print occasional values*/       
            usart_print_string("\n Values "); 
            for(int n=0;n<8;n++)
                usart_print_hex(adc_values[n]);
         
            
            usart_print_string("\n Means  ");     
            for(int j = 0 ;j<8;j++)
            {

                tempval[j]=adc_count_out[j] / 10000;

                usart_print_hex(tempval[j]);

                if(adc_int_counter>0x20000)
                {
                    if(tempval[j]>maximum)
                        maximum=tempval[j];
                    if(tempval[j]<minimum)
                        minimum=tempval[j];
                }
            }

#endif
            usart_print_string("\n Max    ");     
            usart_print_hex(maximum);
            usart_print_string("\n Min    ");     
            usart_print_hex(minimum);

#if 1 

            usart_print_string("\n Median ");  
            temp=0;
            for(k = 0; k < FREQ_WINDOW;k++)
            {
                temp+=stream0_freq[k];
                if(temp>5000)
                {
                    medianValue0=k+MEDIAN_MIN;
                    break;
                }
            }
            temp=0;
            for(k = 0; k < FREQ_WINDOW;k++)
            {
                temp+=stream1_freq[k];
                if(temp>5000)
                {
                    medianValue1=k+MEDIAN_MIN;
                    break;
                }
            }

      #endif      
            
            /* Reset Median Frequency Arrays */
            for(int k=0;k<(FREQ_WINDOW);k++)
                stream0_freq[k]=stream1_freq[k]=0;


            usart_print_2bytes(medianValue0);
            usart_print_2bytes(medianValue1);


            usart_print_string("\n w2     ");    
            for(int j = 0 ;j<8;j++)
            {
                usart_print_byte(white_count2_out[j]);
            }
            
            usart_print_string("\n w3     ");    
            for(int j = 0 ;j<4;j++)
            {
                usart_print_byte(white_count3_out[j]);
            }
            
            usart_print_string("\n Mean   ");    
            usart_print_hex(meanval);
            usart_print_string("\n Vari   ");    
            usart_print_hex(variance);
#if 0 /* not in current spec */
            usart_print_string("\n w4     ");    
            for(int j = 0 ;j<4;j++)
            {
                usart_print_hex(white_count4_out[j]);
            }
#endif
            usart_print_string("\n Alt    ");    
            usart_print_byte(alt1_count_out);
            usart_print_byte(alt2_count_out);
            usart_print_byte(alt1_count_out ^ alt2_count_out);
            usart_send_blocking(USART1,'\n'); 

            //gpio_set(LED_PORT,LED_PIN);
        }

 	}

	return 0;
}
