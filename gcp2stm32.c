/********************************
* GCP 2.0 STM32G030 Firmware    *
* July 2022                     *
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

#define delay(cycles)               for (int i = 0; i < cycles; i++) __asm__("nop")
#define delay_ms(milliseconds)      for (int i = 0; i < (milliseconds*6000); i++) __asm__("nop")

#define PACKED_OUTPUT   1
#define ORIGINAL_HW     1

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

#if ORIGINAL_HW

/* I2C LM75A */
#define TEMP_PORT   GPIOB
#define TEMP_SDA    GPIO0
#define TEMP_SCL    GPIO1

#else

/* I2C LM75A */
#define TEMP_PORT   GPIOB
#define TEMP_SDA    GPIO3
#define TEMP_SCL    GPIO4

#endif

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
unsigned int medianValue2 =     2048;
unsigned int medianValue3 =     2048;
unsigned int previousMedian0 =  2048;
unsigned int previousMedian1 =  2048;
unsigned int previousMedian2 =  2048;
unsigned int previousMedian3 =  2048;

/* Frequency arrays for Median Calculation */
uint16_t stream0_freq[FREQ_WINDOW];
uint16_t stream1_freq[FREQ_WINDOW];
uint16_t stream2_freq[FREQ_WINDOW];
uint16_t stream3_freq[FREQ_WINDOW];

unsigned int alt1_count=0;
unsigned int alt2_count=0;

unsigned int alt1_count_out=0;
unsigned int alt2_count_out=0;

/* Welford Variance Variables */
float tempfloat ,Svar[8];
float meanval,M[8], Mnext[8];
unsigned int k[8], variance[8];

#if ORIGINAL_HW

/* ADC Stuff */
uint8_t channel_array[8] = { 0,1,2,3,4,5,6,7 };

#else

/* ADC Stuff */
uint8_t channel_array[8] = { 0,1,2,4,5,6,7,8 };

#endif

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

volatile unsigned int first_raw[4]={0,0,0,0};
volatile unsigned int white_count2[8]={0,0,0,0,0,0,0,0};
volatile unsigned int white_count3[4]={0,0,0,0};
volatile unsigned int white_count4[4]={0,0,0,0};

volatile unsigned int adc_count_out[8]={0,0,0,0,0,0,0,0 };
volatile unsigned int adc_count_samples_out=0;
volatile unsigned int first_raw_out[4]={0,0,0,0};
volatile unsigned int white_count2_out[8]={0,0,0,0,0,0,0,0};
volatile unsigned int white_count3_out[4]={0,0,0,0};
volatile unsigned int white_count4_out[4]={0,0,0,0};

volatile unsigned int isrtemp0=0;
volatile unsigned int isrtemp1=0;
volatile unsigned int isrtemp2=0;
volatile unsigned int isrtemp3=0;
volatile unsigned int isrtempwhite=0;
volatile unsigned int sampleCounter=0;

#define SAMPLE_BUFFER_SIZE      128
#define SAMPLE_BUFFER_MASK      0x7f
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

        #if ORIGINAL_HW
            if((gpio_port_read(TEMP_PORT) & 0x1)==0x1)
                tempbyte|=1;
            else
                tempbyte|=0;
        #else
            if((gpio_port_read(TEMP_PORT) & 0x8)==0x8)
                tempbyte|=1;
            else
                tempbyte|=0;        
        #endif

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
    //current_temp=tempvar >> 6;
    current_temp=tempvar;
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
    sampleBufferInPtr=(sampleBufferInPtr+1)&SAMPLE_BUFFER_MASK;

    isrtempwhite=gpio_port_read(GPIOB);
 
#if ORIGINAL_HW

    whitebuffer[sampleBufferInPtr]=(isrtempwhite >> 8);
#else

    whitebuffer[sampleBufferInPtr]=((isrtempwhite >> 8) & 0xfb) | (isrtempwhite & 0x20)>>3;

#endif

    samplebuffer0[sampleBufferInPtr]=adc_tempval[0];
    samplebuffer1[sampleBufferInPtr]=adc_tempval[1];
    samplebuffer2[sampleBufferInPtr]=adc_tempval[2];
    samplebuffer3[sampleBufferInPtr]=adc_tempval[7];
    samplebuffer4[sampleBufferInPtr]=adc_tempval[3];
    samplebuffer5[sampleBufferInPtr]=adc_tempval[4];
    samplebuffer6[sampleBufferInPtr]=adc_tempval[5];
    samplebuffer7[sampleBufferInPtr]=adc_tempval[6];


}

static void process_samples(void)
{

    //gpio_clear(LED_PORT,LED_PIN);

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

        sampleBufferOutPtr=(sampleBufferOutPtr+1)&SAMPLE_BUFFER_MASK;

        if(processedSampleCount++<200)
        {
            /* Mean, Min, Max */
            for(int b=0;b<4;b++)
            {
                adc_count[b]+=adc_values[b];
                adc_count_samples++;
            }

            /* If this is the first sample */
            if(processedSampleCount==1)
            {
                first_raw[0]=adc_values[0];
                first_raw[1]=adc_values[1];
                first_raw[2]=adc_values[2];
                first_raw[3]=adc_values[3];
            }

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

            /* Count ones in xor alt sequence */
            /* Data Step 4 */
            if(processedSampleCount&1)
            {
                if(!(whiteport & 0x10)) white_count4[0]++;  /* A */
                if(!(whiteport & 0x04)) white_count4[1]++;  /* B */
                if(!(whiteport & 0x40)) white_count4[2]++;  /* C */
                if(!(whiteport & 0x01)) white_count4[3]++;  /* D */
            }
            else
            {
                if(whiteport & 0x10) white_count4[0]++;  /* A */
                if(whiteport & 0x04) white_count4[1]++;  /* B */
                if(whiteport & 0x40) white_count4[2]++;  /* C */
                if(whiteport & 0x01) white_count4[3]++;  /* D */   
            }

            if((adc_values[0]>previousMedian0) ^ (adc_values[1]>previousMedian1))
                alt1_count++;
        }

        /* Mean, Min, Max */
        for(int b=4;b<8;b++)
        {
            adc_count[b]+=adc_values[b];
            adc_count_samples++;
        }

        isrtemp0=adc_values[0];
        isrtemp1=adc_values[1];
        isrtemp2=adc_values[2];
        isrtemp3=adc_values[3];

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

        /* If the value is outside of the window */
        if(isrtemp2>MEDIAN_MAX)
            isrtemp2=FREQ_WINDOW-1;
        else
        {
            if(isrtemp2<MEDIAN_MIN)
                isrtemp2=0;
            else
                isrtemp2-=MEDIAN_MIN;
        }

        /* If the value is outside of the window */
        if(isrtemp3>MEDIAN_MAX)
            isrtemp3=FREQ_WINDOW-1;
        else
        {
            if(isrtemp3<MEDIAN_MIN)
                isrtemp3=0;
            else
                isrtemp3-=MEDIAN_MIN;
        }

        stream0_freq[isrtemp0]++;
        stream1_freq[isrtemp1]++;
        stream2_freq[isrtemp2]++;
        stream3_freq[isrtemp3]++;

        /* First Sample of Second */
        if(processedSampleCount <= 1)
        {
            M[0]=(float)adc_values[0];
            M[1]=(float)adc_values[1];
            M[2]=(float)adc_values[2];
            M[3]=(float)adc_values[3];
            M[4]=(float)adc_values[4];
            M[5]=(float)adc_values[5];
            M[5]=(float)adc_values[6];
            M[7]=(float)adc_values[7];
            k[0]=k[1]=k[2]=k[3]=k[4]=k[5]=k[6]=k[7]=1;
        }
        else
        {
            switch(processedSampleCount&0x3)
            {
                case 0:
                #undef LOCAL_INDEX
                #define LOCAL_INDEX 0
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];

                #undef LOCAL_INDEX
                #define LOCAL_INDEX 1
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];
                    break;

                case 1:
                #undef LOCAL_INDEX
                #define LOCAL_INDEX 2
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];

                #undef LOCAL_INDEX
                #define LOCAL_INDEX 3
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];
                    break;

                case 2:
                #undef LOCAL_INDEX
                #define LOCAL_INDEX 4
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];

                #undef LOCAL_INDEX
                #define LOCAL_INDEX 5
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];
                    break;

                case 3:
                #undef LOCAL_INDEX
                #define LOCAL_INDEX 6
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];

                #undef LOCAL_INDEX
                #define LOCAL_INDEX 7
                    tempfloat=adc_values[LOCAL_INDEX];
                    Mnext[LOCAL_INDEX]= M[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) / (k[LOCAL_INDEX]++) );
                    Svar[LOCAL_INDEX] = Svar[LOCAL_INDEX] + ( (tempfloat-M[LOCAL_INDEX]) * (tempfloat- Mnext[LOCAL_INDEX]) );
                    M[LOCAL_INDEX]=Mnext[LOCAL_INDEX];
                    break;
           }
        }

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
            first_raw_out[n]=first_raw[n];
            first_raw[n]=0;
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
        variance[0]=((unsigned int)Svar[0])/2499;
        variance[1]=((unsigned int)Svar[1])/2499;
        variance[2]=((unsigned int)Svar[2])/2499;
        variance[3]=((unsigned int)Svar[3])/2499;
        variance[4]=((unsigned int)Svar[4])/2499;
        variance[5]=((unsigned int)Svar[5])/2499;
        variance[6]=((unsigned int)Svar[6])/2499;
        variance[7]=((unsigned int)Svar[7])/2499;
        
        Svar[0]=Svar[1]=Svar[2]=Svar[3]=Svar[4]=Svar[5]=Svar[6]=Svar[7]=0;
        previousMedian0 = medianValue0>>1;
        previousMedian1 = medianValue1>>1;
    }
    //gpio_set(LED_PORT,LED_PIN);
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


#if ORIGINAL_HW

    /* Set up Pin Modes */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SYNC_PIN);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, \
        GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 |  GPIO5 |  GPIO6 | GPIO7 );

    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9 | \
                     GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TXPIN);
    gpio_mode_setup(TEMP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TEMP_SCL | TEMP_SDA);

    /* Setup USART TX pin as alternate function. */
    gpio_set_af(USART_PORT, GPIO_AF1, USART_TXPIN);

    gpio_mode_setup(SYNC_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SYNC_PIN);

#else


    /* Set up Pin Modes */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SYNC_PIN);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, \
        GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 |  GPIO5 |  GPIO6 );

    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 );

    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5 | GPIO8 | GPIO9 | \
                     GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TXPIN);

    gpio_mode_setup(TEMP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TEMP_SCL | TEMP_SDA);

    /* Setup USART TX pin as alternate function. */
    gpio_set_af(USART_PORT, GPIO_AF1, USART_TXPIN);

    gpio_mode_setup(SYNC_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SYNC_PIN);

#endif

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
    unsigned int k=0,temp=0;
    int signed_temp=0;
    unsigned int bcdtemp=0;
    unsigned int fractemp=0;
    unsigned int tempval[8];
    unsigned int minimum=0xffff, maximum=0;
    unsigned int tempword=0;

    /* Reset Median Frequency Arrays */
    for(int k=0;k<(MEDIAN_MAX-MEDIAN_MIN);k++)
    {
        stream0_freq[k]=0;
        stream1_freq[k]=0;
    }

    /* Wait for power to settle down */
    delay_ms(500);

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
            gpio_clear(LED_PORT,LED_PIN);

            processingDone=0;

            /* Read Temperature */
            read_temp();

            //current_temp=0xEFBE;

            /* TEMP1 */
            usart_send_blocking(USART1,(current_temp & 0xff));        /* 01 */
            usart_send_blocking(USART1,((current_temp>>8) & 0xff));   /* 02 */

            /* MEAN_RAW_1-MEAN_RAW_4 */
            for(int j=0;j<4;j++) /* 14 */
            {
                tempword=adc_count_out[j] * 32 / 25; /* count * 256 / 200 */
                usart_send_blocking(USART1,tempword & 0xff);        
                usart_send_blocking(USART1,(tempword>>8) & 0xff);        
                usart_send_blocking(USART1,(tempword>>16) & 0xff);       
            }

            /* MEAN_RAW_5-MEAN_RAW_8 */
            for(int j=4;j<8;j++) /* 26 */
            {
                tempword=(adc_count_out[j] * 16) / 625; /* count * 256 / 10000 */
                usart_send_blocking(USART1,tempword & 0xff);        
                usart_send_blocking(USART1,(tempword>>8) & 0xff);        
                usart_send_blocking(USART1,(tempword>>16) & 0xff);       
            }

            /* VARIANCE_RAW1 - VARIANCE_RAW8 */
            for(int j=0;j<8;j++) /* 50 */
            {
                tempword=variance[j];
                usart_send_blocking(USART1,tempword & 0xff);        
                usart_send_blocking(USART1,(tempword>>8) & 0xff);        
                usart_send_blocking(USART1,(tempword>>16) & 0xff);       
            }

            /* MEDIAN_RAW1 - MEDIAN_RAW4 */ /* 58 */
           temp=0;
            for(k = 0; k < FREQ_WINDOW;k++)
            {
                temp+=stream0_freq[k];
                if(temp>4999)
                {
                    medianValue0=(k+MEDIAN_MIN) << 1;
                    break;
                }
                else
                {
                    if(temp==4999)
                    {
                        medianValue0=((k+MEDIAN_MIN) << 1)+1;
                        break;       
                    }
                }
            }
            temp=0;
            for(k = 0; k < FREQ_WINDOW;k++)
            {
                temp+=stream1_freq[k];
                if(temp>4999)
                {
                    medianValue1=(k+MEDIAN_MIN) << 1;
                    break;
                }
                else
                {
                    if(temp==4999)
                    {
                        medianValue1=((k+MEDIAN_MIN) << 1)+1;
                        break;       
                    }
                }
            }
            temp=0;
            for(k = 0; k < FREQ_WINDOW;k++)
            {
                temp+=stream2_freq[k];
                if(temp>4999)
                {
                    medianValue2=(k+MEDIAN_MIN) << 1;
                    break;
                }
                else
                {
                    if(temp==4999)
                    {
                        medianValue2=((k+MEDIAN_MIN) << 1)+1;
                        break;       
                    }
                }
            }
            temp=0;
            for(k = 0; k < FREQ_WINDOW;k++)
            {
                temp+=stream3_freq[k];
                if(temp>4999)
                {
                    medianValue3=(k+MEDIAN_MIN) << 1;
                    break;
                }
                else
                {
                    if(temp==4999)
                    {
                        medianValue3=((k+MEDIAN_MIN) << 1)+1;
                        break;       
                    }
                }
            }

            /* Reset Median Frequency Arrays */
            for(int k=0;k<(FREQ_WINDOW);k++)
                stream0_freq[k]=stream1_freq[k]=stream2_freq[k]=stream3_freq[k]=0;

            usart_send_blocking(USART1,medianValue0 & 0xff);        
            usart_send_blocking(USART1,(medianValue0>>8) & 0xff); 

            usart_send_blocking(USART1,medianValue1 & 0xff);        
            usart_send_blocking(USART1,(medianValue1>>8) & 0xff);             

            usart_send_blocking(USART1,medianValue2 & 0xff);        
            usart_send_blocking(USART1,(medianValue2>>8) & 0xff); 

            usart_send_blocking(USART1,medianValue3 & 0xff);        
            usart_send_blocking(USART1,(medianValue3>>8) & 0xff); 

            /* FIRST_RAW1 - FIRST_RAW4 */ /* 66 */
            usart_send_blocking(USART1,first_raw_out[0] & 0xff);        
            usart_send_blocking(USART1,(first_raw_out[0]>>8) & 0xf); 

            usart_send_blocking(USART1,first_raw_out[1] & 0xff);        
            usart_send_blocking(USART1,(first_raw_out[1]>>8) & 0xf); 

            usart_send_blocking(USART1,first_raw_out[2] & 0xff);        
            usart_send_blocking(USART1,(first_raw_out[2]>>8) & 0xf); 

            usart_send_blocking(USART1,first_raw_out[3] & 0xff);        
            usart_send_blocking(USART1,(first_raw_out[3]>>8) & 0xf); 

            /* COUNT_FF1_200 - COUNT_FF8_200 */ /* 74 */
            usart_send_blocking(USART1,white_count2_out[0] & 0xff);        
            usart_send_blocking(USART1,white_count2_out[1] & 0xff);        
            usart_send_blocking(USART1,white_count2_out[2] & 0xff);        
            usart_send_blocking(USART1,white_count2_out[3] & 0xff);        
            usart_send_blocking(USART1,white_count2_out[4] & 0xff);        
            usart_send_blocking(USART1,white_count2_out[5] & 0xff);        
            usart_send_blocking(USART1,white_count2_out[6] & 0xff);        
            usart_send_blocking(USART1,white_count2_out[7] & 0xff);        

            /* COUNT_FF1_XOR_FF2 */ /* 78 */
            usart_send_blocking(USART1,white_count3_out[0] & 0xff);        
            usart_send_blocking(USART1,white_count3_out[1] & 0xff);        
            usart_send_blocking(USART1,white_count3_out[2] & 0xff);        
            usart_send_blocking(USART1,white_count3_out[3] & 0xff);        

            /* COUNT_FF1_XOR_FF2 ALT */ /* 82 */
            usart_send_blocking(USART1,white_count4_out[0] & 0xff);        
            usart_send_blocking(USART1,white_count4_out[1] & 0xff);        
            usart_send_blocking(USART1,white_count4_out[2] & 0xff);        
            usart_send_blocking(USART1,white_count4_out[3] & 0xff);        
        
            /* XOR_GT_MEDIANS */ /* 83 */
            usart_send_blocking(USART1,(alt1_count_out) & 0xff);       

            gpio_set(LED_PORT,LED_PIN);
        }
    }
    return 0;
}
