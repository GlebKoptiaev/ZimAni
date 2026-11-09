/* NUCLEO-F429ZI, passive SPI recorder v2.0 HW, remote or robot side.
 * WARNING NEW WIRING: PA0=SCK, PB3=CSN. See README_RU.md.
 * Replace Core/Src/main.c in a fresh STM32F429ZITx CubeMX project.
 * Uses CMSIS device registers, no HAL peripheral initialization required.
 * SPI mode 0, 8-bit, MSB first. Read README_RU.md before connecting.
 * This is a bounded recorder, not a continuous streaming logic analyzer.
 */
#include "stm32f4xx.h"
#include <stdint.h>

#if !defined(STM32F429xx)
#error "Select STM32F429ZITx / STM32F429xx"
#endif

#define BYTE_CAP       32768u  /* each of two DMA buffers; main SRAM, not CCM */
#define FRAME_CAP        2048u
#define WINDOW_US    1000000u  /* maximum acquisition time after arming */
#define ARM_TIMEOUT_US 3000000u
#define IDLE_US            20u /* required stable CSN=1 before arming */


/* MB1137 default solder bridges: active-high user LEDs, not ST-LINK LD4. */
#define LED_GREEN (1u << 0)   /* LD1 / PB0 */
#define LED_BLUE  (1u << 7)   /* LD2 / PB7 */
#define LED_RED   (1u << 14)  /* LD3 / PB14 */
#define LED_ALL   (LED_GREEN | LED_BLUE | LED_RED)
enum { LED_IDLE=0, LED_BUSY, LED_OK, LED_EMPTY, LED_ERROR };
static uint32_t led_state=LED_IDLE, led_epoch, led_last_service;
static uint32_t led_last_mask=0xFFFFFFFFu;

enum { F_ALIGN=1u, F_EDGE_LOSS=2u, F_RX_COUNT=4u,
       F_SPI_OVR=8u, F_DMA=16u, F_ARM=32u, F_RANGE=64u,
       F_CLOCK_TOTAL=128u };
enum { STOP_WINDOW=0, STOP_BYTES, STOP_FRAMES, STOP_ERROR, STOP_USER };


/* Volatile buffers: written by DMA, read after acquisition stops. */
static volatile uint8_t cmd_bytes[BYTE_CAP] __attribute__((aligned(4)));
static volatile uint8_t reply_bytes[BYTE_CAP] __attribute__((aligned(4)));
static volatile uint32_t frame_sck[FRAME_CAP] __attribute__((aligned(4)));
static volatile uint32_t frame_count, flags, running, stop_reason;
static uint32_t t_start, raw_a, raw_b, raw_frames, complete_bytes, total_sck;

void Error_Handler(void);
void SystemClock_Config(void);


static uint32_t now_us(void) { return TIM5->CNT; }
static int cs_high(void)
{
    return ((GPIOA->IDR & (1u<<15)) != 0u)
        && ((GPIOE->IDR & (1u<<4)) != 0u)
        && ((GPIOB->IDR & (1u<<3)) != 0u);
}
static int sck_low(void) { return (GPIOA->IDR & 1u)==0u; }

/* Pure status logic: success needs complete bytes, not CSN edges alone. */
static uint32_t led_classify(uint32_t errors, uint32_t reason,
                             uint32_t count, uint32_t complete_bytes)
{
    if (errors || reason==STOP_ERROR) return LED_ERROR;
    if (count && complete_bytes) return LED_OK;
    return LED_EMPTY;
}

static uint32_t led_pattern(uint32_t state, uint32_t elapsed)
{
    if (state==LED_BUSY) return LED_GREEN;
    uint32_t phase=elapsed % 1000000u;
    uint32_t mask=(phase < 100000u) ? LED_GREEN : 0u; /* alive: 1 flash/s */
    if (state==LED_OK) mask |= LED_BLUE;
    if (state==LED_ERROR) mask |= LED_RED;
    if (state==LED_EMPTY && phase>=500000u && phase<600000u) mask |= LED_RED;
    return mask;
}

static void led_write(uint32_t mask)
{
    if (mask!=led_last_mask) {
        GPIOB->BSRR=((LED_ALL & ~mask) << 16) | mask;
        led_last_mask=mask;
    }
}

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;
    GPIOB->BSRR=LED_ALL << 16;
    GPIOB->OTYPER &= ~LED_ALL;
    const uint32_t fields=(3u<<0) | (3u<<14) | (3u<<28);
    GPIOB->PUPDR &= ~fields;
    GPIOB->OSPEEDR &= ~fields;
    GPIOB->MODER=(GPIOB->MODER & ~fields) | (1u<<0) | (1u<<14) | (1u<<28);
    led_last_mask=0u;
}

static void led_set(uint32_t state)
{
    led_state=state; led_epoch=now_us(); led_last_service=led_epoch;
    led_write(led_pattern(state,0u));
}

/* Foreground only. Never called from the acquisition polling loop. */
static void led_service(void)
{
    uint32_t t=now_us();
    if ((uint32_t)(t-led_last_service)<10000u) return;
    led_last_service=t;
    led_write(led_pattern(led_state,(uint32_t)(t-led_epoch)));
}

static void led_boot_test(void)
{
    /* Only at boot, before capture can start: two flashes of all user LEDs. */
    for (unsigned i=0;i<2u;++i) {
        led_write(LED_ALL);
        uint32_t t=now_us();
        while ((uint32_t)(now_us()-t)<150000u) {}
        led_write(0u); t=now_us();
        while ((uint32_t)(now_us()-t)<150000u) {}
    }
    led_set(LED_IDLE);
}

static void gpio_af(GPIO_TypeDef *g, unsigned pin, unsigned af, unsigned pull)
{
    g->MODER = (g->MODER & ~(3u << (pin*2u))) | (2u << (pin*2u));
    g->PUPDR = (g->PUPDR & ~(3u << (pin*2u))) | (pull << (pin*2u));
    g->AFR[pin/8u] = (g->AFR[pin/8u] & ~(15u << ((pin%8u)*4u)))
                  | (af << ((pin%8u)*4u));
}

void SystemClock_Config(void)
{
    /* HSI -> PLL: 16/16*336/2=168 MHz. APB1=42, APB2=84 MHz.
     * HSI avoids dependence on ST-LINK MCO / board solder bridges.
     */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN
               | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR = 16u | (336u << 6) | (7u << 24); /* PLLP=/2, HSI */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
              | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    SystemCoreClock = 168000000u;
    SysTick->CTRL = 0u; /* timestamps use TIM5, not HAL/SysTick */
}

static void putc_uart(char c)
{
    /* Keep heartbeat alive during a long text dump as well as while idle. */
    while (!(USART3->SR & USART_SR_TXE)) { led_service(); }
    USART3->DR = (uint8_t)c;
}
static void text_uart(const char *s) { while (*s) putc_uart(*s++); }
static void dec_uart(uint32_t x)
{
    char b[10]; unsigned n=0;
    do { b[n++]=(char)('0'+x%10u); x/=10u; } while(x);
    while(n) putc_uart(b[--n]);
}
static void hex_uart(uint32_t x, unsigned digits)
{
    static const char h[]="0123456789ABCDEF";
    while(digits) { --digits; putc_uart(h[(x>>(digits*4u))&15u]); }
}
static int getc_uart(void)
{
    uint32_t s=USART3->SR;
    if(s & (USART_SR_RXNE | USART_SR_ORE)) {
        int c=(int)(USART3->DR & 255u); /* SR then DR clears ORE */
        if(s & USART_SR_RXNE) return c;
    }
    return -1;
}

static void board_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN
                 | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN
                 | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN | RCC_APB1ENR_SPI3EN
                 | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM5EN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN | RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* ST-LINK virtual COM port, PD8 TX / PD9 RX. */
    gpio_af(GPIOD,8,7,0); gpio_af(GPIOD,9,7,1);
    GPIOD->OSPEEDR |= (3u<<16) | (3u<<18);
    USART3->CR1=0; USART3->CR2=0; USART3->CR3=0;
    USART3->BRR=(42000000u+57600u)/115200u;
    USART3->CR1=USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* SPI3 observes robot MCU -> radio (MOSI).
     * SPI4 observes radio -> robot MCU (MISO on the target),
     * connected deliberately to SPI4_MOSI, its receive input.
     * All observed lines are INPUT functions, never driven.
     */
    gpio_af(GPIOA,15,6,1); /* CSN -> SPI3_NSS; weak pull-up */
    gpio_af(GPIOC,10,6,0); /* SCK -> SPI3_SCK */
    gpio_af(GPIOC,12,6,0); /* MOSI -> SPI3_MOSI */
    gpio_af(GPIOE,4,5,1);  /* CSN -> SPI4_NSS */
    gpio_af(GPIOE,2,5,0);  /* SCK -> SPI4_SCK */
    gpio_af(GPIOE,6,5,0);  /* MISO -> SPI4_MOSI */
    /* Unused SPI output pins stay analog; do not wire these pins. */
    GPIOC->MODER |= (3u << 22); /* PC11 */
    GPIOE->MODER |= (3u << 10); /* PE5 */
    GPIOC->PUPDR &= ~(3u << 22);
    GPIOE->PUPDR &= ~(3u << 10);

    /* v2 wiring: PA0 = SCK (TIM2_ETR), PB3 = CSN (TIM2_CH2).
     * PA1 is deliberately unused: connected to Ethernet REF_CLK on Nucleo.
     * Do not enable SWO/JTAG trace on PB3; use ordinary SWD only.
     */
    gpio_af(GPIOA,0,1,0);
    gpio_af(GPIOB,3,1,1);
    TIM2->CR1=0; TIM2->DIER=0; TIM2->CCER=0;
    TIM2->PSC=0; TIM2->ARR=0xFFFFFFFFu;
    TIM2->CCMR1=TIM_CCMR1_CC2S_0; /* IC2 directly from TI2, no filter */
    TIM2->SMCR=TIM_SMCR_ECE; /* count SCK rising edges through ETR */
    TIM2->EGR=TIM_EGR_UG; TIM2->CNT=0; TIM2->SR=0;
    TIM5->CR1=0; TIM5->PSC=83u; TIM5->ARR=0xFFFFFFFFu;
    TIM5->EGR=TIM_EGR_UG; TIM5->CNT=0; TIM5->SR=0;
    TIM5->CR1=TIM_CR1_CEN; /* 84 MHz / 84 -> 1 us, HSI accuracy */
    EXTI->IMR &= ~1u;
    NVIC_DisableIRQ(EXTI0_IRQn);
    EXTI->PR=1u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Stop producers, let already issued DMA requests settle, then stop DMA. */
static void stop_capture(uint32_t reason)
{
    TIM2->CCER=0;
    TIM2->CR1=0;
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI4->CR1 &= ~SPI_CR1_SPE;
    uint32_t t=now_us();
    while ((uint32_t)(now_us()-t)<2u) {}
    TIM2->DIER=0;
    SPI3->CR2=0; SPI4->CR2=0;
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream0->CR | DMA2_Stream0->CR | DMA1_Stream6->CR)
           & DMA_SxCR_EN) {}
    __DSB();
    running=0; stop_reason=reason;
}

static uint32_t hw_errors(void)
{
    uint32_t f=0;
    if ((SPI3->SR | SPI4->SR) & SPI_SR_OVR) f |= F_SPI_OVR;
    if ((DMA1->LISR | DMA2->LISR) &
        (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) f |= F_DMA;
    if (DMA1->HISR &
        (DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6)) f |= F_DMA;
    if (TIM2->SR & TIM_SR_CC2OF) f |= F_EDGE_LOSS;
    return f;
}

static void dma_prepare(DMA_Stream_TypeDef *s, unsigned channel,
                        SPI_TypeDef *spi, volatile uint8_t *dst)
{
    s->CR=0; s->FCR=0;
    s->PAR=(uint32_t)(uintptr_t)&spi->DR;
    s->M0AR=(uint32_t)(uintptr_t)dst;
    s->NDTR=BYTE_CAP;
    s->CR=(channel<<25) | DMA_SxCR_MINC | DMA_SxCR_PL_1; /* high */
}

static void boundary_prepare(void)
{
    DMA1_Stream6->CR=0; DMA1_Stream6->FCR=0;
    DMA1_Stream6->PAR=(uint32_t)(uintptr_t)&TIM2->CCR2;
    DMA1_Stream6->M0AR=(uint32_t)(uintptr_t)frame_sck;
    DMA1_Stream6->NDTR=FRAME_CAP;
    /* RM0090/AN4031: TIM2_CH2 -> DMA1 stream6 channel3, 32-bit words. */
    DMA1_Stream6->CR=(3u<<25) | DMA_SxCR_MINC | DMA_SxCR_PL
                         | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1;
}

static int arm_capture(void)
{
    uint32_t deadline=now_us();
    frame_count=0; complete_bytes=0; raw_a=0; raw_b=0; raw_frames=0; total_sck=0;
    flags=0; running=0; stop_reason=STOP_WINDOW;
    for (;;) {
        stop_capture(STOP_WINDOW);
        RCC->APB1RSTR |= RCC_APB1RSTR_SPI3RST | RCC_APB1RSTR_TIM2RST;
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI4RST;
        RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI3RST | RCC_APB1RSTR_TIM2RST);
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;
        SPI3->CR1=SPI_CR1_RXONLY; SPI4->CR1=SPI_CR1_RXONLY;
        SPI3->CR2=SPI_CR2_RXDMAEN; SPI4->CR2=SPI_CR2_RXDMAEN;
        TIM2->PSC=0; TIM2->ARR=0xFFFFFFFFu;
        TIM2->CCMR1=TIM_CCMR1_CC2S_0;
        TIM2->SMCR=TIM_SMCR_ECE;
        TIM2->EGR=TIM_EGR_UG; TIM2->CNT=0; TIM2->SR=0;
        /* Count SCK already while waiting: reject activity during arming. */
        TIM2->CR1=TIM_CR1_CEN;
        DMA1->LIFCR=0x3Du; DMA2->LIFCR=0x3Du;
        DMA1->HIFCR=0x003D0000u;
        dma_prepare(DMA1_Stream0,0,SPI3,cmd_bytes);
        dma_prepare(DMA2_Stream0,4,SPI4,reply_bytes);
        boundary_prepare();
        uint32_t stable=now_us(), last=TIM2->CNT;
        while (1) {
            if ((uint32_t)(now_us()-deadline)>=ARM_TIMEOUT_US) {
                flags=F_ARM; stop_capture(STOP_ERROR); return 0;
            }
            uint32_t count=TIM2->CNT;
            if (!cs_high() || !sck_low() || count!=last) {
                stable=now_us(); last=count;
            } else if ((uint32_t)(now_us()-stable)>=IDLE_US) break;
        }
        __disable_irq();
        TIM2->CNT=0; TIM2->SR=0;
        DMA1_Stream0->CR |= DMA_SxCR_EN;
        DMA2_Stream0->CR |= DMA_SxCR_EN;
        DMA1_Stream6->CR |= DMA_SxCR_EN;
        SPI3->CR1 |= SPI_CR1_SPE; SPI4->CR1 |= SPI_CR1_SPE;
        TIM2->DIER=TIM_DIER_CC2DE;
        TIM2->CCER=TIM_CCER_CC2E; /* latch SCK count on rising CSN */
        t_start=now_us();
        __DSB();
        if (cs_high() && sck_low() && TIM2->CNT==0u
            && DMA1_Stream0->NDTR==BYTE_CAP
            && DMA2_Stream0->NDTR==BYTE_CAP
            && DMA1_Stream6->NDTR==FRAME_CAP
            && !(TIM2->SR & (TIM_SR_CC2IF | TIM_SR_CC2OF))) {
            running=1; __enable_irq(); return 1;
        }
        stop_capture(STOP_WINDOW);
        __enable_irq();
    }
}

/* Pure post-capture validation. Invalid boundaries never become good frames. */
static uint32_t validate_boundaries(const volatile uint32_t *ends,
                                    uint32_t count, uint32_t available,
                                    uint32_t *valid)
{
    *valid=0;
    if (count>FRAME_CAP || available>BYTE_CAP) return F_RANGE;
    uint32_t previous=0;
    for (uint32_t i=0;i<count;++i) {
        uint32_t bits=ends[i];
        if (bits & 7u) return F_ALIGN;
        if (bits<previous || bits/8u>available) return F_RANGE;
        previous=bits;
        *valid=i+1u;
    }
    return 0;
}

static void capture(void)
{
    led_set(LED_BUSY);
    text_uart("ARMING: all CSN high, SCK low\r\n");
    if (!arm_capture()) { led_set(LED_ERROR); return; }
    while (running) {
        uint32_t f=hw_errors();
        if (f) { flags |= f; stop_capture(STOP_ERROR); break; }
        uint32_t reason=STOP_WINDOW;
        int finish=((uint32_t)(now_us()-t_start)>=WINDOW_US);
        if (!DMA1_Stream0->NDTR || !DMA2_Stream0->NDTR) {
            reason=STOP_BYTES; finish=1;
        }
        if (!DMA1_Stream6->NDTR) { reason=STOP_FRAMES; finish=1; }
        if (getc_uart()=='s') { reason=STOP_USER; finish=1; }
        if (finish) stop_capture(reason);
    }
    flags |= hw_errors();
    raw_a=BYTE_CAP-DMA1_Stream0->NDTR;
    raw_b=BYTE_CAP-DMA2_Stream0->NDTR;
    raw_frames=FRAME_CAP-DMA1_Stream6->NDTR;
    total_sck=TIM2->CNT;
    if (raw_a!=raw_b) flags |= F_RX_COUNT;
    /* Dedicated SPI clock is required: out-of-CSN clocks are unsupported.
     * Stops at a byte boundary can conservatively flag a one-byte skew:
     * TIM2 and the two SPI blocks are stopped sequentially, not atomically.
     */
    if (total_sck/8u!=raw_a || total_sck/8u!=raw_b) flags |= F_CLOCK_TOTAL;
    uint32_t n=0, available=(raw_a<raw_b) ? raw_a : raw_b;
    flags |= validate_boundaries(frame_sck,raw_frames,available,&n);
    frame_count=n;
    complete_bytes=n ? frame_sck[n-1u]/8u : 0u;
    if (flags) stop_reason=STOP_ERROR;
    led_set(led_classify(flags,stop_reason,frame_count,complete_bytes));
}

static void dump(void)
{
    static const char *const reasons[]={"WINDOW","BYTES_FULL","FRAMES_FULL","ERROR","USER"};
    text_uart("#BEGIN version=2 reason="); text_uart(reasons[stop_reason]);
    text_uart(" flags=0x"); hex_uart(flags,8);
    text_uart(" frames="); dec_uart(frame_count);
    text_uart(" complete_bytes="); dec_uart(complete_bytes);
    text_uart(" raw_mosi="); dec_uart(raw_a);
    text_uart(" raw_miso="); dec_uart(raw_b);
    text_uart(" raw_frames="); dec_uart(raw_frames);
    text_uart(" total_sck="); dec_uart(total_sck);
    text_uart(" timestamps=none\r\n#F,index,end_sck,MOSI_hex,MISO_hex\r\n");
    uint32_t start=0;
    for (uint32_t n=0;n<frame_count;++n) {
        uint32_t bits=frame_sck[n], end=bits/8u;
        text_uart("F,"); dec_uart(n); putc_uart(','); dec_uart(bits); putc_uart(',');
        for (uint32_t i=start;i<end;++i) hex_uart(cmd_bytes[i],2);
        putc_uart(',');
        for (uint32_t i=start;i<end;++i) hex_uart(reply_bytes[i],2);
        text_uart("\r\n"); start=end;
    }
    if (flags) {
        text_uart("#ERROR raw_boundaries_prefix=");
        uint32_t n=(raw_frames<8u) ? raw_frames : 8u;
        for (uint32_t i=0;i<n;++i) {
            if (i) putc_uart(',');
            dec_uart(frame_sck[i]);
        }
        text_uart("\r\n");
    }
    text_uart("#END\r\n");
}

static void help(void)
{
    text_uart("\r\nF429 SPI RECORDER v2.0 HW; 115200 8N1\r\n"
              "NEW WIRING: CSN -> PA15,PE4,PB3 (CN12-31).\r\n"
              "SCK -> PC10,PE2,PA0 (CN11-28); NEVER join CSN and SCK.\r\n"
              "MOSI -> PC12; MISO -> PE6; common GND. Mode0 MSB8.\r\n"
              "a: capture up to 1s then dump; s: stop during capture\r\n"
              "d: repeat dump; h: help. No idle s diagnostic in v2.\r\n"
              "Boundaries: TIM2 SCK counter + CSN input capture + DMA.\r\n"
              "Format v2: end_sck is cumulative SCK edges, NOT microseconds.\r\n"
              "Flags: 1=bit alignment,2=boundary overcapture,4=RX counts,\r\n"
              "8=SPI overrun,16=DMA error,32=arming timeout,64=boundary range,\r\n"
              "128=SCK total vs RX count (includes possible stop-edge skew).\r\n"
              "LED: green alive/busy; blue=data/no detected error; red=error.\r\n");
#ifdef __OPTIMIZE__
    text_uart("#BUILD optimization_enabled=1; use -O2, SWD without SWO\r\n");
#else
    text_uart("#BUILD optimization_enabled=0; rebuild with -O2\r\n");
#endif
}

int main(void)
{
    SystemClock_Config(); board_init(); led_init(); led_boot_test(); help();
    int have_capture=0;
    for (;;) {
        led_service();
        int c=getc_uart();
        if (c=='a') { capture(); have_capture=1; dump(); text_uart("READY\r\n"); }
        else if (c=='d' && have_capture) dump();
        else if (c=='h') help();
    }
}

void Error_Handler(void)
{
    __disable_irq(); led_init(); led_write(LED_RED);
    for (;;) {}
}
