/* NUCLEO-F429ZI: SCK frequency meter for the robot's SPI bus (receive-only).
 *
 * How to use: create a NEW CubeMX project for STM32F429ZITx (same steps as for
 * the recorder: Debug = Serial Wire, nothing else enabled), then replace
 * Core/Src/main.c with this file. Open the ST-LINK Virtual COM Port at
 * 115200 8N1.
 *
 * Wiring: robot SCK -> PA0 (CN11 pin 28, verify against UM1974), GND -> GND.
 * Nothing else is needed. PA0 is an INPUT, never driven.
 *
 * Keys in the terminal:
 *   m : measure (every SCK edge is captured, best for up to ~3 MHz)
 *   f : fast measure (captures every 8th edge, for ~2..10 MHz)
 *   h : help
 * Trigger some robot activity (e.g. move the stick on the remote) while
 * measuring: SPI clock exists only while a transaction is running.
 *
 * Method: TIM2 channel 1 captures a timestamp (84 MHz timer clock) on every
 * rising SCK edge, DMA stores up to 2048 timestamps. Then the differences
 * between neighbouring timestamps are put in a histogram. The most common
 * difference is one SCK period. Gaps between bytes/transactions are outliers.
 * Accuracy is limited by the internal HSI oscillator (about +-1 %).
 *
 * NOT compiled or tested on hardware by the author of this file.
 */
#include "stm32f4xx.h"
#include <stdint.h>

#if !defined(STM32F429xx)
#error "Select STM32F429ZITx / STM32F429xx"
#endif

#define N_CAP        2048u
#define HIST_SIZE    8192u      /* deltas up to 8191 ticks (>= ~10 kHz SCK) */
#define TIMER_HZ     84000000u
#define TIMEOUT_US   5000000u   /* wait for edges at most 5 s */

static volatile uint32_t cap[N_CAP] __attribute__((aligned(4)));
static uint16_t hist[HIST_SIZE];

void Error_Handler(void);
void SystemClock_Config(void);

static uint32_t now_us(void) { return TIM5->CNT; }

static void gpio_af(GPIO_TypeDef *g, unsigned pin, unsigned af, unsigned pull)
{
    g->MODER = (g->MODER & ~(3u << (pin*2u))) | (2u << (pin*2u));
    g->PUPDR = (g->PUPDR & ~(3u << (pin*2u))) | (pull << (pin*2u));
    g->AFR[pin/8u] = (g->AFR[pin/8u] & ~(15u << ((pin%8u)*4u)))
                   | (af << ((pin%8u)*4u));
}

/* HSI 16 MHz -> PLL -> 168 MHz. APB1 = 42 MHz (timers 84 MHz), APB2 = 84 MHz. */
void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN
               | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR = 16u | (336u << 6) | (7u << 24); /* M=16, N=336, P=2, HSI */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
              | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    SystemCoreClock = 168000000u;
    SysTick->CTRL = 0u;
}

static void putc_uart(char c)
{
    while (!(USART3->SR & USART_SR_TXE)) {}
    USART3->DR = (uint8_t)c;
}
static void text_uart(const char *s) { while (*s) putc_uart(*s++); }
static void dec_uart(uint32_t x)
{
    char b[10]; unsigned n = 0;
    do { b[n++] = (char)('0' + x % 10u); x /= 10u; } while (x);
    while (n) putc_uart(b[--n]);
}
static int getc_uart(void)
{
    uint32_t s = USART3->SR;
    if (s & (USART_SR_RXNE | USART_SR_ORE)) {
        int c = (int)(USART3->DR & 255u);
        if (s & USART_SR_RXNE) return c;
    }
    return -1;
}

static void board_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN
                 | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN | RCC_APB1ENR_TIM2EN
                 | RCC_APB1ENR_TIM5EN;
    (void)RCC->APB1ENR;

    /* ST-LINK virtual COM port: PD8 TX, PD9 RX. */
    gpio_af(GPIOD, 8, 7, 0); gpio_af(GPIOD, 9, 7, 1);
    GPIOD->OSPEEDR |= (3u << 16) | (3u << 18);
    USART3->CR1 = 0; USART3->CR2 = 0; USART3->CR3 = 0;
    USART3->BRR = (42000000u + 57600u) / 115200u;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* SCK input: PA0 = TIM2_CH1 (AF1), no pull (SPI mode 0 idles low; a
     * weak pull-down is also fine if the line floats when robot is off). */
    gpio_af(GPIOA, 0, 1, 0);

    /* TIM5: 1 us time base for timeouts. */
    TIM5->CR1 = 0; TIM5->PSC = 83u; TIM5->ARR = 0xFFFFFFFFu;
    TIM5->EGR = TIM_EGR_UG; TIM5->CNT = 0; TIM5->SR = 0;
    TIM5->CR1 = TIM_CR1_CEN;
}

/* Capture up to N_CAP rising SCK edges (every 'div'-th edge, div = 1 or 8).
 * Returns number of timestamps stored. Sets *overcap if DMA lost captures. */
static uint32_t acquire(unsigned div, int *overcap, int *user_stop)
{
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {}
    DMA1->HIFCR = 0x3Fu << 6;            /* clear all flags of stream 5 */

    TIM2->CR1 = 0; TIM2->CCER = 0; TIM2->DIER = 0;
    TIM2->PSC = 0; TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCMR1 = 1u | (div == 8u ? (3u << 2) : 0u); /* CC1S=TI1, ICPSC */
    TIM2->EGR = TIM_EGR_UG; TIM2->CNT = 0; TIM2->SR = 0;

    DMA1_Stream5->CR = 0; DMA1_Stream5->FCR = 0;
    DMA1_Stream5->PAR  = (uint32_t)(uintptr_t)&TIM2->CCR1;
    DMA1_Stream5->M0AR = (uint32_t)(uintptr_t)cap;
    DMA1_Stream5->NDTR = N_CAP;
    DMA1_Stream5->CR = (3u << 25) | DMA_SxCR_MINC | DMA_SxCR_PL
                     | (2u << DMA_SxCR_PSIZE_Pos) | (2u << DMA_SxCR_MSIZE_Pos);
    DMA1_Stream5->CR |= DMA_SxCR_EN;

    TIM2->CCER = TIM_CCER_CC1E;          /* rising edge */
    TIM2->DIER = TIM_DIER_CC1DE;
    TIM2->CR1  = TIM_CR1_CEN;

    uint32_t t0 = now_us();
    *user_stop = 0;
    while (DMA1_Stream5->NDTR != 0u) {
        if ((uint32_t)(now_us() - t0) >= TIMEOUT_US) break;
        if (getc_uart() == 's') { *user_stop = 1; break; }
    }

    *overcap = (TIM2->SR & TIM_SR_CC1OF) ? 1 : 0;
    if (DMA1->HISR & ((1u << 9) | (1u << 8) | (1u << 6))) *overcap = 1;
    TIM2->CR1 = 0; TIM2->CCER = 0; TIM2->DIER = 0;
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {}
    return N_CAP - DMA1_Stream5->NDTR;
}

static void print_khz(uint32_t hz)
{
    dec_uart(hz / 1000u); putc_uart('.');
    uint32_t r = hz % 1000u;
    if (r < 100u) putc_uart('0');
    if (r < 10u) putc_uart('0');
    dec_uart(r);
}

static void measure(unsigned div)
{
    int overcap, user_stop;
    text_uart(div == 8u ? "FAST mode (every 8th edge). " : "NORMAL mode (every edge). ");
    text_uart("Waiting for SCK on PA0 (max 5 s, 's' = stop)...\r\n");
    uint32_t n = acquire(div, &overcap, &user_stop);

    text_uart("edges_captured="); dec_uart(n); text_uart("\r\n");
    if (n < 20u) {
        text_uart("RESULT: too few edges. Is SCK connected to PA0 and GND common?\r\n"
                  "Is the robot doing SPI right now (move the remote's stick)?\r\n");
        return;
    }

    for (uint32_t i = 0; i < HIST_SIZE; ++i) hist[i] = 0;
    uint32_t big = 0;
    for (uint32_t i = 1; i < n; ++i) {
        uint32_t d = cap[i] - cap[i-1u];
        if (d < HIST_SIZE) hist[d]++; else big++;
    }

    /* Mode of the histogram, smoothed over 3 neighbouring bins (jitter). */
    uint32_t best = 0, mode = 0;
    for (uint32_t k = 2; k + 1u < HIST_SIZE; ++k) {
        uint32_t s = (uint32_t)hist[k-1u] + hist[k] + hist[k+1u];
        if (s > best) { best = s; mode = k; }
    }
    if (mode == 0u) {
        text_uart("RESULT: no stable period found (signal too slow or noisy).\r\n");
        return;
    }

    /* Mean of all deltas within +-25 % of the mode = one clock period. */
    uint32_t lo = mode - mode / 4u, hi = mode + mode / 4u + 1u;
    uint64_t sum = 0; uint32_t cnt = 0, outliers = big;
    for (uint32_t k = 1; k < HIST_SIZE; ++k) {
        if (k >= lo && k <= hi) { sum += (uint64_t)k * hist[k]; cnt += hist[k]; }
        else outliers += hist[k];
    }
    uint32_t hz = (uint32_t)((uint64_t)TIMER_HZ * div * cnt / sum);

    text_uart("period_ticks(84MHz)="); dec_uart(mode);
    text_uart("  periods_used="); dec_uart(cnt);
    text_uart("  outliers(gaps)="); dec_uart(outliers); text_uart("\r\n");
    text_uart("RESULT: SCK ~ "); dec_uart(hz); text_uart(" Hz  ("); print_khz(hz);
    text_uart(" kHz)\r\n");
    text_uart("Accuracy about +-1% (internal HSI oscillator).\r\n");
    if (div == 1u && outliers == 0u && cnt + 1u >= N_CAP)
        text_uart("Note: no gaps seen; the whole buffer was one burst.\r\n");
    if (overcap) {
        text_uart("WARNING: some edges were lost (DMA too slow for this SCK).\r\n"
                  "Result is NOT reliable. Repeat with 'f' (fast mode).\r\n");
    }
    if (div == 8u)
        text_uart("(In fast mode gaps between bytes are not analysed.)\r\n");
}

static void help(void)
{
    text_uart("\r\nF429 SCK METER v1; 115200 8N1\r\n"
              "Wiring: robot SCK -> PA0 (CN11-28), robot GND -> GND. Input only.\r\n"
              "m: measure (every edge)   f: fast measure (every 8th edge)\r\n"
              "s: stop while waiting      h: help\r\n"
              "Move the remote's stick while measuring.\r\n");
}

int main(void)
{
    SystemClock_Config(); board_init(); help();
    for (;;) {
        int c = getc_uart();
        if (c == 'm') { measure(1u); text_uart("READY\r\n"); }
        else if (c == 'f') { measure(8u); text_uart("READY\r\n"); }
        else if (c == 'h') help();
    }
}

void Error_Handler(void) { __disable_irq(); for (;;) {} }
