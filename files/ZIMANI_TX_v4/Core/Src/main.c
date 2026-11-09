/* NUCLEO-F429ZI, ZimAni transmitter v4.0.0. NOT the passive recorder.
 * Original remote MCU must be electrically disconnected from radio pins.
 * Read README_RU.md before wiring. Core clock 168MHz HSI, SPI3 mode0 ~1.31MHz.
 */
#include "stm32f4xx.h"
#include "zimani_control.h"
#if !defined(STM32F429xx)
#error Select STM32F429xx
#endif
#define RX_SIZE 512u
#define TX_SIZE 1024u
static ZControl control;
static volatile uint8_t uart_rx[RX_SIZE];
static uint8_t uart_tx[TX_SIZE],host_frame[HOST_FRAME_SIZE];
static uint32_t rx_pos,tx_in,tx_out,host_used,uart_errors;
static uint32_t ack_count,fail_count,reply_count,bad_reply_count,spi_errors;
static uint8_t radio_ok,spi_ok,last_status,last_reply[23],reply_seen;
static uint32_t last_reply_us;
static uint32_t now_us(void) {return TIM5->CNT;}
static int due(uint32_t now,uint32_t at) {return (int32_t)(now-at)>=0;}
static void delay_us(uint32_t n) {uint32_t t=now_us();while((uint32_t)(now_us()-t)<n) {}}
static void ce(int high) {GPIOE->BSRR=high?(1u<<3):(1u<<19);}
static void cs(int high) {GPIOA->BSRR=high?(1u<<15):(1u<<31);}
static void gpio_af(GPIO_TypeDef *g,unsigned pin,unsigned af,unsigned pull)
{
 g->MODER=(g->MODER&~(3u<<(2u*pin)))|(2u<<(2u*pin));
 g->PUPDR=(g->PUPDR&~(3u<<(2u*pin)))|(pull<<(2u*pin));
 g->AFR[pin/8]=(g->AFR[pin/8]&~(15u<<(4u*(pin%8))))|(af<<(4u*(pin%8)));
}
static void early_gpio(void)
{
 RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOEEN;
 (void)RCC->AHB1ENR;
 ce(0);cs(1); /* Set levels BEFORE changing mode to output. */
 GPIOE->MODER=(GPIOE->MODER&~(3u<<6))|(1u<<6);
 GPIOE->OTYPER&=~(1u<<3);
 GPIOA->MODER=(GPIOA->MODER&~(3u<<30))|(1u<<30);
 GPIOA->OTYPER&=~(1u<<15);
 const uint32_t fields=(3u<<0)|(3u<<14)|(3u<<28);
 GPIOB->MODER=(GPIOB->MODER&~fields)|(1u<<0)|(1u<<14)|(1u<<28);
 GPIOB->OTYPER&=~((1u<<0)|(1u<<7)|(1u<<14));
 GPIOB->BSRR=((1u<<0)|(1u<<7)|(1u<<14))<<16;
 GPIOC->MODER&=~(3u<<26);GPIOC->PUPDR=(GPIOC->PUPDR&~(3u<<26))|(2u<<26);
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
static void peripherals(void)
{
 RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
 RCC->APB1ENR|=RCC_APB1ENR_TIM5EN|RCC_APB1ENR_USART3EN|RCC_APB1ENR_SPI3EN;
 (void)RCC->APB1ENR;
 TIM5->PSC=83;TIM5->ARR=0xFFFFFFFFu;TIM5->EGR=TIM_EGR_UG;TIM5->CR1=TIM_CR1_CEN;
 gpio_af(GPIOD,8,7,0);gpio_af(GPIOD,9,7,1);
 USART3->CR1=0;USART3->CR2=0;USART3->CR3=USART_CR3_DMAR;
 USART3->BRR=(42000000u+57600u)/115200u;
 DMA1_Stream1->CR=0;while(DMA1_Stream1->CR&DMA_SxCR_EN) {}
 DMA1->LIFCR=0xF40u;DMA1_Stream1->FCR=0;
 DMA1_Stream1->PAR=(uint32_t)(uintptr_t)&USART3->DR;
 DMA1_Stream1->M0AR=(uint32_t)(uintptr_t)uart_rx;DMA1_Stream1->NDTR=RX_SIZE;
 /* AN4031: USART3_RX = DMA1 stream1 channel4. Circular, byte transfers. */
 DMA1_Stream1->CR=(4u<<25)|DMA_SxCR_MINC|DMA_SxCR_CIRC|DMA_SxCR_PL_1|DMA_SxCR_EN;
 USART3->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
 gpio_af(GPIOC,10,6,0);gpio_af(GPIOC,11,6,1);gpio_af(GPIOC,12,6,0);
 GPIOC->OSPEEDR|=(2u<<20)|(2u<<24);
 SPI3->CR1=SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_2;
 SPI3->CR2=0;SPI3->CR1|=SPI_CR1_SPE;
}
static void uart_byte(uint8_t b)
{
 if(!host_used && b!=0xC3u) return;
 host_frame[host_used++]=b;
 if(host_used==2u && host_frame[1]!=0x3Cu) {
   host_used=(b==0xC3u)?1u:0u;host_frame[0]=b;return;
 }
 if(host_used==HOST_FRAME_SIZE) {
   if(z_accept(&control,host_frame,now_us())) host_used=0;
   else {
     for(unsigned i=1;i<HOST_FRAME_SIZE;i++) host_frame[i-1]=host_frame[i];
     host_used=HOST_FRAME_SIZE-1u;
     /* Search remaining suffix for the next sync prefix. */
     while(host_used && host_frame[0]!=0xC3u) {
       for(unsigned i=1;i<host_used;i++) host_frame[i-1]=host_frame[i];
       host_used--;
     }
   }
 }
}
static void uart_service(void)
{
 uint32_t sr=USART3->SR;
 if(sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE)) {
   (void)USART3->DR;uart_errors++;z_stop(&control,FAULT_UART);host_used=0;
   rx_pos=(RX_SIZE-DMA1_Stream1->NDTR)%RX_SIZE;
 }
 if(DMA1->LISR&(DMA_LISR_TEIF1|DMA_LISR_DMEIF1|DMA_LISR_FEIF1)) {
   DMA1->LIFCR=0xF40u;uart_errors++;z_stop(&control,FAULT_UART);
 }
 uint32_t end=(RX_SIZE-DMA1_Stream1->NDTR)%RX_SIZE;
 while(rx_pos!=end) {uint8_t b=uart_rx[rx_pos];rx_pos=(rx_pos+1u)%RX_SIZE;uart_byte(b);}
 if(tx_out!=tx_in && (USART3->SR&USART_SR_TXE)) {
   USART3->DR=uart_tx[tx_out];tx_out=(tx_out+1u)%TX_SIZE;
 }
}
static void uart_queue(const char *s)
{
 uint32_t n=0;while(s[n]) n++;
 uint32_t free=(tx_out+TX_SIZE-tx_in-1u)%TX_SIZE;
 if(n>free) return; /* Drop whole telemetry line; never stall command handling. */
 while(*s) {uart_tx[tx_in]=(uint8_t)*s++;tx_in=(tx_in+1u)%TX_SIZE;}
}
static uint8_t spi(uint8_t b)
{
 uint32_t t=now_us();
 while(!(SPI3->SR&SPI_SR_TXE)) if((uint32_t)(now_us()-t)>500u) {spi_ok=0;return 0xFF;}
 *(volatile uint8_t *)&SPI3->DR=b;
 t=now_us();
 while(!(SPI3->SR&SPI_SR_RXNE)) if((uint32_t)(now_us()-t)>500u) {spi_ok=0;return 0xFF;}
 return *(volatile uint8_t *)&SPI3->DR;
}
static void end_spi(void)
{
 uint32_t t=now_us();
 while(SPI3->SR&SPI_SR_BSY) if((uint32_t)(now_us()-t)>500u) {spi_ok=0;break;}
 cs(1);delay_us(1);
}
static uint8_t radio_cmd(uint8_t cmd)
{cs(0);uint8_t s=spi(cmd);end_spi();return s;}
static void write_reg(uint8_t reg,uint8_t value)
{cs(0);spi((uint8_t)(0x20u|reg));spi(value);end_spi();}
static uint8_t read_reg(uint8_t reg)
{cs(0);spi(reg);uint8_t v=spi(0xFF);end_spi();return v;}
static void write_many(uint8_t cmd,const uint8_t *data,unsigned n)
{cs(0);spi(cmd);for(unsigned i=0;i<n;i++) spi(data[i]);end_spi();}
static void read_many(uint8_t cmd,uint8_t *data,unsigned n)
{cs(0);spi(cmd);for(unsigned i=0;i<n;i++) data[i]=spi(0xFF);end_spi();}
static int radio_init(void)
{
 static const uint8_t addr[]={ZIMANI_RADIO_ADDRESS};
 uint8_t got[5];ce(0);cs(1);spi_ok=1;
 /* Reinitialize peripheral after any previous bounded SPI timeout. */
 RCC->APB1RSTR|=RCC_APB1RSTR_SPI3RST;RCC->APB1RSTR&=~RCC_APB1RSTR_SPI3RST;
 SPI3->CR1=SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_BR_2|SPI_CR1_SPE;
 /* Captured startup: test address, then two configuration passes (22 -> 23).
  * CE stays low. Timings are conservative design choices, not measured in logs.
  */
 static const uint8_t probe[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
 write_many(0x30,probe,5);read_many(0x10,got,5);
 for(unsigned i=0;i<5;i++) if(got[i]!=probe[i]) return 0;
 for(unsigned pass=0;pass<2;pass++) {
   write_reg(7,0x70);write_reg(0x1C,1);write_reg(0x1D,7);
   /* Compatibility fallback only if FEATURE is locked; not observed in captures. */
   if(read_reg(0x1D)!=7u) {uint8_t a=0x73;write_many(0x50,&a,1);write_reg(0x1D,7);write_reg(0x1C,1);}
   write_reg(0,0x0A);write_reg(1,1);write_reg(2,1);write_reg(3,3);
   write_reg(4,RADIO_RETRIES);write_reg(5,RADIO_CHANNEL);
   write_reg(6,pass?RADIO_RF_SETUP:0x22u);
   write_many(0x30,addr,5);write_many(0x2A,addr,5);
   write_reg(7,0x70);radio_cmd(0xE1);
 }
 radio_cmd(0xE2);
 write_reg(0,0x0A);delay_us(5000);
 if(read_reg(0)!=0x0A||read_reg(1)!=1||read_reg(2)!=1||read_reg(3)!=3
    ||read_reg(4)!=RADIO_RETRIES||read_reg(5)!=RADIO_CHANNEL
    ||(read_reg(6)&0x3Eu)!=(RADIO_RF_SETUP&0x3Eu)
    ||read_reg(0x1D)!=7||read_reg(0x1C)!=1) return 0;
 read_many(0x10,got,5);for(unsigned i=0;i<5;i++) if(got[i]!=addr[i]) return 0;
 read_many(0x0A,got,5);for(unsigned i=0;i<5;i++) if(got[i]!=addr[i]) return 0;
 if(!spi_ok) return 0;
 write_reg(0,0x0B);ce(1);delay_us(150);return spi_ok;
}
static void receive_payloads(void)
{
 for(unsigned k=0;k<3;k++) {
   if(read_reg(0x17)&1u) break;
   uint8_t size=0;read_many(0x60,&size,1);
   if(!size||size>32u) {radio_cmd(0xE2);bad_reply_count++;break;}
   uint8_t data[32];read_many(0x61,data,size);
   if(z_reply_valid(data,size)) {
     for(unsigned i=0;i<23;i++) last_reply[i]=data[i];
     last_reply_us=now_us();reply_seen=1;reply_count++;z_link(&control,last_reply_us);
   } else bad_reply_count++;
 }
 write_reg(7,0x40);
}
static uint32_t radio_state,next_tx,tx_started,pulse_end,next_poll,next_init;
static void open_receive(void)
{
 ce(0);receive_payloads();write_reg(7,0x30);write_reg(0,0x0B);ce(1);
 uint32_t t=now_us();next_poll=t+150u;
 uint32_t normal=tx_started+RADIO_PERIOD_US;
 next_tx=due(t+RADIO_MIN_RX_US,normal)?t+RADIO_MIN_RX_US:normal;
 radio_state=0;
}
static void radio_service(void)
{
 uint32_t t=now_us();
 if(!radio_ok) {
   if(!due(t,next_init)) return;
   z_stop(&control,FAULT_RADIO);control.link_seen=0;
   radio_ok=(uint8_t)radio_init();next_init=now_us()+1000000u;
   if(radio_ok) {last_status=0;radio_state=0;next_tx=now_us()+3000u;next_poll=now_us()+150u;}
   else {ce(0);cs(1);spi_errors++;}
   return;
 }
 if(radio_state==1u && due(t,pulse_end)) {ce(0);radio_state=2;next_poll=t;}
 if(radio_state==2u && due(t,next_poll)) {
   last_status=radio_cmd(0xFF);next_poll=now_us()+200u;
   if(last_status&0x20u) {
     ack_count++;open_receive(); /* ACK is not the separate BB application reply. */
   } else if((last_status&0x10u)||(uint32_t)(t-tx_started)>=RADIO_TX_TIMEOUT_US) {
     fail_count++;ce(0);radio_cmd(0xE1);open_receive();
   }
 } else if(radio_state==0u) {
   if(due(t,next_poll)) {
     last_status=radio_cmd(0xFF);next_poll=now_us()+500u;
     if((last_status&0x40u)||!(read_reg(0x17)&1u)) receive_payloads();
   }
   if(due(now_us(),next_tx)) {
     uint8_t payload[26];
     z_tick(&control,now_us());
     if(GPIOC->IDR&(1u<<13)) z_stop(&control,FAULT_BUTTON);
     z_payload(&control.command,payload);
     ce(0);receive_payloads();write_reg(0,0x0A);write_reg(7,0x70);radio_cmd(0xE1);
     write_many(0xA0,payload,26);
     tx_started=now_us();ce(1);pulse_end=tx_started+20u;radio_state=1;
   }
 }
 if(!spi_ok||last_status==0xFFu) {
   spi_errors++;radio_ok=0;ce(0);cs(1);z_stop(&control,FAULT_RADIO);
   control.link_seen=0;next_init=now_us()+1000000u;
 }
}
static char status_line[640];static unsigned slen;
static void append(const char *s)
{while(*s && slen<sizeof(status_line)-1u) status_line[slen++]=*s++;status_line[slen]=0;}
static void number(uint32_t n)
{char b[10];unsigned k=0;do {b[k++]=(char)('0'+n%10u);n/=10u;}while(n);while(k) {char x[2]={b[--k],0};append(x);}}
static void signed_number(int32_t n) {if(n<0) {append("-");n=-n;}number((uint32_t)n);}
static void telemetry(void)
{
 uint32_t t=now_us();slen=0;
 append("S v=4 session=");number(control.session);append(" seq=");number(control.seq);
 append(" armed=");number(control.armed);append(" radio=");number(radio_ok);
 append(" fault=");number(control.fault);append(" host_ms=");number(control.host_seen?(t-control.last_host_us)/1000u:999999u);
 append(" link_ms=");number(control.link_seen?(t-control.last_link_us)/1000u:999999u);
 append(" ack=");number(ack_count);append(" fail=");number(fail_count);append(" rx=");number(reply_count);
 append(" badrx=");number(bad_reply_count);append(" reject=");number(control.rejected);
 append(" uart=");number(uart_errors);append(" spi=");number(spi_errors);
 append(" f=");signed_number(control.command.forward);append(" t=");signed_number(control.command.turn);
 append(" blade=");number(control.command.blade);append(" cruise=");number(control.command.cruise);
 append(" light=");number(control.command.light);append(" engine=");number(control.command.engine);
 append(" adjust=");number(control.command.engine_adjust);append(" estop=");number(control.command.estop);
 append(" can_arm=");number(control.can_arm);append(" stop_rx=");number(reply_seen?last_reply[11]:255u);
 append(" rx_ms=");number(reply_seen?(t-last_reply_us)/1000u:999999u);append(" reply=");
 if(reply_seen) {const char *h="0123456789ABCDEF";for(unsigned i=0;i<23;i++) {char b[3]={h[last_reply[i]>>4],h[last_reply[i]&15u],0};append(b);}}
 else append("-");
 append("\r\n");uart_queue(status_line);
}
static void leds(void)
{
 uint32_t t=now_us(),mask=0;
 if(control.armed||t%1000000u<100000u) mask|=1u;
 if(radio_ok&&control.link_seen) mask|=1u<<7;
 if(!radio_ok||control.fault||control.command.estop) mask|=1u<<14;
 const uint32_t all=(1u<<0)|(1u<<7)|(1u<<14);
 GPIOB->BSRR=((all&~mask)<<16)|mask;
}
static void watchdog_start(void)
{
 IWDG->KR=0x5555;IWDG->PR=4;IWDG->RLR=499;
 uint32_t t=now_us();while(IWDG->SR) if((uint32_t)(now_us()-t)>10000u) break;
 IWDG->KR=0xAAAA;IWDG->KR=0xCCCC; /* nominal ~1s, LSI tolerance applies */
}
int main(void)
{
 early_gpio();SystemClock_Config();peripherals();z_init(&control);
 delay_us(100000u);watchdog_start();
 uart_queue("ZIMANI TX v" ZIMANI_VERSION " NUCLEO-F429ZI; host binary v2; 115200 8N1\r\n");
 uint32_t next_status=now_us();next_init=now_us();
 for(;;) {
   uart_service();z_tick(&control,now_us());
   if(GPIOC->IDR&(1u<<13)) z_stop(&control,FAULT_BUTTON);
   radio_service();leds();
   if(due(now_us(),next_status)) {telemetry();next_status=now_us()+200000u;}
   IWDG->KR=0xAAAA;
 }
}
void Error_Handler(void) {ce(0);for(;;) {GPIOB->BSRR=1u<<14;}}
