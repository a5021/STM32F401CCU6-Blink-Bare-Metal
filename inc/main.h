#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "stm32f4xx.h"

#define ALL_ANALOG            UINT32_MAX
#define PIN_CONF(PIN, MODE)   ((uint32_t) MODE << (PIN * 2))
#define PIN_AF(PIN, AF)       ((uint64_t) AF << (4 * (PIN)))

#define PIN(PIN_NO)           (PIN_NO)
#define AF(PIN_NO)            (PIN_NO)

#define PIN_MODE_INPUT        ((uint32_t) 0)
#define PIN_MODE_OUTPUT       ((uint32_t) 1)
#define PIN_MODE_ALT_FUNC     ((uint32_t) 2)
#define PIN_MODE_ANALOG       ((uint32_t) 3)

#define PINV_INPUT            ((uint32_t) 3)
#define PINV_OUTPUT           ((uint32_t) 2)
#define PINV_ALT_FUNC         ((uint32_t) 1)
#define PINV_ANALOG           ((uint32_t) 0)

#define PIN_SPEED_LOW         ((uint32_t) 0)
#define PIN_SPEED_MEDIUM      ((uint32_t) 1)
#define PIN_SPEED_HIGH        ((uint32_t) 2)
#define PIN_SPEED_HIGHER      ((uint32_t) 3)

#define PIN_SPEED(PORT, CFG)  GPIO ## PORT->OSPEEDR = (CFG)

#define PIN_PULL_NO           ((uint32_t) 0)
#define PIN_PULL_UP           ((uint32_t) 1)
#define PIN_PULL_DOWN         ((uint32_t) 2)

#define PIN_LOW(PIN)          GPIO_BSRR_BR_ ## PIN
#define PIN_HIGH(PIN)         GPIO_BSRR_BS_ ## PIN

#define LOW                   GPIO_BSRR_BR_
#define HIGH                  GPIO_BSRR_BS_
#define LO                    0
#define HI                    1

#define PIN_STATE(PIN, STATE) (1 << ((PIN) + (16 * !(STATE))))

#define CAT(A, B)             A ## B
#define SWITCH_PIN(PORT, PIN, STATE) GPIO ## PORT->BSRR = CAT(STATE, PIN)
#define SW_PIN(PORT, STATE)   GPIO ## PORT->BSRR = (STATE)

#define DELAY_MS(MS)          for(int x = 0; x < MS; x += SysTick->CTRL >> SysTick_CTRL_COUNTFLAG_Pos){}

#define MHZ                   *1000000UL

#define UART_BAUDRATE(FCLK, P_SPEED)    (((FCLK) + ((P_SPEED)/2U)) / (P_SPEED))

#define READ_UART(_UART_NO, BUF, LEN)   for (int i = 0; i < (LEN); i++) {                                          \
                                          while((USART##_UART_NO->SR & USART_SR_RXNE) != USART_SR_RXNE) { /****/ } \
                                          (BUF)[i] = USART##_UART_NO->DR;                                          \
                                        }

#define WRITE_UART(_UART_NO, SRC, LEN)  for (int i = 0; i < LEN; i++) {                                            \
                                          while((USART##_UART_NO->SR & USART_SR_TXE) != USART_SR_TXE) { /****/ }   \
                                          USART##_UART_NO->DR = SRC[i];                                            \
                                        }

__STATIC_INLINE void u_putc(const char c) {
  while ((USART6->SR & USART_SR_TXE) == 0);
  USART6->DR = c;
}

__STATIC_INLINE void u_puts(const char *s) {
  while (*s != 0) {
    u_putc(*s++);
  }
}

  /* Convert a nibble to HEX and send it via UART */
__STATIC_INLINE void u_put_x(const uint8_t c) {
  /* Convert a nibble to HEX char and send it via UART */
  u_putc(((c < 10) ? c + '0' : c + '7'));
}

  /* Convert a byte to HEX and send it via UART */
__STATIC_INLINE void u_putx(const uint8_t c) {
  u_put_x(c >> 4);
  u_put_x(c & 0x0F);
}

  /* MACRO to emulate printf() via UART */
#define uprintf(...) for(char _b[160]; snprintf(_b, sizeof(_b), __VA_ARGS__), u_puts(_b), 0;){}

__STATIC_INLINE void init_sys(void) {

  RCC->AHB1ENR = (
    1 * RCC_AHB1ENR_GPIOAEN          | /*  0x00000001                                            */
    0 * RCC_AHB1ENR_GPIOBEN          | /*  0x00000002                                            */
    1 * RCC_AHB1ENR_GPIOCEN          | /*  0x00000004                                            */
    0 * RCC_AHB1ENR_GPIODEN          | /*  0x00000008                                            */
    0 * RCC_AHB1ENR_GPIOEEN          | /*  0x00000010                                            */
    0 * RCC_AHB1ENR_GPIOHEN          | /*  0x00000080                                            */
    0 * RCC_AHB1ENR_CRCEN            | /*  0x00001000                                            */
    0 * RCC_AHB1ENR_DMA1EN           | /*  0x00200000                                            */
    0 * RCC_AHB1ENR_DMA2EN             /*  0x00400000                                            */
  );

  RCC->AHB2ENR = (
    0 * RCC_AHB2ENR_OTGFSEN           /*  0x00000080                                             */
  );

  RCC->APB1ENR = (
    0 * RCC_APB1ENR_TIM2EN           | /*  0x00000001                                            */
    0 * RCC_APB1ENR_TIM3EN           | /*  0x00000002                                            */
    0 * RCC_APB1ENR_TIM4EN           | /*  0x00000004                                            */
    0 * RCC_APB1ENR_TIM5EN           | /*  0x00000008                                            */
    0 * RCC_APB1ENR_WWDGEN           | /*  0x00000800                                            */
    0 * RCC_APB1ENR_SPI2EN           | /*  0x00004000                                            */
    0 * RCC_APB1ENR_SPI3EN           | /*  0x00008000                                            */
    0 * RCC_APB1ENR_USART2EN         | /*  0x00020000                                            */
    0 * RCC_APB1ENR_I2C1EN           | /*  0x00200000                                            */
    0 * RCC_APB1ENR_I2C2EN           | /*  0x00400000                                            */
    0 * RCC_APB1ENR_I2C3EN           | /*  0x00800000                                            */
    1 * RCC_APB1ENR_PWREN              /*  0x10000000                                            */
  );

  RCC->APB2ENR = (
    0 * RCC_APB2ENR_TIM1EN           | /*  0x00000001                                            */
    0 * RCC_APB2ENR_USART1EN         | /*  0x00000010                                            */
    1 * RCC_APB2ENR_USART6EN         | /*  0x00000020                                            */
    0 * RCC_APB2ENR_ADC1EN           | /*  0x00000100                                            */
    0 * RCC_APB2ENR_SDIOEN           | /*  0x00000800                                            */
    0 * RCC_APB2ENR_SPI1EN           | /*  0x00001000                                            */
    0 * RCC_APB2ENR_SPI4EN           | /*  0x00002000                                            */
    0 * RCC_APB2ENR_SYSCFGEN         | /*  0x00004000                                            */
    0 * RCC_APB2ENR_TIM9EN           | /*  0x00010000                                            */
    0 * RCC_APB2ENR_TIM10EN          | /*  0x00020000                                            */
    0 * RCC_APB2ENR_TIM11EN            /*  0x00040000                                            */
  );

  /*  HSE / M * N / P                                                                            */

  /*  25Mhz / 15 * 120 / 2 = 100Mhz                                                              */

  RCC->PLLCFGR = (

    /* PLLM: Division factor for the main PLL (PLL) and audio PLL (PLLI2S) input clock           */
    /*      Caution:The software has to set these bits correctly to ensure that the VCO          */
    /*      input frequency ranges from 1 to 2 MHz. It is recommended to select                  */
    /*      a frequency of 2 MHz to limit PLL jitter.                                            */

    0 * RCC_PLLCFGR_PLLM             | /*  0x0000003F                                            */
    1 * RCC_PLLCFGR_PLLM_0           | /*    0x00000001                                          */
    1 * RCC_PLLCFGR_PLLM_1           | /*    0x00000002                                          */
    1 * RCC_PLLCFGR_PLLM_2           | /*    0x00000004                                          */
    1 * RCC_PLLCFGR_PLLM_3           | /*    0x00000008                                          */
    0 * RCC_PLLCFGR_PLLM_4           | /*    0x00000010                                          */
    0 * RCC_PLLCFGR_PLLM_5           | /*    0x00000020                                          */

    /* PLLN: Main PLL (PLL) multiplication factor for VCO                                        */
    /*      Caution:The software has to set these bits correctly to ensure that                  */
    /*      the VCO output frequency is between 192 and 432 MHz.                                 */

    0 * RCC_PLLCFGR_PLLN             | /*  0x00007FC0                                            */
    0 * RCC_PLLCFGR_PLLN_0           | /*    0x00000040                                          */
    0 * RCC_PLLCFGR_PLLN_1           | /*    0x00000080                                          */
    0 * RCC_PLLCFGR_PLLN_2           | /*    0x00000100                                          */
    1 * RCC_PLLCFGR_PLLN_3           | /*    0x00000200                                          */
    1 * RCC_PLLCFGR_PLLN_4           | /*    0x00000400                                          */
    1 * RCC_PLLCFGR_PLLN_5           | /*    0x00000800                                          */
    1 * RCC_PLLCFGR_PLLN_6           | /*    0x00001000                                          */
    0 * RCC_PLLCFGR_PLLN_7           | /*    0x00002000                                          */
    0 * RCC_PLLCFGR_PLLN_8           | /*    0x00004000                                          */

    /* PLLP: Main PLL (PLL) division factor for main system clock                                */
    /*      PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8          */
    /*      00: PLLP = 2                                                                         */
    /*      01: PLLP = 4                                                                         */
    /*      10: PLLP = 6                                                                         */
    /*      11: PLLP = 8                                                                         */

    0 * RCC_PLLCFGR_PLLP             | /*  0x00030000                                            */
    0 * RCC_PLLCFGR_PLLP_0           | /*    0x00010000                                          */
    0 * RCC_PLLCFGR_PLLP_1           | /*    0x00020000                                          */

    /* PLLSRC: Main PLL(PLL) and audio PLL (PLLI2S) entry clock source                           */
    /*      Set and cleared by software to select PLL and PLLI2S clock source.                   */
    /*      This bit can be written only when PLL and PLLI2S are disabled.                       */
    /*      0: HSI clock selected as PLL and PLLI2S clock entry                                  */
    /*      1: HSE oscillator clock selected as PLL and PLLI2S clock entry                       */

    1 * RCC_PLLCFGR_PLLSRC_HSE       | /*  0x00400000                                            */
    0 * RCC_PLLCFGR_PLLSRC_HSI       | /*                                                        */

    /* PLLQ: Main PLL (PLL) division factor for USB OTG FS, SDIO and RNG clocks                  */

    0 * RCC_PLLCFGR_PLLQ             | /*  0x0F000000                                            */
    0 * RCC_PLLCFGR_PLLQ_0           | /*    0x01000000                                          */
    0 * RCC_PLLCFGR_PLLQ_1           | /*    0x02000000                                          */
    0 * RCC_PLLCFGR_PLLQ_2           | /*    0x04000000                                          */
    0 * RCC_PLLCFGR_PLLQ_3             /*    0x08000000                                          */
  );

  RCC->CR = (
    RCC_CR_HSEON                     | /*  Switch HSE ON                                         */
    RCC_CR_PLLON                       /*  Switch PLL ON                                         */
  );

  PWR->CR |= PWR_CR_DBP;               /*  Enable Backup Domain Access                           */
  RCC->BDCR |= RCC_BDCR_LSEON;
  while(!(RCC->BDCR & RCC_BDCR_LSERDY)) { /*  */ }

  RCC->BDCR |= RCC_BDCR_RTCSEL_0;      /*  LSE oscillator clock used as RTC clock                */
  RCC->BDCR |= RCC_BDCR_RTCEN;         /*  Enable RTC                                            */

  PWR->CR &= ~PWR_CR_DBP;              /*  Disable Backup Domain Access                          */
  RCC->APB1ENR = 0;                    /*  Disable PWR interface                                 */

  while(!(RCC->CR & RCC_CR_HSERDY))  { /*  Wait till HSE is ready */ }
  while(!(RCC->CR & RCC_CR_PLLRDY))  { /*  Wait till PLL is ready */ }

  FLASH->ACR = (
    0 * FLASH_ACR_LATENCY_0WS        | /*                                                        */
    0 * FLASH_ACR_LATENCY_1WS        | /*                                                        */
    1 * FLASH_ACR_LATENCY_2WS        | /*                                                        */
    0 * FLASH_ACR_LATENCY_3WS        | /*                                                        */
    0 * FLASH_ACR_LATENCY_4WS        | /*                                                        */
    0 * FLASH_ACR_LATENCY_5WS        | /*                                                        */
    0 * FLASH_ACR_LATENCY_6WS        | /*                                                        */
    0 * FLASH_ACR_LATENCY_7WS        | /*                                                        */
    1 * FLASH_ACR_PRFTEN             | /*  0x00000100                                            */
    1 * FLASH_ACR_ICEN               | /*  0x00000200                                            */
    1 * FLASH_ACR_DCEN               | /*  0x00000400                                            */
    0 * FLASH_ACR_ICRST              | /*  0x00000800                                            */
    0 * FLASH_ACR_DCRST              | /*  0x00001000                                            */
    0 * FLASH_ACR_BYTE0_ADDRESS      | /*  0x40023C00                                            */
    0 * FLASH_ACR_BYTE2_ADDRESS        /*  0x40023C03                                            */
  );

  RCC->CFGR = (
    0 * RCC_CFGR_SW                  | /*  0x00000003 SW[1:0] bits (System clock Switch)         */
    0 * RCC_CFGR_SW_0                | /*    0x00000001                                          */
    0 * RCC_CFGR_SW_1                | /*    0x00000002                                          */
    0 * RCC_CFGR_SW_HSI              | /*    HSI selected as system clock                        */
    0 * RCC_CFGR_SW_HSE              | /*    HSE selected as system clock                        */
    1 * RCC_CFGR_SW_PLL              | /*    PLL selected as system clock                        */

    0 * RCC_CFGR_SWS                 | /*  0x0000000C SWS[1:0] bits (System Clock Switch Status) */
    0 * RCC_CFGR_SWS_0               | /*    0x00000004                                          */
    0 * RCC_CFGR_SWS_1               | /*    0x00000008                                          */
    0 * RCC_CFGR_SWS_HSI             | /*    HSI oscillator used as system clock                 */
    0 * RCC_CFGR_SWS_HSE             | /*    HSE oscillator used as system clock                 */
    0 * RCC_CFGR_SWS_PLL             | /*    PLL used as system clock                            */

    /* HPRE: AHB prescaler                                                                       */
    /*      Set and cleared by software to control AHB clock division factor.                    */
    /*      Caution: The clocks are divided with the new prescaler factor                        */
    /*      from 1 to 16 AHB cycles after HPRE write.                                            */
    /*                                                                                           */
    /*        0xxx: system clock not divided                                                     */
    /*        1000: system clock divided by 2                                                    */
    /*        1001: system clock divided by 4                                                    */
    /*        1010: system clock divided by 8                                                    */
    /*        1011: system clock divided by 16                                                   */
    /*        1100: system clock divided by 64                                                   */
    /*        1101: system clock divided by 128                                                  */
    /*        1110: system clock divided by 256                                                  */
    /*        1111: system clock divided by 512                                                  */

    0 * RCC_CFGR_HPRE                | /*  0x000000F0 HPRE[3:0] bits (AHB prescaler)             */
    0 * RCC_CFGR_HPRE_0              | /*    0x00000010                                          */
    0 * RCC_CFGR_HPRE_1              | /*    0x00000020                                          */
    0 * RCC_CFGR_HPRE_2              | /*    0x00000040                                          */
    0 * RCC_CFGR_HPRE_3              | /*    0x00000080                                          */
    1 * RCC_CFGR_HPRE_DIV1           | /*    SYSCLK not divided                                  */
    0 * RCC_CFGR_HPRE_DIV2           | /*    SYSCLK divided by 2                                 */
    0 * RCC_CFGR_HPRE_DIV4           | /*    SYSCLK divided by 4                                 */
    0 * RCC_CFGR_HPRE_DIV8           | /*    SYSCLK divided by 8                                 */
    0 * RCC_CFGR_HPRE_DIV16          | /*    SYSCLK divided by 16                                */
    0 * RCC_CFGR_HPRE_DIV64          | /*    SYSCLK divided by 64                                */
    0 * RCC_CFGR_HPRE_DIV128         | /*    SYSCLK divided by 128                               */
    0 * RCC_CFGR_HPRE_DIV256         | /*    SYSCLK divided by 256                               */
    0 * RCC_CFGR_HPRE_DIV512         | /*    SYSCLK divided by 512                               */

    0 * RCC_CFGR_PPRE1               | /*  0x00001C00 PRE1[2:0] bits (APB1 prescaler)            */
    0 * RCC_CFGR_PPRE1_0             | /*    0x00000400                                          */
    0 * RCC_CFGR_PPRE1_1             | /*    0x00000800                                          */
    0 * RCC_CFGR_PPRE1_2             | /*    0x00001000                                          */
    0 * RCC_CFGR_PPRE1_DIV1          | /*    HCLK not divided                                    */
    1 * RCC_CFGR_PPRE1_DIV2          | /*    HCLK divided by 2                                   */
    0 * RCC_CFGR_PPRE1_DIV4          | /*    HCLK divided by 4                                   */
    0 * RCC_CFGR_PPRE1_DIV8          | /*    HCLK divided by 8                                   */
    0 * RCC_CFGR_PPRE1_DIV16         | /*    HCLK divided by 16                                  */

    0 * RCC_CFGR_PPRE2               | /*  0x0000E000 PRE2[2:0] bits (APB2 prescaler)            */
    0 * RCC_CFGR_PPRE2_0             | /*    0x00002000                                          */
    0 * RCC_CFGR_PPRE2_1             | /*    0x00004000                                          */
    0 * RCC_CFGR_PPRE2_2             | /*    0x00008000                                          */
    1 * RCC_CFGR_PPRE2_DIV1          | /*    HCLK not divided                                    */
    0 * RCC_CFGR_PPRE2_DIV2          | /*    HCLK divided by 2                                   */
    0 * RCC_CFGR_PPRE2_DIV4          | /*    HCLK divided by 4                                   */
    0 * RCC_CFGR_PPRE2_DIV8          | /*    HCLK divided by 8                                   */
    0 * RCC_CFGR_PPRE2_DIV16         | /*    HCLK divided by 16                                  */

    0 * RCC_CFGR_RTCPRE              | /*  0x001F0000                                            */
    0 * RCC_CFGR_RTCPRE_0            | /*    0x00010000                                          */
    0 * RCC_CFGR_RTCPRE_1            | /*    0x00020000                                          */
    0 * RCC_CFGR_RTCPRE_2            | /*    0x00040000                                          */
    0 * RCC_CFGR_RTCPRE_3            | /*    0x00080000                                          */
    0 * RCC_CFGR_RTCPRE_4            | /*    0x00100000                                          */

    0 * RCC_CFGR_MCO1                | /*  0x00600000                                            */
    0 * RCC_CFGR_MCO1_0              | /*    0x00200000                                          */
    0 * RCC_CFGR_MCO1_1              | /*    0x00400000                                          */

    0 * RCC_CFGR_I2SSRC              | /*  0x00800000                                            */

    0 * RCC_CFGR_MCO1PRE             | /*  0x07000000                                            */
    0 * RCC_CFGR_MCO1PRE_0           | /*    0x01000000                                          */
    0 * RCC_CFGR_MCO1PRE_1           | /*    0x02000000                                          */
    0 * RCC_CFGR_MCO1PRE_2           | /*    0x04000000                                          */

    0 * RCC_CFGR_MCO2PRE             | /*  0x38000000                                            */
    0 * RCC_CFGR_MCO2PRE_0           | /*    0x08000000                                          */
    0 * RCC_CFGR_MCO2PRE_1           | /*    0x10000000                                          */
    0 * RCC_CFGR_MCO2PRE_2           | /*    0x20000000                                          */

    0 * RCC_CFGR_MCO2                | /*  0xC0000000                                            */
    0 * RCC_CFGR_MCO2_0              | /*    0x40000000                                          */
    0 * RCC_CFGR_MCO2_1                /*    0x80000000                                          */
  );
                                                                              /*
                Wait till System clock is ready
                ===============================                               */

  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { /*  */ }
                                                                              /*
                   Configure SysTick timer
   By default the clock source of SysTick is AHB/8 = 12.5 MHz.
   ===========================================================                */

  SysTick->LOAD = 12500UL - 1;                /* set reload register */
  SysTick->VAL  = 1000UL - 1;                 /* load counter value  */
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;    /* start SysTick timer */

}

__STATIC_INLINE void init_gpio(void) {

  #ifdef __clang__
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wcast-align"
  #endif
                                                                              /*
                Configure aternate functions of GPIOA
                =====================================                         */
  
  *(volatile uint64_t *)&GPIOA->AFR = (
    PIN_AF(PIN(14),  AF(0))             | /* PA14:  AF0 SYS_SWDCLK            */
    PIN_AF(PIN(13),  AF(0))             | /* PA13:  AF0 SYS_SWDIO             */
    PIN_AF(PIN(11),  AF(8))               /* PA11:  AF8 USART6_TX             */
  );
  
  #if defined(__clang__) 
    #pragma clang diagnostic pop 
  #endif
                                                                              /*
                 Set mode for each ping of GPIOA
                 ===============================                              */

  GPIOA->MODER = ALL_ANALOG - (
#ifndef SWD_DISABLED
    PIN_CONF(PIN(14), PINV_ALT_FUNC)    | /* PA14 AF0 -- SYS_SWDCLK           */
    PIN_CONF(PIN(13), PINV_ALT_FUNC)    | /* PA13 AF0 -- SYS_SWDIO            */
#endif
    PIN_CONF(PIN(11), PINV_ALT_FUNC)    | /* PA11 AF8 -- USART6_TX            */
    PIN_CONF(PIN(1),  PINV_INPUT)       | /* PA1  INPUT MODE                  */
    PIN_CONF(PIN(0),  PINV_OUTPUT)        /* PA0  OUTPUT PUSH PULL MODE       */
  );
                                                                              /*
                 Set speed for each pin of GPIOA
                 ===============================                              */

  GPIOA->OSPEEDR = (
#ifndef SWD_DISABLED
    PIN_CONF(PIN(14), PIN_SPEED_LOW)    | /* PA14 AF0 -- SYS_SWDCLK           */
    PIN_CONF(PIN(13), PIN_SPEED_HIGHER) | /* PA13 AF0 -- SYS_SWDIO            */
#endif
    PIN_CONF(PIN(11), PIN_SPEED_LOW)    | /* PA11 AF8 -- USART6_TX            */
    PIN_CONF(PIN(0),  PIN_SPEED_LOW)      /* PA0  OUTPUT PUSH PULL            */
  );
                                                                              /*
                Pull pins up or down if required
                ================================                              */

  GPIOA->PUPDR = (
#ifndef SWD_DISABLED
    PIN_CONF(PIN(14), PIN_PULL_UP)      | /* PA14 AF0 -- SYS_SWDCLK           */
    PIN_CONF(PIN(13), PIN_PULL_DOWN)    | /* PA13 AF0 -- SYS_SWDIO            */
#endif
    PIN_CONF(PIN(1),  PIN_PULL_UP)        /* PA1  INPUT PULL-UP               */
  );

  //SW_PIN(C, 13, HIGH);                    /* SET GPIOC PIN 13 HIGH            */
  SW_PIN(C, PIN_HIGH(13));

  GPIOC->MODER = ALL_ANALOG - (           /* Configure GPIOC                  */
    PIN_CONF(PIN(13), PINV_OUTPUT)        /* PA13 OUTPUT PUSH-PULL            */
  );
}

__STATIC_INLINE void init_usart(uint32_t baudrate) {
  USART6->BRR = UART_BAUDRATE(100 MHZ, baudrate);
  USART6->CR1 = (
    0 * USART_CR1_SBK        | /* 0x00000001 Send Break                             */
    0 * USART_CR1_RWU        | /* 0x00000002 Receiver wakeup                        */
    0 * USART_CR1_RE         | /* 0x00000004 Receiver Enable                        */
    1 * USART_CR1_TE         | /* 0x00000008 Transmitter Enable                     */
    0 * USART_CR1_IDLEIE     | /* 0x00000010 IDLE Interrupt Enable                  */
    0 * USART_CR1_RXNEIE     | /* 0x00000020 RXNE Interrupt Enable                  */
    0 * USART_CR1_TCIE       | /* 0x00000040 Transmission Complete Interrupt Enable */
    0 * USART_CR1_TXEIE      | /* 0x00000080 TXE Interrupt Enable                   */
    0 * USART_CR1_PEIE       | /* 0x00000100 PE Interrupt Enable                    */
    0 * USART_CR1_PS         | /* 0x00000200 Parity Selection                       */
    0 * USART_CR1_PCE        | /* 0x00000400 Parity Control Enable                  */
    0 * USART_CR1_WAKE       | /* 0x00000800 Wakeup method                          */
    0 * USART_CR1_M          | /* 0x00001000 Word length                            */
    1 * USART_CR1_UE         | /* 0x00002000 USART Enable                           */
    0 * USART_CR1_OVER8        /* 0x00008000 USART Oversampling by 8 enable         */
  );
}

#if defined(__clang__) && !defined(__CC_ARM)
  #pragma clang diagnostic ignored "-Wextra-semi-stmt"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
