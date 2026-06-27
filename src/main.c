#include "main.h"

/*
 * Clock:  100 MHz from HSE 25 MHz via PLL (25 / 15 x 120 / 2)
 * RTC:    LSE 32768 Hz, wakeup every ~1 s (WUCKSEL=000 -> RTCCLK/16 -> 2048 Hz / 2048 = 1 Hz)
 * UART:   USART6 on PA11, 115200 baud, 8N1
 * LED:    PC13 (on-board), active low
 */

int main(void) {

  init_sys();         /* RCC, PLL, FLASH caches, RTC, SysTick       */
  init_gpio();        /* PA11 (USART6 TX), PC13 (LED), SWD          */
  init_usart(115200); /* USART6, TX only                            */

  for (;;) {

    /* Wait for RTC wakeup flag; reading ISR clears it atomically   */
    while ((RTC->ISR = UINT32_MAX * !(RTC->ISR & RTC_ISR_WUTF)));

    /* Capture time and date once per cycle                         */
    uint32_t tr = RTC->TR, dr = RTC->DR;
    uint32_t v[] = { RTC_DAY(dr), RTC_MONTH(dr), RTC_YEAR(dr),
                     RTC_HOUR(tr), RTC_MINUTE(tr), RTC_SECOND(tr) };

    /* 3 LED flashes interleaved with date output over UART         */
    for (uint32_t i = 0; i < 6; i++) {
      GPIOC->ODR ^= GPIO_ODR_OD13;   /* toggle LED                 */
      u_put_bcd(v[i]);               /* one BCD field               */
      u_putc(".. ::\n"[i]);          /* corresponding separator     */
      DELAY_MS(30);                  /* hold half-flash             */
    }
  }
}

