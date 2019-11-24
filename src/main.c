#include "main.h"

__STATIC_INLINE void print_date(void) {
   uint32_t tr = RTC->TR, dr = RTC->DR;

   u_putx(RTC_DAY(dr)); u_putc('.'); u_putx(RTC_MONTH(dr)); u_putc('.'); u_putc('2'); u_putc('0'); u_putx(RTC_YEAR(dr)); u_putc(' ');
   u_putx(RTC_HOUR(tr)); u_putc(':'); u_putx(RTC_MINUTE(tr)); u_putc(':'); u_putx(RTC_SECOND(tr)); u_putc(10);
 }

int main(void) {

  init_sys();         /* Set MCU speed, enable preipherials, etc. */
  init_gpio();        /* Configure ports and pins                 */
  init_usart(115200); /* Configure USART6 for 115200 baud         */
                                                                  /*
                      Main program loop
====================================================================
                                                                  */
  for (;;) {
                                                                  /*
            Wait for wakeup flag and then clear it
            ======================================                */

    while ((RTC->ISR = UINT32_MAX * !(RTC->ISR & RTC_ISR_WUTF)));
                                                                  /*
               Make 3 fast LED flashes
               =======================                            */
    for (int i = 6; i; i--) {
      GPIOC->ODR ^= GPIO_ODR_OD13; /* Invert LED pin state        */
      DELAY_MS(30);                /* Wait 30ms                   */
    }                                                             /*
               Print the date and time
               =======================                            */

    print_date();
  }                                                               /*
                     Supress debug message
       =================================================
                                                                  */
  #if defined(__clang__) && !defined(__CC_ARM)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wunreachable-code-return"
  #elif defined(__CC_ARM)
    #pragma diag_suppress 111
  #elif defined(__ICCARM__)
    #pragma diag_suppress = Pe111
  #endif

  return 0;

  #if defined(__clang__) && !defined(__CC_ARM)
    #pragma clang diagnostic pop
  #endif
}

