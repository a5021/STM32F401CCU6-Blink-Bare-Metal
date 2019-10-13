#include "main.h"

int main(void) {
  uint8_t it = 0;

  init_sys();   /* Set MCU speed, enable preipherials, etc.       */
  init_gpio();  /* Configure ports and pins (PA11 & PC13)         */
  init_usart(); /* Configure USART6 for 115200 baud               */

  /********             Main program loop                **********/

  for (;;) {
    for (int i = 6; i; i--) { /* Make 3 fast LED flashes          */
      GPIOC->ODR ^= GPIO_ODR_OD13; /* invert LED pin state        */
      DELAY_MS(30); /* Wait 30ms between LED state changings      */
    }

    DELAY_MS(820);  /* Make LED off for the rest of a second      */

    uprintf("Iteration #%u\n", it++);  /* print debug message     */
  }

/**************        Supress debug message         **************/

#if __CC_ARM
  #pragma diag_suppress 111
#elif __ICCARM__
  #pragma diag_suppress = Pe111
#endif

  return 0;
}

