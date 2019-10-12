#include "main.h"

int main(void) {
  uint8_t it = 0;

  init_sys();
  init_gpio();
  init_usart();

  for (;;) {
    for (int i = 6; i; i--) {
      GPIOC->ODR ^= GPIO_ODR_OD13;
      DELAY_MS(30);
    }

    DELAY_MS(820);

    uprintf("Itteration #%u\n", it++);
  }

#if __CC_ARM
  #pragma diag_suppress 111
#elif __ICCARM__
  #pragma diag_suppress = Pe111
#endif

  return 0;
}

