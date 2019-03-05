#include "stm32f0xx_hal.h"
#include "main.h"

void platform_usleep(uint64_t us) {
  delayUs(us);
}

