#ifndef ASAC_ESC_H
#define ASAC_ESC_H

#include "machine.h"
#include "stdio.h"
#include "state.h"
#include "stm32c0xx_hal.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define enable_interrupts() __enable_irq()
#define disable_interrupts() __disable_irq()

inline uint32_t micros();

inline uint32_t millis();

inline void sleep_us(const uint32_t delay_us);

inline void sleep_ms(const uint32_t delay_ms);

inline void reset_commutation_state();

#define HIN_A_MODER_SHIFT_MASK (1 * 2)  // PIN 1
#define HIN_B_MODER_SHIFT_MASK (14 * 2) // PIN 14
#define HIN_C_MODER_SHIFT_MASK (6 * 2)  // PIN 6



#endif /* ASAC_ESC_H */
