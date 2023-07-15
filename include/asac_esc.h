#ifndef ASAC_ESC_H
#define ASAC_ESC_H

#include "machine.h"
#include "stdio.h"
#include "state.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


inline uint32_t micros();

inline uint32_t millis();

#define HIN_A_MODER_SHIFT_MASK (1 * 2)  // PIN 1
#define HIN_B_MODER_SHIFT_MASK (14 * 2) // PIN 14
#define HIN_C_MODER_SHIFT_MASK (6 * 2)  // PIN 6



#endif /* ASAC_ESC_H */
