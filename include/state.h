#ifndef STATE_H
#define STATE_H

#include "stdint.h"
#include "stdbool.h"


typedef enum
{
    MODE_IDLE,
    MODE_RUNNING,
} state_mode_t;

typedef enum
{
    BLDC_STATE_1 = 0,
    BLDC_STATE_2,
    BLDC_STATE_3,
    BLDC_STATE_4,
    BLDC_STATE_5,
    BLDC_STATE_6,
} bldc_state_t;

typedef struct
{
    state_mode_t mode;
    uint32_t     signal_last_low_us;
    uint32_t     signal_last_high_us;
    uint32_t     signal_last_pulse_length_us;
    bldc_state_t bldc_state;
    bool         bemf_rising;
    uint32_t     last_bldc_state_transition;
    float        throttle;
    uint32_t     input_adc_channel;
} state_t;

extern state_t state;


#endif /* STATE_H */
