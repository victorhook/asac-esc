#include "state.h"


state_t state = {
    .mode                        = MODE_IDLE,
    .signal_last_low_us          = 0,
    .signal_last_high_us         = 0,
    .signal_last_pulse_length_us = 0,
    .throttle                    = 0,
    .bldc_state                  = BLDC_STATE_1,
    .bemf_rising                 = false,
    .last_bldc_state_transition  = 0,
    .input_adc_channel           = 0
};


state_t state;
