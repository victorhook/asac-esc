#include "state.h"


state_t state = {
    // Modes
    .mode                            = MODE_IDLE,
    .commutation_mode                = COMMUTATION_OPEN_LOOP,

    // Input signal
    .signal_last_low_us              = 0,
    .signal_last_high_us             = 0,
    .signal_pulse_length_us          = 0,
    .signal_last_pulse_length_us     = 0,
    .signal_last_ok_pulse            = 0,

    // Commutation
    .bldc_state                      = BLDC_STATE_1,
    .bemf_rising                     = false,
    .last_commutation                = 0,
    .next_commutation                = 0,
    .next_commutation_time_set       = false,
    .last_commutation_duration_us    = 0,
    .desired_throttle                = 0,
    .throttle                        = 0,
    .direction                       = DIRECTION_FORWARD,

    // Open loop commutation params
    .open_loop_throttle              = OPEN_LOOP_THROTTLE,
    .open_loop_delay_us_min          = 700,
    .open_loop_commutation_period_us = OPEN_LOOP_START_COMMUTATION_TIME_US,
    .open_loop_min_commutations      = 30,
    .open_loop_commutations          = 0,

    // BEMF ADC
    .input_adc_channel               = 0,
    .highest_adc_value               = 0,
    .zero_crossing_point             = 0,
    .bemf_adc_filter_order           = 5,
    .bemf_adc_filter                 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

    // MISC
    .pulse_errors                    = 0,
    ._micros                         = 0,
};


state_t state;
