#ifndef STATE_H
#define STATE_H

#include "stdint.h"
#include "stdbool.h"



#define OPEN_LOOP_START_COMMUTATION_TIME_US 5000


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

typedef enum
{
    COMMUTATION_OPEN_LOOP   = 0,
    COMMUTATION_CLOSED_LOOP = 1
} commutation_mode_t;

typedef struct
{
    // Modes
    state_mode_t       mode;
    commutation_mode_t commutation_mode;

    // Input signal
    uint32_t           signal_last_low_us;
    uint32_t           signal_last_high_us;
    uint32_t           signal_pulse_length_us;
    uint32_t           signal_last_pulse_length_us;
    uint32_t           signal_last_ok_pulse;

    // Commutation
    bldc_state_t       bldc_state;
    bool               bemf_rising;
    uint32_t           last_commutation;
    uint32_t           next_commutation;
    bool               next_commutation_time_set;
    uint32_t           last_commutation_duration_us;
    uint16_t           throttle;

    // Open loop commutation params
    uint16_t           open_loop_throttle;
    uint16_t           open_loop_delay_us_min;
    uint16_t           open_loop_commutation_period_us;
    uint16_t           open_loop_max_commutations;
    uint16_t           open_loop_commutations;

    // BEMF ADC
    uint32_t           input_adc_channel;
    uint32_t           highest_adc_value;
    uint32_t           zero_crossing_point;
    uint32_t           bemf_adc_filter_order;
    uint32_t           bemf_adc_filter[5];

    // MISC
    uint32_t           pulse_errors;
    uint32_t           _micros;      // Counter for storing microseconds since boot
} state_t;

extern state_t state;


#endif /* STATE_H */
