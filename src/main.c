#include "asac_esc.h"
#include "hal.h"
#include "audio.h"

// Callbacks
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin);

static void blink_boot_up_sequence();
static inline void update_micros_timer();
static inline void switch_to_open_loop();
static inline void update();
static uint32_t read_bemf();
static inline void arm();
static inline void disarm();

/* Returns -1 if the pulse length is invalid */
static inline int16_t pulse_length_to_throttle(const uint32_t time_since_last_pulse);
static void inline set_gpio_mode(GPIO_TypeDef* gpio_port, const uint8_t mode, const uint8_t shift_mask);

static inline void phase_high(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin);
static inline void phase_low(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin);
static inline void phase_input(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin, uint32_t adc_channel);


#define LED_BLUE_HIGH() HAL_GPIO_WritePin(PORT_LED_BLUE, PIN_LED_BLUE, GPIO_PIN_SET)
#define LED_BLUE_LOW() HAL_GPIO_WritePin(PORT_LED_BLUE, PIN_LED_BLUE, GPIO_PIN_RESET)
#define LED_RED_HIGH() HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_SET)
#define LED_RED_LOW() HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_RESET)

// Slowest allowed signal is 50 Hz,
#define NO_SIGNAL_RECEIVED_DISARM_TIMEOUT_US (20000 + 10000) // Add some margin

#define MIN_PULSE_LENGTH_US 20
#define MAX_PULSE_LENGTH_US 2500

#define MINIMUM_PULSE_LENGTH_PWM_US        1000
#define MAXIMUM_PULSE_LENGTH_PWM_US        2000
#define MINIMUM_PULSE_LENGTH_ONESHOT125_US 125
#define MAXIMUM_PULSE_LENGTH_ONESHOT125_US 250
#define MINIMUM_PULSE_LENGTH_ONESHOT42_US  42
#define MAXIMUM_PULSE_LENGTH_ONESHOT42_US  84
#define INVALID_PULSE_LENGTH               -1

#define PWM_MIN_DETECTION_LIMIT     750
#define ONESHOT_125_DETECTION_LIMIT 100
#define ONESHOT_4_DETECTION_LIMIT   30

#define MAX_THROTTLE_DUTY_VALUE 2048
#define MIN_THROTTLE_DUTY_VALUE 400

#define PULSE_LENGTH_TO_THROTTLE_FACTOR_PWM        ((1.0 / (float) MINIMUM_PULSE_LENGTH_PWM_US)        * (MAX_THROTTLE_DUTY_VALUE - MIN_THROTTLE_DUTY_VALUE))
#define PULSE_LENGTH_TO_THROTTLE_FACTOR_ONESHOT125 ((1.0 / (float) MINIMUM_PULSE_LENGTH_ONESHOT125_US) * (MAX_THROTTLE_DUTY_VALUE - MIN_THROTTLE_DUTY_VALUE))
#define PULSE_LENGTH_TO_THROTTLE_FACTOR_ONESHOT42  ((1.0 / (float) MINIMUM_PULSE_LENGTH_ONESHOT42_US)  * (MAX_THROTTLE_DUTY_VALUE - MIN_THROTTLE_DUTY_VALUE))

#define phase_a_high()  phase_high( &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_low()   phase_low(  &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_input() phase_input(&PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A, ADC_CHANNEL_PHASE_A)
#define phase_b_high()  phase_high( &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_low()   phase_low(  &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_input() phase_input(&PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B, ADC_CHANNEL_PHASE_B)
#define phase_c_high()  phase_high( &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_low()   phase_low(  &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_input() phase_input(&PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C, ADC_CHANNEL_PHASE_C)


void all_outputs_low()
{
    phase_a_low();
    phase_b_low();
    phase_c_low();
}

// -- Audio -- //

// TODO: Make this a setting?
direction_t direction = DIRECTION_FORWARD;

//#define DO_DEBUG
#define DO_USE_AUDIO


note_t startup_melody[] = {
    {
        .tone            = TONE_B,
        .on_duration_ms  = 100,
        .off_duration_ms = 100
    },
    {
        .tone            = TONE_B,
        .on_duration_ms  = 100,
        .off_duration_ms = 100
    },
    {
        .tone            = TONE_F,
        .on_duration_ms  = 300,
        .off_duration_ms = 0
    }
};

note_t throttle_detected_note = {
    .tone            = TONE_G,
    .on_duration_ms  = 300,
    .off_duration_ms = 300
};

note_t armed_note = {
    .tone            = TONE_B,
    .on_duration_ms  = 300,
    .off_duration_ms = 300
};


int main()
{
    hal_init();
    all_outputs_low();

    blink_boot_up_sequence();
    LED_RED_LOW();
    LED_BLUE_HIGH();

    printf("Starting up...\n");

    #ifdef DO_USE_AUDIO
    play_melody(startup_melody, 3);
    #endif

    while (1)
    {
        update_micros_timer();
        //uint32_t t0 = micros();
        update();
        //uint32_t t1 = micros();
        //uint32_t dt = t1 - t0;
        //printf("%d\n", dt);

        #ifdef DO_DEBUG
        static uint32_t t0 = 0;
        if ((micros() - t0) > 1000000)
        {
            printf("%d, %d\n", state.commutation_mode, state.open_loop_commutations);
            t0 = micros();
        }
        #endif
    }
}

static inline void update()
{
    state_mode_t next_mode;
    uint32_t now = micros();

    long time_since_last_pulse = now - state.signal_last_ok_pulse;

    if ((state.signal_last_ok_pulse > 0) && (time_since_last_pulse < NO_SIGNAL_RECEIVED_DISARM_TIMEOUT_US))
    {
        switch (state.mode)
        {
            case MODE_IDLE:
                // Starting from idle, let's start arming sequence
                next_mode = MODE_ARMING_SEQUENCE_STARTED;
                break;
            case MODE_ARMING_SEQUENCE_STARTED:
                if (state.throttle == 0)
                {
                    next_mode = MODE_ARMED;
                }
                else
                {
                    next_mode = MODE_ARMING_SEQUENCE_STARTED;
                }
                break;
            case MODE_ARMED:
                next_mode = MODE_ARMED;
                break;
            default:
                // Should never reach this
                next_mode = MODE_IDLE;
        }
    }
    else
    {
        next_mode = MODE_IDLE;
    }

    switch (state.mode)
    {
        case MODE_IDLE:
            #ifdef DO_USE_AUDIO
            if (next_mode == MODE_ARMING_SEQUENCE_STARTED)
            {
                play_melody(&throttle_detected_note, 1);
            }
            #endif
            break;
        case MODE_ARMING_SEQUENCE_STARTED:
            if (next_mode == MODE_ARMED)
            {
                arm();
            }
            break;
        case MODE_ARMED:
            if (next_mode == MODE_IDLE)
            {
                disarm();
            }
            else
            {
                if (state.throttle == 0)
                {   // If throttle is zero, we'll switch to open loop control
                    switch_to_open_loop();
                }
                else
                {
                    // Read BEMF ADC value and update highest seen ADC value since last
                    // falling bemf commutation (max 2).
                    uint32_t adc_value = read_bemf();

                    if (adc_value > state.highest_adc_value)
                    {
                        state.highest_adc_value = adc_value;
                    }

                    if (state.commutation_mode == COMMUTATION_OPEN_LOOP)
                    {
                        // -- Open loop commutation -- //

                        bool zero_cross_detected = false;

                        // Check if we've done enough commutations in open-loop
                        // mode and if we've detected a zero cross that seems
                        // to be reasonable.
                        if ((state.open_loop_commutations > state.open_loop_min_commutations) &&
                            (
                                ((state.bemf_rising)  && (adc_value > state.zero_crossing_point)) ||
                                ((!state.bemf_rising) && (adc_value < state.zero_crossing_point))
                            ))
                        {
                            // Zero cross detected. Let's check that the time is reasonable.
                            // We're expecting to detect a zero cross roughly in the middle of
                            // a commutation period.
                            uint32_t expected_zero_cross = state.last_commutation + (state.last_commutation_duration_us / 2);

                            const int offset_limit_plus = 100;
                            const int offset_limit_minus = -100;

                            now = micros();
                            int dt = now - expected_zero_cross;
                            if (dt < 0)
                            {
                                zero_cross_detected = dt > offset_limit_minus;
                            }
                            else
                            {
                                zero_cross_detected = dt < offset_limit_plus;
                            }
                        }

                        if (zero_cross_detected)
                        {   // Detected zero cross in closed-loop!
                            // Let's switch to closed-loop control instead
                            state.commutation_mode = COMMUTATION_CLOSED_LOOP;
                        }
                        else
                        {
                            if (!state.next_commutation_time_set)
                            {
                                state.throttle = state.open_loop_throttle;

                                // Decrease the period a bit to make commutations faster
                                if (state.open_loop_commutation_period_us > 1600)
                                {
                                    state.open_loop_commutation_period_us -= 100;
                                }

                                // We'll set the next commutation to pre-decided time
                                state.next_commutation = now + state.open_loop_commutation_period_us;
                                state.next_commutation_time_set = true;
                                state.open_loop_commutations++;
                            }
                        }
                    }
                    else
                    {   // -- Closed loop commutation -- //

                        // https://dspguru.com/dsp/faqs/fir/properties/
                        // Phase delay of filter: (N – 1) / (2 * Fs) = (5 - 1) / (2 * 60000) ≃ 33us
                        static const uint32_t phase_delay_us = 33;
                        uint16_t loop_delay_us = 12;

                        if ( (!state.next_commutation_time_set) &&
                            (((state.bemf_rising)  && (adc_value > state.zero_crossing_point)) ||
                            ((!state.bemf_rising) &&  (adc_value < state.zero_crossing_point))))
                        {
                            now = micros();
                            state.next_commutation = now + (state.last_commutation_duration_us / 2) - phase_delay_us - loop_delay_us;
                            state.next_commutation_time_set = true;
                        }

                    }

                    if (state.next_commutation_time_set)
                    {
                        now = micros();
                        if (now >= state.next_commutation)
                        {
                            commutate();
                        }
                    }
                }
            }
            break;
    }

    // Transition to next mode
    state.mode = next_mode;
}

void commutate()
{
    /*
        State   A   B   C
          1     1   0   Z
          2     1   Z   0
          3     Z   1   0
          4     0   1   Z
          5     0   Z   1
          6     Z   0   1
    */
    switch (state.bldc_state)
    {
        case BLDC_STATE_1:
            phase_b_low();
            phase_a_high();
            phase_c_input();
            state.bemf_rising = false;
            state.zero_crossing_point = state.highest_adc_value / 2;
            state.highest_adc_value = 0;
            break;
        case BLDC_STATE_2:
            phase_c_low();
            phase_a_high();
            phase_b_input();
            state.bemf_rising = true;
            break;
        case BLDC_STATE_3:
            phase_a_input();
            phase_b_high();
            phase_c_low();
            state.bemf_rising = false;
            state.zero_crossing_point = state.highest_adc_value / 2;
            state.highest_adc_value = 0;
            break;
        case BLDC_STATE_4:
            phase_a_low();
            phase_b_high();
            phase_c_input();
            state.bemf_rising = true;
            break;
        case BLDC_STATE_5:
            phase_a_low();
            phase_b_input();
            phase_c_high();
            state.bemf_rising = false;
            state.zero_crossing_point = state.highest_adc_value / 2;
            state.highest_adc_value = 0;
            break;
        case BLDC_STATE_6:
            phase_a_input();
            phase_b_low();
            phase_c_high();
            state.bemf_rising = true;
            break;
    }

    // Switch to next commutation state
    if (state.direction == DIRECTION_FORWARD)
    {
        state.bldc_state = (state.bldc_state + state.direction) % 6;
    }
    else
    {
        if (state.bldc_state == BLDC_STATE_1)
        {
            state.bldc_state = BLDC_STATE_6;
        }
        else
        {
            state.bldc_state--;
        }
        // In reverse direction, the bemf must also be considered, since the
        // slopes will be reversed.
        state.bemf_rising = !state.bemf_rising;
    }

    // Update time since last transition
    uint32_t now = micros();
    state.last_commutation_duration_us = now - state.last_commutation;
    state.next_commutation_time_set = false;
    state.last_commutation = now;
}

static inline void update_micros_timer()
{
    // If the timer 14 (that handles micro seconds timing) reaches above a
    // certain value, we'll
    const uint32_t UPPER_VALUE = 30000;
    if (TIM14->CNT > UPPER_VALUE)
    {
        state._micros += (TIM14->CNT);
        TIM14->CNT = 0;
    }
}

static inline void switch_to_open_loop()
{
    state.commutation_mode = COMMUTATION_OPEN_LOOP;
    state.open_loop_commutations = 0;
    state.open_loop_commutation_period_us = OPEN_LOOP_START_COMMUTATION_TIME_US;
    state.open_loop_throttle = OPEN_LOOP_THROTTLE;

    reset_commutation_state();
}

void reset_commutation_state()
{
    state.bldc_state = BLDC_STATE_1;
    state.bemf_rising = false;
    all_outputs_low();
}

static uint32_t read_bemf()
{
    while (!(ADC1->ISR & ADC_ISR_EOC));
    uint32_t raw = ADC1->DR;

    // Filter ADC
    for (int i = state.bemf_adc_filter_order-1; i > 0; i--)
    {
        state.bemf_adc_filter[i] = state.bemf_adc_filter[i-1];
    }
    state.bemf_adc_filter[0] = raw;

    uint32_t adc_value = 0;
    for (int i = 0; i < state.bemf_adc_filter_order; i++)
    {
        adc_value += state.bemf_adc_filter[i];
    }

    return adc_value / state.bemf_adc_filter_order;
}

static inline void phase_high(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin)
{
    // Close LIN -> Set NLIN HIGH (Active low)
    nlin_port->BSRR = nlin_pin;
    // Set PWM on HIN to throttle value
    *pwm_comp_reg = state.throttle;
}

static inline void phase_low(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin)
{
    // Open LIN -> Set NLIN LOW (Active low)
    nlin_port->BRR = nlin_pin;
    // Set PWM on HIN to 0
    *pwm_comp_reg = 0;
}

static inline void phase_input(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin, uint32_t adc_channel)
{
    // Close LIN -> Set NLIN LOW (Active low)
    nlin_port->BSRR = nlin_pin;
    // Set PWM on HIN to 0
    *pwm_comp_reg = 0;

    // Stop ADC
    ADC1->CR &= ~ADC_CR_ADSTART;
    // Set channel
    ADC1->CHSELR = adc_channel;
    // Wait for channel selection to be set
    while (!(ADC1->ISR & ADC_ISR_CCRDY));
    // Start ADC again
    ADC1->CR |= ADC_CR_ADSTART;

    state.input_adc_channel = adc_channel;
}

static inline void arm()
{
    #ifdef DO_USE_AUDIO
    play_melody(&armed_note, 1);
    #endif

    state.commutation_mode = COMMUTATION_OPEN_LOOP;
    state.open_loop_commutation_period_us = OPEN_LOOP_START_COMMUTATION_TIME_US;
    LED_RED_HIGH();
    printf("[%lu] ARM\n", millis());
}

static inline void disarm()
{
    state.commutation_mode = COMMUTATION_OPEN_LOOP;
    LED_RED_LOW();
    phase_a_low();
    phase_b_low();
    phase_c_low();
    printf("[%lu] DISARM\n", millis());
}

static inline int16_t pulse_length_to_throttle(const uint32_t pulse_us)
{
    uint32_t pulse;
    uint16_t min;
    float pulse_length_to_throttle_factor;

    /*
        We support the following pulse lengths:
            - 1000-2000 us - Normal 50 Hz PWM
            - 125-250 us - Oneshot 125
            - 42-84 us - Oneshot 42
    */

    if (pulse_us > PWM_MIN_DETECTION_LIMIT)
    {    // PWM
        pulse = constrain(pulse_us, MINIMUM_PULSE_LENGTH_PWM_US, MAXIMUM_PULSE_LENGTH_PWM_US);
        pulse_length_to_throttle_factor = PULSE_LENGTH_TO_THROTTLE_FACTOR_PWM;
        min = MINIMUM_PULSE_LENGTH_PWM_US;
    }
    else if (pulse_us > ONESHOT_125_DETECTION_LIMIT)
    {    // Oneshot 125
        pulse = constrain(pulse_us, MINIMUM_PULSE_LENGTH_ONESHOT125_US, MAXIMUM_PULSE_LENGTH_ONESHOT125_US);
        pulse_length_to_throttle_factor = PULSE_LENGTH_TO_THROTTLE_FACTOR_ONESHOT125;
        min = MINIMUM_PULSE_LENGTH_ONESHOT125_US;
    }
    else if (pulse_us > ONESHOT_4_DETECTION_LIMIT)
    {    // Oneshot 42
        pulse = constrain(pulse_us, MINIMUM_PULSE_LENGTH_ONESHOT42_US, MAXIMUM_PULSE_LENGTH_ONESHOT42_US);
        pulse_length_to_throttle_factor = PULSE_LENGTH_TO_THROTTLE_FACTOR_ONESHOT42;
        min = MINIMUM_PULSE_LENGTH_ONESHOT42_US;
    }
    else
    {    // Unknown??
        return -1;
    }

    // This ensures that as long as there is a desired throttle above 0, it's
    // going to be at least MIN_THROTTLE_DUTY_VALUE. This is because bldc motors
    // are typically very hard to spin at low speeds.
    if (pulse > min)
    {
        return MIN_THROTTLE_DUTY_VALUE + ((uint16_t) ((pulse - min) * pulse_length_to_throttle_factor));
    }
    else
    {
        return 0;
    }
}

static void blink_boot_up_sequence()
{
  for (int i = 0; i < 3; i++)
  {
    LED_RED_HIGH();
    LED_BLUE_LOW();
    HAL_Delay(100);
    LED_RED_LOW();
    LED_BLUE_HIGH();
    HAL_Delay(100);
  }

}

static void inline set_gpio_mode(GPIO_TypeDef* gpio_port, const uint8_t mode, const uint8_t shift_mask)
{
    // To switch between pin mode, eg OUTPUT/INPUT/Alternate function, we
    // set the correct bits in the MODER register:
    // 00: Input
    // 01: Output
    // 10: Alternate function
    // 11: Analog

    // First we'll clear the two bits corresponding to the specific pin
    gpio_port->MODER &= ~(0b11 << shift_mask);
    // Set bits
    gpio_port->MODER |=  (mode << shift_mask);
}


void Error_Handler(void)
{
  LED_BLUE_LOW();
  while (1)
  {
    LED_RED_HIGH();
    HAL_Delay(500);
    LED_RED_LOW();
    HAL_Delay(500);
  }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    disable_interrupts();
    state.signal_last_high_us = micros();
    enable_interrupts();
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    disable_interrupts();
    state.signal_last_low_us = micros();

    uint32_t signal_pulse_length_us = state.signal_last_low_us - state.signal_last_high_us;

    if ((signal_pulse_length_us >= MIN_PULSE_LENGTH_US) &&
        (signal_pulse_length_us <= MAX_PULSE_LENGTH_US))
    {
        // Set last signal pulse length to current one
        state.signal_last_pulse_length_us = state.signal_pulse_length_us;

        // Update current pulse length
        state.signal_pulse_length_us = signal_pulse_length_us;

        // Save timestamp of las received OK signal
        state.signal_last_ok_pulse = state.signal_last_low_us;

        // If the newly received signal differs from the previous one, we'll
        // update the throttle value
        if (state.signal_pulse_length_us != state.signal_last_pulse_length_us)
        {   // New pulse received, calculate throttle
            int16_t new_throttle = pulse_length_to_throttle(state.signal_last_pulse_length_us);
            if (new_throttle != INVALID_PULSE_LENGTH)
            {
                // New throttle value set
                state.throttle = new_throttle;
            }
            else
            {
                // Pulse length invalid!
                state.pulse_errors++;
            }
        }
    }
    else
    {
        state.pulse_errors++;
    }
    enable_interrupts();
}

uint32_t micros()
{
  return state._micros + (TIM14->CNT);
}

uint32_t millis()
{
  return micros() / 1000;
}

void sleep_us(const uint32_t delay_us)
{
    uint32_t t0 = micros();
    while ((micros() - t0) < delay_us)
    {
        // Ensure we update microsecond timer while we wait.
        update_micros_timer();
    }
}

void sleep_ms(const uint32_t delay_ms)
{
    sleep_us(delay_ms * 1000);
}
