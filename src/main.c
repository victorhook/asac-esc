#include "asac_esc.h"
#include "hal.h"

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
static inline uint16_t pulse_length_to_throttle(const uint32_t time_since_last_pulse);
static inline void commutate();
static void inline set_gpio_mode(GPIO_TypeDef* gpio_port, const uint8_t mode, const uint8_t shift_mask);

static inline void phase_high(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin);
static inline void phase_low(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin);
static inline void phase_input(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin, uint32_t adc_channel);


#define LED_BLUE_HIGH() HAL_GPIO_WritePin(PORT_LED_BLUE, PIN_LED_BLUE, GPIO_PIN_SET)
#define LED_BLUE_LOW() HAL_GPIO_WritePin(PORT_LED_BLUE, PIN_LED_BLUE, GPIO_PIN_RESET)
#define LED_RED_HIGH() HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_SET)
#define LED_RED_LOW() HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_RESET)

// Slowest allowed signal is 50 Hz,
#define NO_SIGNAL_RECEIVED_DISARM_TIMEOUT_US (20000 + 3000) // Add some margin

#define MIN_PULSE_LENGTH_US 20
#define MAX_PULSE_LENGTH_US 2500

#define MINIMUM_PULSE_LENGTH_US 1000
#define MAXIMUM_PULSE_LENGTH_US 2000
#define MAX_THROTTLE_DUTY_VALUE 2048
#define PULSE_LENGTH_TO_THROTTLE_FACTOR ((1.0 / (float) MINIMUM_PULSE_LENGTH_US) * MAX_THROTTLE_DUTY_VALUE)

#define phase_a_high()  phase_high( &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_low()   phase_low(  &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_input() phase_input(&PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A, ADC_CHANNEL_PHASE_A)
#define phase_b_high()  phase_high( &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_low()   phase_low(  &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_input() phase_input(&PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B, ADC_CHANNEL_PHASE_B)
#define phase_c_high()  phase_high( &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_low()   phase_low(  &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_input() phase_input(&PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C, ADC_CHANNEL_PHASE_C)
#define all_outputs_low() \
    phase_a_low();        \
    phase_b_low();        \
    phase_c_low()

//#define DO_DEBUG


int main()
{
    hal_init();
    all_outputs_low();

    blink_boot_up_sequence();
    LED_RED_LOW();
    LED_BLUE_HIGH();

    printf("Starting up...\n");


    while (1)
    {
        update_micros_timer();
        uint32_t t0 = micros();
        update();
        uint32_t t1 = micros();
        uint32_t dt = t1 - t0;
        //printf("DT: %u\n", dt);
    }
}

static inline void update()
{
    uint32_t now;



    now = micros();
    long time_since_last_pulse = now - state.signal_last_ok_pulse;

    #ifdef DO_DEBUG
    static uint32_t t0 = 0;
    if ((micros() - t0) > 1000000)
    {
        printf("%d, %d\n", state.commutation_mode, state.open_loop_commutations);
        t0 = micros();
    }
    #endif

    state_mode_t next_mode;

    if ((state.signal_last_ok_pulse > 0) && (time_since_last_pulse < NO_SIGNAL_RECEIVED_DISARM_TIMEOUT_US))
    {
        next_mode = MODE_RUNNING;
    }
    else
    {
        next_mode = MODE_IDLE;
    }

    switch (state.mode)
    {
        case MODE_IDLE:
            if (next_mode == MODE_RUNNING)
            {
                arm();
            }
            break;
        case MODE_RUNNING:
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
                        if ((state.open_loop_commutations > state.open_loop_max_commutations) &&
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
                            const int offset_limit_minus = 100;

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
                                    state.open_loop_commutation_period_us -= 30;
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

                        uint16_t loop_delay_us = 4;


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

static inline void commutate()
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
    state.bldc_state = (state.bldc_state + state.direction) % 6;

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
    state.open_loop_throttle = 500;
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

    // Select phase A as ADC channel.
    // TODO: Quicker implementation with direct register manipulation.
    /*
    ADC_ChannelConfTypeDef sConfig = {
        .Channel = adc_channel,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLINGTIME_COMMON_1
    };
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    */

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

static inline uint16_t pulse_length_to_throttle(const uint32_t pulse_us)
{
    float throttle;

    if (pulse_us < MINIMUM_PULSE_LENGTH_US)
    {
        throttle = MINIMUM_PULSE_LENGTH_US;
    }
    else if (pulse_us > MAXIMUM_PULSE_LENGTH_US)
    {
        throttle = MAXIMUM_PULSE_LENGTH_US;
    }
    else
    {
        throttle = pulse_us;
    }

    throttle -= MINIMUM_PULSE_LENGTH_US;

    return (uint16_t) (throttle * PULSE_LENGTH_TO_THROTTLE_FACTOR);
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
    state.signal_last_high_us = micros();
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
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
            state.throttle = pulse_length_to_throttle(state.signal_last_pulse_length_us);
        }
    }
    else
    {
        state.pulse_errors++;
    }
}

uint32_t micros()
{
  return state._micros + (TIM14->CNT);
}

uint32_t millis()
{
  return micros() / 1000;
}
