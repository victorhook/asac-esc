#include "asac_esc.h"
#include "hal.h"

#define LED_BLUE_HIGH() HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_SET)
#define LED_BLUE_LOW() HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_RESET)
#define LED_RED_HIGH() HAL_GPIO_WritePin(PORT_LED_BLUE, PIN_LED_BLUE, GPIO_PIN_SET)
#define LED_RED_LOW() HAL_GPIO_WritePin(PORT_LED_BLUE, PIN_LED_BLUE, GPIO_PIN_RESET)

// Slowest allowed signal is 50 Hz,
#define NO_SIGNAL_RECEIVED_DISARM_TIMEOUT_US (20000 + 3000) // Add some margin

#define MIN_THROTTLE 0
#define MAX_THROTTLE 2048
#define MIN_PULSE_LENGTH_US 20
#define MAX_PULSE_LENGTH_US 2500

// Callbacks
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin);

static void blink_boot_up_sequence();
static inline void update();
static inline void arm();
static inline void disarm();
static inline float pulse_length_to_throttle(const uint32_t time_since_last_pulse);
static inline void handle_bldc_output();
//static inline void set_pwm_duty_cycle(TIM_TypeDef* TIMx, const uint32_t duty);
static void inline set_gpio_mode(GPIO_TypeDef* gpio_port, const uint8_t mode, const uint8_t shift_mask);


typedef enum
{
    COMMUTATION_OPEN_LOOP   = 0,
    COMMUTATION_CLOSED_LOOP = 1
} commutation_mode_t;
uint32_t lowest_adc_value  = 0xFFFFFFFF;
uint32_t highest_adc_value = 0;
uint32_t zero_crossing_point = 0;
commutation_mode_t commutation_mode = COMMUTATION_OPEN_LOOP;
uint32_t open_loop_commutation_steps = 0;
uint32_t max_open_loop_commutation_steps = 500;
float open_loop_delay_us = 5000;
uint32_t open_loop_delay_us_min = 250;
uint32_t pulse_errors = 0;





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
    ADC_ChannelConfTypeDef sConfig = {
        .Channel = adc_channel,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLINGTIME_COMMON_1
    };
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    state.input_adc_channel = adc_channel;
}


#define phase_a_high()  phase_high( &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_low()   phase_low(  &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_input() phase_input(&PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A, ADC_CHANNEL_PHASE_A)
#define phase_b_high()  phase_high( &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_low()   phase_low(  &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_input() phase_input(&PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B, ADC_CHANNEL_PHASE_B)
#define phase_c_high()  phase_high( &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_low()   phase_low(  &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_input() phase_input(&PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C, ADC_CHANNEL_PHASE_C)


static uint32_t _micros = 0;
uint32_t time_since_last_transition_us;


int main()
{
    hal_init();

    blink_boot_up_sequence();
    LED_RED_LOW();
    LED_BLUE_HIGH();

    printf("Starting up...\n");

    disarm();

    phase_a_input();
    phase_b_input();
    phase_c_input();

    HAL_ADCEx_Calibration_Start(&hadc1);
    uint32_t calib = HAL_ADCEx_Calibration_GetValue(&hadc1);

    while (1)
    {
        /*
        phase_a_input();
        HAL_ADC_Start(&hadc1);
        HAL_StatusTypeDef res = HAL_ADC_PollForConversion(&hadc1, 10);
        if (res != HAL_OK)
        {
            printf("ADC ERR\n");
        }
        else
        {
            uint32_t raw = HAL_ADC_GetValue(&hadc1);
            // Voltage divider:
            // R1 = 4700
            // R2 = 1000
            // Gain = 1 / (R1 / (R1 + R2)) = 1 / (1000 (4700 + 1000)) = 5.7
            float voltage = 3.3 * ((float) raw / 4095.0) * 5.7;
            printf("A:%u, %d\n", raw, (int) (voltage*1000));
        }
        */
        update();
    }
}


static inline void update()
{
    // If the timer 14 (that handles micro seconds timing) reaches above a
    // certain value, we'll
    const UPPER_VALUE = 30000;
    if (TIM14->CNT > UPPER_VALUE)
    {
        _micros += (TIM14->CNT);
        TIM14->CNT = 0;
    }

    static uint32_t t0 = 0;

    uint32_t now = micros();
    int time_since_last_pulse = now - state.signal_last_low_us;
    float throttle = pulse_length_to_throttle(state.signal_last_pulse_length_us);

    if ((now - t0) > 1000000)
    {
        //printf("[Mode: %d] Pulse: %lu, throttle: %d, low: %u, high: %u, zc: %u, cMode: %d, S: %u\n",
        //       state.mode, state.signal_last_pulse_length_us, (int) throttle,
        //       lowest_adc_value, highest_adc_value, zero_crossing_point,
        //       commutation_mode, open_loop_commutation_steps
        //       );
        t0 = micros();
    }

    state_mode_t next_mode;
    uint32_t transition_delay_us = 5000;


    if (time_since_last_pulse > NO_SIGNAL_RECEIVED_DISARM_TIMEOUT_US)
    {   // Disconnect?
        next_mode = MODE_IDLE;
    }
    else
    {
        state.throttle = throttle;
        next_mode = MODE_RUNNING;
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
                {   // If throttle is zero, we'll set commutation to open loop again
                    commutation_mode = COMMUTATION_OPEN_LOOP;
                    open_loop_commutation_steps = 0;
                }
                else
                {

                    // -- Open loop commutation -- //
                    if (commutation_mode == COMMUTATION_OPEN_LOOP)
                    {
                        transition_delay_us = open_loop_delay_us;
                        state.throttle = 1000;
                        open_loop_delay_us -= .1;


                        time_since_last_transition_us = micros() - state.last_bldc_state_transition;
                        if ((time_since_last_transition_us) > transition_delay_us)
                        {
                            handle_bldc_output();
                            open_loop_commutation_steps++;
                        }

                        //if (open_loop_commutation_steps > max_open_loop_commutation_steps)
                        if (open_loop_delay_us < open_loop_delay_us_min)
                        {
                            commutation_mode = COMMUTATION_CLOSED_LOOP;
                            open_loop_commutation_steps = 0;
                            open_loop_delay_us = 5000;
                        }
                    }
                    else
                    {   // -- Closed loop commutation -- //
                        HAL_ADC_Start(&hadc1);
                        HAL_StatusTypeDef res = HAL_ADC_PollForConversion(&hadc1, 10);
                        if (res != HAL_OK)
                        {
                            printf("ADC ERR\n");
                        }
                        else
                        {
                            uint32_t raw = HAL_ADC_GetValue(&hadc1);
                            if (raw < lowest_adc_value)
                            {
                                lowest_adc_value = raw;
                            }
                            if (raw > highest_adc_value)
                            {
                                highest_adc_value = raw;
                            }

                            const uint32_t MAX_EXPECTED_ADC_DIFF = 4095;
                            uint32_t adc_diff = highest_adc_value - lowest_adc_value;
                            if (adc_diff > MAX_EXPECTED_ADC_DIFF)
                            {   // WHAT TO DO?
                            }
                            else
                            {
                                zero_crossing_point = adc_diff / 2;
                            }

                            // TODO: Remove
                            //zero_crossing_point = 400;

                            if (state.bemf_rising)
                            {
                                if (raw > zero_crossing_point)
                                {
                                    handle_bldc_output();
                                }
                            }
                            else
                            {
                                if (raw < zero_crossing_point)
                                {
                                    handle_bldc_output();
                                }
                            }
                            // Voltage divider:
                            // R1 = 4700
                            // R2 = 1000
                            // Gain = 1 / (R1 / (R1 + R2)) = 1 / (1000 (4700 + 1000)) = 5.7
                            /*

                            float voltage = ((float) raw / 4095.0) * 5.7;
                            switch (state.input_adc_channel)
                            {
                                case ADC_CHANNEL_PHASE_A:
                                    printf("A:%u, %d\n", raw, (int) (voltage*1000));
                                    break;
                                case ADC_CHANNEL_PHASE_B:
                                    //printf("B:%d,", raw);
                                    break;
                                case ADC_CHANNEL_PHASE_C:
                                    //printf("C:%d\n", raw);
                                    break;
                            }
                            */
                        }
                    }
                }
            }
            break;
    }

    // Transition to next mode
    state.mode = next_mode;
}

static inline void handle_bldc_output()
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
            state.bldc_state = BLDC_STATE_2;
            break;
        case BLDC_STATE_2:
            phase_c_low();
            phase_a_high();
            phase_b_input();
            state.bemf_rising = true;
            state.bldc_state = BLDC_STATE_3;
            break;
        case BLDC_STATE_3:
            phase_a_input();
            phase_b_high();
            phase_c_low();
            state.bemf_rising = false;
            state.bldc_state = BLDC_STATE_4;
            break;
        case BLDC_STATE_4:
            phase_a_low();
            phase_b_high();
            phase_c_input();
            state.bemf_rising = true;
            state.bldc_state = BLDC_STATE_5;
            break;
        case BLDC_STATE_5:
            phase_a_low();
            phase_b_input();
            phase_c_high();
            state.bemf_rising = false;
            state.bldc_state = BLDC_STATE_6;
            break;
        case BLDC_STATE_6:
            phase_a_input();
            phase_b_low();
            phase_c_high();
            state.bemf_rising = false;
            state.bldc_state = BLDC_STATE_1;
            break;
    }

    // Reset zero crossing variables
    lowest_adc_value  = 0xFFFFFFFF;
    highest_adc_value = 0;

    // Update time since last transition
    state.last_bldc_state_transition = micros();
}

static inline void arm()
{
    commutation_mode = COMMUTATION_OPEN_LOOP;
    LED_RED_HIGH();
}
static inline void disarm()
{
    commutation_mode = COMMUTATION_OPEN_LOOP;
    LED_RED_LOW();
    phase_a_low();
    phase_b_low();
    phase_c_low();
}

static inline float pulse_length_to_throttle(const uint32_t pulse_us)
{
    int min = 1000;
    int max = 2000;
    int diff_min_max = (max - min);

    float throttle = constrain(pulse_us, min, max);
    throttle = (throttle - min) / diff_min_max;

    // Assumption: Normal PWM, 50 Hz
    return throttle * MAX_THROTTLE;
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
  __disable_irq();
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

    uint32_t signal_last_pulse_length_us = state.signal_last_low_us - state.signal_last_high_us;

    if ((signal_last_pulse_length_us >= MIN_PULSE_LENGTH_US) && 
        (signal_last_pulse_length_us <= MAX_PULSE_LENGTH_US))
    {   // Ensure signal pulse length is valid
        state.signal_last_pulse_length_us = signal_last_pulse_length_us;
    }
    else
    {
        pulse_errors++;
    }
}

uint32_t micros()
{
  return _micros + (TIM14->CNT);
}

uint32_t millis()
{
  return micros() * 1000;
}
