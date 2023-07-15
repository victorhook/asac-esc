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

static inline void phase_input(__IO uint32_t* pwm_comp_reg, GPIO_TypeDef* nlin_port, uint16_t nlin_pin)
{
    // Close LIN -> Set NLIN LOW (Active low)
    nlin_port->BSRR = nlin_pin;
    // Set PWM on HIN to 0
    *pwm_comp_reg = 0;

    //set_gpio_mode
    // HIN_C_MODER_SHIFT_MASK
}


#define phase_a_high()  phase_high( &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_low()   phase_low(  &PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_a_input() phase_input(&PWM_REG_HIN_A, PORT_LIN_A, PIN_LIN_A)
#define phase_b_high()  phase_high( &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_low()   phase_low(  &PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_b_input() phase_input(&PWM_REG_HIN_B, PORT_LIN_B, PIN_LIN_B)
#define phase_c_high()  phase_high( &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_low()   phase_low(  &PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)
#define phase_c_input() phase_input(&PWM_REG_HIN_C, PORT_LIN_C, PIN_LIN_C)


static uint32_t _micros = 0;
uint32_t time_since_last_transition_us;


int main(void)
{
    hal_init();

    blink_boot_up_sequence();
    LED_RED_LOW();
    LED_BLUE_HIGH();


    while (1)
    {
        update();
        continue;
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
    uint32_t time_since_last_pulse = now - state.signal_last_low_us;
    float throttle = pulse_length_to_throttle(state.signal_last_pulse_length_us);

    if ((now - t0) > 1000000)
    {
        printf("[Mode: %d] Pulse: %lu, throttle: %d\n", state.mode, state.signal_last_pulse_length_us, (int) throttle);
        t0 = micros();
    }

    state_mode_t next_mode;
    uint32_t transition_delay_us = 5000;


    if ((state.signal_last_pulse_length_us < MIN_PULSE_LENGTH_US) ||
        (state.signal_last_pulse_length_us > MAX_PULSE_LENGTH_US) ||
        (time_since_last_pulse > NO_SIGNAL_RECEIVED_DISARM_TIMEOUT_US))
    {
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
                time_since_last_transition_us = micros() - state.last_bldc_state_transition;
                if ((time_since_last_transition_us) > transition_delay_us)
                {
                    handle_bldc_output();
                    state.last_bldc_state_transition = micros();
                }
            }
            break;
    }

    // Transition to next mode
    state.mode = next_mode;
}

static inline void set_pwm_duty_cycle(TIM_TypeDef* TIMx, const uint8_t channel, const uint32_t duty)
{
    // 1. Stop PWM
    //TIMx->CCER &= ~TIM_CCER_CC1E;

    // 2. Set the Capture Compare Register value
    TIMx->CCR1 = duty;

    // 3. Start PWM
    //TIMx->CCER |= TIM_CCER_CC1E;
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

static inline void arm()
{
    LED_RED_HIGH();
}
static inline void disarm()
{
    LED_RED_LOW();
    phase_a_low();
    phase_b_low();
    phase_c_low();
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
            state.bldc_state = BLDC_STATE_2;
            break;
        case BLDC_STATE_2:
            phase_c_low();
            phase_a_high();
            phase_b_input();
            state.bldc_state = BLDC_STATE_3;
            break;
        case BLDC_STATE_3:
            phase_a_input();
            phase_b_high();
            phase_c_low();
            state.bldc_state = BLDC_STATE_4;
            break;
        case BLDC_STATE_4:
            phase_a_low();
            phase_b_high();
            phase_c_input();
            state.bldc_state = BLDC_STATE_5;
            break;
        case BLDC_STATE_5:
            phase_a_low();
            phase_b_input();
            phase_c_high();
            state.bldc_state = BLDC_STATE_6;
            break;
        case BLDC_STATE_6:
            phase_a_input();
            phase_b_low();
            phase_c_high();
            state.bldc_state = BLDC_STATE_1;
            break;
    }
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
    state.signal_last_pulse_length_us = state.signal_last_low_us - state.signal_last_high_us;
}

uint32_t micros()
{
  return _micros + (TIM14->CNT);
}

uint32_t millis()
{
  return micros() * 1000;
}
