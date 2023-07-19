#ifndef HAL_H
#define HAL_H


#include "stm32c0xx_hal.h"


//#define GPIO_MODE_INPUT              0b00
//#define GPIO_MODE_OUTPUT             0b01
//#define GPIO_MODE_ALTERNATE_FUNCTION 0b10
//#define GPIO_MODE_ANALOG             0b11

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


void hal_init();

void Error_Handler();

extern ADC_HandleTypeDef  hadc1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim14;

#define TIM_HIN_A htim1
#define TIM_HIN_B htim3
#define TIM_HIN_C htim1

#define PWM_REG_HIN_A TIM1->CCR2
#define PWM_REG_HIN_B TIM3->CCR2
#define PWM_REG_HIN_C TIM1->CCR3


void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);


#endif /* HAL_H */
