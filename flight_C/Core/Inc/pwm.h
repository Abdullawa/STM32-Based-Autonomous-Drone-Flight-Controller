#ifndef PWM_H
#define PWM_H

#include <stdint.h>

// Initialize TIM2 CH1 (PA0) for PWM output at ~50Hz (1000-2000us range)
void MX_TIM2_Init(void);

// Set PWM pulse width in microseconds (1000..2000)
void PWM_SetPulse_us(uint32_t us);

#endif // PWM_H
