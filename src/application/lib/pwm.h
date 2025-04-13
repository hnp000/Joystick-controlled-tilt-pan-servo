#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>





#define PWM_time_period 20  /// in milisecond
#define PWM_period_reload_value  ((PWM_time_period*1000) - 1);

void MX_TIM1_Init(void);

void set_duty_cycle(uint8_t duty_cycle);
void set_pwm_from_ms_width( float pulse_ms);





#endif