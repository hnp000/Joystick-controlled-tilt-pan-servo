#include "pwm.h"







// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
// {
  
// }






    
 



// void set_pwm_from_ms_width( float pulse_ms)
// {
//     // Convert pulse width from ms to microseconds.
//     uint32_t pulse_us = (uint32_t)(pulse_ms * 1000.0f);

//     // Optionally, clamp the pulse width to the PWM period.
//     // The full period (in microseconds) is PWM_time_period * 1000.
//     uint32_t full_period_us = PWM_time_period * 1000;
//     if(pulse_us > full_period_us)
//     {
//         pulse_us = full_period_us;
//     }

//     // Set the PWM compare value.
//     // This sets the duty cycle: if ARR = PWM_period_reload_value, then
//     // pulse_us represents the high time in microseconds.
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
// }