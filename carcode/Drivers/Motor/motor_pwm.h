#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

void MotorPwm_Init(void);
void MotorPwm_SetLeft(uint8_t duty_percent);
void MotorPwm_SetRight(uint8_t duty_percent);

#endif
