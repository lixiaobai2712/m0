#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetSignedSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);

#endif
