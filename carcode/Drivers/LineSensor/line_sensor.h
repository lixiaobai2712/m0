#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

#define LINE_SENSOR_COUNT 8U

void LineSensor_Init(void);
void LineSensor_Read(uint8_t values[LINE_SENSOR_COUNT]);
void Encoder_Init(void);
void Encoder_Service(void);
void Encoder_Zero(void);
int32_t Encoder_GetMotor1Count(void);
int32_t Encoder_GetMotor2Count(void);

#endif
