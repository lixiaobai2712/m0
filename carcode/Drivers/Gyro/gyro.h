#ifndef GYRO_H
#define GYRO_H

#include <stdbool.h>
#include <stdint.h>

void Gyro_Init(void);
void Gyro_Service(uint32_t now_ms);
bool Gyro_HasData(void);
float Gyro_GetAngleDeg(void);
float Gyro_GetRateDps(void);
uint16_t Gyro_GetFrameSequence(void);
uint32_t Gyro_GetValidFrameCount(void);
uint32_t Gyro_GetCrcErrorCount(void);
void Gyro_CopyLastFrame(uint8_t frame[16]);
void Gyro_RequestZero(void);
void Gyro_RequestBiasCalibration(void);

#endif
