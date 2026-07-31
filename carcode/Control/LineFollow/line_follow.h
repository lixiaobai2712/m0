#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

void LineFollow_Init(void);
void LineFollow_Update(void);
bool LineFollow_SetPid(int16_t kp, int16_t ki, int16_t kd);
void LineFollow_GetPid(int16_t *kp, int16_t *ki, int16_t *kd);
int16_t LineFollow_GetPosition(void);
int16_t LineFollow_GetCorrection(void);
bool LineFollow_HasLine(void);
bool LineFollow_StopLineDetected(void);
int8_t LineFollow_GetLastTurnDirection(void);
void LineFollow_SetHeadingCorrection(int16_t correction);
uint8_t LineFollow_GetSensorMask(void);

#endif
