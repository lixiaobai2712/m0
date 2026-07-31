#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Camera / vision module driver — UART3 on PB2 (TX) / PB3 (RX).
 *
 * Protocol (8-byte frames, both directions):
 *   AA 66  PAYLOAD[4]  DD EE
 *
 * RX (camera → MCU):  STA  POS_H  POS_L  CHK
 *   STA  bit0 = ball detected (1) or not (0)
 *   POS  int16 big-endian, ball X offset in pixels (±256)
 *   CHK  = (STA + POS_H + POS_L) & 0xFF
 *
 * TX (MCU → camera):  CMD  PARAM1  PARAM2  CHK
 *   CMD  0x01 = start, 0x02 = stop, 0x03 = config
 *   CHK  = (CMD + PARAM1 + PARAM2) & 0xFF
 *
 * All pin / peripheral defines come from SysConfig (CAMERA_UART).
 */

/* ---- Public API ---- */

void Camera_Init(void);
void Camera_Service(uint32_t now_ms);

/* ---- Ball data ---- */
bool    Camera_HasBall(void);         /* true when STA bit0 == 1 */
int16_t Camera_GetBallX(void);        /* ±256, only valid if HasBall() */
uint32_t Camera_GetValidFrameCount(void);
uint32_t Camera_GetLastValidMs(void);

/* ---- Communication watchdog ---- */
bool    Camera_IsOnline(uint32_t now_ms);  /* true if last valid frame < 100 ms */

/* ---- Commands to camera ---- */
void Camera_Start(void);              /* CMD 0x01 */
void Camera_Stop(void);               /* CMD 0x02 */
void Camera_Config(uint8_t p1, uint8_t p2);  /* CMD 0x03 */

#endif
