#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Stepper motor driver — UART-controlled closed-loop stepper.
 *
 * Hardware:  MSPM0 UART2 on PA23 (TX) / PA24 (RX)
 * Baud rate: 115200, 8N1
 * Protocol:  binary frames (AA 55 ID CMD LEN DATA... CHKSUM),
 *            motor replies with ASCII lines terminated by \r\n.
 *
 * Stepper_Init() performs a FULL manual UART2 configuration so the motor
 * works even without a SysConfig-generated init function for UART2.
 */

/* ---- Public API ---- */

void Stepper_Init(void);
void Stepper_Service(void);
void Stepper_Sync(void);
void Stepper_BitBangAbsoluteTest(int32_t degrees);
void Stepper_BlockingAbsoluteTest(int32_t degrees);
void Stepper_BlockingRelativeTest(int32_t degrees);
void Stepper_BlockingSetOrigin(void);
void Stepper_BlockingWake(void);
void Stepper_BlockingBeginCalibration(void);

/* High-level commands (fire-and-forget, non-blocking). */
void Stepper_Wake(void);
void Stepper_Sleep(void);
void Stepper_SetOrigin(void);
void Stepper_RelativeRotate(int32_t degrees);
void Stepper_AbsoluteRotate(int32_t degrees);
void Stepper_SetCurrent(uint8_t current);
void Stepper_SetMode(uint8_t mode);
void Stepper_SetUserDirection(uint8_t dir);
void Stepper_Reboot(void);

/* Raw frame send — data may be NULL when len == 0. */
void Stepper_SendRaw(uint8_t cmd, const uint8_t *data, uint8_t len);

/* Send completely raw bytes to the motor — no header, no checksum added.
   For protocol debugging via "RAW AA 55 01 ..." Bluetooth commands. */
void Stepper_SendRawBytes(const uint8_t *bytes, uint8_t len);

/* Debug: hex dump of the last frame queued for TX (for Bluetooth forwarding). */
bool Stepper_HasDebugHex(void);
void Stepper_CopyDebugHex(char *dst, uint8_t max_len);

/* Response forwarding.
 * Stepper_HasResponse() returns true when a complete \r\n line is ready.
 * Call Stepper_ReadResponseByte() in a loop (returns 0 when drained). */
bool    Stepper_HasResponse(void);
uint8_t Stepper_ReadResponseByte(void);
uint32_t Stepper_GetTxByteCount(void);
uint32_t Stepper_GetRxByteCount(void);
uint32_t Stepper_GetRxOverflowCount(void);
uint16_t Stepper_GetBufferedResponseCount(void);
uint8_t Stepper_ReadBufferedResponseByte(void);
uint16_t Stepper_GetTraceCount(void);
uint8_t Stepper_ReadTraceByte(void);

#endif
