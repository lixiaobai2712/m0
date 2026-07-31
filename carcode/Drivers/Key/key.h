#ifndef KEY_H
#define KEY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Button definitions for the Tianmengxing car (MSPM0G3507).
 *
 * All buttons are active-low with internal pull-ups enabled via SysConfig.
 *
 * KEY_START : PB21  — run/stop toggle (primary)
 * KEY_K1    : PB13  — reserved
 * KEY_K2    : PB23  — reserved
 * KEY_K3    : PB26  — reserved
 * KEY_K4    : PB27  — run/stop toggle (secondary, same function as START)
 */

typedef enum {
    KEY_START = 0,
    KEY_K1    = 1,
    KEY_K2    = 2,
    KEY_K3    = 3,
    KEY_K4    = 4,
    KEY_COUNT = 5
} KeyId;

/*
 * Initialize all five buttons.  START_KEY (PB21) is already configured by
 * SYSCFG_DL_GPIO_init(); the other four are set up here explicitly so the
 * driver works even without a SysConfig regeneration.
 */
void Key_Init(void);

/*
 * Non-blocking state-machine tick.  Must be called every control cycle
 * (every ~1 ms) so debounce timing is accurate.  time_ms is the application
 * millisecond counter (app_time_ms in car_app.c).
 */
void Key_Service(uint32_t time_ms);

/*
 * Returns true exactly once per press — on the confirmed falling edge after
 * debounce.  Does NOT block; returns immediately.
 */
bool Key_WasPressed(KeyId id);

/* Convenience wrappers */
static inline bool Key_StartPressed(void) { return Key_WasPressed(KEY_START); }
static inline bool Key_K1Pressed(void)    { return Key_WasPressed(KEY_K1); }
static inline bool Key_K2Pressed(void)    { return Key_WasPressed(KEY_K2); }
static inline bool Key_K3Pressed(void)    { return Key_WasPressed(KEY_K3); }
static inline bool Key_K4Pressed(void)    { return Key_WasPressed(KEY_K4); }

#endif
