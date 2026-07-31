#include "key.h"
#include "ti_msp_dl_config.h"

/*
 * Non-blocking 5-button driver with timestamp-based debounce.
 *
 * Every button uses a 4-state machine:
 *   IDLE → PRESS_DEBOUNCE → HELD → RELEASE_DEBOUNCE → IDLE
 *
 * Key_WasPressed() returns true exactly once when the state transitions
 * from PRESS_DEBOUNCE to HELD.  The function NEVER blocks — the caller's
 * main loop continues running, UART / Bluetooth stays alive, and the
 * motor control loop is not interrupted.
 *
 * Original blocking code (REMOVED):
 *   while (DL_GPIO_readPins(port, pin) == 0U) {}  // ← froze Bluetooth!
 */

#define KEY_DEBOUNCE_MS  30U

typedef enum {
    KEY_STATE_IDLE,
    KEY_STATE_PRESS_DEBOUNCE,
    KEY_STATE_HELD,
    KEY_STATE_RELEASE_DEBOUNCE
} KeyState;

typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
    KeyState   state;
    uint32_t   debounce_start_ms;
    bool       pressed_flag;   /* latched: cleared after WasPressed() reads it */
} KeyCtx;

static KeyCtx key_ctx[KEY_COUNT];

/* ==================================================================
 *  Public API
 * ================================================================== */

void Key_Init(void)
{
    uint8_t i;

    /* START_KEY (PB21) — already configured by SYSCFG_DL_GPIO_init() */
    key_ctx[KEY_START].port = CAR_GPIO_START_KEY_PORT;
    key_ctx[KEY_START].pin  = CAR_GPIO_START_KEY_PIN;

    /* K1 (PB13) — configure here so the driver is self-contained */
    key_ctx[KEY_K1].port = CAR_GPIO_KEY_K1_PORT;
    key_ctx[KEY_K1].pin  = CAR_GPIO_KEY_K1_PIN;
    DL_GPIO_initDigitalInputFeatures(CAR_GPIO_KEY_K1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* K2 (PB23) */
    key_ctx[KEY_K2].port = CAR_GPIO_KEY_K2_PORT;
    key_ctx[KEY_K2].pin  = CAR_GPIO_KEY_K2_PIN;
    DL_GPIO_initDigitalInputFeatures(CAR_GPIO_KEY_K2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* K3 (PB26) */
    key_ctx[KEY_K3].port = CAR_GPIO_KEY_K3_PORT;
    key_ctx[KEY_K3].pin  = CAR_GPIO_KEY_K3_PIN;
    DL_GPIO_initDigitalInputFeatures(CAR_GPIO_KEY_K3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* K4 (PB27) */
    key_ctx[KEY_K4].port = CAR_GPIO_KEY_K4_PORT;
    key_ctx[KEY_K4].pin  = CAR_GPIO_KEY_K4_PIN;
    DL_GPIO_initDigitalInputFeatures(CAR_GPIO_KEY_K4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    for (i = 0U; i < KEY_COUNT; i++) {
        key_ctx[i].state            = KEY_STATE_IDLE;
        key_ctx[i].debounce_start_ms = 0U;
        key_ctx[i].pressed_flag     = false;
    }
}

void Key_Service(uint32_t time_ms)
{
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; i++) {
        /* Active-low: pin reads 0 when pressed. */
        bool pin_active = (DL_GPIO_readPins(key_ctx[i].port,
                                            key_ctx[i].pin) == 0U);

        switch (key_ctx[i].state) {

        case KEY_STATE_IDLE:
            if (pin_active) {
                key_ctx[i].state = KEY_STATE_PRESS_DEBOUNCE;
                key_ctx[i].debounce_start_ms = time_ms;
            }
            break;

        case KEY_STATE_PRESS_DEBOUNCE:
            if (!pin_active) {
                /* Glitch — went back high before debounce window closed. */
                key_ctx[i].state = KEY_STATE_IDLE;
            } else if ((time_ms - key_ctx[i].debounce_start_ms)
                       >= KEY_DEBOUNCE_MS) {
                /* Confirmed press. */
                key_ctx[i].state = KEY_STATE_HELD;
                key_ctx[i].pressed_flag = true;
            }
            break;

        case KEY_STATE_HELD:
            if (!pin_active) {
                key_ctx[i].state = KEY_STATE_RELEASE_DEBOUNCE;
                key_ctx[i].debounce_start_ms = time_ms;
            }
            break;

        case KEY_STATE_RELEASE_DEBOUNCE:
            if (pin_active) {
                /* Glitch — went back low during release debounce. */
                key_ctx[i].state = KEY_STATE_HELD;
            } else if ((time_ms - key_ctx[i].debounce_start_ms)
                       >= KEY_DEBOUNCE_MS) {
                /* Confirmed release — ready for next press. */
                key_ctx[i].state = KEY_STATE_IDLE;
            }
            break;
        }
    }
}

bool Key_WasPressed(KeyId id)
{
    if (id >= KEY_COUNT) {
        return false;
    }
    if (key_ctx[id].pressed_flag) {
        key_ctx[id].pressed_flag = false;
        return true;
    }
    return false;
}
