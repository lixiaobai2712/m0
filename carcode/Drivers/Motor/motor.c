#include "motor.h"
#include "motor_pwm.h"
#include "ti_msp_dl_config.h"

static uint8_t ClampMagnitude(int16_t speed)
{
    int16_t magnitude = speed < 0 ? -speed : speed;
    return (uint8_t)(magnitude > 99 ? 99 : magnitude);
}

static void SetLeftDirection(int16_t speed)
{
    if (speed > 0) {
        DL_GPIO_setPins(CAR_GPIO_AIN1_PORT, CAR_GPIO_AIN1_PIN);
        DL_GPIO_clearPins(CAR_GPIO_AIN2_PORT, CAR_GPIO_AIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(CAR_GPIO_AIN1_PORT, CAR_GPIO_AIN1_PIN);
        DL_GPIO_setPins(CAR_GPIO_AIN2_PORT, CAR_GPIO_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(CAR_GPIO_AIN1_PORT, CAR_GPIO_AIN1_PIN);
        DL_GPIO_clearPins(CAR_GPIO_AIN2_PORT, CAR_GPIO_AIN2_PIN);
    }
}

static void SetRightDirection(int16_t speed)
{
    if (speed > 0) {
        DL_GPIO_setPins(CAR_GPIO_BIN1_PORT, CAR_GPIO_BIN1_PIN);
        DL_GPIO_clearPins(CAR_GPIO_BIN2_PORT, CAR_GPIO_BIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(CAR_GPIO_BIN1_PORT, CAR_GPIO_BIN1_PIN);
        DL_GPIO_setPins(CAR_GPIO_BIN2_PORT, CAR_GPIO_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(CAR_GPIO_BIN1_PORT, CAR_GPIO_BIN1_PIN);
        DL_GPIO_clearPins(CAR_GPIO_BIN2_PORT, CAR_GPIO_BIN2_PIN);
    }
}

void Motor_Init(void)
{
    MotorPwm_Init();
    Motor_Stop();
}

void Motor_SetSignedSpeed(int16_t left_speed, int16_t right_speed)
{
    /* The driver A/B channels are wired to the opposite physical wheels. */
    SetLeftDirection(right_speed);
    SetRightDirection(left_speed);
    MotorPwm_SetLeft(ClampMagnitude(right_speed));
    MotorPwm_SetRight(ClampMagnitude(left_speed));
}

void Motor_Stop(void)
{
    Motor_SetSignedSpeed(0, 0);
}
