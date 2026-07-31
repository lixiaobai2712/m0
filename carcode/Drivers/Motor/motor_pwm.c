#include "motor_pwm.h"
#include "ti_msp_dl_config.h"

#define MOTOR_PWM_PERIOD_COUNTS 1600U
#define MOTOR_PWM_MAX_DUTY      99U

static uint16_t DutyToCompare(uint8_t duty)
{
    uint32_t active_counts;

    if (duty > MOTOR_PWM_MAX_DUTY) {
        duty = MOTOR_PWM_MAX_DUTY;
    }
    active_counts = ((uint32_t)duty * MOTOR_PWM_PERIOD_COUNTS) / 100U;
    return (uint16_t)(MOTOR_PWM_PERIOD_COUNTS - active_counts);
}

void MotorPwm_Init(void)
{
    MotorPwm_SetLeft(0U);
    MotorPwm_SetRight(0U);
    DL_TimerG_startCounter(MOTOR_PWM_INST);
}

void MotorPwm_SetLeft(uint8_t duty_percent)
{
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        DutyToCompare(duty_percent), GPIO_MOTOR_PWM_C0_IDX);
}

void MotorPwm_SetRight(uint8_t duty_percent)
{
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        DutyToCompare(duty_percent), GPIO_MOTOR_PWM_C1_IDX);
}
