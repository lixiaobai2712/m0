#include "line_follow.h"
#include "car_config.h"
#include "line_sensor.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define LINE_POSITION_LOST ((int8_t)-128)

static uint8_t sensors[LINE_SENSOR_COUNT];
static uint8_t sensor_history[LINE_SENSOR_COUNT];
static int8_t previous_position;
static int8_t last_turn_direction;
static int16_t filtered_position;
static int32_t integral;
static int16_t derivative_filtered;
static int16_t previous_correction;
static int16_t current_base_speed;
static int16_t pid_kp = CAR_PID_KP;
static int16_t pid_ki = CAR_PID_KI;
static int16_t pid_kd = CAR_PID_KD;
static int16_t telemetry_position;
static int16_t telemetry_correction;
static uint8_t line_visible;
static uint8_t stop_line_detected;
static uint8_t stop_line_armed;
static uint16_t stop_line_samples;
static int16_t heading_correction;
static uint8_t sensor_mask;

static int16_t LimitCorrectionRate(int16_t correction)
{
    if (correction > previous_correction + CAR_CORRECTION_STEP) {
        correction = previous_correction + CAR_CORRECTION_STEP;
    } else if (correction < previous_correction - CAR_CORRECTION_STEP) {
        correction = previous_correction - CAR_CORRECTION_STEP;
    }
    previous_correction = correction;
    return correction;
}

static int16_t UpdateBaseSpeed(int16_t target_speed)
{
    if (current_base_speed < target_speed) {
        current_base_speed += CAR_BASE_SPEED_STEP;
    } else if (current_base_speed > target_speed) {
        current_base_speed -= CAR_BASE_SPEED_STEP;
    }
    return current_base_speed;
}

static void ReadFilteredSensors(void)
{
    uint8_t raw[LINE_SENSOR_COUNT];
    uint8_t i;

    LineSensor_Read(raw);
    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        sensor_history[i] = (uint8_t)(((sensor_history[i] << 1) | raw[i]) &
            CAR_SENSOR_FILTER_MASK);
        /* Majority vote removes a single bad 1 ms sample. */
        sensors[i] = ((sensor_history[i] & 1U) +
            ((sensor_history[i] >> 1) & 1U) +
            ((sensor_history[i] >> 2) & 1U)) >= 2U;
    }
    sensor_mask = 0U;
    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if (sensors[i] == CAR_LINE_ACTIVE_LEVEL) {
            sensor_mask |= (uint8_t)(1U << i);
        }
    }
    /* PB22 is now an independent board LED. Show either extreme sensor. */
    if ((sensor_mask & 0x81U) != 0U) {
        DL_GPIO_setPins(CAR_GPIO_LINE_LED_PORT, CAR_GPIO_LINE_LED_PIN);
    } else {
        DL_GPIO_clearPins(CAR_GPIO_LINE_LED_PORT, CAR_GPIO_LINE_LED_PIN);
    }
}

static int8_t CalculatePosition(void)
{
    static const int8_t weights[LINE_SENSOR_COUNT] = {
        100, 75, 48, 16, -16, -48, -75, -100
    };
    int16_t weighted_sum = 0;
    uint8_t active_count = 0U;
    uint8_t i;

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if (sensors[i] == CAR_LINE_ACTIVE_LEVEL) {
            weighted_sum += weights[i];
            active_count++;
        }
    }
    return active_count == 0U ? LINE_POSITION_LOST :
        (int8_t)(weighted_sum / active_count);
}

static uint8_t IsStopLine(void)
{
    uint8_t active_count = 0U;
    uint8_t i;

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if (sensors[i] == CAR_LINE_ACTIVE_LEVEL) active_count++;
    }
    /* Normal tracking lights at most two channels. Four or more channels
     * identify the transverse finish mark, with margin for alignment. */
    return active_count >= 4U;
}

static int16_t CalculatePid(int8_t position)
{
    int32_t proportional;
    int32_t derivative_raw;
    int32_t output;

    if (position >= -CAR_PID_DEADBAND && position <= CAR_PID_DEADBAND) {
        position = 0;
    }
    proportional = (int32_t)pid_kp * position;

    if (position > -30 && position < 30) {
        integral += position;
        if (integral > 8000) integral = 8000;
        if (integral < -8000) integral = -8000;
    } else {
        integral = integral * 7 / 8;
    }

    derivative_raw = pid_kd * (position - previous_position);
    derivative_filtered = (int16_t)((derivative_filtered * 3 + derivative_raw) / 4);
    previous_position = position;

    output = (proportional + pid_ki * integral + derivative_filtered) >>
        CAR_PID_SCALE_SHIFT;
    if (output > 100) output = 100;
    if (output < -100) output = -100;
    return (int16_t)output;
}

void LineFollow_Init(void)
{
    uint8_t raw[LINE_SENSOR_COUNT];
    uint8_t i;

    LineSensor_Read(raw);
    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        /* Seed all filter samples from the real level so PB21 cannot create
         * a transient all-black or all-white reading. */
        sensor_history[i] = raw[i] != 0U ? CAR_SENSOR_FILTER_MASK : 0U;
    }
    previous_position = 0;
    last_turn_direction = 0;
    filtered_position = 0;
    integral = 0;
    derivative_filtered = 0;
    previous_correction = 0;
    current_base_speed = CAR_BASE_SPEED;
    telemetry_position = 0;
    telemetry_correction = 0;
    line_visible = 0U;
    stop_line_detected = 0U;
    stop_line_armed = 0U;
    stop_line_samples = 0U;
    heading_correction = 0;
    sensor_mask = 0U;
}

bool LineFollow_SetPid(int16_t kp, int16_t ki, int16_t kd)
{
    if (kp < 1 || kp > 120 || ki < 0 || ki > 20 || kd < 0 || kd > 80) {
        return false;
    }
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    integral = 0;
    previous_position = 0;
    derivative_filtered = 0;
    previous_correction = 0;
    return true;
}

void LineFollow_GetPid(int16_t *kp, int16_t *ki, int16_t *kd)
{
    *kp = pid_kp;
    *ki = pid_ki;
    *kd = pid_kd;
}

int16_t LineFollow_GetPosition(void)
{
    return telemetry_position;
}

int16_t LineFollow_GetCorrection(void)
{
    return telemetry_correction;
}

bool LineFollow_HasLine(void)
{
    return line_visible != 0U;
}

bool LineFollow_StopLineDetected(void)
{
    return stop_line_detected != 0U;
}

int8_t LineFollow_GetLastTurnDirection(void)
{
    return last_turn_direction;
}

void LineFollow_SetHeadingCorrection(int16_t correction)
{
    heading_correction = correction;
}

uint8_t LineFollow_GetSensorMask(void)
{
    return sensor_mask;
}

void LineFollow_Update(void)
{
    int8_t position;
    int16_t control_position;
    int16_t absolute_position;
    int16_t base_speed;
    int16_t correction;
    int16_t left_speed;
    int16_t right_speed;

    ReadFilteredSensors();
    stop_line_detected = 0U;
    if (IsStopLine()) {
        telemetry_position = 0;
        line_visible = 1U;
        if (stop_line_samples < CAR_STOP_LINE_CONFIRM_MS) {
            stop_line_samples++;
        }
        if (stop_line_armed != 0U &&
            stop_line_samples >= CAR_STOP_LINE_CONFIRM_MS) {
            telemetry_correction = 0;
            stop_line_detected = 1U;
            Motor_Stop();
        } else if (stop_line_armed != 0U) {
            /* A brief four-sensor pattern can occur in a curve. Hold the
             * prior steering while waiting for finish-line confirmation. */
            telemetry_correction = previous_correction;
            Motor_SetSignedSpeed(current_base_speed + previous_correction,
                current_base_speed - previous_correction);
        } else {
            /* The car starts over this same mark; cross it once before the
             * mark becomes an active finish line. */
            telemetry_correction = 0;
            Motor_SetSignedSpeed(CAR_BASE_SPEED, CAR_BASE_SPEED);
        }
        return;
    }
    stop_line_samples = 0U;
    stop_line_armed = 1U;

    position = CalculatePosition();
    line_visible = position != LINE_POSITION_LOST;
    if (position == LINE_POSITION_LOST) {
        /* The oval has no intentional gaps. Stop instead of steering forever
         * from a stale last direction when the sensor leaves the track. */
        telemetry_position = 0;
        telemetry_correction = 0;
        integral = 0;
        derivative_filtered = 0;
        previous_position = 0;
        previous_correction = 0;
        filtered_position = 0;
        last_turn_direction = 0;
        Motor_Stop();
        return;
    } else {
        telemetry_position = position;
        if (position > CAR_TURN_LATCH_ERROR) {
            last_turn_direction = 1;
        } else if (position < -CAR_TURN_LATCH_ERROR) {
            last_turn_direction = -1;
        }
    }

    filtered_position += ((int16_t)position - filtered_position) /
        CAR_POSITION_FILTER;
    control_position = filtered_position;
    absolute_position = filtered_position < 0 ?
        -filtered_position : filtered_position;

    base_speed = UpdateBaseSpeed(absolute_position >= CAR_CURVE_THRESHOLD ?
        CAR_CURVE_SPEED : CAR_BASE_SPEED);

    correction = CalculatePid((int8_t)control_position) *
        CAR_TURN_FACTOR / 100;
    correction += heading_correction;
    if (correction > CAR_MAX_CORRECTION) correction = CAR_MAX_CORRECTION;
    if (correction < -CAR_MAX_CORRECTION) correction = -CAR_MAX_CORRECTION;
    correction = LimitCorrectionRate(correction);
    telemetry_correction = correction;

    /* Left-side line: negative correction, left slows and right accelerates. */
    left_speed = base_speed + correction;
    right_speed = base_speed - correction;

    if (left_speed > CAR_MAX_SPEED) left_speed = CAR_MAX_SPEED;
    if (right_speed > CAR_MAX_SPEED) right_speed = CAR_MAX_SPEED;
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;

    Motor_SetSignedSpeed(left_speed, right_speed);
}
