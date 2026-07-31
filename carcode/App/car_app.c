#include "car_app.h"
#include "car_config.h"
#include "delay.h"
#include "key.h"
#include "gyro.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "motor.h"
#include "oled.h"
#include "stepper.h"
#include "camera.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TUNER_RX_RING_SIZE 64U
#define OLED_STATUS_PERIOD_MS 250U

typedef enum {
    GYRO_BOOT_WAIT_DATA,
    GYRO_BOOT_WAIT_STILL,
    GYRO_BOOT_CALIBRATING,
    GYRO_BOOT_READY
} GyroBootState;

static bool car_running;
static uint32_t app_time_ms;
static char rx_line[64];
static uint8_t rx_length;
static uint32_t rx_last_byte_ms;
static uint32_t rx_byte_count;
static volatile uint8_t tuner_rx_ring[TUNER_RX_RING_SIZE];
static volatile uint8_t tuner_rx_write;
static volatile uint8_t tuner_rx_read;
static volatile uint32_t tuner_rx_overflow_count;
static uint32_t telemetry_pause_until_ms;
static bool angle_turn_active;
static bool angle_turn_line_mode;
static int32_t angle_turn_target_mdeg;
static int32_t angle_turn_start_mdeg;
static int8_t angle_turn_direction;
static uint32_t angle_turn_started_ms;
static uint32_t angle_turn_settle_started_ms;
static uint32_t line_lost_started_ms;
static uint32_t line_reacquired_started_ms;
static uint32_t corner_rearm_started_ms;
static bool corner_detection_armed;
static uint32_t line_advance_detected_ms;
static bool corner_candidate_active;
static bool corner_candidate_confirmed;
static int8_t corner_candidate_direction;
static int8_t last_corner_direction;
static uint32_t corner_candidate_started_ms;
static uint32_t corner_pattern_started_ms;
static uint32_t corner_center_started_ms;
static uint32_t corner_white_started_ms;
static uint16_t corner_observation_samples;
static uint16_t corner_white_samples;
static uint16_t corner_center_samples;
static bool corner_preview_active;
static int32_t corner_preview_m1_start;
static int32_t corner_preview_m2_start;
static int32_t corner_preview_heading_mdeg;
static bool encoder_drive_active;
static int32_t encoder_drive_m1_start;
static int32_t encoder_drive_m2_start;
static uint32_t encoder_drive_started_ms;
static int32_t encoder_drive_distance_mm;
static bool encoder_drive_line_mode;
static int8_t encoder_drive_turn_direction;
static int32_t encoder_drive_heading_mdeg;
static int16_t encoder_drive_heading_correction;
static uint32_t gyro_last_frame_ms;
static uint32_t gyro_last_frame_count;
static bool heading_locked;
static int32_t heading_target_mdeg;
static GyroBootState gyro_boot_state;
static int32_t gyro_still_reference_mdps;
static uint32_t gyro_still_started_ms;
static uint32_t gyro_cal_started_ms;
static uint32_t oled_last_update_ms;
#if CAR_CAMERA_ENABLE
static bool camera_mode;
static bool camera_was_online;
#endif
#if CAR_TRACK_ENABLE
static bool    track_mode;
static uint8_t track_start_state;
static uint32_t track_state_started_ms;
static uint32_t track_last_adjust_ms;
static int16_t track_tilt_deg;
static int16_t track_kp_divisor;
static bool    track_debug;
static uint32_t track_ball_lost_ms;
#endif
static bool    step_test_mode;
static int32_t step_test_last_ms;
static int8_t  step_test_direction;
static bool    step_raw_dump;
static char tx_buffer[96];
static uint8_t tx_length;
static uint8_t tx_index;
static void UartQueue(const char *text);

#if CAR_TRACK_ENABLE
enum {
    TRACK_START_IDLE,           /* active control running                        */
    TRACK_START_WAIT_CAMERA,    /* waiting for camera to come online             */
    TRACK_START_WAIT_ORIGIN,    /* waiting CAR_TRACK_START_DELAY_MS before origin */
    TRACK_START_WAIT_CONTROL    /* waiting CAR_TRACK_ORIGIN_DELAY_MS after origin */
};

static void StopBallBalance(const char *message)
{
    track_mode = false;
    track_start_state = TRACK_START_IDLE;
    /* Return to level using relative rotation (avoid multi-turn wrap). */
    if (track_tilt_deg != 0) {
        Stepper_RelativeRotate((int32_t)(-track_tilt_deg));
    }
    track_tilt_deg = 0;
    Camera_Stop();
    UartQueue(message);
}

static void StartBallBalance(void)
{
    step_test_mode = false;
    Camera_Start();
    Stepper_Wake();                        /* non-blocking motor wake */
    track_mode = true;
    track_start_state = TRACK_START_WAIT_CAMERA;
    track_state_started_ms = app_time_ms;
    UartQueue("# BALANCE STARTING; WAIT CAMERA");
}
#endif

static void Buzzer_Set(bool enabled)
{
    if (enabled) {
        DL_GPIO_clearPins(CAR_GPIO_BUZZER_PORT, CAR_GPIO_BUZZER_PIN);
    } else {
        DL_GPIO_setPins(CAR_GPIO_BUZZER_PORT, CAR_GPIO_BUZZER_PIN);
    }
}

static void UartQueue(const char *text)
{
    if (tx_index < tx_length) return;
    tx_length = (uint8_t)snprintf(tx_buffer, sizeof(tx_buffer), "%s\r\n", text);
    if (tx_length >= sizeof(tx_buffer)) tx_length = sizeof(tx_buffer) - 1U;
    tx_index = 0U;
}

static void UartServiceTx(void)
{
    while (tx_index < tx_length &&
        DL_UART_Main_transmitDataCheck(TUNER_UART_INST,
            (uint8_t)tx_buffer[tx_index])) {
        tx_index++;
    }
}

static int16_t ParseGain(const char *tag, int16_t fallback)
{
    const char *value = strstr(rx_line, tag);
    int32_t result = 0;
    if (value == NULL) return fallback;
    value += strlen(tag);
    while (*value >= '0' && *value <= '9') {
        result = result * 10 + (*value++ - '0');
    }
    return (int16_t)result;
}

static const char *GyroBootStateText(void)
{
    switch (gyro_boot_state) {
        case GYRO_BOOT_WAIT_DATA:
            return "DATA";
        case GYRO_BOOT_WAIT_STILL:
            return "STILL";
        case GYRO_BOOT_CALIBRATING:
            return "CAL";
        case GYRO_BOOT_READY:
            return "READY";
        default:
            return "UNK";
    }
}

static void FormatSignedMilli(char *buffer, uint8_t size, int32_t value)
{
    int32_t fraction = value < 0 ? -(value % 1000L) : value % 1000L;

    snprintf(buffer, size, "%ld.%03ld",
        (long)(value / 1000L), (long)fraction);
}

static void OledShowLine(uint8_t page, const char *text)
{
    char line[17];
    uint8_t i = 0U;

    while (i < 16U && text[i] != '\0') {
        line[i] = text[i];
        i++;
    }
    while (i < 16U) {
        line[i++] = ' ';
    }
    line[16] = '\0';
    OLED_ShowString(0U, page, line);
}

static void UpdateOledStatus(void)
{
    char line[17];
    char value[12];
    int32_t angle_mdeg;
    int32_t rate_mdps;

    if ((uint32_t)(app_time_ms - oled_last_update_ms) < OLED_STATUS_PERIOD_MS) {
        return;
    }
    oled_last_update_ms = app_time_ms == 0U ? 1U : app_time_ms;

    if (!Gyro_HasData()) {
        OledShowLine(0U, "GYRO WAIT DATA");
        OledShowLine(2U, "ANG ---.---");
        OledShowLine(4U, "RATE ---.---");
        OledShowLine(6U, "F=0 CRC=0");
        return;
    }

    angle_mdeg = (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    rate_mdps = (int32_t)(Gyro_GetRateDps() * 1000.0F);

    {
        const char *mode_str;
#if CAR_TRACK_ENABLE
        if (track_mode)
            mode_str = "TRK";
        else
#endif
        if (car_running)
            mode_str = "RUN";
        else
            mode_str = "STOP";
        snprintf(line, sizeof(line), "GYRO %-5s %s",
            GyroBootStateText(), mode_str);
    }
    OledShowLine(0U, line);

    FormatSignedMilli(value, sizeof(value), angle_mdeg);
    snprintf(line, sizeof(line), "ANG %s", value);
    OledShowLine(2U, line);

    FormatSignedMilli(value, sizeof(value), rate_mdps);
    snprintf(line, sizeof(line), "RATE %s", value);
    OledShowLine(4U, line);

#if CAR_TRACK_ENABLE
    if (track_mode) {
        int16_t ball_x = Camera_GetBallX();
        snprintf(line, sizeof(line), "%s X=%d",
            Camera_HasBall() ? "BALL" : "LOST", ball_x);
        OledShowLine(6U, line);
    } else
#endif
    {
        if (car_running) {
            snprintf(line, sizeof(line), "IR=%02X P=%d",
                LineFollow_GetSensorMask() & 0xFFU,
                LineFollow_GetPosition());
        } else {
            snprintf(line, sizeof(line), "F=%lu C=%lu",
                (unsigned long)Gyro_GetValidFrameCount(),
                (unsigned long)Gyro_GetCrcErrorCount());
        }
        OledShowLine(6U, line);
    }
}

static void QueueGyroStatus(void)
{
    int32_t angle_mdeg = (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    int32_t rate_mdps = (int32_t)(Gyro_GetRateDps() * 1000.0F);
    int32_t angle_fraction = angle_mdeg < 0 ? -(angle_mdeg % 1000) : angle_mdeg % 1000;
    int32_t rate_fraction = rate_mdps < 0 ? -(rate_mdps % 1000) : rate_mdps % 1000;

    snprintf(tx_buffer, sizeof(tx_buffer),
        "# GYRO OK=%u READY=%u ANG=%ld.%03ld RATE=%ld.%03ld SEQ=%u FRAMES=%lu CRCERR=%lu\r\n",
        Gyro_HasData() ? 1U : 0U,
        gyro_boot_state == GYRO_BOOT_READY ? 1U : 0U,
        (long)(angle_mdeg / 1000), (long)angle_fraction,
        (long)(rate_mdps / 1000), (long)rate_fraction,
        Gyro_GetFrameSequence(), (unsigned long)Gyro_GetValidFrameCount(),
        (unsigned long)Gyro_GetCrcErrorCount());
    tx_length = (uint8_t)strlen(tx_buffer);
    tx_index = 0U;
}

static void UpdateGyroAutoCalibration(void)
{
    int32_t rate_mdps;

    if (!Gyro_HasData() ||
        (uint32_t)(app_time_ms - gyro_last_frame_ms) > CAR_GYRO_STALE_TIMEOUT_MS) {
        if (gyro_boot_state != GYRO_BOOT_CALIBRATING) {
            gyro_boot_state = GYRO_BOOT_WAIT_DATA;
        }
        return;
    }

    rate_mdps = (int32_t)(Gyro_GetRateDps() * 1000.0F);
    if (gyro_boot_state == GYRO_BOOT_WAIT_DATA) {
        gyro_still_reference_mdps = rate_mdps;
        gyro_still_started_ms = app_time_ms;
        gyro_boot_state = GYRO_BOOT_WAIT_STILL;
    } else if (gyro_boot_state == GYRO_BOOT_WAIT_STILL) {
        if (labs(rate_mdps - gyro_still_reference_mdps) >
            CAR_GYRO_STATIC_DELTA_MDPS) {
            gyro_still_reference_mdps = rate_mdps;
            gyro_still_started_ms = app_time_ms;
        } else if ((uint32_t)(app_time_ms - gyro_still_started_ms) >=
            CAR_GYRO_STATIC_CONFIRM_MS) {
            Gyro_RequestBiasCalibration();
            gyro_cal_started_ms = app_time_ms;
            gyro_boot_state = GYRO_BOOT_CALIBRATING;
            UartQueue("# GYRO AUTO CAL START; KEEP STILL");
        }
    } else if (gyro_boot_state == GYRO_BOOT_CALIBRATING &&
        (uint32_t)(app_time_ms - gyro_cal_started_ms) >=
            CAR_GYRO_AUTO_CAL_WAIT_MS) {
        if (labs(rate_mdps) <= CAR_GYRO_READY_RATE_MDPS) {
            gyro_boot_state = GYRO_BOOT_READY;
            UartQueue("# GYRO AUTO CAL DONE; READY");
        } else {
            gyro_still_reference_mdps = rate_mdps;
            gyro_still_started_ms = app_time_ms;
            gyro_boot_state = GYRO_BOOT_WAIT_STILL;
            UartQueue("# GYRO AUTO CAL RETRY; KEEP STILL");
        }
    }
}

static void QueueGyroRawFrame(void)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t frame[16];
    uint8_t index;
    uint8_t output = 0U;

    Gyro_CopyLastFrame(frame);
    tx_buffer[output++] = '#';
    tx_buffer[output++] = ' ';
    tx_buffer[output++] = 'R';
    tx_buffer[output++] = 'A';
    tx_buffer[output++] = 'W';
    tx_buffer[output++] = ' ';
    for (index = 0U; index < 16U; index++) {
        tx_buffer[output++] = hex[frame[index] >> 4U];
        tx_buffer[output++] = hex[frame[index] & 0x0FU];
        tx_buffer[output++] = index == 15U ? '\r' : ' ';
    }
    tx_buffer[output++] = '\n';
    tx_buffer[output] = '\0';
    tx_length = output;
    tx_index = 0U;
}

static void StopAngleTurn(const char *message)
{
    angle_turn_active = false;
    angle_turn_line_mode = false;
    angle_turn_direction = 0;
    angle_turn_settle_started_ms = 0U;
    Motor_Stop();
    Buzzer_Set(false);
    UartQueue(message);
}

static void StartAngleTurn(int32_t relative_degrees)
{
    int32_t current_mdeg;
    int32_t rate_mdps;

    if (car_running) {
        UartQueue("# TURN ERROR STOP LINE FOLLOW FIRST");
        return;
    }
    if (gyro_boot_state != GYRO_BOOT_READY) {
        UartQueue("# TURN ERROR AUTO CAL NOT READY; KEEP STILL");
        return;
    }
    if (!Gyro_HasData() ||
        (uint32_t)(app_time_ms - gyro_last_frame_ms) > CAR_GYRO_STALE_TIMEOUT_MS) {
        UartQueue("# TURN ERROR GYRO NOT READY");
        return;
    }
    rate_mdps = (int32_t)(Gyro_GetRateDps() * 1000.0F);
    if (labs(rate_mdps) > CAR_GYRO_READY_RATE_MDPS) {
        UartQueue("# TURN ERROR GYRO BIAS; KEEP STILL, SEND GYRO CAL");
        return;
    }
    if (relative_degrees == 0 || relative_degrees < -180 || relative_degrees > 180) {
        UartQueue("# TURN ERROR RANGE -180..180");
        return;
    }

    current_mdeg = (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    angle_turn_start_mdeg = current_mdeg;
    angle_turn_target_mdeg = current_mdeg + relative_degrees * 1000L;
    angle_turn_direction = relative_degrees > 0 ? 1 : -1;
    angle_turn_started_ms = app_time_ms;
    angle_turn_settle_started_ms = 0U;
    angle_turn_active = true;
    angle_turn_line_mode = false;
    snprintf(tx_buffer, sizeof(tx_buffer), "# TURN START REL=%ld TARGET=%ld.%03ld\r\n",
        (long)relative_degrees, (long)(angle_turn_target_mdeg / 1000L),
        (long)labs(angle_turn_target_mdeg % 1000L));
    tx_length = (uint8_t)strlen(tx_buffer);
    tx_index = 0U;
}

static int32_t NormalizeAngleError(int32_t error_mdeg)
{
    while (error_mdeg > 180000L) error_mdeg -= 360000L;
    while (error_mdeg < -180000L) error_mdeg += 360000L;
    return error_mdeg;
}

static int32_t AbsoluteDelta(int32_t current, int32_t start)
{
    int32_t delta = current - start;
    return delta < 0 ? -delta : delta;
}

static void StopEncoderDrive(const char *message)
{
    encoder_drive_active = false;
    encoder_drive_line_mode = false;
    Motor_Stop();
    UartQueue(message);
}

static void StartEncoderDrive(int32_t distance_mm)
{
    if (car_running || angle_turn_active) {
        UartQueue("# DRIVE ERROR STOP OTHER MODE FIRST");
        return;
    }
    if (distance_mm == 0 || distance_mm < -1000L || distance_mm > 1000L) {
        UartQueue("# DRIVE ERROR RANGE -100..100 CM; ZERO INVALID");
        return;
    }
    encoder_drive_m1_start = Encoder_GetMotor1Count();
    encoder_drive_m2_start = Encoder_GetMotor2Count();
    encoder_drive_started_ms = app_time_ms;
    encoder_drive_distance_mm = distance_mm;
    encoder_drive_line_mode = false;
    encoder_drive_turn_direction = 0;
    encoder_drive_heading_mdeg =
        (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    encoder_drive_heading_correction = 0;
    encoder_drive_active = true;
    snprintf(tx_buffer, sizeof(tx_buffer), "# DRIVE %ld START\r\n",
        (long)(distance_mm / 10L));
    tx_length = (uint8_t)strlen(tx_buffer);
    tx_index = 0U;
}

#if CAR_LINE_CORNER_TURN_ENABLE
static void StartLineAngleTurn(int8_t direction)
{
    int32_t current_mdeg = (int32_t)(Gyro_GetAngleDeg() * 1000.0F);

    /* Positive sensor direction means the line is on the right. */
    angle_turn_start_mdeg = current_mdeg;
    angle_turn_target_mdeg = current_mdeg +
        (int32_t)direction * CAR_LINE_TURN_ANGLE_DEG * 1000L;
    angle_turn_direction = direction > 0 ? 1 : -1;
    angle_turn_started_ms = app_time_ms;
    angle_turn_settle_started_ms = 0U;
    angle_turn_active = true;
    angle_turn_line_mode = true;
    line_reacquired_started_ms = 0U;
    line_lost_started_ms = 0U;
    heading_locked = false;
    LineFollow_SetHeadingCorrection(0);
    Buzzer_Set(true);
}

static void StartLineAdvance(int8_t direction)
{
    encoder_drive_m1_start = corner_preview_active ?
        corner_preview_m1_start : Encoder_GetMotor1Count();
    encoder_drive_m2_start = corner_preview_active ?
        corner_preview_m2_start : Encoder_GetMotor2Count();
    encoder_drive_started_ms = app_time_ms;
    encoder_drive_distance_mm = CAR_ENCODER_ADVANCE_MM;
    encoder_drive_turn_direction = direction;
    encoder_drive_heading_mdeg = corner_preview_active ?
        corner_preview_heading_mdeg :
        (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    encoder_drive_line_mode = true;
    encoder_drive_active = true;
    encoder_drive_heading_correction = 0;
    line_advance_detected_ms = 0U;
    heading_locked = false;
    LineFollow_SetHeadingCorrection(0);
    corner_preview_active = false;
}

static int8_t GetCornerPatternDirection(uint8_t mask)
{
    const uint8_t right_outer = 0x03U;
    const uint8_t left_outer = 0xC0U;

    /* Enter protection before the center sensors leave the old line. */
    if ((mask & right_outer) != 0U && (mask & left_outer) == 0U) return 1;
    if ((mask & left_outer) != 0U && (mask & right_outer) == 0U) return -1;
    /* A dirty/wide junction can momentarily report all eight sensors high.
     * Never infer left from that ambiguous mask: reuse the direction seen
     * immediately before the blackout, if one was established. */
    if (mask == 0xFFU && last_corner_direction != 0) {
        return last_corner_direction;
    }
    return 0;
}

static void UpdateCornerCandidate(void)
{
    uint8_t mask = LineFollow_GetSensorMask();

    if (mask == 0U) {
        if (corner_white_started_ms == 0U) {
            corner_white_started_ms = app_time_ms;
        } else if ((uint32_t)(app_time_ms - corner_white_started_ms) >=
            CAR_LINE_LOST_CONFIRM_MS) {
            corner_candidate_confirmed = true;
        }
    } else {
        corner_white_started_ms = 0U;
    }
}

static void DetectCornerCandidate(void)
{
    uint8_t mask = LineFollow_GetSensorMask();
    int8_t direction = GetCornerPatternDirection(mask);

    if (corner_candidate_active) {
        Motor_SetSignedSpeed(CAR_CORNER_ADVANCE_SPEED,
            CAR_CORNER_ADVANCE_SPEED);
        corner_observation_samples++;
        if (mask == 0U) corner_white_samples++;
        if ((mask & 0x3CU) != 0U) corner_center_samples++;

        if ((uint32_t)(app_time_ms - corner_candidate_started_ms) >=
            CAR_CORNER_WHITE_WINDOW_MS) {
            uint32_t white_percent = corner_observation_samples == 0U ? 0U :
                (uint32_t)corner_white_samples * 100U /
                    corner_observation_samples;

            if (white_percent >= CAR_CORNER_WHITE_PERCENT) {
                int8_t locked_direction = corner_candidate_direction;
                corner_candidate_active = false;
                StartLineAdvance(locked_direction);
            } else {
                corner_candidate_active = false;
                corner_candidate_direction = 0;
                corner_pattern_started_ms = 0U;
                corner_preview_active = false;
                last_corner_direction = 0;
                LineFollow_Init();
            }
        }
        return;
    }

    if (direction != 0 && mask != 0xFFU) {
        last_corner_direction = direction;
    }

    /* A real corner begins with an outer sensor while the old straight line
     * is still visible in the center. An isolated dirty outer sensor cannot
     * satisfy the following mostly-white observation window. */
    if (direction == 0 || (mask & 0x3CU) == 0U) {
        corner_pattern_started_ms = 0U;
        corner_preview_active = false;
        return;
    }
    /* Lock the first unambiguous side immediately. The sensor input is
     * already majority-filtered; requiring this exact pattern for several
     * more milliseconds caused real corners to disappear before arming. */
    corner_candidate_direction = direction;
    corner_pattern_started_ms = app_time_ms;
    corner_preview_active = true;
    corner_preview_m1_start = Encoder_GetMotor1Count();
    corner_preview_m2_start = Encoder_GetMotor2Count();
    corner_preview_heading_mdeg =
        (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    corner_candidate_active = true;
    corner_candidate_confirmed = false;
    corner_candidate_started_ms = app_time_ms;
    corner_center_started_ms = 0U;
    corner_white_started_ms = 0U;
    corner_observation_samples = 1U;
    corner_white_samples = 0U;
    corner_center_samples = 1U;
    Motor_SetSignedSpeed(CAR_CORNER_ADVANCE_SPEED,
        CAR_CORNER_ADVANCE_SPEED);
}
#endif

static void UpdateEncoderDrive(void)
{
    const int16_t drive_direction = encoder_drive_distance_mm < 0 ? -1 : 1;
    const int32_t distance_mm = labs(encoder_drive_distance_mm);
    const int32_t m2_target = CAR_ENCODER_M2_COUNTS_PER_REV *
        distance_mm / 204L;
    const int32_t m1_target = CAR_ENCODER_M1_COUNTS_PER_REV *
        distance_mm / 204L;
    const int32_t m2_stop = m2_target * CAR_ENCODER_STOP_COMP_PERCENT / 100L;
    const int32_t m1_stop = m1_target * CAR_ENCODER_STOP_COMP_PERCENT / 100L;
    int32_t m1 = AbsoluteDelta(Encoder_GetMotor1Count(),
        encoder_drive_m1_start);
    int32_t m2 = AbsoluteDelta(Encoder_GetMotor2Count(),
        encoder_drive_m2_start);
    int32_t m2_progress = m2 * 1000L / m2_target;
    int16_t base_speed;
    int16_t left_speed;
    int16_t right_speed;
    int16_t heading = 0;

    if (encoder_drive_line_mode) {
        base_speed = m2_progress >= CAR_ENCODER_SLOWDOWN_PERCENT * 10L ?
            CAR_CORNER_ADVANCE_SLOW_SPEED : CAR_CORNER_ADVANCE_SPEED;
    } else {
        base_speed = m2_progress >= CAR_ENCODER_SLOWDOWN_PERCENT * 10L ?
            CAR_ENCODER_SLOW_SPEED : CAR_ENCODER_TEST_SPEED;
    }

    if (encoder_drive_line_mode) {
        LineFollow_Update();
        if (LineFollow_StopLineDetected()) {
            car_running = false;
            Motor_Stop();
            UartQueue("# FINISH LINE; STOPPED");
        }
#if CAR_LINE_CORNER_TURN_ENABLE
        if (corner_candidate_active) UpdateCornerCandidate();
#endif
        if (!encoder_drive_active) return;
    }

    if (encoder_drive_line_mode && Gyro_HasData() &&
        (uint32_t)(app_time_ms - gyro_last_frame_ms) <=
            CAR_GYRO_STALE_TIMEOUT_MS) {
        int32_t heading_error = NormalizeAngleError(
            (int32_t)(Gyro_GetAngleDeg() * 1000.0F) -
            encoder_drive_heading_mdeg);
        int32_t rate_mdps = (int32_t)(Gyro_GetRateDps() * 1000.0F);
        int16_t target_heading = (int16_t)(
            heading_error / CAR_ADVANCE_HEADING_DIVISOR +
            rate_mdps / CAR_ADVANCE_RATE_DIVISOR);

        if (target_heading > encoder_drive_heading_correction +
            CAR_ADVANCE_HEADING_STEP) {
            target_heading = encoder_drive_heading_correction +
                CAR_ADVANCE_HEADING_STEP;
        } else if (target_heading < encoder_drive_heading_correction -
            CAR_ADVANCE_HEADING_STEP) {
            target_heading = encoder_drive_heading_correction -
                CAR_ADVANCE_HEADING_STEP;
        }
        encoder_drive_heading_correction = target_heading;
        heading = encoder_drive_heading_correction;
        if (heading > CAR_ADVANCE_HEADING_MAX) {
            heading = CAR_ADVANCE_HEADING_MAX;
        } else if (heading < -CAR_ADVANCE_HEADING_MAX) {
            heading = -CAR_ADVANCE_HEADING_MAX;
        }
    }

    /* Serial DRIVE is deliberately open-loop: identical signed PWM on both
     * wheels, with the right-wheel M1 encoder used only for distance. */
    left_speed = (int16_t)(drive_direction * base_speed);
    right_speed = (int16_t)(drive_direction * base_speed);
    if (encoder_drive_line_mode) {
        if (drive_direction > 0) {
            if (heading > 0) {
                left_speed += heading;
            } else if (heading < 0) {
                right_speed -= heading;
            }
        }
    }

    /* Automatic corner advance keeps M2 QEI; serial DRIVE uses right-wheel
     * M1 as explicitly requested. */
    if ((encoder_drive_line_mode && m2 >= m2_stop) ||
        (!encoder_drive_line_mode && m1 >= m1_stop)) {
        if (encoder_drive_line_mode) {
#if CAR_LINE_CORNER_TURN_ENABLE
            int8_t direction = encoder_drive_turn_direction;
            encoder_drive_active = false;
            encoder_drive_line_mode = false;
            corner_candidate_active = false;
            corner_candidate_confirmed = false;
            StartLineAngleTurn(direction);
#else
            StopEncoderDrive("# DRIVE DONE");
#endif
        } else {
            StopEncoderDrive("# DRIVE DONE");
        }
    } else if ((uint32_t)(app_time_ms - encoder_drive_started_ms) >=
        (encoder_drive_line_mode ? CAR_CORNER_ADVANCE_TIMEOUT_MS :
            CAR_ENCODER_DRIVE_TIMEOUT_MS)) {
        if (encoder_drive_line_mode) car_running = false;
        StopEncoderDrive("# DRIVE ERROR TIMEOUT");
    } else if (encoder_drive_line_mode &&
        (!Gyro_HasData() ||
        (uint32_t)(app_time_ms - gyro_last_frame_ms) >
            CAR_GYRO_STALE_TIMEOUT_MS)) {
        car_running = false;
        StopEncoderDrive("# DRIVE ERROR GYRO STALE");
    } else {
        Motor_SetSignedSpeed(left_speed, right_speed);
    }
}

static void UpdateAngleTurn(void)
{
    int32_t current_mdeg = (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    int32_t rate_mdps = (int32_t)(Gyro_GetRateDps() * 1000.0F);
    int32_t error_mdeg = NormalizeAngleError(
        angle_turn_target_mdeg - current_mdeg);
    int32_t absolute_error = labs(error_mdeg);
    int32_t absolute_rate = labs(rate_mdps);
    int32_t effort;
    int16_t speed;

    if (angle_turn_line_mode) {
        /* Keep the sensor filter current, but the gyro alone decides when
         * the fixed 90-degree turn is complete. */
        LineFollow_Update();
    }

    if (!angle_turn_line_mode &&
        (uint32_t)(app_time_ms - angle_turn_started_ms) >= CAR_ANGLE_TURN_TIMEOUT_MS) {
        StopAngleTurn("# TURN ERROR TIMEOUT");
        return;
    }
    if ((uint32_t)(app_time_ms - gyro_last_frame_ms) > CAR_GYRO_STALE_TIMEOUT_MS) {
        if (angle_turn_line_mode) car_running = false;
        StopAngleTurn("# TURN ERROR GYRO STALE");
        return;
    }

    if (absolute_error <= CAR_ANGLE_TURN_TOLERANCE_MDEG &&
        (angle_turn_line_mode || absolute_rate <= CAR_ANGLE_TURN_RATE_MDPS)) {
        if (angle_turn_line_mode) {
            /* The gyro turn owns exactly 90 degrees. Hand control to a fresh
             * line-follow state regardless of which sensor sees the line. */
            Motor_Stop();
            Buzzer_Set(false);
            angle_turn_active = false;
            angle_turn_line_mode = false;
            angle_turn_direction = 0;
            line_reacquired_started_ms = 0U;
            corner_detection_armed = false;
            corner_rearm_started_ms = 0U;
            corner_pattern_started_ms = 0U;
            corner_preview_active = false;
            last_corner_direction = 0;
            LineFollow_Init();
            return;
        }
        Motor_Stop();
        if (angle_turn_settle_started_ms == 0U) {
            angle_turn_settle_started_ms = app_time_ms;
        } else if ((uint32_t)(app_time_ms - angle_turn_settle_started_ms) >=
            CAR_ANGLE_TURN_SETTLE_MS) {
            StopAngleTurn("# TURN DONE");
        }
        return;
    }
    angle_turn_settle_started_ms = 0U;

    /* PD effort in speed units: angle drives rotation, rate damps overshoot. */
    effort = error_mdeg / 5000L - rate_mdps / 20000L;
    if (angle_turn_line_mode) {
        int16_t minimum_speed = absolute_error <=
            CAR_LINE_ANGLE_SLOWDOWN_DEG * 1000L ?
            CAR_LINE_ANGLE_FINISH_SPEED : CAR_LINE_ANGLE_TURN_MIN_SPEED;

        if (effort > CAR_LINE_ANGLE_TURN_MAX_SPEED) {
            effort = CAR_LINE_ANGLE_TURN_MAX_SPEED;
        }
        if (effort < -CAR_LINE_ANGLE_TURN_MAX_SPEED) {
            effort = -CAR_LINE_ANGLE_TURN_MAX_SPEED;
        }
        if (effort > 0 && effort < minimum_speed) {
            effort = minimum_speed;
        }
        if (effort < 0 && effort > -minimum_speed) {
            effort = -minimum_speed;
        }
        /* A line-triggered corner has a latched direction. Gyro damping may
         * reduce its speed, but must never reverse it near the target and
         * make the chassis oscillate before the new line is acquired. */
        if (angle_turn_direction > 0 && effort <
            minimum_speed) {
            effort = minimum_speed;
        } else if (angle_turn_direction < 0 && effort >
            -minimum_speed) {
            effort = -minimum_speed;
        }
    } else {
        if (effort > CAR_ANGLE_TURN_MAX_SPEED) effort = CAR_ANGLE_TURN_MAX_SPEED;
        if (effort < -CAR_ANGLE_TURN_MAX_SPEED) effort = -CAR_ANGLE_TURN_MAX_SPEED;
        if (effort > 0 && effort < CAR_ANGLE_TURN_MIN_SPEED) effort = CAR_ANGLE_TURN_MIN_SPEED;
        if (effort < 0 && effort > -CAR_ANGLE_TURN_MIN_SPEED) effort = -CAR_ANGLE_TURN_MIN_SPEED;
    }
    speed = (int16_t)effort;

    /* Positive gyro angle is a physical left turn. */
    Motor_SetSignedSpeed(speed, (int16_t)-speed);
}

#if CAR_LINE_GYRO_HEADING_ENABLE
static void UpdateHeadingHold(void)
{
    int32_t current_mdeg;
    int32_t rate_mdps;
    int32_t error_mdeg;
    int32_t absolute_position = labs(LineFollow_GetPosition());
    int32_t correction;

    if (!Gyro_HasData() ||
        (uint32_t)(app_time_ms - gyro_last_frame_ms) > CAR_GYRO_STALE_TIMEOUT_MS ||
        absolute_position > CAR_CENTER_EXIT_ERROR) {
        heading_locked = false;
        LineFollow_SetHeadingCorrection(0);
        return;
    }

    current_mdeg = (int32_t)(Gyro_GetAngleDeg() * 1000.0F);
    if (!heading_locked) {
        heading_target_mdeg = current_mdeg;
        heading_locked = true;
        LineFollow_SetHeadingCorrection(0);
        return;
    }

    rate_mdps = (int32_t)(Gyro_GetRateDps() * 1000.0F);
    error_mdeg = NormalizeAngleError(heading_target_mdeg - current_mdeg);
    if (labs(error_mdeg) <= CAR_HEADING_DEADBAND_MDEG) error_mdeg = 0;
    correction = -(error_mdeg / CAR_HEADING_KP_DIVISOR -
        rate_mdps / CAR_HEADING_KD_DIVISOR);
    if (correction > CAR_HEADING_MAX_CORRECTION) {
        correction = CAR_HEADING_MAX_CORRECTION;
    } else if (correction < -CAR_HEADING_MAX_CORRECTION) {
        correction = -CAR_HEADING_MAX_CORRECTION;
    }
    LineFollow_SetHeadingCorrection((int16_t)correction);
}
#endif

static void ProcessCommand(void)
{
    int16_t kp, ki, kd;
    LineFollow_GetPid(&kp, &ki, &kd);

    if (strncmp(rx_line, "SET ", 4) == 0) {
        kp = ParseGain("P:", kp);
        ki = ParseGain("I:", ki);
        kd = ParseGain("D:", kd);
        UartQueue(LineFollow_SetPid(kp, ki, kd) ?
            "# PID UPDATED" : "# ERROR PID RANGE");
    } else if (strcmp(rx_line, "STOP") == 0) {
        car_running = false;
        angle_turn_active = false;
        angle_turn_line_mode = false;
        angle_turn_direction = 0;
        encoder_drive_active = false;
        encoder_drive_line_mode = false;
        encoder_drive_turn_direction = 0;
        heading_locked = false;
        LineFollow_SetHeadingCorrection(0);
        Motor_Stop();
        Buzzer_Set(false);
        UartQueue("# STOPPED");
    } else if (strcmp(rx_line, "RUN") == 0) {
        car_running = true;
        LineFollow_Init();
        heading_locked = false;
        UartQueue("# RUNNING; INFRARED LINE FOLLOW");
    } else if (strcmp(rx_line, "STATUS") == 0) {
        snprintf(tx_buffer, sizeof(tx_buffer), "# STATUS P=%d I=%d D=%d RUN=%u\r\n",
            kp, ki, kd, car_running ? 1U : 0U);
        tx_length = (uint8_t)strlen(tx_buffer);
        tx_index = 0U;
    } else if (strcmp(rx_line, "GYRO?") == 0 || strcmp(rx_line, "?") == 0) {
        QueueGyroStatus();
    } else if (strcmp(rx_line, "GYRO RAW") == 0 || strcmp(rx_line, "!") == 0) {
        QueueGyroRawFrame();
    } else if (strcmp(rx_line, "GYRO ZERO") == 0) {
        Gyro_RequestZero();
        UartQueue("# GYRO ZERO SENT");
    } else if (strcmp(rx_line, "GYRO CAL") == 0) {
        Gyro_RequestBiasCalibration();
        gyro_cal_started_ms = app_time_ms;
        gyro_boot_state = GYRO_BOOT_CALIBRATING;
        UartQueue("# GYRO CAL SENT");
    } else if (strcmp(rx_line, "ENC?") == 0) {
        snprintf(tx_buffer, sizeof(tx_buffer), "# ENC M1=%ld M2=%ld\r\n",
            (long)Encoder_GetMotor1Count(),
            (long)Encoder_GetMotor2Count());
        tx_length = (uint8_t)strlen(tx_buffer);
        tx_index = 0U;
    } else if (strcmp(rx_line, "STEP?") == 0) {
        snprintf(tx_buffer, sizeof(tx_buffer),
            "# STEP TX=%lu RX=%lu OVF=%lu BUF=%u TRACE=%u\r\n",
            (unsigned long)Stepper_GetTxByteCount(),
            (unsigned long)Stepper_GetRxByteCount(),
            (unsigned long)Stepper_GetRxOverflowCount(),
            Stepper_GetBufferedResponseCount(), Stepper_GetTraceCount());
        tx_length = (uint8_t)strlen(tx_buffer);
        tx_index = 0U;
    } else if (strcmp(rx_line, "STEP RAW?") == 0) {
        step_raw_dump = true;
        UartQueue("# STEP RAW DUMP");
    } else if (strcmp(rx_line, "ENC ZERO") == 0) {
        Encoder_Zero();
        UartQueue("# ENC ZEROED");
    } else if (strncmp(rx_line, "DRIVE ", 6) == 0) {
        char *end;
        long distance_cm = strtol(&rx_line[6], &end, 10);

        if (end == &rx_line[6] || *end != '\0') {
            UartQueue("# DRIVE ERROR FORMAT: DRIVE -100..100");
        } else if (distance_cm == 0 || distance_cm < -100L ||
            distance_cm > 100L) {
            UartQueue("# DRIVE ERROR RANGE -100..100 CM; ZERO INVALID");
        } else {
            StartEncoderDrive(distance_cm * 10L);
        }
    } else if (strncmp(rx_line, "TURN ", 5) == 0) {
        StartAngleTurn(strtol(&rx_line[5], NULL, 10));
    } else if (strcmp(rx_line, "STEP TEST") == 0) {
        step_test_mode = !step_test_mode;
        if (step_test_mode) {
            step_test_last_ms = (int32_t)app_time_ms;
            step_test_direction = -1;
            UartQueue("# STEP TEST STARTED (1s +/-10 deg)");
        } else {
            UartQueue("# STEP TEST STOPPED");
        }
    } else if (strcmp(rx_line, "STEP WAKE") == 0) {
        Stepper_Wake();
        UartQueue("# STEP WAKE SENT");
    } else if (strcmp(rx_line, "STEP SLEEP") == 0) {
        Stepper_Sleep();
        UartQueue("# STEP SLEEP SENT");
    } else if (strcmp(rx_line, "STEP ZERO") == 0) {
        Stepper_SetOrigin();
        UartQueue("# STEP ZERO SENT");
    } else if (strncmp(rx_line, "STEP A ", 7) == 0) {
        Stepper_AbsoluteRotate(strtol(&rx_line[7], NULL, 10));
        UartQueue("# STEP ABS SENT");
    } else if (strncmp(rx_line, "STEP ", 5) == 0) {
        Stepper_RelativeRotate(strtol(&rx_line[5], NULL, 10));
        UartQueue("# STEP REL SENT");
    } else if (strcmp(rx_line, "CAM START") == 0) {
#if CAR_CAMERA_ENABLE
        if (gyro_boot_state == GYRO_BOOT_READY) {
            Camera_Start();
            camera_mode = true;
            car_running = true;
            heading_locked = false;
            UartQueue("# CAMERA MODE; BALL TRACKING");
        } else {
            UartQueue("# CAM BLOCKED; GYRO NOT READY");
        }
#else
        UartQueue("# CAMERA DISABLED IN CONFIG");
#endif
    } else if (strcmp(rx_line, "CAM STOP") == 0) {
#if CAR_CAMERA_ENABLE
        Camera_Stop();
        camera_mode = false;
        car_running = false;
        Motor_Stop();
        UartQueue("# CAMERA STOPPED");
#else
        UartQueue("# CAMERA DISABLED IN CONFIG");
#endif
    } else if (strcmp(rx_line, "CAM?") == 0) {
#if CAR_CAMERA_ENABLE
        snprintf(tx_buffer, sizeof(tx_buffer),
            "# CAM ON=%u BALL=%u X=%d FR=%lu AGE=%lu\r\n",
            Camera_IsOnline(app_time_ms) ? 1U : 0U,
            Camera_HasBall() ? 1U : 0U,
            Camera_GetBallX(),
            (unsigned long)Camera_GetValidFrameCount(),
            (unsigned long)(app_time_ms - Camera_GetLastValidMs()));
        tx_length = (uint8_t)strlen(tx_buffer);
        tx_index = 0U;
#else
        UartQueue("# CAMERA DISABLED IN CONFIG");
#endif
    } else if (strcmp(rx_line, "TRACK START") == 0) {
#if CAR_TRACK_ENABLE
        car_running = false;
        Motor_Stop();
        StartBallBalance();
#else
        UartQueue("# TRACK DISABLED IN CONFIG");
#endif
    } else if (strcmp(rx_line, "TRACK STOP") == 0) {
#if CAR_TRACK_ENABLE
        StopBallBalance("# BALANCE STOPPED");
#else
        UartQueue("# TRACK DISABLED IN CONFIG");
#endif
    } else if (strcmp(rx_line, "TRACK?") == 0) {
#if CAR_TRACK_ENABLE
        snprintf(tx_buffer, sizeof(tx_buffer),
            "# TRACK MODE=%u ST=%u ONLINE=%u BALL=%u X=%d TILT=%d KP=%d CD=%lu\r\n",
            track_mode ? 1U : 0U,
            track_start_state,
            Camera_IsOnline(app_time_ms) ? 1U : 0U,
            Camera_HasBall() ? 1U : 0U,
            Camera_GetBallX(),
            track_tilt_deg,
            track_kp_divisor,
            track_last_adjust_ms == 0 ? 0UL :
                (unsigned long)(app_time_ms -
                    (uint32_t)track_last_adjust_ms));
        tx_length = (uint8_t)strlen(tx_buffer);
        tx_index = 0U;
#else
        UartQueue("# TRACK DISABLED IN CONFIG");
#endif
    } else if (strcmp(rx_line, "TRACK DEBUG") == 0) {
#if CAR_TRACK_ENABLE
        track_debug = !track_debug;
        snprintf(tx_buffer, sizeof(tx_buffer),
            "# TRACK DEBUG %s\r\n", track_debug ? "ON" : "OFF");
        tx_length = (uint8_t)strlen(tx_buffer);
        tx_index = 0U;
#else
        UartQueue("# TRACK DISABLED IN CONFIG");
#endif
    } else if (strncmp(rx_line, "TRACK KP ", 9) == 0) {
#if CAR_TRACK_ENABLE
        char *end;
        long val = strtol(&rx_line[9], &end, 10);
        if (end == &rx_line[9] || *end != '\0' || val < 1 || val > 200) {
            UartQueue("# TRACK KP ERROR RANGE 1..200");
        } else {
            track_kp_divisor = (int16_t)val;
            snprintf(tx_buffer, sizeof(tx_buffer),
                "# TRACK KP SET TO %ld\r\n", val);
            tx_length = (uint8_t)strlen(tx_buffer);
            tx_index = 0U;
        }
#else
        UartQueue("# TRACK DISABLED IN CONFIG");
#endif
    } else if (strncmp(rx_line, "RAW ", 4) == 0) {
        /* RAW <hex bytes> — send raw hex to stepper motor for testing.
           Example: RAW AA 55 01 11 05 01 00 00 00 10 28   */
        uint8_t raw[32];
        uint8_t raw_len = 0U;
        const char *p = &rx_line[4];
        while (*p != '\0' && raw_len < sizeof(raw)) {
            uint8_t hi, lo;
            while (*p == ' ') p++;
            if (*p == '\0') break;
            hi = (uint8_t)(*p >= '0' && *p <= '9' ? *p - '0' :
                           *p >= 'A' && *p <= 'F' ? *p - 'A' + 10 :
                           *p >= 'a' && *p <= 'f' ? *p - 'a' + 10 : 0xFFU);
            p++;
            if (*p == '\0' || *p == ' ') {
                lo = hi; hi = 0U;
            } else {
                lo = (uint8_t)(*p >= '0' && *p <= '9' ? *p - '0' :
                               *p >= 'A' && *p <= 'F' ? *p - 'A' + 10 :
                               *p >= 'a' && *p <= 'f' ? *p - 'a' + 10 : 0xFFU);
                p++;
            }
            raw[raw_len++] = (uint8_t)((hi << 4U) | lo);
        }
        if (raw_len > 0U) {
            Stepper_SendRawBytes(raw, raw_len);
            UartQueue("# RAW SENT");
        }
    } else {
        UartQueue("# ERROR UNKNOWN COMMAND");
    }
}

static void CompleteRxLine(void)
{
    if (rx_length == 0U) return;
    rx_line[rx_length] = '\0';
    ProcessCommand();
    rx_length = 0U;
}

static void UartServiceRx(void)
{
    uint8_t byte;
    uint8_t next;

    /* Polling fallback keeps Bluetooth commands working even if an RX IRQ is
     * missed or left pending by the UART peripheral. */
    while (DL_UART_Main_receiveDataCheck(TUNER_UART_INST, &byte)) {
        next = (uint8_t)((tuner_rx_write + 1U) % TUNER_RX_RING_SIZE);
        if (next == tuner_rx_read) {
            tuner_rx_overflow_count++;
        } else {
            tuner_rx_ring[tuner_rx_write] = byte;
            tuner_rx_write = next;
        }
    }
    while (tuner_rx_read != tuner_rx_write) {
        byte = tuner_rx_ring[tuner_rx_read];
        tuner_rx_read = (uint8_t)((tuner_rx_read + 1U) % TUNER_RX_RING_SIZE);
        rx_byte_count++;
        telemetry_pause_until_ms = app_time_ms + 200U;
        if (byte == '\r' || byte == '\n') {
            CompleteRxLine();
        } else if (rx_length < sizeof(rx_line) - 1U) {
            rx_line[rx_length++] = (char)byte;
            rx_last_byte_ms = app_time_ms;
            if (byte == '?' || byte == '!') {
                CompleteRxLine();
            }
        } else {
            rx_length = 0U;
        }
    }

    /* Some serial assistants send neither CR nor LF. */
    if (rx_length != 0U &&
        (uint32_t)(app_time_ms - rx_last_byte_ms) >= 30U) {
        CompleteRxLine();
    }
}

void UART1_IRQHandler(void)
{
    uint8_t byte;
    uint8_t next;

    (void)DL_UART_Main_getPendingInterrupt(TUNER_UART_INST);
    while (DL_UART_Main_receiveDataCheck(TUNER_UART_INST, &byte)) {
        next = (uint8_t)((tuner_rx_write + 1U) % TUNER_RX_RING_SIZE);
        if (next == tuner_rx_read) {
            tuner_rx_overflow_count++;
        } else {
            tuner_rx_ring[tuner_rx_write] = byte;
            tuner_rx_write = next;
        }
    }
}

#if CAR_TELEMETRY_ENABLE
static void QueueTelemetry(void)
{
    if (tx_index < tx_length) return;

#if CAR_TRACK_ENABLE
    if (track_mode) {
        /* TRACK mode: ball position + stepper angle telemetry. */
        int16_t ball_x = Camera_GetBallX();
        snprintf(tx_buffer, sizeof(tx_buffer),
            "%lu,1,%d,%u,%d,%d,%d,%d,%d,%u\r\n",
            (unsigned long)app_time_ms,
            ball_x,
            Camera_HasBall() ? 1U : 0U,
            track_tilt_deg,
            track_kp_divisor,
            CAR_TRACK_MAX_TILT_DEG,
            track_start_state,
            Camera_IsOnline(app_time_ms) ? 1U : 0U,
            track_mode ? 1U : 0U);
    } else
#endif
    {
        int16_t kp, ki, kd;
        int16_t position = LineFollow_GetPosition();
        LineFollow_GetPid(&kp, &ki, &kd);
        snprintf(tx_buffer, sizeof(tx_buffer),
            "%lu,0,%d,%d,%d,%d,%d,%d,%u,%lu\r\n",
            (unsigned long)app_time_ms, position, LineFollow_GetCorrection(),
            -position, kp, ki, kd, car_running ? 1U : 0U,
            (unsigned long)rx_byte_count);
    }
    tx_length = (uint8_t)strlen(tx_buffer);
    tx_index = 0U;
}
#endif

void CarApp_Init(void)
{
    DL_UART_Main_enableFIFOs(TUNER_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(
        TUNER_UART_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableFIFOs(GYRO_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(
        GYRO_UART_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    Motor_Init();
    Key_Init();
    LineSensor_Init();
    Encoder_Init();
    LineFollow_Init();
    Gyro_Init();
    Buzzer_Set(false);
    car_running = false;
    app_time_ms = 0U;
    rx_length = 0U;
    rx_last_byte_ms = 0U;
    rx_byte_count = 0U;
    tuner_rx_write = 0U;
    tuner_rx_read = 0U;
    tuner_rx_overflow_count = 0U;
    telemetry_pause_until_ms = 0U;
    tx_length = 0U;
    tx_index = 0U;
    angle_turn_active = false;
    angle_turn_line_mode = false;
    angle_turn_target_mdeg = 0;
    angle_turn_start_mdeg = 0;
    angle_turn_direction = 0;
    angle_turn_started_ms = 0U;
    angle_turn_settle_started_ms = 0U;
    line_lost_started_ms = 0U;
    line_reacquired_started_ms = 0U;
    corner_rearm_started_ms = 0U;
    corner_detection_armed = true;
    line_advance_detected_ms = 0U;
    corner_candidate_active = false;
    corner_candidate_confirmed = false;
    corner_candidate_direction = 0;
    last_corner_direction = 0;
    corner_candidate_started_ms = 0U;
    corner_pattern_started_ms = 0U;
    corner_center_started_ms = 0U;
    corner_white_started_ms = 0U;
    corner_observation_samples = 0U;
    corner_white_samples = 0U;
    corner_center_samples = 0U;
    corner_preview_active = false;
    corner_preview_m1_start = 0;
    corner_preview_m2_start = 0;
    corner_preview_heading_mdeg = 0;
    encoder_drive_active = false;
    encoder_drive_m1_start = 0;
    encoder_drive_m2_start = 0;
    encoder_drive_started_ms = 0U;
    encoder_drive_distance_mm = 0;
    encoder_drive_line_mode = false;
    encoder_drive_turn_direction = 0;
    encoder_drive_heading_mdeg = 0;
    encoder_drive_heading_correction = 0;
    gyro_last_frame_ms = 0U;
    gyro_last_frame_count = 0U;
    heading_locked = false;
    heading_target_mdeg = 0;
    gyro_boot_state = GYRO_BOOT_WAIT_DATA;
    gyro_still_reference_mdps = 0;
    gyro_still_started_ms = 0U;
    gyro_cal_started_ms = 0U;
    oled_last_update_ms = 0U;
    Stepper_Init();
#if CAR_CAMERA_ENABLE
    Camera_Init();
    camera_mode = false;
    camera_was_online = false;
#endif
#if CAR_TRACK_ENABLE
    track_mode = false;
    track_start_state = TRACK_START_IDLE;
    track_state_started_ms = 0U;
    track_last_adjust_ms = 0U;
    track_tilt_deg = 0;
    track_kp_divisor = CAR_TRACK_KP_DIVISOR;
    track_debug = false;
    track_ball_lost_ms = 0U;
#endif
    step_test_mode = false;
    step_test_last_ms = 0;
    step_test_direction = 0;
    step_raw_dump = false;
    OLED_Init();
    OledShowLine(0U, "GYRO POWER ON");
    OledShowLine(2U, "ZERO REQUEST");
    OledShowLine(4U, "ANGLE ABS");
    OledShowLine(6U, "WAIT DATA");
    Gyro_RequestZero();
    DL_UART_Main_enableInterrupt(TUNER_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(TUNER_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(TUNER_UART_INST_INT_IRQN);
}

void CarApp_RunCycle(void)
{
    app_time_ms += CAR_CONTROL_PERIOD_MS;
    Encoder_Service();
    Gyro_Service(app_time_ms);
    Stepper_Service();
#if CAR_CAMERA_ENABLE
    Camera_Service(app_time_ms);
#endif
#if CAR_TRACK_ENABLE
    if (track_mode) {
        if (!Camera_IsOnline(app_time_ms) &&
            track_start_state != TRACK_START_WAIT_CAMERA) {
            StopBallBalance("# BALANCE TIMEOUT; STOPPED");
        } else if (track_start_state == TRACK_START_WAIT_CAMERA) {
            if (Camera_IsOnline(app_time_ms)) {
                track_start_state = TRACK_START_WAIT_ORIGIN;
                track_state_started_ms = app_time_ms;
                UartQueue("# BALANCE CAM ONLINE; SET ORIGIN PENDING");
            } else if ((uint32_t)(app_time_ms - track_state_started_ms) >=
                5000U) {
                StopBallBalance("# BALANCE TIMEOUT; CAMERA NOT FOUND");
            }
        } else if (track_start_state == TRACK_START_WAIT_ORIGIN &&
            (uint32_t)(app_time_ms - track_state_started_ms) >=
                CAR_TRACK_START_DELAY_MS) {
            /* The rod must be manually level when K1 starts this sequence. */
            Stepper_SetOrigin();
            track_start_state = TRACK_START_WAIT_CONTROL;
            track_state_started_ms = app_time_ms;
            UartQueue("# BALANCE ORIGIN SET");
        } else if (track_start_state == TRACK_START_WAIT_CONTROL &&
            (uint32_t)(app_time_ms - track_state_started_ms) >=
                CAR_TRACK_ORIGIN_DELAY_MS) {
            track_start_state = TRACK_START_IDLE;
            track_last_adjust_ms = app_time_ms;
            Stepper_AbsoluteRotate(0);
            UartQueue("# BALANCE ACTIVE");
        } else if (track_start_state == TRACK_START_IDLE &&
            (uint32_t)(app_time_ms - track_last_adjust_ms) >=
                CAR_TRACK_UPDATE_MS) {
            int16_t x = Camera_GetBallX();
            int16_t target_deg = 0;

            if (Camera_HasBall()) {
                track_ball_lost_ms = 0U;

                if (x > CAR_TRACK_DEADBAND || x < -CAR_TRACK_DEADBAND) {
                    /* Right-side ball needs the free end raised. */
                    target_deg = (int16_t)(x / track_kp_divisor);
                    if (target_deg == 0)
                        target_deg = x > 0 ? 1 : -1;
                    target_deg = (int16_t)(target_deg * CAR_TRACK_MOTOR_SIGN);
                    if (target_deg > CAR_TRACK_MAX_TILT_DEG)
                        target_deg = CAR_TRACK_MAX_TILT_DEG;
                    else if (target_deg < -CAR_TRACK_MAX_TILT_DEG)
                        target_deg = -CAR_TRACK_MAX_TILT_DEG;
                }
                /* else: ball in deadband → target stays 0 (return to level) */
            } else if (track_tilt_deg != 0) {
                /* Ball lost — start counting, then return platform to level
                   so the ball can roll back into view. */
                if (track_ball_lost_ms == 0U) {
                    track_ball_lost_ms = app_time_ms;
                } else if ((uint32_t)(app_time_ms - track_ball_lost_ms)
                    >= 500U) {
                    target_deg = 0;   /* return to horizontal */
                    track_ball_lost_ms = 0U;
                    UartQueue("# BALL LOST; RETURNING TO LEVEL");
                }
            }

            if (target_deg != track_tilt_deg) {
                /* Use relative rotation to avoid multi-turn wrap-around
                   that occurs with AbsoluteRotate(0) after crossing a
                   full revolution. */
                int16_t delta = (int16_t)(target_deg - track_tilt_deg);
                Stepper_RelativeRotate((int32_t)delta);
                track_tilt_deg = target_deg;
            }
            track_last_adjust_ms = app_time_ms;
        }
        /* Debug output: every 250ms when enabled, regardless of ball state. */
        if (track_debug && (app_time_ms % 250U) == 0U) {
            snprintf(tx_buffer, sizeof(tx_buffer),
                "# DBG X=%d BALL=%u TILT=%d KP=%d ST=%u CAM=%u FR=%lu\r\n",
                Camera_GetBallX(),
                Camera_HasBall() ? 1U : 0U,
                track_tilt_deg,
                track_kp_divisor,
                track_start_state,
                Camera_IsOnline(app_time_ms) ? 1U : 0U,
                (unsigned long)Camera_GetValidFrameCount());
            tx_length = (uint8_t)strlen(tx_buffer);
            tx_index = 0U;
        }
    }
#endif
    /* Blind 1-second step test — no camera needed, just toggles ±10°. */
    if (step_test_mode &&
        (int32_t)(app_time_ms - (uint32_t)step_test_last_ms) >= 1000) {
        step_test_last_ms = (int32_t)app_time_ms;
        step_test_direction = (int8_t)(-step_test_direction);
        Stepper_RelativeRotate((int32_t)step_test_direction * 10);
    }
    if (Gyro_GetValidFrameCount() != gyro_last_frame_count) {
        gyro_last_frame_count = Gyro_GetValidFrameCount();
        gyro_last_frame_ms = app_time_ms;
    }
    UpdateGyroAutoCalibration();
    UartServiceRx();
    Key_Service(app_time_ms);
    if (Key_StartPressed() || Key_K4Pressed()) {
        if (encoder_drive_active) {
            car_running = false;
            encoder_drive_turn_direction = 0;
            StopEncoderDrive("# DRIVE ABORTED BY PB21");
        } else if (angle_turn_active) {
            car_running = false;
            StopAngleTurn("# TURN ABORTED BY PB21");
        } else {
            if (car_running) {
                car_running = false;
                heading_locked = false;
                LineFollow_SetHeadingCorrection(0);
                Motor_Stop();
            } else {
                car_running = true;
                LineFollow_Init();
                heading_locked = false;
            }
        }
    }

    if (Key_K1Pressed()) {
#if CAR_TRACK_ENABLE
        if (track_mode) {
            StopBallBalance("# BALANCE STOPPED BY K1");
        } else {
            StartBallBalance();
        }
#else
        UartQueue("# TRACK DISABLED IN CONFIG");
#endif
    }
    if (Key_K2Pressed()) {
        /* Manual direction calibration: absolute +5 degrees from the
         * horizontal origin established by K1/PB13. */
#if CAR_TRACK_ENABLE
        track_mode = false;
        track_start_state = TRACK_START_IDLE;
#endif
        Stepper_BlockingRelativeTest(CAR_TRACK_CAL_ANGLE_DEG);
        UartQueue("# K2 RELATIVE +1 DEG");
    }
    if (Key_K3Pressed()) {
        /* Manual direction calibration: absolute -5 degrees. */
#if CAR_TRACK_ENABLE
        track_mode = false;
        track_start_state = TRACK_START_IDLE;
#endif
        Stepper_BlockingRelativeTest(-CAR_TRACK_CAL_ANGLE_DEG);
        UartQueue("# K3 RELATIVE -1 DEG");
    }

    if (encoder_drive_active) {
        UpdateEncoderDrive();
    } else if (angle_turn_active) {
        UpdateAngleTurn();
    }
#if CAR_CAMERA_ENABLE
    else if (car_running && camera_mode &&
             !Camera_IsOnline(app_time_ms)) {
        car_running = false;
        camera_mode = false;
        Motor_Stop();
        UartQueue("# CAMERA TIMEOUT; STOPPED");
    } else if (car_running && camera_mode && Camera_HasBall()) {
        int16_t ball_x = Camera_GetBallX();
        int16_t correction = ball_x / CAR_CAMERA_KP_DIVISOR;
        int16_t base = CAR_CAMERA_BALL_TRACK_SPEED;
        int16_t left, right;

        if (correction > CAR_CAMERA_MAX_CORRECTION)
            correction = CAR_CAMERA_MAX_CORRECTION;
        else if (correction < -CAR_CAMERA_MAX_CORRECTION)
            correction = -CAR_CAMERA_MAX_CORRECTION;

        /* Positive ball X = ball is right → turn right. */
        left  = base + correction;
        right = base - correction;
        if (left  > CAR_MAX_SPEED) left  = CAR_MAX_SPEED;
        if (right > CAR_MAX_SPEED) right = CAR_MAX_SPEED;
        if (left  < CAR_MIN_SPEED) left  = CAR_MIN_SPEED;
        if (right < CAR_MIN_SPEED) right = CAR_MIN_SPEED;
        Motor_SetSignedSpeed(left, right);
    }
#endif
    else if (car_running) {
#if CAR_LINE_FOLLOW_ENABLE
#if CAR_LINE_GYRO_HEADING_ENABLE
        UpdateHeadingHold();
#else
        LineFollow_SetHeadingCorrection(0);
#endif
        LineFollow_Update();
        if (LineFollow_StopLineDetected()) {
            car_running = false;
            Motor_Stop();
            UartQueue("# FINISH LINE; STOPPED");
        }
#if CAR_LINE_CORNER_TURN_ENABLE
        if (!corner_detection_armed) {
            uint8_t mask = LineFollow_GetSensorMask();

            /* Do not interpret the same corner line twice. Rearm only after
             * the new line stays on the four center sensors without either
             * outer pair being active. */
            if ((mask & 0x3CU) != 0U && (mask & 0xC3U) == 0U) {
                if (corner_rearm_started_ms == 0U) {
                    corner_rearm_started_ms = app_time_ms;
                } else if ((uint32_t)(app_time_ms - corner_rearm_started_ms) >=
                    CAR_CORNER_REARM_CONFIRM_MS) {
                    corner_detection_armed = true;
                    corner_rearm_started_ms = 0U;
                }
            } else {
                corner_rearm_started_ms = 0U;
            }
        } else if (Gyro_HasData() &&
            (uint32_t)(app_time_ms - gyro_last_frame_ms) <=
                CAR_GYRO_STALE_TIMEOUT_MS) {
            DetectCornerCandidate();
        }
#endif
#else
        Motor_SetSignedSpeed(CAR_BASE_SPEED, CAR_BASE_SPEED);
#endif
    }

    if (tx_index >= tx_length) {
        if (step_raw_dump && Stepper_GetTraceCount() != 0U) {
            static const char hex[] = "0123456789ABCDEF";
            uint8_t i = 0U;
            tx_buffer[i++] = '#';
            tx_buffer[i++] = ' ';
            tx_buffer[i++] = 'R';
            tx_buffer[i++] = 'X';
            tx_buffer[i++] = ' ';
            while (i < sizeof(tx_buffer) - 5U &&
                   Stepper_GetTraceCount() != 0U) {
                uint8_t byte = Stepper_ReadTraceByte();
                tx_buffer[i++] = hex[byte >> 4U];
                tx_buffer[i++] = hex[byte & 0x0FU];
                tx_buffer[i++] = ' ';
            }
            tx_buffer[i++] = '\r';
            tx_buffer[i++] = '\n';
            tx_buffer[i] = '\0';
            tx_length = i;
            tx_index = 0U;
        } else if (step_raw_dump) {
            step_raw_dump = false;
        } else if (Stepper_HasResponse()) {
            /* Motor responses FIRST — critical feedback for the user. */
            uint8_t i = 0U;
            uint8_t byte;

            tx_buffer[0] = '#';
            tx_buffer[1] = ' ';
            i = 2U;
            while (i < sizeof(tx_buffer) - 2U &&
                   (byte = Stepper_ReadResponseByte()) != 0U) {
                tx_buffer[i++] = (char)byte;
                if (byte == '\n') break;
            }
            tx_buffer[i] = '\0';
            tx_length = i;
            tx_index  = 0U;
        } else if (Stepper_HasDebugHex()) {
            /* Debug hex second — useful but don't starve motor replies. */
            tx_buffer[0] = '#';
            tx_buffer[1] = ' ';
            Stepper_CopyDebugHex(&tx_buffer[2], sizeof(tx_buffer) - 3U);
            tx_length = (uint8_t)strlen(tx_buffer);
            tx_index  = 0U;
        }
    }
#if CAR_TELEMETRY_ENABLE
    /* Never let periodic telemetry starve DCC-101 response forwarding. */
    if (tx_index >= tx_length && (app_time_ms % 20U) == 0U &&
        (int32_t)(app_time_ms - telemetry_pause_until_ms) >= 0) {
        QueueTelemetry();
    }
#endif
    UpdateOledStatus();
    UartServiceTx();

    Delay_Ms(CAR_CONTROL_PERIOD_MS);
}
