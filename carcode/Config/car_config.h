#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#define CAR_CONTROL_PERIOD_MS  1U

/* Enable the eight-channel line sensor controller. */
#define CAR_LINE_FOLLOW_ENABLE 1U

/* Camera / vision module (UART3, PB2/PB3).  When enabled the car tracks
   the ball X position instead of the line sensor while CAMERA mode is on. */
#define CAR_CAMERA_ENABLE           1U
#define CAR_CAMERA_TIMEOUT_MS       100U
#define CAR_CAMERA_BALL_TRACK_SPEED  25
#define CAR_CAMERA_KP_DIVISOR       8L
#define CAR_CAMERA_MAX_CORRECTION   20

/* Stepper-based ball balancing. The motor's internal encoder closes the
   angle loop; the MCU maps the predicted ball position to an absolute tilt. */
#define CAR_TRACK_ENABLE            1U
#define CAR_TRACK_DEADBAND           8
#define CAR_TRACK_KP_DIVISOR        64L
#define CAR_TRACK_MAX_TILT_DEG       5
#define CAR_TRACK_UPDATE_MS         80U
#define CAR_TRACK_ORIGIN_DELAY_MS   120U
#define CAR_TRACK_START_DELAY_MS    120U
/* Set to -1 if a positive absolute angle lowers the rod's free end. */
#define CAR_TRACK_MOTOR_SIGN        -1
#define CAR_TRACK_MAX_SAFE_ANGLE_DEG 10
#define CAR_TRACK_CAL_ANGLE_DEG       1

/* Oval course: steering comes only from the infrared line sensors. */
#define CAR_LINE_CORNER_TURN_ENABLE 0U
#define CAR_LINE_GYRO_HEADING_ENABLE 0U

/* Periodic PID telemetry; keep commands and diagnostic replies enabled. */
#define CAR_TELEMETRY_ENABLE   0U

#define CAR_BASE_SPEED         28
#define CAR_MIN_SPEED          10
#define CAR_MAX_SPEED          50

/* PID: output = (Kp*error + Ki*sum(error) + Kd*delta(error)) / 128. */
#define CAR_PID_KP             50
#define CAR_PID_KI             0
#define CAR_PID_KD             8
#define CAR_PID_SCALE_SHIFT    7

#define CAR_TURN_FACTOR        90
#define CAR_PID_DEADBAND       6
#define CAR_CENTER_EXIT_ERROR  8
#define CAR_CORRECTION_STEP    1
#define CAR_MAX_CORRECTION     32
#define CAR_POSITION_FILTER    4
#define CAR_CURVE_THRESHOLD    18
#define CAR_CURVE_SPEED        CAR_BASE_SPEED
#define CAR_BASE_SPEED_STEP    1
#define CAR_SEARCH_SPEED       24
#define CAR_TURN_LATCH_ERROR   20
#define CAR_STOP_LINE_CONFIRM_MS 20U

/* Independent gyro angle-turn test mode. */
#define CAR_ANGLE_TURN_MIN_SPEED       12
#define CAR_ANGLE_TURN_MAX_SPEED       22
#define CAR_LINE_ANGLE_TURN_MIN_SPEED  18
#define CAR_LINE_ANGLE_TURN_MAX_SPEED  31
#define CAR_LINE_ANGLE_FINISH_SPEED     9
#define CAR_LINE_ANGLE_SLOWDOWN_DEG     20
#define CAR_ANGLE_TURN_TOLERANCE_MDEG  5000
#define CAR_ANGLE_TURN_RATE_MDPS       12000
#define CAR_ANGLE_TURN_SETTLE_MS       60U
#define CAR_ANGLE_TURN_TIMEOUT_MS      5000U
#define CAR_GYRO_STALE_TIMEOUT_MS      150U
#define CAR_GYRO_READY_RATE_MDPS       3000
#define CAR_GYRO_STATIC_DELTA_MDPS     500
#define CAR_GYRO_STATIC_CONFIRM_MS     1000U
#define CAR_GYRO_AUTO_CAL_WAIT_MS      3000U

/* Gyro assistance for line following. Positive gyro angle is a left turn. */
#define CAR_LINE_TURN_ANGLE_DEG         90
#define CAR_LINE_TURN_MIN_PROGRESS_DEG  90
#define CAR_LINE_LOST_CONFIRM_MS        20U
#define CAR_LINE_REACQUIRE_CONFIRM_MS   5U
#define CAR_CORNER_REARM_CONFIRM_MS      300U
#define CAR_FALSE_TURN_REJECT_MS        80U
#define CAR_CORNER_CANDIDATE_CONFIRM_MS 3U
#define CAR_CORNER_CANDIDATE_TIMEOUT_MS 80U
#define CAR_CORNER_WHITE_WINDOW_MS       100U
#define CAR_CORNER_WHITE_PERCENT         15U
#define CAR_ENCODER_M1_COUNTS_PER_REV   364L
#define CAR_ENCODER_M2_COUNTS_PER_REV   1430L
#define CAR_ENCODER_ADVANCE_MM          270L
#define CAR_ENCODER_TEST_SPEED          25
#define CAR_ENCODER_SLOW_SPEED          CAR_ENCODER_TEST_SPEED
#define CAR_CORNER_ADVANCE_SPEED        CAR_BASE_SPEED
#define CAR_CORNER_ADVANCE_SLOW_SPEED   CAR_CORNER_ADVANCE_SPEED
#define CAR_ENCODER_SLOWDOWN_PERCENT    75L
#define CAR_ENCODER_STOP_COMP_PERCENT   89L
#define CAR_ADVANCE_HEADING_DIVISOR     3000L
#define CAR_ADVANCE_RATE_DIVISOR        30000L
#define CAR_ADVANCE_HEADING_MAX         4
#define CAR_ADVANCE_HEADING_STEP        1
#define CAR_ENCODER_DRIVE_TIMEOUT_MS    12000U
#define CAR_CORNER_ADVANCE_TIMEOUT_MS   4000U
#define CAR_HEADING_DEADBAND_MDEG       500
#define CAR_HEADING_KP_DIVISOR          2500L
#define CAR_HEADING_KD_DIVISOR          30000L
#define CAR_HEADING_MAX_CORRECTION      5

/* 1 ms samples with 2-of-3 filtering. */
#define CAR_SENSOR_FILTER_MASK 0x07U

/* Infrared outputs are low over the black track and high over white. */
#define CAR_LINE_ACTIVE_LEVEL  0U

#endif
