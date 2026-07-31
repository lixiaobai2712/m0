#include "line_sensor.h"
#include "ti_msp_dl_config.h"

#define ENCODER_M1_A_PIN DL_GPIO_PIN_25
#define ENCODER_M1_B_PIN DL_GPIO_PIN_14
#define ENCODER_M2_A_PIN DL_GPIO_PIN_26
#define ENCODER_M2_B_PIN DL_GPIO_PIN_27

static volatile int32_t encoder_motor1_count;
static int32_t encoder_motor2_count;
static uint16_t encoder_motor1_previous;
static uint16_t encoder_motor2_previous;

void LineSensor_Init(void)
{
    static const uint32_t pincm[LINE_SENSOR_COUNT] = {
        IOMUX_PINCM56, IOMUX_PINCM44, IOMUX_PINCM5, IOMUX_PINCM12,
        IOMUX_PINCM13, IOMUX_PINCM27, IOMUX_PINCM28, IOMUX_PINCM31
    };
    uint8_t i;

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        DL_GPIO_initDigitalInputFeatures(pincm[i],
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    }
}

void LineSensor_Read(uint8_t values[LINE_SENSOR_COUNT])
{
    uint32_t port_b = DL_GPIO_readPins(GPIOB,
        DL_GPIO_PIN_14 | DL_GPIO_PIN_11 | DL_GPIO_PIN_10 | DL_GPIO_PIN_1 |
        DL_GPIO_PIN_0 | DL_GPIO_PIN_18 | DL_GPIO_PIN_25);
    uint32_t port_a = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_30);

    /* Logical order is car-right to car-left. */
    values[0] = (port_b & DL_GPIO_PIN_25) != 0U;
    values[1] = (port_b & DL_GPIO_PIN_18) != 0U;
    values[2] = (port_a & DL_GPIO_PIN_30) != 0U;
    values[3] = (port_b & DL_GPIO_PIN_0) != 0U;
    values[4] = (port_b & DL_GPIO_PIN_1) != 0U;
    values[5] = (port_b & DL_GPIO_PIN_10) != 0U;
    values[6] = (port_b & DL_GPIO_PIN_11) != 0U;
    values[7] = (port_b & DL_GPIO_PIN_14) != 0U;
}

void Encoder_Init(void)
{
    static const DL_TimerG_ClockConfig motor1_clock = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    static const DL_TimerG_CompareConfig motor1_edge_counter = {
        .compareMode = DL_TIMER_COMPARE_MODE_EDGE_COUNT,
        .count = 65535U,
        .startTimer = DL_TIMER_START,
        .edgeDetectMode = DL_TIMER_COMPARE_EDGE_DETECTION_MODE_RISING,
        .inputChan = DL_TIMER_INPUT_CHAN_1,
        .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT
    };

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM55,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM36,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    encoder_motor1_count = 0;
    encoder_motor2_count = 0;
    DL_TimerG_reset(TIMG12);
    DL_TimerG_enablePower(TIMG12);
    delay_cycles(16U);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM55,
        IOMUX_PINCM55_PF_TIMG12_CCP1);
    DL_TimerG_setClockConfig(TIMG12,
        (DL_TimerG_ClockConfig *)&motor1_clock);
    DL_TimerG_initCompareMode(TIMG12, &motor1_edge_counter);
    DL_TimerG_enableClock(TIMG12);
    encoder_motor1_previous = (uint16_t)DL_TimerG_getTimerCount(TIMG12);

    DL_TimerG_setTimerCount(ENCODER_M2_INST, 32768U);
    DL_TimerG_startCounter(ENCODER_M2_INST);
    encoder_motor2_previous = 32768U;
}

void Encoder_Service(void)
{
    uint16_t motor1_current = (uint16_t)DL_TimerG_getTimerCount(TIMG12);
    uint16_t current = (uint16_t)DL_TimerG_getTimerCount(ENCODER_M2_INST);
    encoder_motor1_count += (uint16_t)(encoder_motor1_previous -
        motor1_current);
    encoder_motor1_previous = motor1_current;
    encoder_motor2_count += (int16_t)(current - encoder_motor2_previous);
    encoder_motor2_previous = current;
}

void Encoder_Zero(void)
{
    encoder_motor1_count = 0;
    encoder_motor2_count = 0;
    DL_TimerG_setTimerCount(TIMG12, 65535U);
    encoder_motor1_previous = 65535U;
    DL_TimerG_setTimerCount(ENCODER_M2_INST, 32768U);
    encoder_motor2_previous = 32768U;
}

int32_t Encoder_GetMotor1Count(void)
{
    return encoder_motor1_count;
}

int32_t Encoder_GetMotor2Count(void)
{
    return encoder_motor2_count;
}
