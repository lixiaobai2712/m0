#include "oled.h"
#include "delay.h"
#include "oledfont.h"
#include "ti_msp_dl_config.h"

#define OLED_I2C_ADDRESS_7BIT 0x3CU
#define OLED_I2C_TIMEOUT      320000U
#define OLED_CONTROL_CMD      0x00U
#define OLED_CONTROL_DATA     0x40U
#define OLED_I2C_MAX_PACKET   8U

static bool OLED_I2CWaitIdle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
        DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout-- == 0U) return false;
    }

    timeout = OLED_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
        DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (timeout-- == 0U) return false;
    }

    timeout = OLED_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
        DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout-- == 0U) return false;
    }

    return (DL_I2C_getControllerStatus(OLED_I2C_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
}

static bool OLED_WritePacket(uint8_t control, const uint8_t *data, uint8_t length)
{
    uint8_t packet[OLED_I2C_MAX_PACKET];
    DL_I2C_ClockConfig clock_config;
    uint32_t i2c_clock_hz = CPUCLK_FREQ;
    uint32_t delay_cycles_after_start;
    uint8_t i;

    packet[0] = control;
    for (i = 0U; i < length; i++) {
        packet[i + 1U] = data[i];
    }

    if (!OLED_I2CWaitIdle()) return false;
    DL_I2C_flushControllerTXFIFO(OLED_I2C_INST);
    DL_I2C_flushControllerRXFIFO(OLED_I2C_INST);
    DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, packet, (uint16_t)length + 1U);
    DL_I2C_startControllerTransfer(OLED_I2C_INST, OLED_I2C_ADDRESS_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint16_t)length + 1U);

    DL_I2C_getClockConfig(OLED_I2C_INST, &clock_config);
    if (clock_config.clockSel == DL_I2C_CLOCK_MFCLK) {
        i2c_clock_hz = 4000000U;
    }
    delay_cycles_after_start = (3U * ((uint32_t)clock_config.divideRatio + 1U)) *
        (CPUCLK_FREQ / i2c_clock_hz);
    delay_cycles(delay_cycles_after_start);

    return OLED_I2CWaitIdle();
}

static bool OLED_WriteBytes(uint8_t control, const uint8_t *data, uint16_t length)
{
    while (length > 0U) {
        uint8_t chunk = length > (OLED_I2C_MAX_PACKET - 1U) ?
            (OLED_I2C_MAX_PACKET - 1U) : (uint8_t)length;

        if (!OLED_WritePacket(control, data, chunk)) return false;
        data += chunk;
        length -= chunk;
    }

    return true;
}

static void OLED_WriteCmd(uint8_t cmd)
{
    (void)OLED_WritePacket(OLED_CONTROL_CMD, &cmd, 1U);
}

static void OLED_WriteData(uint8_t data)
{
    (void)OLED_WritePacket(OLED_CONTROL_DATA, &data, 1U);
}

void OLED_SetPos(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH || y >= OLED_PAGE_COUNT) return;
    OLED_WriteCmd(0xB0U + y);
    OLED_WriteCmd(((x & 0xF0U) >> 4U) | 0x10U);
    OLED_WriteCmd((x & 0x0FU) | 0x01U);
}

void OLED_Clear(void)
{
    for (uint8_t i = 0U; i < OLED_PAGE_COUNT; i++) {
        OLED_SetPos(0U, i);
        for (uint8_t n = 0U; n < OLED_WIDTH; n++) {
            OLED_WriteData(0U);
        }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr)
{
    uint8_t c;

    if (x > (OLED_WIDTH - 8U) || y > (OLED_PAGE_COUNT - 2U)) return;
    if (chr < ' ' || chr > '~') chr = ' ';
    c = (uint8_t)(chr - ' ');

    OLED_SetPos(x, y);
    (void)OLED_WriteBytes(OLED_CONTROL_DATA, &OLED_F8x16[(uint16_t)c * 16U], 8U);

    OLED_SetPos(x, y + 1U);
    (void)OLED_WriteBytes(OLED_CONTROL_DATA, &OLED_F8x16[(uint16_t)c * 16U + 8U], 8U);
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *chr)
{
    uint8_t row_top[OLED_WIDTH];
    uint8_t row_bottom[OLED_WIDTH];
    uint8_t column;

    while (*chr != '\0' && y <= (OLED_PAGE_COUNT - 2U)) {
        uint8_t count = 0U;

        column = x;
        while (*chr != '\0' && column <= (OLED_WIDTH - 8U)) {
            uint8_t ch = (uint8_t)*chr++;
            uint8_t c;

            if (ch < ' ' || ch > '~') ch = ' ';
            c = (uint8_t)(ch - ' ');
            for (uint8_t i = 0U; i < 8U; i++) {
                row_top[(uint16_t)count * 8U + i] =
                    OLED_F8x16[(uint16_t)c * 16U + i];
                row_bottom[(uint16_t)count * 8U + i] =
                    OLED_F8x16[(uint16_t)c * 16U + i + 8U];
            }
            count++;
            column += 8U;
        }

        if (count != 0U) {
            OLED_SetPos(x, y);
            (void)OLED_WriteBytes(OLED_CONTROL_DATA, row_top,
                (uint16_t)count * 8U);
            OLED_SetPos(x, y + 1U);
            (void)OLED_WriteBytes(OLED_CONTROL_DATA, row_bottom,
                (uint16_t)count * 8U);
        }

        x = 0U;
        y += 2U;
    }
}

void OLED_Init(void)
{
    const uint8_t init_cmds[] = {
        0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0xFF,
        0xA1, 0xA6, 0xA8, 0x3F, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1,
        0xDA, 0x12, 0xDB, 0x30, 0x8D, 0x14, 0xAF
    };

    Delay_Ms(100U);
    for (uint8_t i = 0U; i < sizeof(init_cmds); i++) {
        OLED_WriteCmd(init_cmds[i]);
    }
    OLED_Clear();
}
