#include "stepper.h"
#include "delay.h"
#include <string.h>

/*
 * Stepper motor driver — UART-controlled closed-loop stepper.
 *
 * All pin / peripheral defines come from SysConfig (STEPPER_UART instance).
 * The SysConfig name "STEPPER_UART" produces macros like:
 *   STEPPER_UART_INST, GPIO_STEPPER_UART_RX_PIN, STEPPER_UART_IBRD_..., etc.
 *
 * Stepper_Init() performs a FULL manual UART2 configuration (reset, power,
 * pin mux, clock, mode, baud rate, enable) so that the stepper motor works
 * even if SysConfig has not been regenerated for this peripheral.
 */

/* ==================================================================
 *  Protocol constants
 * ================================================================== */
#define FRAME_HDR0          0xAAU
#define FRAME_HDR1          0x55U
#define DEVICE_ID           0x01U

#define CMD_REL_ROTATE      0x11U
#define CMD_SET_ORIGIN      0x12U
#define CMD_ABS_ROTATE      0x13U
#define CMD_SET_CURRENT     0x14U
#define CMD_SET_MODE        0x15U
#define CMD_SET_USER_DIR    0x16U
#define CMD_REBOOT          0x17U
#define CMD_SLEEP           0x19U

#define ENC_PULSES_PER_REV  16384L
#define STEP_MAX_DEG         45     /* mechanical safety: ±45° hard limit */
#define RX_RING_SIZE        128U
#define RESP_BUF_SIZE       256U
#define TRACE_BUF_SIZE      256U
#define FRAME_MAX_DATA      16U

/* ==================================================================
 *  Static state
 * ================================================================== */
static uint8_t  tx_buf[32];
static uint8_t  tx_len;
static uint8_t  tx_idx;

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint8_t  rx_w;
static volatile uint8_t  rx_r;
static volatile uint32_t rx_overflow;
static volatile uint32_t rx_byte_count;
static uint32_t tx_byte_count;

static uint8_t  resp[RESP_BUF_SIZE];
static volatile uint16_t resp_w;
static volatile uint16_t resp_r;
static volatile bool     resp_ready;
static volatile uint8_t  trace[TRACE_BUF_SIZE];
static volatile uint16_t trace_w;
static volatile uint16_t trace_r;

/* Debug: hex dump of last queued frame, forwarded over Bluetooth. */
static char    debug_hex[96];
static bool    debug_pending;

/* ==================================================================
 *  Internal helpers
 * ================================================================== */

static uint8_t checksum(const uint8_t *p, uint8_t n)
{
    uint16_t s = 0U;
    uint8_t  i;
    for (i = 0U; i < n; i++) s += p[i];
    return (uint8_t)(s & 0xFFU);
}

static bool tx_idle(void)
{
    return tx_idx >= tx_len;
}

static uint16_t deg_to_pulses(int32_t deg);

/* Build a frame and queue it for TX.
   Standard frame:  AA 55  ID  CMD  LEN  DATA[0..LEN-1]  CHKSUM
   Total length = 6 + len bytes. */
static void queue_frame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t pos;

    if (!tx_idle()) return;

    tx_buf[0] = FRAME_HDR0;     /* AA                         */
    tx_buf[1] = FRAME_HDR1;     /* 55                         */
    tx_buf[2] = DEVICE_ID;      /* ID = 0x01                  */
    tx_buf[3] = cmd;
    tx_buf[4] = len;
    for (i = 0U; i < len; i++) {
        tx_buf[5U + i] = data[i];
    }
    /* Checksum over ID + CMD + LEN + DATA */
    tx_buf[5U + len] = checksum(&tx_buf[2], (uint8_t)(3U + len));
    tx_len = (uint8_t)(6U + len);

    /*
     * Transmit EVERY byte in a tight polling loop.
     *
     * The motor's inter-byte timeout is only ~2 byte-times (@ 115200 ≈ 174 µs).
     * If we let the periodic Stepper_Service() push bytes one 1-ms tick at a
     * time, gaps of 300+ µs appear between FIFO bursts and the motor declares
     * "ERR: Frame incomplete".  Blocking here for ~1 ms on a 11-byte frame is
     * acceptable because STEP commands are rare and the main loop still runs
     * at 1 kHz — interrupts stay enabled.
     */
    for (tx_idx = 0U; tx_idx < tx_len; ) {
        if (DL_UART_Main_transmitDataCheck(STEPPER_UART_INST,
                                           tx_buf[tx_idx])) {
            tx_idx++;
            tx_byte_count++;
        }
    }

    /* Build hex dump for Bluetooth debugging. */
    {
        static const char hex[] = "0123456789ABCDEF";
        pos = 0U;
        for (i = 0U; i < tx_len && pos < sizeof(debug_hex) - 4U; i++) {
            debug_hex[pos++] = hex[tx_buf[i] >> 4U];
            debug_hex[pos++] = hex[tx_buf[i] & 0x0FU];
            debug_hex[pos++] = ' ';
        }
        debug_hex[pos] = '\0';
        debug_pending = true;
    }
}

/* ==================================================================
 *  TX / RX service routines
 * ================================================================== */

static void service_tx(void)
{
    while (tx_idx < tx_len &&
           DL_UART_Main_transmitDataCheck(STEPPER_UART_INST,
                                          tx_buf[tx_idx])) {
        tx_idx++;
    }
}

static void service_rx(void)
{
    while (rx_r != rx_w) {
        uint8_t byte = rx_ring[rx_r];
        rx_r = (uint8_t)((rx_r + 1U) % RX_RING_SIZE);

        {
            uint16_t nxt = (uint16_t)((resp_w + 1U) % RESP_BUF_SIZE);
            if (nxt != resp_r) {
                resp[resp_w] = byte;
                resp_w = nxt;
                if (byte == '\n') resp_ready = true;
            }
        }
    }
}

/* ---- UART2 RX ISR ---- */
void UART2_IRQHandler(void)
{
    uint8_t byte;
    uint8_t nxt;

    (void)DL_UART_Main_getPendingInterrupt(STEPPER_UART_INST);
    while (DL_UART_Main_receiveDataCheck(STEPPER_UART_INST, &byte)) {
        rx_byte_count++;
        {
            uint16_t trace_next = (uint16_t)((trace_w + 1U) % TRACE_BUF_SIZE);
            if (trace_next != trace_r) {
                trace[trace_w] = byte;
                trace_w = trace_next;
            }
        }
        nxt = (uint8_t)((rx_w + 1U) % RX_RING_SIZE);
        if (nxt == rx_r) {
            rx_overflow++;
        } else {
            rx_ring[rx_w] = byte;
            rx_w = nxt;
        }
    }
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void Stepper_Init(void)
{
    /*
     * Full manual UART2 init — the exact sequence proven to work.
     *
     * RESET must come first: SysConfig already powered up UART2, so without a
     * preceding reset the subsequent power / clock / init calls may silently
     * keep the old register state.  The 16-cycle delay lets the analogue
     * supply settle after power-on.
     */
    DL_UART_Main_reset(STEPPER_UART_INST);
    DL_UART_Main_enablePower(STEPPER_UART_INST);
    delay_cycles(16U);

    DL_GPIO_initPeripheralOutputFunction(GPIO_STEPPER_UART_IOMUX_TX,
        GPIO_STEPPER_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_STEPPER_UART_IOMUX_RX,
        GPIO_STEPPER_UART_IOMUX_RX_FUNC);

    {
        const DL_UART_Main_ClockConfig clk_cfg = {
            .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
            .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
        };
        DL_UART_Main_setClockConfig(STEPPER_UART_INST,
            (DL_UART_Main_ClockConfig *)&clk_cfg);
    }
    {
        const DL_UART_Main_Config cfg = {
            .mode        = DL_UART_MAIN_MODE_NORMAL,
            .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
            .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
            .parity      = DL_UART_MAIN_PARITY_NONE,
            .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
            .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
        };
        DL_UART_Main_init(STEPPER_UART_INST,
            (DL_UART_Main_Config *)&cfg);
    }
    DL_UART_Main_setOversampling(STEPPER_UART_INST,
        DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(STEPPER_UART_INST,
        STEPPER_UART_IBRD_32_MHZ_115200_BAUD,
        STEPPER_UART_FBRD_32_MHZ_115200_BAUD);
    DL_UART_Main_enable(STEPPER_UART_INST);

    /* Clear software state */
    tx_len = 0U;
    tx_idx = 0U;
    rx_w   = 0U;
    rx_r   = 0U;
    rx_overflow = 0U;
    rx_byte_count = 0U;
    tx_byte_count = 0U;
    resp_w = 0U;
    resp_r = 0U;
    resp_ready = false;
    trace_w = 0U;
    trace_r = 0U;

    /* Enable RX interrupt (1-byte FIFO threshold) */
    DL_UART_Main_enableFIFOs(STEPPER_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(STEPPER_UART_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(STEPPER_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(STEPPER_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_UART_INST_INT_IRQN);

    /*
     * Motor startup sequence (order matters):
     *   1. AA AA AA — sync UART protocol parser
     *   2. WAKE     — power up driver stage (needs ~300 ms)
     *   3. ORIGIN   — lock current position as zero reference
     *   4. CURRENT  — enable drive current (50 %)
     *
     * CURRENT must come AFTER WAKE because a sleeping motor ignores
     * every command except WAKE.
     */
    tx_buf[0] = 0xAAU;
    tx_buf[1] = 0xAAU;
    tx_buf[2] = 0xAAU;
    tx_len    = 3U;
    tx_idx    = 0U;
    while (!tx_idle()) {
        service_tx();
    }
    Delay_Ms(20U);

    Stepper_BlockingWake();           /* CMD_SLEEP data=0x01 */
    Delay_Ms(300U);                   /* driver-stage power-up time */
    Stepper_BlockingSetOrigin();      /* CMD_SET_ORIGIN — lock zero   */
    Delay_Ms(10U);

    /* Enable drive current now that the driver stage is awake.
       CURRENT must be set AFTER WAKE — a sleeping motor ignores it. */
    Stepper_SetCurrent(50U);
    Delay_Ms(10U);
}

void Stepper_Service(void)
{
    service_rx();
    service_tx();
}

void Stepper_Sync(void)
{
    uint8_t sync[3] = {0xAAU, 0xAAU, 0xAAU};
    uint8_t i;

    while (!tx_idle()) service_tx();
    for (i = 0U; i < sizeof(sync); i++) {
        while (!DL_UART_Main_transmitDataCheck(STEPPER_UART_INST, sync[i])) {
        }
        tx_byte_count++;
    }
}

static void bitbang_byte(uint8_t data)
{
    uint8_t bit;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    DL_GPIO_clearPins(GPIO_STEPPER_UART_TX_PORT, GPIO_STEPPER_UART_TX_PIN);
    delay_cycles(270U);
    for (bit = 0U; bit < 8U; bit++) {
        if ((data & (1U << bit)) != 0U) {
            DL_GPIO_setPins(GPIO_STEPPER_UART_TX_PORT,
                GPIO_STEPPER_UART_TX_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_STEPPER_UART_TX_PORT,
                GPIO_STEPPER_UART_TX_PIN);
        }
        delay_cycles(270U);
    }
    DL_GPIO_setPins(GPIO_STEPPER_UART_TX_PORT, GPIO_STEPPER_UART_TX_PIN);
    delay_cycles(270U);
    if (primask == 0U) __enable_irq();
}

void Stepper_BitBangAbsoluteTest(int32_t degrees)
{
    uint8_t frame[9];
    uint8_t i;
    uint16_t pulses = deg_to_pulses(degrees);

    DL_UART_Main_disable(STEPPER_UART_INST);
    DL_GPIO_initDigitalOutputFeatures(GPIO_STEPPER_UART_IOMUX_TX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_setPins(GPIO_STEPPER_UART_TX_PORT, GPIO_STEPPER_UART_TX_PIN);
    Delay_Ms(2U);
    bitbang_byte(0xAAU);
    bitbang_byte(0xAAU);
    bitbang_byte(0xAAU);
    Delay_Ms(20U);

    frame[0] = 0xAAU; frame[1] = 0x55U; frame[2] = DEVICE_ID;
    frame[3] = CMD_ABS_ROTATE; frame[4] = 3U;
    frame[5] = degrees >= 0 ? 1U : 0U;
    frame[6] = (uint8_t)pulses; frame[7] = (uint8_t)(pulses >> 8U);
    frame[8] = checksum(&frame[2], 6U);
    for (i = 0U; i < sizeof(frame); i++) bitbang_byte(frame[i]);

    DL_GPIO_initPeripheralOutputFunction(GPIO_STEPPER_UART_IOMUX_TX,
        GPIO_STEPPER_UART_IOMUX_TX_FUNC);
    DL_UART_Main_enable(STEPPER_UART_INST);
}

void Stepper_BlockingAbsoluteTest(int32_t degrees)
{
    uint8_t frame[9];
    uint8_t i;
    uint16_t pulses = deg_to_pulses(degrees);

    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, 0xAAU);
    Delay_Ms(20U);

    frame[0] = 0xAAU; frame[1] = 0x55U; frame[2] = DEVICE_ID;
    frame[3] = CMD_ABS_ROTATE; frame[4] = 3U;
    frame[5] = degrees >= 0 ? 1U : 0U;
    frame[6] = (uint8_t)pulses; frame[7] = (uint8_t)(pulses >> 8U);
    frame[8] = checksum(&frame[2], 6U);
    for (i = 0U; i < sizeof(frame); i++) {
        DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, frame[i]);
        tx_byte_count++;
    }
}

static void blocking_frame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t sum = (uint8_t)(DEVICE_ID + cmd + len);

    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, FRAME_HDR0);
    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, FRAME_HDR1);
    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, DEVICE_ID);
    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, cmd);
    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, len);
    for (i = 0U; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, data[i]);
        sum = (uint8_t)(sum + data[i]);
    }
    DL_UART_Main_transmitDataBlocking(STEPPER_UART_INST, sum);
    tx_byte_count += (uint32_t)len + 6U;
}

void Stepper_BlockingRelativeTest(int32_t degrees)
{
    uint16_t pulses;

    /* Mechanical safety: hard-clamp to ±STEP_MAX_DEG degrees. */
    if (degrees > STEP_MAX_DEG)  degrees = STEP_MAX_DEG;
    if (degrees < -STEP_MAX_DEG) degrees = -STEP_MAX_DEG;

    pulses = deg_to_pulses(degrees);
    uint8_t data[5] = {
        degrees > 0 ? 1U : 0U, 0U, 0U,
        (uint8_t)pulses, (uint8_t)(pulses >> 8U)
    };
    if (degrees != 0) blocking_frame(CMD_REL_ROTATE, data, sizeof(data));
}

void Stepper_BlockingSetOrigin(void)
{
    blocking_frame(CMD_SET_ORIGIN, NULL, 0U);
}

void Stepper_BlockingWake(void)
{
    const uint8_t enable = 1U;
    blocking_frame(CMD_SLEEP, &enable, 1U);
}

void Stepper_BlockingBeginCalibration(void)
{
    /* Motor already received AA AA AA sync in Stepper_Init().
       Only need wake + settle + origin. */
    while (DL_UART_Main_isBusy(STEPPER_UART_INST)) {
    }
    Stepper_BlockingWake();
    Delay_Ms(300U);
    Stepper_BlockingSetOrigin();
}

/* ==================================================================
 *  High-level commands
 * ================================================================== */

static uint16_t deg_to_pulses(int32_t deg)
{
    int32_t p = deg < 0 ? -deg : deg;
    p = (int32_t)(((int64_t)p * ENC_PULSES_PER_REV + 180L) / 360L);
    if (p > 65535L) p = 65535L;
    return (uint16_t)p;
}

void Stepper_Wake(void)
{
    const uint8_t d = 0x01U;
    queue_frame(CMD_SLEEP, &d, 1U);
}

void Stepper_Sleep(void)
{
    const uint8_t d = 0x00U;
    queue_frame(CMD_SLEEP, &d, 1U);
}

void Stepper_SetOrigin(void)
{
    queue_frame(CMD_SET_ORIGIN, NULL, 0U);
}

void Stepper_RelativeRotate(int32_t degrees)
{
    uint8_t  data[5];
    uint16_t pulses;

    if (degrees == 0) return;

    /* Mechanical safety: hard-clamp to ±STEP_MAX_DEG degrees. */
    if (degrees > STEP_MAX_DEG)  degrees = STEP_MAX_DEG;
    if (degrees < -STEP_MAX_DEG) degrees = -STEP_MAX_DEG;

    data[0] = (uint8_t)(degrees > 0 ? 0x01U : 0x00U);  /* dir */
    data[1] = 0x00U;
    data[2] = 0x00U;
    pulses  = deg_to_pulses(degrees);
    data[3] = (uint8_t)(pulses & 0xFFU);
    data[4] = (uint8_t)((pulses >> 8U) & 0xFFU);

    queue_frame(CMD_REL_ROTATE, data, 5U);
}

void Stepper_AbsoluteRotate(int32_t degrees)
{
    uint8_t  data[3];
    uint16_t pulses;

    /* Mechanical safety: hard-clamp to ±STEP_MAX_DEG degrees. */
    if (degrees > STEP_MAX_DEG)  degrees = STEP_MAX_DEG;
    if (degrees < -STEP_MAX_DEG) degrees = -STEP_MAX_DEG;

    data[0] = (uint8_t)(degrees >= 0 ? 0x01U : 0x00U);
    pulses  = deg_to_pulses(degrees);
    data[1] = (uint8_t)(pulses & 0xFFU);
    data[2] = (uint8_t)((pulses >> 8U) & 0xFFU);

    queue_frame(CMD_ABS_ROTATE, data, 3U);
}

void Stepper_SetCurrent(uint8_t current)
{
    queue_frame(CMD_SET_CURRENT, &current, 1U);
}

void Stepper_SetMode(uint8_t mode)
{
    queue_frame(CMD_SET_MODE, &mode, 1U);
}

void Stepper_SetUserDirection(uint8_t dir)
{
    queue_frame(CMD_SET_USER_DIR, &dir, 1U);
}

void Stepper_Reboot(void)
{
    queue_frame(CMD_REBOOT, NULL, 0U);
}

void Stepper_SendRaw(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    queue_frame(cmd, data, len);
}

void Stepper_SendRawBytes(const uint8_t *bytes, uint8_t len)
{
    uint8_t i;
    static const char hex[] = "0123456789ABCDEF";
    uint8_t pos;

    if (!tx_idle()) return;
    if (len > sizeof(tx_buf)) len = sizeof(tx_buf);

    for (i = 0U; i < len; i++) {
        tx_buf[i] = bytes[i];
    }
    tx_len = len;

    /* Tight-loop TX — same reason as queue_frame(). */
    for (tx_idx = 0U; tx_idx < tx_len; ) {
        if (DL_UART_Main_transmitDataCheck(STEPPER_UART_INST,
                                           tx_buf[tx_idx])) {
            tx_idx++;
        }
    }

    /* Build hex dump for debugging. */
    pos = 0U;
    for (i = 0U; i < tx_len && pos < sizeof(debug_hex) - 4U; i++) {
        debug_hex[pos++] = hex[tx_buf[i] >> 4U];
        debug_hex[pos++] = hex[tx_buf[i] & 0x0FU];
        debug_hex[pos++] = ' ';
    }
    debug_hex[pos] = '\0';
    debug_pending = true;
}

/* ==================================================================
 *  Debug hex dump
 * ================================================================== */

bool Stepper_HasDebugHex(void)
{
    return debug_pending;
}

void Stepper_CopyDebugHex(char *dst, uint8_t max_len)
{
    uint8_t i = 0U;
    while (i < max_len - 1U && debug_hex[i] != '\0') {
        dst[i] = debug_hex[i];
        i++;
    }
    dst[i] = '\0';
    debug_pending = false;
}

/* ==================================================================
 *  Response forwarding
 * ================================================================== */

bool Stepper_HasResponse(void)
{
    return resp_ready;
}

uint8_t Stepper_ReadResponseByte(void)
{
    uint8_t byte;
    if (resp_r == resp_w) {
        resp_ready = false;
        return 0U;
    }
    byte = resp[resp_r];
    resp_r = (uint16_t)((resp_r + 1U) % RESP_BUF_SIZE);
    /* Expose one complete line at a time to the Bluetooth forwarder. */
    if (byte == '\n') {
        resp_ready = false;
        if (resp_r != resp_w) {
            uint16_t scan = resp_r;
            while (scan != resp_w) {
                if (resp[scan] == '\n') {
                    resp_ready = true;
                    break;
                }
                scan = (uint16_t)((scan + 1U) % RESP_BUF_SIZE);
            }
        }
    } else if (resp_r == resp_w) {
        resp_ready = false;
    }
    return byte;
}

uint32_t Stepper_GetTxByteCount(void) { return tx_byte_count; }
uint32_t Stepper_GetRxByteCount(void) { return rx_byte_count; }
uint32_t Stepper_GetRxOverflowCount(void) { return rx_overflow; }

uint16_t Stepper_GetBufferedResponseCount(void)
{
    if (resp_w >= resp_r) return (uint16_t)(resp_w - resp_r);
    return (uint16_t)(RESP_BUF_SIZE - resp_r + resp_w);
}

uint8_t Stepper_ReadBufferedResponseByte(void)
{
    uint8_t byte;
    if (resp_r == resp_w) return 0U;
    byte = resp[resp_r];
    resp_r = (uint16_t)((resp_r + 1U) % RESP_BUF_SIZE);
    if (resp_r == resp_w) resp_ready = false;
    return byte;
}

uint16_t Stepper_GetTraceCount(void)
{
    uint16_t write = trace_w;
    uint16_t read = trace_r;
    if (write >= read) return (uint16_t)(write - read);
    return (uint16_t)(TRACE_BUF_SIZE - read + write);
}

uint8_t Stepper_ReadTraceByte(void)
{
    uint8_t byte;
    if (trace_r == trace_w) return 0U;
    byte = trace[trace_r];
    trace_r = (uint16_t)((trace_r + 1U) % TRACE_BUF_SIZE);
    return byte;
}
