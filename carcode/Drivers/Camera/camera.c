#include "camera.h"

/* ==================================================================
 *  Protocol constants
 * ================================================================== */
#define CAM_SYNC0      0xAAU
#define CAM_SYNC1      0x66U       /* vision packet identifier        */
#define CAM_FOOT0      0xDDU
#define CAM_FOOT1      0xEEU
#define CAM_FRAME_LEN  8U

#define CMD_START      0x01U
#define CMD_STOP       0x02U
#define CMD_CONFIG     0x03U

/* Communication timeout: stop motors if no valid frame within 100 ms. */
#define CAM_TIMEOUT_MS 100U

/* ==================================================================
 *  RX ring buffer (interrupt-driven)
 * ================================================================== */
#define RX_RING_SIZE   128U

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint8_t  rx_w;
static volatile uint8_t  rx_r;

/* ==================================================================
 *  Frame parser state
 * ================================================================== */
typedef enum {
    PARSE_SYNC0,        /* waiting for 0xAA                           */
    PARSE_SYNC1,        /* got AA, checking for 0x66                  */
    PARSE_PAYLOAD,      /* reading 4 payload bytes                    */
    PARSE_FOOT0,        /* checking for 0xDD                          */
    PARSE_FOOT1         /* checking for 0xEE                          */
} ParseState;

static ParseState parse_state = PARSE_SYNC0;
static uint8_t    payload[4];
static uint8_t    payload_idx;

/* ==================================================================
 *  Parsed data (updated atomically from service routine)
 * ================================================================== */
static volatile bool    ball_detected;
static volatile int16_t ball_x;
static volatile uint32_t last_valid_ms;
static volatile uint32_t valid_frame_count;

/* ==================================================================
 *  TX state (non-blocking, single-frame)
 * ================================================================== */
static uint8_t  tx_buf[CAM_FRAME_LEN];
static uint8_t  tx_len;
static uint8_t  tx_idx;

/* ==================================================================
 *  Internal helpers
 * ================================================================== */

static bool tx_idle(void)
{
    return tx_idx >= tx_len;
}

static void queue_cmd(uint8_t cmd, uint8_t p1, uint8_t p2)
{
    uint8_t chk;

    if (!tx_idle()) return;

    chk = (uint8_t)((cmd + p1 + p2) & 0xFFU);

    tx_buf[0] = CAM_SYNC0;
    tx_buf[1] = CAM_SYNC1;
    tx_buf[2] = cmd;
    tx_buf[3] = p1;
    tx_buf[4] = p2;
    tx_buf[5] = chk;
    tx_buf[6] = CAM_FOOT0;
    tx_buf[7] = CAM_FOOT1;
    tx_len    = CAM_FRAME_LEN;

    /* Tight-loop TX — no inter-byte gaps (motor protocol taught us this). */
    for (tx_idx = 0U; tx_idx < tx_len; ) {
        if (DL_UART_Main_transmitDataCheck(CAMERA_UART_INST,
                                           tx_buf[tx_idx])) {
            tx_idx++;
        }
    }
}

static void service_tx(void)
{
    while (tx_idx < tx_len &&
           DL_UART_Main_transmitDataCheck(CAMERA_UART_INST,
                                          tx_buf[tx_idx])) {
        tx_idx++;
    }
}

/* ==================================================================
 *  RX frame parser
 * ================================================================== */

static void parse_byte(uint8_t byte, uint32_t now_ms)
{
    switch (parse_state) {
    case PARSE_SYNC0:
        if (byte == CAM_SYNC0) parse_state = PARSE_SYNC1;
        break;

    case PARSE_SYNC1:
        if (byte == CAM_SYNC1) {
            parse_state  = PARSE_PAYLOAD;
            payload_idx  = 0U;
        } else {
            /* Not a vision frame — go back to hunting for AA. */
            parse_state = PARSE_SYNC0;
            if (byte == CAM_SYNC0) parse_state = PARSE_SYNC1;
        }
        break;

    case PARSE_PAYLOAD:
        payload[payload_idx++] = byte;
        if (payload_idx >= 4U) parse_state = PARSE_FOOT0;
        break;

    case PARSE_FOOT0:
        if (byte == CAM_FOOT0) {
            parse_state = PARSE_FOOT1;
        } else {
            parse_state = PARSE_SYNC0;
            if (byte == CAM_SYNC0) parse_state = PARSE_SYNC1;
        }
        break;

    case PARSE_FOOT1:
        parse_state = PARSE_SYNC0;
        if (byte == CAM_FOOT1) {
            /* Full frame received — validate checksum. */
            uint8_t chk = (uint8_t)((payload[0] + payload[1] +
                                     payload[2]) & 0xFFU);
            if (chk == payload[3]) {
                /* STA[0], POS_H[1], POS_L[2], CHK[3] */
                ball_detected = (payload[0] & 0x01U) != 0U;
                ball_x = (int16_t)(((uint16_t)payload[1] << 8U) |
                                    (uint16_t)payload[2]);
                last_valid_ms = now_ms;
                valid_frame_count++;
            }
            /* Checksum failure → silent drop. */
        }
        /* Footer failure or end of frame — restart hunting. */
        if (byte == CAM_SYNC0) parse_state = PARSE_SYNC1;
        break;
    }
}

static void service_rx(uint32_t now_ms)
{
    while (rx_r != rx_w) {
        uint8_t byte = rx_ring[rx_r];
        rx_r = (uint8_t)((rx_r + 1U) % RX_RING_SIZE);
        parse_byte(byte, now_ms);
    }
}

/* ==================================================================
 *  UART3 RX ISR
 * ================================================================== */

void UART3_IRQHandler(void)
{
    uint8_t byte;
    uint8_t nxt;

    (void)DL_UART_Main_getPendingInterrupt(CAMERA_UART_INST);
    while (DL_UART_Main_receiveDataCheck(CAMERA_UART_INST, &byte)) {
        nxt = (uint8_t)((rx_w + 1U) % RX_RING_SIZE);
        if (nxt == rx_r) {
            /* Overflow — oldest byte is silently lost. */
        } else {
            rx_ring[rx_w] = byte;
            rx_w = nxt;
        }
    }
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void Camera_Init(void)
{
    /* SYSCFG_DL_CAMERA_UART_init() already powers, configures, and enables
       UART3.  Here we only add what the generated init leaves out:
       FIFOs and the RX interrupt. */

    /* 1. Enable FIFOs (1-byte RX threshold for per-byte interrupt). */
    DL_UART_Main_enableFIFOs(CAMERA_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(CAMERA_UART_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);

    /* 2. Clear state */
    rx_w = 0U;
    rx_r = 0U;
    parse_state = PARSE_SYNC0;
    ball_detected = false;
    ball_x  = 0;
    last_valid_ms = 0U;
    valid_frame_count = 0U;
    tx_len = 0U;
    tx_idx = 0U;

    /* 3. Enable RX interrupt */
    DL_UART_Main_enableInterrupt(CAMERA_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(CAMERA_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CAMERA_UART_INST_INT_IRQN);
}

void Camera_Service(uint32_t now_ms)
{
    service_rx(now_ms);
    service_tx();
}

/* ---- Ball data ---- */

bool Camera_HasBall(void)
{
    return ball_detected;
}

int16_t Camera_GetBallX(void)
{
    return ball_x;
}

uint32_t Camera_GetValidFrameCount(void) { return valid_frame_count; }
uint32_t Camera_GetLastValidMs(void) { return last_valid_ms; }

/* ---- Communication watchdog ---- */

bool Camera_IsOnline(uint32_t now_ms)
{
    if (last_valid_ms == 0U) return false;
    return ((uint32_t)(now_ms - last_valid_ms)) < CAM_TIMEOUT_MS;
}

/* ---- Commands to camera ---- */

void Camera_Start(void)
{
    queue_cmd(CMD_START, 0U, 0U);
}

void Camera_Stop(void)
{
    queue_cmd(CMD_STOP, 0U, 0U);
}

void Camera_Config(uint8_t p1, uint8_t p2)
{
    queue_cmd(CMD_CONFIG, p1, p2);
}
