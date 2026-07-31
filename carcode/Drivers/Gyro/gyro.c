#include "gyro.h"
#include "ti_msp_dl_config.h"
#include <string.h>

#define GYRO_DATA_FRAME_SIZE 16U
#define GYRO_COMMAND_SIZE     8U
#define GYRO_READ_PERIOD_MS  20U
#define GYRO_RX_RING_SIZE    64U

static uint8_t rx_frame[GYRO_DATA_FRAME_SIZE];
static uint8_t rx_length;
static uint8_t tx_frame[GYRO_COMMAND_SIZE];
static uint8_t tx_index;
static uint8_t command_sequence;
static uint32_t last_read_ms;
static float angle_deg;
static float rate_dps;
static uint16_t frame_sequence;
static uint32_t valid_frame_count;
static uint32_t crc_error_count;
static bool has_data;
static uint8_t pending_command;
static uint8_t pending_parameter;
static volatile uint8_t rx_ring[GYRO_RX_RING_SIZE];
static volatile uint8_t rx_ring_write;
static volatile uint8_t rx_ring_read;
static volatile uint32_t rx_ring_overflow_count;

static uint16_t Crc16Modbus(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++) {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static bool TransmitterIdle(void)
{
    return tx_index >= GYRO_COMMAND_SIZE;
}

static void QueueCommand(uint8_t command, uint8_t parameter)
{
    uint16_t crc;
    if (!TransmitterIdle()) return;

    tx_frame[0] = 0xA5U;
    tx_frame[1] = 0x5AU;
    tx_frame[2] = command;
    tx_frame[3] = parameter;
    tx_frame[4] = command_sequence++;
    crc = Crc16Modbus(&tx_frame[2], 3U);
    tx_frame[5] = (uint8_t)crc;
    tx_frame[6] = (uint8_t)(crc >> 8U);
    tx_frame[7] = 0x5AU;
    tx_index = 0U;
}

static void ServiceTx(void)
{
    while (tx_index < GYRO_COMMAND_SIZE &&
        DL_UART_Main_transmitDataCheck(GYRO_UART_INST, tx_frame[tx_index])) {
        tx_index++;
    }
}

static void AcceptDataFrame(void)
{
    uint16_t received_crc = (uint16_t)rx_frame[12] |
        ((uint16_t)rx_frame[13] << 8U);

    if (rx_frame[14] != 0x55U || rx_frame[15] != 0xAAU ||
        Crc16Modbus(&rx_frame[2], 10U) != received_crc) {
        crc_error_count++;
        return;
    }

    frame_sequence = (uint16_t)rx_frame[2] | ((uint16_t)rx_frame[3] << 8U);
    memcpy(&angle_deg, &rx_frame[4], sizeof(angle_deg));
    memcpy(&rate_dps, &rx_frame[8], sizeof(rate_dps));
    valid_frame_count++;
    has_data = true;
}

static void ConsumeByte(uint8_t byte)
{
    if (rx_length == 0U) {
        if (byte == 0xAAU) rx_frame[rx_length++] = byte;
        return;
    }
    if (rx_length == 1U) {
        if (byte == 0x55U) {
            rx_frame[rx_length++] = byte;
        } else {
            rx_length = (byte == 0xAAU) ? 1U : 0U;
        }
        return;
    }

    rx_frame[rx_length++] = byte;
    if (rx_length == GYRO_DATA_FRAME_SIZE) {
        AcceptDataFrame();
        rx_length = 0U;
    }
}

static void ServiceRx(void)
{
    while (rx_ring_read != rx_ring_write) {
        uint8_t byte = rx_ring[rx_ring_read];
        rx_ring_read = (uint8_t)((rx_ring_read + 1U) % GYRO_RX_RING_SIZE);
        ConsumeByte(byte);
    }
}

void UART0_IRQHandler(void)
{
    uint8_t byte;
    uint8_t next;

    (void)DL_UART_Main_getPendingInterrupt(GYRO_UART_INST);
    while (DL_UART_Main_receiveDataCheck(GYRO_UART_INST, &byte)) {
        next = (uint8_t)((rx_ring_write + 1U) % GYRO_RX_RING_SIZE);
        if (next == rx_ring_read) {
            rx_ring_overflow_count++;
        } else {
            rx_ring[rx_ring_write] = byte;
            rx_ring_write = next;
        }
    }
}

void Gyro_Init(void)
{
    rx_length = 0U;
    tx_index = GYRO_COMMAND_SIZE;
    command_sequence = 0U;
    last_read_ms = 0U;
    angle_deg = 0.0F;
    rate_dps = 0.0F;
    frame_sequence = 0U;
    valid_frame_count = 0U;
    crc_error_count = 0U;
    has_data = false;
    pending_command = 0U;
    pending_parameter = 0U;
    rx_ring_write = 0U;
    rx_ring_read = 0U;
    rx_ring_overflow_count = 0U;
    DL_UART_Main_enableInterrupt(GYRO_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(GYRO_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(GYRO_UART_INST_INT_IRQN);
}

void Gyro_Service(uint32_t now_ms)
{
    ServiceRx();
    ServiceTx();
    if (TransmitterIdle() && pending_command != 0U) {
        QueueCommand(pending_command, pending_parameter);
        pending_command = 0U;
        ServiceTx();
    } else if (TransmitterIdle() &&
        (uint32_t)(now_ms - last_read_ms) >= GYRO_READ_PERIOD_MS) {
        last_read_ms = now_ms;
        QueueCommand(0x04U, 0x00U);
        ServiceTx();
    }
}

bool Gyro_HasData(void) { return has_data; }
float Gyro_GetAngleDeg(void) { return angle_deg; }
float Gyro_GetRateDps(void) { return rate_dps; }
uint16_t Gyro_GetFrameSequence(void) { return frame_sequence; }
uint32_t Gyro_GetValidFrameCount(void) { return valid_frame_count; }
uint32_t Gyro_GetCrcErrorCount(void) { return crc_error_count; }
void Gyro_CopyLastFrame(uint8_t frame[16])
{
    memcpy(frame, rx_frame, GYRO_DATA_FRAME_SIZE);
}
void Gyro_RequestZero(void)
{
    pending_command = 0x01U;
    pending_parameter = 0x01U;
}
void Gyro_RequestBiasCalibration(void)
{
    pending_command = 0x01U;
    pending_parameter = 0x02U;
}
