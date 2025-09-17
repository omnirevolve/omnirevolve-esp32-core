// UART framed link (ESP32 side). Provides TX of commands and RX callbacks.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "omnirevolve/protocol/cmd_ids.h"
#include "omnirevolve/protocol/uart_framing.h"
#include "omnirevolve/link/uart_link.h"

#define UART_BUFFER_SIZE 1024

static uart_port_t s_port = UART_NUM_2;
static uint32_t s_req_counter = 1;

static uart_resp_cb_t s_resp_cb = NULL;
static uart_debug_cb_t s_dbg_cb = NULL;

// CRC16-CCITT (0xFFFF)
static inline uint16_t crc16_ccitt_step(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
    return crc;
}
static uint16_t crc16_ccitt_buf(const uint8_t *p, uint16_t n) {
    uint16_t crc = 0xFFFF;
    while (n--) crc = crc16_ccitt_step(crc, *p++);
    return crc;
}
static inline void wr_u16le(uint8_t *p, uint16_t v){ p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)(v>>8); }
static inline void wr_u32le(uint8_t *p, uint32_t v){ p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)((v>>8)&0xFF); p[2]=(uint8_t)((v>>16)&0xFF); p[3]=(uint8_t)((v>>24)&0xFF); }

void uart_link_set_callbacks(uart_resp_cb_t resp_cb, uart_debug_cb_t dbg_cb) {
    s_resp_cb = resp_cb;
    s_dbg_cb = dbg_cb;
}

void uart_link_init(uart_port_t port, int tx_pin, int rx_pin, int baud) {
    s_port = port;
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    uart_driver_install(s_port, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);
    uart_param_config(s_port, &cfg);
    uart_set_pin(s_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("[uart_link] port=%d tx=%d rx=%d baud=%d\n", s_port, tx_pin, rx_pin, baud);
}

static void uart_send_frame(uint8_t type, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[5] = {UFR_SYNC0, UFR_SYNC1, type, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint16_t crc = crc16_ccitt_buf(payload, len);
    uint8_t tail[2] = { (uint8_t)(crc>>8), (uint8_t)(crc & 0xFF) };
    uart_write_bytes(s_port, (const char*)hdr, sizeof(hdr));
    if (len) uart_write_bytes(s_port, (const char*)payload, len);
    uart_write_bytes(s_port, (const char*)tail, sizeof(tail));
}

uint32_t uart_link_send_cmd(command_id_t cmd_id, const char* params) {
    // CTRL_CMD payload: [req_id(4)][cmd(1)][flags(1)=0][plen(2)][p...]
    uint8_t pl[64];
    uint16_t off = 0;
    uint32_t req_id = s_req_counter++;

    // header
    wr_u32le(pl + off, req_id); off += 4;
    pl[off++] = (uint8_t)cmd_id;
    pl[off++] = 0; // flags

    // params
    uint16_t p_len = 0;
    uint8_t pbuf[16];
    switch (cmd_id) {
        case CMD_SET_COLOR: {
            uint8_t idx = 0;
            if (params && (params[0]=='C' || params[0]=='c')) {
                int v = atoi(params+1);
                if (v < 0) { v = 0; }
                if (v > 255) { v = 255; }
                idx = (uint8_t)v;
            }
            pbuf[0] = idx; p_len = 1;
            break;
        }
        case CMD_GET_CMD_STATUS: {
            uint32_t q = 0;
            if (params && *params) q = (uint32_t)strtoul(params, NULL, 10);
            wr_u32le(pbuf, q); p_len = 4;
            break;
        }
        default: p_len = 0; break;
    }
    wr_u16le(pl + off, p_len); off += 2;
    if (p_len) { memcpy(pl + off, pbuf, p_len); off += p_len; }

    uart_send_frame(UFR_TYPE_CMD, pl, off);
    return req_id;
}

static void handle_debug_text(const uint8_t *p, uint16_t n) {
    if (!s_dbg_cb) return;
    char line[200];
    uint16_t m = (n < sizeof(line)-1) ? n : (sizeof(line)-1);
    memcpy(line, p, m); line[m] = 0;
    s_dbg_cb(line);
}

static void rx_task(void *arg) {
    (void)arg;
    printf("[uart_link] RX task\n");
    rx_ctx_t srx; memset(&srx, 0, sizeof(srx)); srx.st = RX_SYNC0;
    uint8_t b;
    for (;;) {
        int got = uart_read_bytes(s_port, &b, 1, pdMS_TO_TICKS(100));
        if (got == 1) {
            switch (srx.st) {
                case RX_SYNC0: srx.st = (b==UFR_SYNC0) ? RX_SYNC1 : RX_SYNC0; break;
                case RX_SYNC1: srx.st = (b==UFR_SYNC1) ? RX_TYPE  : RX_SYNC0; break;
                case RX_TYPE:  srx.type = b; srx.st = RX_LEN0; break;
                case RX_LEN0:  srx.len = b; srx.st = RX_LEN1; break;
                case RX_LEN1:  srx.len |= (uint16_t)b<<8; srx.idx = 0; srx.crc_calc = 0xFFFF; srx.st = (srx.len ? RX_PAYLOAD : RX_CRC_HI); break;
                case RX_PAYLOAD:
                    if (srx.idx < sizeof(srx.payload)) {
                        srx.payload[srx.idx++] = b;
                        srx.crc_calc = crc16_ccitt_step(srx.crc_calc, b);
                        if (srx.idx >= srx.len) srx.st = RX_CRC_HI;
                    } else { memset(&srx, 0, sizeof(srx)); srx.st = RX_SYNC0; }
                    break;
                case RX_CRC_HI:
                    srx.crc_hi = b; srx.st = RX_CRC_LO; break;
                case RX_CRC_LO: {
                    uint16_t crc_recv = ((uint16_t)srx.crc_hi<<8) | b;
                    if (crc_recv == srx.crc_calc) {
                        if (srx.type == UFR_TYPE_RESP)      { if (s_resp_cb) s_resp_cb(srx.payload, srx.len); }
                        else if (srx.type == UFR_TYPE_DEBUG){ handle_debug_text(srx.payload, srx.len); }
                    }
                    memset(&srx, 0, sizeof(srx)); srx.st = RX_SYNC0;
                    break;
                }
            }
        }
        taskYIELD();
    }
}

void uart_link_start(void) {
    xTaskCreatePinnedToCore(rx_task, "stm32_uart", 4096, NULL, 3, NULL, 1);
}
