#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "omnirevolve/hal/pins.h"
#include "omnirevolve/protocol/uart_framing.h"
#include "omnirevolve/protocol/cmd_ids.h"
#include "omnirevolve/core/state.h"

static inline uint16_t crc16_step(uint16_t crc, uint8_t b){
    crc ^= (uint16_t)b << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
    return crc;
}
static uint16_t crc16_buf(const uint8_t *p, uint16_t n){
    uint16_t crc = 0xFFFF; while(n--) crc = crc16_step(crc, *p++); return crc;
}

static void uart_send_frame(uint8_t type, const uint8_t *payload, uint16_t len){
    uint8_t hdr[5] = {UFR_SYNC0, UFR_SYNC1, type, (uint8_t)(len&0xFF), (uint8_t)(len>>8)};
    uint16_t crc = crc16_buf(payload, len);
    uint8_t tail[2] = { (uint8_t)(crc>>8), (uint8_t)(crc & 0xFF) };
    uart_write_bytes(STM32_UART_PORT, (const char*)hdr, sizeof(hdr));
    if (len) uart_write_bytes(STM32_UART_PORT, (const char*)payload, len);
    uart_write_bytes(STM32_UART_PORT, (const char*)tail, sizeof(tail));
}

void or_uart_init(void){
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    uart_driver_install(STM32_UART_PORT, 1024, 1024, 0, NULL, 0);
    uart_param_config(STM32_UART_PORT, &cfg);
    uart_set_pin(STM32_UART_PORT, STM32_TX_PIN, STM32_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("UART2 configured: TX=%d RX=%d\n", STM32_TX_PIN, STM32_RX_PIN);
}

uint32_t or_send_cmd(command_id_t cmd_id, const char* params){
    uint8_t pl[64]; uint16_t off=0;
    uint32_t rid = g_or_plotter.request_counter++;
    pl[off++] = (uint8_t)(rid & 0xFF);
    pl[off++] = (uint8_t)((rid>>8)&0xFF);
    pl[off++] = (uint8_t)((rid>>16)&0xFF);
    pl[off++] = (uint8_t)((rid>>24)&0xFF);
    pl[off++] = (uint8_t)cmd_id;
    pl[off++] = 0; // flags

    uint8_t pbuf[16]; uint16_t p_len=0;
    switch(cmd_id){
        case CMD_SET_COLOR:{
            uint8_t idx=0;
            if (params && (params[0]=='C'||params[0]=='c')) {
                int v = atoi(params+1); if (v<0) v=0; if (v>255) v=255; idx=(uint8_t)v;
            }
            pbuf[0]=idx; p_len=1; break;
        }
        case CMD_GET_CMD_STATUS:{
            uint32_t q = 0; if (params && *params) q = (uint32_t)strtoul(params,NULL,10);
            pbuf[0]=(uint8_t)(q&0xFF); pbuf[1]=(uint8_t)((q>>8)&0xFF); pbuf[2]=(uint8_t)((q>>16)&0xFF); pbuf[3]=(uint8_t)((q>>24)&0xFF);
            p_len=4; break;
        }
        default: p_len=0; break;
    }
    pl[off++] = (uint8_t)(p_len&0xFF);
    pl[off++] = (uint8_t)(p_len>>8);
    if (p_len){ memcpy(pl+off, pbuf, p_len); off+=p_len; }

    if (cmd_id < CMD_DUMMY_ACTIVE_COMMANDS_DELIMITER){
        g_or_plotter.current_cmd_status.request_id = rid;
        g_or_plotter.current_cmd_status.cmd_state  = PLOTTER_CMD_STATE__UNDEFINED;
    }
    uart_send_frame(UFR_TYPE_CMD, pl, off);
    return rid;
}

static rx_ctx_t srx;

static void rx_reset(void){ memset(&srx, 0, sizeof(srx)); srx.st = RX_SYNC0; }

static uint16_t rd_u16le(const uint8_t *p){ return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static uint32_t rd_u32le(const uint8_t *p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

static void process_cmd_response(uint8_t status, uint32_t req_id){
    if (g_or_plotter.current_cmd_status.request_id == req_id){
        if (status==0x00) g_or_plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__RECEIVED;
        else if (status==0x01){ g_or_plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__FINISHED; g_or_plotter.state.current_cmd = CURRENT_CMD_IDLE; }
        else if (status==0x02){ /* busy */ }
        else if (status==0xFF){ g_or_plotter.state.current_cmd = CURRENT_CMD_ERROR; g_or_plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__FAILED; }
    }
}

static void handle_resp(const uint8_t *p, uint16_t n){
    if (n<8) return;
    uint32_t req_id = rd_u32le(p+0);
    uint8_t  cmd_id = p[4];
    uint8_t  stat   = p[5];
    uint16_t dlen   = rd_u16le(p+6);
    if (8u + dlen != n) return;
    const uint8_t *d = p+8;

    TickType_t now = xTaskGetTickCount();
    g_or_plotter.last_rx_any_ticks = now;
    if (cmd_id == CMD_HEARTBEAT) g_or_plotter.last_rx_heartbeat_ticks = now;

    switch(cmd_id){
        case CMD_GET_STATUS:
            if (dlen>=5){
                uint8_t flags = d[0];
                uint32_t bytes = rd_u32le(d+1);
                g_or_plotter.state.is_calibrated    = (flags & 0x80)?1:0;
                g_or_plotter.state.is_homed         = (flags & 0x40)?1:0;
                g_or_plotter.state.is_processing_cmd= (flags & 0x10)?1:0;
                g_or_plotter.state.is_idle          = (!(flags & 0x10) && (flags & 0x20))?1:0;
                g_or_plotter.state.bytes_processed  = bytes;
            }
            break;
        case CMD_GET_CMD_STATUS:
            if (dlen>=5){
                uint32_t qid = rd_u32le(d+0);
                uint8_t st   = d[4];
                if (g_or_plotter.current_cmd_status.request_id == qid){
                    g_or_plotter.current_cmd_status.cmd_state = (CmdState_t)st;
                    if (st == PLOTTER_CMD_STATE__FINISHED) g_or_plotter.state.current_cmd = CURRENT_CMD_IDLE;
                    else if (st == PLOTTER_CMD_STATE__FAILED) g_or_plotter.state.current_cmd = CURRENT_CMD_ERROR;
                }
            }
            break;
        case CMD_GET_POSITION:
            if (dlen>=8){
                g_or_plotter.state.x_pos = (int32_t)rd_u32le(d+0);
                g_or_plotter.state.y_pos = (int32_t)rd_u32le(d+4);
            } else { g_or_plotter.state.x_pos = -1; g_or_plotter.state.y_pos = -1; }
            break;
        case CMD_GET_LIMITS:
            if (dlen>=8){
                float xmm, ymm;
                memcpy(&xmm, d+0, 4);
                memcpy(&ymm, d+4, 4);
                g_or_plotter.state.x_max = xmm; g_or_plotter.state.y_max = ymm;
            }
            break;
        case CMD_GET_COLOR:
            if (dlen>=1) g_or_plotter.state.current_color = d[0];
            break;
        case CMD_CALIBRATE:
            process_cmd_response(stat, req_id);
            if (stat==0x00) g_or_plotter.state.current_cmd = CURRENT_CMD_CALIBRATING;
            if (stat==0x01) or_send_cmd(CMD_GET_LIMITS, NULL);
            break;
        case CMD_HOME:
            process_cmd_response(stat, req_id);
            if (stat==0x00) g_or_plotter.state.current_cmd = CURRENT_CMD_HOMING;
            break;
        case CMD_DRAW_BEGIN:
            process_cmd_response(stat, req_id);
            if (stat==0x00) g_or_plotter.state.current_cmd = CURRENT_CMD_DRAWING;
            break;
        case CMD_PEN_UP:
        case CMD_PEN_DOWN:
        case CMD_SET_COLOR:
        case CMD_EMERGENCY_STOP:
            process_cmd_response(stat, req_id);
            break;
        default: break;
    }
    g_or_plotter.state.is_connected = 1;
}

static void handle_debug(const uint8_t *p, uint16_t n){
    char line[200]; uint16_t m = (n<sizeof(line)-1)? n : (sizeof(line)-1);
    memcpy(line, p, m); line[m]=0;
    printf("STM32[DBG]: %s\n", line);
}

void or_uart_task(void *ignored){
    (void)ignored;
    printf("UART framed RX loop\n");
    rx_reset();
    uint8_t b;
    for(;;){
        int got = uart_read_bytes(STM32_UART_PORT, &b, 1, pdMS_TO_TICKS(100));
        if (got==1){
            switch(srx.st){
                case RX_SYNC0: srx.st = (b==UFR_SYNC0)? RX_SYNC1 : RX_SYNC0; break;
                case RX_SYNC1: srx.st = (b==UFR_SYNC1)? RX_TYPE  : RX_SYNC0; break;
                case RX_TYPE:  srx.type=b; srx.st=RX_LEN0; break;
                case RX_LEN0:  srx.len=b; srx.st=RX_LEN1; break;
                case RX_LEN1:  srx.len|=((uint16_t)b<<8); srx.idx=0; srx.crc_calc=0xFFFF; srx.st = (srx.len? RX_PAYLOAD : RX_CRC_HI); break;
                case RX_PAYLOAD:
                    if (srx.idx < sizeof(srx.payload)){
                        srx.payload[srx.idx++] = b;
                        srx.crc_calc = (uint16_t)crc16_step(srx.crc_calc, b);
                        if (srx.idx >= srx.len) srx.st = RX_CRC_HI;
                    } else { rx_reset(); }
                    break;
                case RX_CRC_HI: srx.crc_hi=b; srx.st=RX_CRC_LO; break;
                case RX_CRC_LO: {
                    uint16_t recv = ((uint16_t)srx.crc_hi<<8) | b;
                    if (recv == srx.crc_calc){
                        if      (srx.type==UFR_TYPE_RESP)  handle_resp(srx.payload, srx.len);
                        else if (srx.type==UFR_TYPE_DEBUG) handle_debug(srx.payload, srx.len);
                    }
                    rx_reset();
                    break;
                }
            }
        }
        taskYIELD();
    }
}
