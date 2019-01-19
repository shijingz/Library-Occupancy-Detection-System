#include "nrfx_uart.h"
#include "nrf_log.h"
#include "nrf_drv_uart.h"

#define MESH2PI_TX_PIN 6
#define MESH2PI_RX_PIN 31
#define TX_BUFF_SIZE 11

void uart_init(void);
void uart_evt_handler(nrfx_uart_event_t const *p_event, void *p_context);
ret_code_t uart_send_packet(uint8_t *p_Mac_addr, uint8_t battery_level, uint8_t occupancy, uint16_t temperature);