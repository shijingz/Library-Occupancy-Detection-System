#include "mesh2pi_uart.h"

nrfx_uart_t m_uart0 =  NRFX_UART_INSTANCE(0);

static uint8_t mesh_tx_buffer[TX_BUFF_SIZE] = {0xA5, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00};

void uart_init(void)
{
    nrfx_uart_config_t uart_config = {
        .pseltxd= MESH2PI_TX_PIN,         ///< TXD pin number.
        .pselrxd= MESH2PI_RX_PIN,           ///< RXD pin number.
        .pselcts=8,            ///< CTS pin number.
        .pselrts=7,            ///< RTS pin number.
        .p_context=NULL,          ///< Context passed to interrupt handler.
        .hwfc= NRF_UART_HWFC_DISABLED,               ///< Flow control configuration.
        .parity= false,             ///< Parity configuration.
        .baudrate=NRF_UART_BAUDRATE_115200,           ///< Baudrate.
        .interrupt_priority= NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY, ///< Interrupt priority
    };
    nrfx_uart_init(&m_uart0, &uart_config, uart_evt_handler);

}

void uart_evt_handler(nrfx_uart_event_t const *p_event, void *p_context)
{
    //NRFX_UART_EVT_TX_DONE, ///< Requested TX transfer completed.
    //NRFX_UART_EVT_RX_DONE, ///< Requested RX transfer completed.
    //NRFX_UART_EVT_ERROR,   ///< Error reported by UART peripheral.
    if(p_event->type == NRFX_UART_EVT_TX_DONE){
        NRF_LOG_INFO("uart tx done");

    } else if (p_event->type == NRFX_UART_EVT_RX_DONE) {
        NRF_LOG_INFO("uart rx done");

    } else if (p_event->type == NRFX_UART_EVT_ERROR) {
        NRF_LOG_INFO("uart error");
    }

}

ret_code_t uart_send_packet(uint8_t *p_Mac_addr, uint8_t battery_level, uint8_t occupancy, uint16_t temperature)
{
    ret_code_t err_code;
    
    int i;
    for(i=0;i<4;i++)
    {
        //mesh_tx_buffer[3+i] = p_Mac_addr[3-i];
        mesh_tx_buffer[6-i] = p_Mac_addr[i];
    }
    mesh_tx_buffer[7] = battery_level;
    mesh_tx_buffer[8] = occupancy;
    
    err_code = nrfx_uart_tx(&m_uart0, mesh_tx_buffer, TX_BUFF_SIZE);

    APP_ERROR_CHECK(err_code);
    return err_code;
}