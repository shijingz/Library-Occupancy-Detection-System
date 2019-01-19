/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_qwr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_sortlist.h"
#include "nrfx_rtc.h"
#include "nrf_drv_rtc.h"

#include "mesh_main.h"
#include "mesh_app_utils.h"

#include "mesh2pi_uart.h"




#define APP_BLE_CONN_CFG_TAG      1                                     /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define CENTRAL_SCANNING_LED     30
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that will write to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCAN_INTERVAL             0x00A0*5                                /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0010                                /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION             0x0000                                /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)      /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)       /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                     /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)       /**< Determines supervision time-out in units of 10 milliseconds. */
#define CHECK_NODE_INTERVAL         APP_TIMER_TICKS(1000)
#define TIME_THRES                  APP_TIMER_TICKS(300000)


NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_LBS_C_ARRAY_DEF(m_lbs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED Button client instances. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
APP_TIMER_DEF(m_check_node_timer_id);
NRF_BLE_QWR_DEF(m_qwr);                                   /**< Check node timer. */

	

static char const m_target_periph_name[] = "Nordic_HRM";             /**< Name of the device we try to connect to. This name is searched for in the scan report data*/

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);

// *********************************************************************************
//define the list for the receive data
typedef struct
{
    int8_t rssi;//indicate the signal strength
    char unique_id[10];
    uint32_t last_active_time;
    bool active;
    nrf_sortlist_item_t sensor_node;
} sensor_node_s;

sensor_node_s node[100];

static bool compare_function(nrf_sortlist_item_t * p_item0, nrf_sortlist_item_t *p_item1);
NRF_SORTLIST_DEF(sensor_node_sortlist, compare_function);

static bool compare_function(nrf_sortlist_item_t * p_item0, nrf_sortlist_item_t * p_item1)
{
    sensor_node_s * p_item_0 = CONTAINER_OF(p_item0, sensor_node_s, sensor_node);
    sensor_node_s * p_item_1 = CONTAINER_OF(p_item1, sensor_node_s, sensor_node);
    return (p_item_0->rssi > p_item_1->rssi) ? true : false;
}

static bool match_function(nrf_sortlist_item_t * p_item, char * item)
{
    sensor_node_s * p_item_0 = CONTAINER_OF(p_item, sensor_node_s, sensor_node);
    if(strcmp(p_item_0->unique_id , item)==0)
    {
        p_item_0->last_active_time = app_timer_cnt_get();
        p_item_0->active = true;
        return true;
    }
    else{
        return false;
    }
}


static bool search_function(nrf_sortlist_t const * p_list, char * item)
{
    nrf_sortlist_item_t ** pp_curr = &(p_list->p_cb->p_head);

    while(*pp_curr != NULL)
    {
        if((match_function(*pp_curr, item)))
        {
            //break;
            return true;
        }
        pp_curr = &((*pp_curr)->p_next);
    }

    return false;

    //p_item->p_next = *pp_curr;
    //*pp_curr = p_item;

    //NRF_LOG_INFO("List:%s, adding element:%08X after:%08X, before:%08X",
                                  //p_list->p_name, p_item, *pp_curr, p_item->p_next);
}

static void cal_time(nrf_sortlist_item_t * p_item, uint32_t time)
{
    sensor_node_s * p_item_0 = CONTAINER_OF(p_item, sensor_node_s, sensor_node);
    uint32_t dif_time;
    if(time >= p_item_0->last_active_time)
    {
        dif_time = time - p_item_0->last_active_time;
    }
    else
    {
        dif_time = p_item_0->last_active_time - time;
    }
    if((dif_time>TIME_THRES)&&(p_item_0->active == true))
    {
        p_item_0->active = false;
        NRF_LOG_INFO("inactive node: %s",p_item_0->unique_id);
    }
}


static void check_node_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    static uint8_t tx_buffer[4] = {'a', 'a', 'a', 'a'};


    uint32_t current_RTC_count = app_timer_cnt_get();

    NRF_LOG_INFO("current RTC count: %d",current_RTC_count);
    
  
    nrf_sortlist_t const * p_list = &sensor_node_sortlist;
    
    nrf_sortlist_item_t ** pp_curr = &(p_list->p_cb->p_head);

    while(*pp_curr != NULL)
    {
        cal_time(* pp_curr, current_RTC_count);
        pp_curr = &((*pp_curr)->p_next);
    }
}
// *********************************************************************************

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_MIN
};

/**@brief Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,

    .timeout           = SCAN_DURATION,
    .scan_phys         = BLE_GAP_PHY_1MBPS,
    .filter_policy     = BLE_GAP_SCAN_FP_ACCEPT_ALL,

};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};


/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    // Initialize Queued Write Module.
    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    (void) sd_ble_gap_scan_stop();

    //NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name);
    ret = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_off(CENTRAL_SCANNING_LED);
}


/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_lbs_c     The instance of LBS_C that triggered the event.
 * @param[in] p_lbs_c_evt The LBS_C event.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            NRF_LOG_INFO("LED Button service discovered on conn_handle 0x%x",
                         p_lbs_c_evt->conn_handle);

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
            NRF_LOG_INFO("Link 0x%x, Button state changed on peer to 0x%x",
                         p_lbs_c_evt->conn_handle,
                         p_lbs_c_evt->params.button.button_state);

            if (p_lbs_c_evt->params.button.button_state)
            {
                bsp_board_led_on(LEDBUTTON_LED);
            }
            else
            {
                bsp_board_led_off(LEDBUTTON_LED);
            }
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t err_code;
    uint8_t *ad_data_p;
    static int idx = 0;

    if (ble_advdata_name_find(p_adv_report->data.p_data,
                              p_adv_report->data.len,
                              m_target_periph_name))
    {
        //NRF_LOG_INFO("name found");
        // Name is a match, initiate connection.
        ad_data_p = ble_advdata_parse(p_adv_report->data.p_data, p_adv_report->data.len, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA);
        //NRF_LOG_INFO("manufacturer data: %x%x%x", ad_data_p[0],ad_data_p[1],ad_data_p[2]);
        
        // compare
        char node_tmp[10];
        
        if(p_adv_report->rssi > -80)
        {
            for(int i=0; i<3; i++)
            {
                if(ad_data_p[i]<10)
                {
                    node_tmp[i] = ad_data_p[i] + '0';
                }
                else
                {
                    node_tmp[i] = ad_data_p[i] - 10 + 'A';
                }
                //NRF_LOG_INFO("temp: %c", node_tmp[i]);
            }
            //NRF_LOG_INFO("manufacturer data: %s", node_tmp);
            if(!search_function(&sensor_node_sortlist, node_tmp))
            {
                // assign
                node[idx].rssi = p_adv_report->rssi;
                node[idx].last_active_time = app_timer_cnt_get();
                node[idx].active = true;
                strcpy(node[idx].unique_id, node_tmp);
                nrf_sortlist_add(&sensor_node_sortlist, &node[idx].sensor_node);
                NRF_LOG_INFO("node unique id: %s",node[idx].unique_id);
                NRF_LOG_INFO("rssi: %d",node[idx].rssi);
                NRF_LOG_INFO("RTC count: %d",node[idx].last_active_time);
                idx++;
                //nrf_sortlist_peek(&sensor_node_sortlist)
            }
        }
        
        /*
        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
        }
        */
    }
    else
    {
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected, initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                         p_gap_evt->conn_handle);
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

            err_code = ble_lbs_c_handles_assign(&m_lbs_c[p_gap_evt->conn_handle],
                                                p_gap_evt->conn_handle,
                                                NULL);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                              p_gap_evt->conn_handle);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                bsp_board_led_off(CENTRAL_SCANNING_LED);
                //scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {

            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            if (ble_conn_state_central_conn_count() == 0)
            {
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);

                // Turn off connection indication LED
                bsp_board_led_off(CENTRAL_CONNECTED_LED);
            }

            // Start scanning
            //scan_start();

            // Turn on LED for indicating scanning
            bsp_board_led_off(CENTRAL_SCANNING_LED);

        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
        
        case BLE_GAP_EVT_ADV_REPORT:
            //on_adv_report(&p_gap_evt->params.adv_report);
            //scan_start();
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;


        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief LED Button collector initialization. */
static void lbs_c_init(void)
{
    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_c_init(&m_lbs_c[i], &lbs_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);


    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for writing to the LED characteristic of all connected clients.
 *
 * @details Based on if the button is pressed or released, this function writes a high or low
 *          LED status to the server.
 *
 * @param[in] button_action The button action (press/release).
 *            Determines if the LEDs of the servers will be ON or OFF.
 *
 * @return If successful NRF_SUCCESS is returned. Otherwise, the error code from @ref ble_lbs_led_status_send.
 */
static ret_code_t led_status_send_to_all(uint8_t button_action)
{
    ret_code_t err_code;

    for (uint32_t i = 0; i< NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_led_status_send(&m_lbs_c[i], button_action);
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            return err_code;
        }
    }
        return NRF_SUCCESS;
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            err_code = led_status_send_to_all(button_action);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

   // The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

    ble_lbs_on_db_disc_evt(&m_lbs_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/*
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    
}

static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrfx_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 5;
    
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    //NRF_LOG_INFO("rtc_config");
    //APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    //nrf_drv_rtc_tick_enable(&rtc,false);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    //err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
    //APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    //nrf_drv_rtc_enable(&rtc);
}
*/

/** @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_check_node_timer_id,
                               APP_TIMER_MODE_REPEATED,
                               check_node_timeout_handler);
}

static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_check_node_timer_id, CHECK_NODE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}





int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    leds_init();
    qwr_init();
    uart_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    lbs_c_init();
    ble_conn_state_init();
    //rtc_config();
    //application_timers_start();
    //NRF_LOG_INFO("Multilink example started");
    mesh_main_initialize();
    //application_timers_start();
    // Start execution.
    NRF_LOG_INFO("Start listening ...");
    //scan_start();
    execution_start(mesh_main_start);

    for (;;)
    {
        idle_state_handle();
        //(void)sd_app_evt_wait();
    }
}
