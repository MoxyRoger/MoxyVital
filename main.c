/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
//#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "nrf_gpio.h"
#include "string_util.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static void enable_moxy_notifications(uint16_t conn_handle, uint16_t cccd_handle);
static void moxy_data_parse(const uint8_t * p_data, uint16_t len);
static void enqueue_message(const char * p_msg);
static void fit_negative_exponential(float * p_A, float * p_B, uint8_t count);
static float calculate_r_squared(float m, float c, uint8_t n);


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        512                                     /**< UART RX buffer size. */
#define UART_CMD_SIZE            32                                     // Max length of a command string
#define MSG_BUF_COUNT            16                                     // Max number of send messages we can queue up
#define MSG_BUF_SIZE             40                                     // Max length of a single send message string
#define NUM_LOADS                 8                                     // Number of Load/Rest Steps

/* Moxy Base UUID: 6404D801-4CB9-11E8-B566-0800200C9A66 */
#define MOXY_BASE_UUID {{0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x66, 0xB5, 0xE8, 0x11, 0xB9, 0x4C, 0x00, 0x00, 0x04, 0x64}}
#define MOXY_SERVICE_UUID             0xD801  // SMO2 Service
#define MOXY_CHAR_SENSORDATA_UUID     0xD804  // Sensor Data Characteristic

typedef enum {
    CUFF_MODE_NONE,
    CUFF_MODE_SMO2,
    CUFF_MODE_TIME
} cuff_mode_t;

typedef enum {
    TEST_STATE_IDLE,
    TEST_STATE_PREP,
    TEST_STATE_LOAD,
    TEST_STATE_REST,
    TEST_STATE_CUFF,
    TEST_STATE_RELEASE,
    TEST_STATE_COOLDOWN,
    TEST_STATE_CALCULATE
} test_state_t;

static uint8_t m_moxy_uuid_type; // Global variable to store the type index
static uint16_t m_moxy_data_handle = BLE_GATT_HANDLE_INVALID;
static uint16_t m_moxy_cccd_handle = BLE_GATT_HANDLE_INVALID;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

static char m_cmd_chars[UART_CMD_SIZE];   // A buffer for commands received from uart
static char m_msg_queue[MSG_BUF_COUNT][MSG_BUF_SIZE];
static uint8_t m_cmd_index = 0;
static uint8_t m_msg_head = 0;
static uint8_t m_msg_tail = 0;
static volatile bool m_cmd_ready = false;
static volatile bool m_msg_ready = false;

static uint16_t loadtime[NUM_LOADS];
static uint16_t resttime[NUM_LOADS];
static uint16_t time[256];
static uint16_t smo2[256];
static uint8_t  smo2_index = 0;
static cuff_mode_t  cuffmode = CUFF_MODE_TIME;
static uint8_t  maxcuffs = 12;
static uint32_t slope_time[50];
static float    slope[50];
static uint16_t slope_delay = 500;
static uint8_t  cufftime =  5;
static uint8_t  rcvrtime =  5;
static uint8_t  rcvrsmo2 = 50;
static uint8_t  loadnum = 0;
static uint8_t  cuffnum = 0;
static uint32_t uptime = 0;
static uint32_t starttime = 0;
static uint32_t elapsed_time = 0;
static test_state_t teststate = TEST_STATE_IDLE;

static uint32_t step_duration = 0; // How long to stay in the current state (ms)
static uint8_t  step_index = 0;
static bool     step_init = false;

static uint8_t substep = 0;
static uint8_t substep2 = 0;
static bool pedal_state = false;
char cmd_resp[MSG_BUF_SIZE];
static uint16_t smo2_raw = 0;
static uint16_t thb_raw = 0;



NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
              char msg[MSG_BUF_SIZE];
              // Scan is automatically stopped by the connection.
              NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
              snprintf(msg, sizeof(msg),"Connecting to %02x%02x%02x%02x%02x%02x\r\n",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
              enqueue_message(msg);
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // --- UPDATED FOR MOXY ---
    ble_uuid_t moxy_uuid;
    moxy_uuid.uuid = MOXY_SERVICE_UUID;   // 0xD801
    moxy_uuid.type = m_moxy_uuid_type;       // The VS type we got from sd_ble_uuid_vs_add

    //err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &hr_uuid);
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &moxy_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == MOXY_SERVICE_UUID &&
        p_evt->params.discovered_db.srv_uuid.type == m_moxy_uuid_type)
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == MOXY_CHAR_SENSORDATA_UUID)
            {
                m_moxy_data_handle = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                m_moxy_cccd_handle = p_evt->params.discovered_db.charateristics[i].cccd_handle;

                NRF_LOG_INFO("Found Moxy Data Char! Handle: %d, CCCD: %d", m_moxy_data_handle, m_moxy_cccd_handle);

                // Enable notifications
                enable_moxy_notifications(p_evt->conn_handle, m_moxy_cccd_handle);
            }
        }
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    NRF_LOG_DEBUG("uart evt");
    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
        {
            uint8_t dummy;
            app_uart_get(&dummy);
            m_cmd_chars[m_cmd_index] = dummy;
            m_cmd_index++;
            if (dummy == '\r' || m_cmd_index >= UART_CMD_SIZE)
            {
                m_cmd_ready = true;
                NRF_LOG_DEBUG("Command Ready");
            }
            break;
        }
        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            m_cmd_ready = true;
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            m_cmd_ready = true;
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            // start discovery of services. The Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            scan_start();
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

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

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTC_EVT_HVX:
            // Check if this notification is from the Moxy Data Characteristic
            if (p_ble_evt->evt.gattc_evt.params.hvx.handle == m_moxy_data_handle)
            {
                // This is the actual data! Send it to your parser.
                moxy_data_parse(p_ble_evt->evt.gattc_evt.params.hvx.data, 
                                p_ble_evt->evt.gattc_evt.params.hvx.len);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    //nrf_gpio_cfg_output(11);
    nrf_gpio_cfg(
        11,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_H0H1,  // <-- H0H1 means "High Drive 0, High Drive 1"
        NRF_GPIO_PIN_NOSENSE
    );
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    ble_uuid_t moxy_uuid;
    moxy_uuid.uuid = MOXY_SERVICE_UUID;
    moxy_uuid.type = m_moxy_uuid_type;

    err_code = ble_db_discovery_evt_register(&moxy_uuid);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        //nrf_pwr_mgmt_run();
    }
}

void GUI_handle_uart_cmds(void)
{
    char str[20], s0[10], s1[10], s2[10], s3[10];
    int val_0, val_1, val_2, val_3;
    int arg = STR_scanf(m_cmd_chars, str, s0, s1, s2, s3);
    

    if (arg >= 2 && STR_strcmp(str, "test") == 0)
    {
        if (STR_CharToInt(s0, &val_0) == 0)
        {
            NRF_LOG_DEBUG("Received Test %d",val_0);
            snprintf(cmd_resp, sizeof(cmd_resp),"Received Test %d\r\n\r\n",val_0);
            enqueue_message(cmd_resp);
        }
    }
    else if (arg >= 1 && STR_strcmp(str, "cuffmode") == 0)
    {
        if (arg >= 2)
        {
            if (STR_CharToInt(s0, &val_0) == 0)
            {
                NRF_LOG_DEBUG("Received cuffmode %d",val_0);
                if (val_0 == CUFF_MODE_SMO2)
                {
                    cuffmode = CUFF_MODE_SMO2;
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received cuffmode %d\r\n",val_0);
                }
                else if (val_0 == CUFF_MODE_TIME)
                {
                    cuffmode = CUFF_MODE_TIME;
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received cuffmode %d\r\n",val_0);
                }
                else
                {
                    cuffmode = CUFF_MODE_NONE;
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received cuffmode invalid\r\n");
                }
            }
            else
            {
                snprintf(cmd_resp, sizeof(cmd_resp),"Received cuffmode invalid\r\n");
            }
            enqueue_message(cmd_resp);
        }
        snprintf(cmd_resp, sizeof(cmd_resp),"Curent cuffmode %d\r\n\r\n",cuffmode);
        enqueue_message(cmd_resp);
    }
    else if (arg >= 1 && STR_strcmp(str, "cufftime") == 0)
    {
        if (arg >= 2)
        {
            if (STR_CharToInt(s0, &val_0) == 0)
            {
                NRF_LOG_DEBUG("Received cufftime %d",val_0);
                if (val_0 >= 1 && val_0 <= 20)
                {
                    cufftime = val_0;
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received cufftime %d\r\n",val_0);
                }
                else
                {
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received cufftime invalid\r\n");
                }
            }
            else
            {
                snprintf(cmd_resp, sizeof(cmd_resp),"Received cufftime invalid\r\n");
            }
            enqueue_message(cmd_resp);
        }
        snprintf(cmd_resp, sizeof(cmd_resp),"Curent cufftime %d\r\n\r\n",cufftime);
        enqueue_message(cmd_resp);
    }
    else if (arg >= 2 && STR_strcmp(str, "load") == 0)
    {
        if (STR_CharToInt(s0, &val_0) == 0 && val_0 >= 1 && val_0 <= NUM_LOADS)
        {
            if (arg >= 4 && STR_CharToInt(s1, &val_1) == 0 && STR_CharToInt(s2, &val_2) == 0)
            {
                loadtime[val_0 - 1] = val_1;
                resttime[val_0 - 1] = val_2;
                snprintf(cmd_resp, sizeof(cmd_resp),"Load %d %d %d\r\n\r\n",val_0, val_1, val_2);
            }
            else
            {
                snprintf(cmd_resp, sizeof(cmd_resp),"Load %d %d %d\r\n\r\n",val_0, loadtime[val_0 - 1], resttime[val_0 - 1]);
            }
        }
        else
        {
            snprintf(cmd_resp, sizeof(cmd_resp),"Invalid load number\r\n\r\n");
        }
        enqueue_message(cmd_resp);
    }
    else if (arg >= 1 && STR_strcmp(str, "maxcuffs") == 0)
    {
        if (arg >= 2)
        {
            if (STR_CharToInt(s0, &val_0) == 0)
            {
                NRF_LOG_DEBUG("Received maxcuffs %d",val_0);
                if (val_0 >= 1 && val_0 <= 50)
                {
                    maxcuffs = val_0;
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received maxcuffs %d\r\n",val_0);
                }
                else
                {
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received maxcuffs invalid\r\n");
                }
            }
            else
            {
                snprintf(cmd_resp, sizeof(cmd_resp),"Received maxcuffs invalid\r\n");
            }
            enqueue_message(cmd_resp);
        }
        snprintf(cmd_resp, sizeof(cmd_resp),"Curent maxcuffs %d\r\n\r\n",maxcuffs);
        enqueue_message(cmd_resp);
    }
    else if (arg >= 1 && STR_strcmp(str, "rcvrtime") == 0)
    {
        if (arg >= 2)
        {
            if (STR_CharToInt(s0, &val_0) == 0)
            {
                NRF_LOG_DEBUG("Received rcvrtime %d",val_0);
                if (val_0 >= 1 && val_0 <= 20)
                {
                    rcvrtime = val_0;
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received rcvrtime %d\r\n",val_0);
                }
                else
                {
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received rcvrtime invalid\r\n");
                }
            }
            else
            {
                snprintf(cmd_resp, sizeof(cmd_resp),"Received rcvrtime invalid\r\n");
            }
            enqueue_message(cmd_resp);
        }
        snprintf(cmd_resp, sizeof(cmd_resp),"Curent rcvrtime %d\r\n\r\n",rcvrtime);
        enqueue_message(cmd_resp);
    }
    else if (arg >= 1 && STR_strcmp(str, "rcvrsmo2") == 0)
    {
        if (arg >= 2)
        {
            if (STR_CharToInt(s0, &val_0) == 0)
            {
                NRF_LOG_DEBUG("Received rcvrsmo2 %d",val_0);
                if (val_0 >= 1 && val_0 <= 100)
                {
                    rcvrsmo2 = val_0;
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received rcvrsmo2 %d\r\n",val_0);
                }
                else
                {
                    snprintf(cmd_resp, sizeof(cmd_resp),"Received rcvrsmo2 invalid\r\n");
                }
            }
            else
            {
                snprintf(cmd_resp, sizeof(cmd_resp),"Received rcvrsmo2 invalid\r\n");
            }
            enqueue_message(cmd_resp);
        }
        snprintf(cmd_resp, sizeof(cmd_resp),"Curent rcvrsmo2 %d\r\n\r\n",rcvrsmo2);
        enqueue_message(cmd_resp);
    }
    else if (arg >= 1 && STR_strcmp(str, "show") == 0)
    {
        for (int i = 0; i < NUM_LOADS; i++)
        {
            snprintf(cmd_resp, sizeof(cmd_resp),"Step %d %d %d\r\n",(i+1), loadtime[i], resttime[i]);
            enqueue_message(cmd_resp);
        }
        snprintf(cmd_resp, sizeof(cmd_resp),"\r\n");
        enqueue_message(cmd_resp);
    }
    else if (arg >= 1 && STR_strcmp(str, "start") == 0)
    {
        if (teststate == TEST_STATE_IDLE)
        {
            teststate = TEST_STATE_PREP;
            starttime = uptime;
            snprintf(cmd_resp, sizeof(cmd_resp),"Received Start\r\n\r\n");
        }
        else
        {
            snprintf(cmd_resp, sizeof(cmd_resp),"Test Already Running\r\n\r\n");
        }
        enqueue_message(cmd_resp);
    }
    else if (arg >= 1 && STR_strcmp(str, "time") == 0)
    {
        snprintf(cmd_resp, sizeof(cmd_resp),"Time %d\r\n\r\n", uptime);
        enqueue_message(cmd_resp);
    }
    else
    {
        snprintf(cmd_resp, sizeof(cmd_resp),"Invalid Command\r\n\r\n");
        enqueue_message(cmd_resp);
    }
    
    for(int i = 0; i < UART_CMD_SIZE ; i++)
    {
      m_cmd_chars[i] = '\0';
    }
    m_cmd_index = 0;
    m_cmd_ready = false;
}

void app_uart_put_string(const uint8_t * p_str)
{
    while (*p_str != '\0')
    {
        // Loop until a null terminator is found
        while (app_uart_put(*p_str) != NRF_SUCCESS);
        p_str++;
    }
}


static void enqueue_message(const char * p_msg)
{
    // Copy the string into the current write slot
    strncpy(m_msg_queue[m_msg_head], p_msg, MSG_BUF_SIZE - 1);
    m_msg_queue[m_msg_head][MSG_BUF_SIZE - 1] = '\0'; // Ensure null termination
    m_msg_head = (m_msg_head + 1) % MSG_BUF_COUNT;
    m_msg_ready = true;
}


// Helper to pulse the GPIO Pin 11 and LED
static void pedal_pulse(void)
{
    nrf_gpio_pin_set(11);
    bsp_board_led_on(1);
    pedal_state = true;
    // Note: The turn-off logic remains in the main loop to avoid blocking
}

// Helper to reset the GPIO Pin 11 and LED
static void pedal_reset(void)
{
    nrf_gpio_pin_clear(11);
    bsp_board_led_off(1);
    pedal_state = false;
}

// Convert RTC ticks to milliseconds
static uint32_t get_now_ms(void)
{
    return (app_timer_cnt_get() * 1000) / (APP_TIMER_TICKS(1000) / 1);
}


/**
 * @brief Main State Machine Logic
 * Called every iteration of the for(;;) loop.
 */
static void test_start_step(void)
{
    // Check if this is a new step
    if (step_init == false)
    {
        char msg[MSG_BUF_SIZE];
        switch (teststate)
        {
            case TEST_STATE_PREP:
                snprintf(msg, sizeof(msg), "%d Get Ready...\r\n", 5 - step_index);
                enqueue_message(msg);
                step_duration = 1000;
                break;

            case TEST_STATE_LOAD:
                snprintf(msg, sizeof(msg), "%d seconds - Load %d\r\n", (loadtime[loadnum] - step_index), loadnum);
                enqueue_message(msg);
                step_duration = 1000;
                break;

            case TEST_STATE_REST:
                snprintf(msg, sizeof(msg), "%d seconds - Rest %d\r\n", (resttime[loadnum] - step_index), loadnum);
                enqueue_message(msg);
                step_duration = 1000;
                break;

            case TEST_STATE_CUFF:
                snprintf(msg, sizeof(msg), "\r\nCuff %d\r\n", cuffnum + 1);
                enqueue_message(msg);
                pedal_pulse();
                step_duration = 1000 * cufftime;
                smo2_index = 0;
                slope_time[cuffnum] = uptime + slope_delay;
                break;

            case TEST_STATE_RELEASE:
                snprintf(msg, sizeof(msg), "\r\nRelease %d\r\n", cuffnum + 1);
                enqueue_message(msg);
                pedal_pulse();
                step_duration = 1000 * rcvrtime;
                uint32_t t0 = time[0]; 
                float f_sumx  = 0;
                float f_sumxx = 0;
                float f_sumxy = 0;
                float f_sumy  = 0;
                float n = (float)smo2_index;
                float f_slope = 0;

                for (uint8_t i = 0; i < smo2_index; i++) {
                    // x is time relative to the start of this recovery phase
                    float x = (float)(time[i] - t0) / 1000.0; 
                    float y = (float)smo2[i] / 10.0;

                    f_sumx  += x;
                    f_sumxx += x * x;
                    f_sumxy += x * y;
                    f_sumy  += y;
                }
                float denominator = (n * f_sumxx - f_sumx * f_sumx);
                if (denominator != 0) {
                    f_slope = (n * f_sumxy - f_sumx * f_sumy) / denominator;
                } else {
                    f_slope = 0;
                }
                slope[cuffnum] = f_slope;
                snprintf(msg, sizeof(msg), "Time: %d, Slope: %.3f\r\n\r\n", slope_time[cuffnum], f_slope);
                enqueue_message(msg);
                break;

            case TEST_STATE_COOLDOWN:
                snprintf(msg, sizeof(msg), "%d Seconds Cooldown\r\n", 5 - step_index);
                enqueue_message(msg);
                step_duration = 1000;
                break;

            case TEST_STATE_CALCULATE:
                {
                float curve_A = 0;
                float curve_B = 0;
                fit_negative_exponential(&curve_A, &curve_B, cuffnum);
                snprintf(msg, sizeof(msg), "\r\nFit: y = %.2f * e^(%.4f * x)\r\n", curve_A, curve_B);
                enqueue_message(msg);
                float r2 = calculate_r_squared(curve_B, logf(fabsf(curve_A)), cuffnum);
                snprintf(msg, sizeof(msg), "R-sq: %.3f)\r\n", r2);
                enqueue_message(msg);
                step_duration = 1000;
                break;
                }

            case TEST_STATE_IDLE:
                break;
        }
        step_init = true;
    }
}
    

static void test_check_end(void)
{
    switch (teststate)
    {
        case TEST_STATE_PREP:
            if (elapsed_time >= step_duration)
            {
                step_index++;
                if (step_index >= 5)
                {
                    teststate = TEST_STATE_LOAD;
                    step_index = 0;
                }
                starttime = uptime;
                step_init = false;
            }
            break;

        case TEST_STATE_LOAD:
            if (elapsed_time >= step_duration)
            {
                step_index++;
                if (step_index >= loadtime[loadnum])
                {
                    step_index = 0;
                    if (resttime[loadnum] == 0)
                    {
                        teststate = TEST_STATE_CUFF;
                    }
                    else
                    {
                        teststate = TEST_STATE_REST;
                    }   
                }
                starttime =uptime;
                step_init = false;
            }
            break;

        case TEST_STATE_REST:
            if (elapsed_time >= step_duration)
            {
                step_index++;
                if (step_index >= resttime[loadnum])
                {
                    step_index = 0;
                    loadnum++;
                    if (loadtime[loadnum] == 0)
                    {
                        teststate = TEST_STATE_CUFF;
                    }
                    else
                    {
                        teststate = TEST_STATE_LOAD;
                    }   
                }
                starttime =uptime;
                step_init = false;
            }
            break;

        case TEST_STATE_CUFF:
            if (elapsed_time >= step_duration || (cuffmode == CUFF_MODE_SMO2 && smo2_raw > rcvrsmo2 * 10))
            {
                starttime = uptime;
                step_init = false;
                teststate = TEST_STATE_RELEASE;
            }
            break;

        case TEST_STATE_RELEASE:
            if (elapsed_time >= step_duration)
            {
                cuffnum++;
                if (cuffnum >= maxcuffs)
                {
                    teststate = TEST_STATE_COOLDOWN;
                    step_index = 0;
                }
                else
                {
                    teststate = TEST_STATE_CUFF;
                }
                starttime = uptime;
                step_init = false;
                
            }
            break;
                
        case TEST_STATE_COOLDOWN:
            if (elapsed_time >= step_duration)
            {
                step_index++;
                if (step_index >= 5)
                {
                    teststate = TEST_STATE_CALCULATE;
                    step_index = 0;
                    step_init = false;
                }
                starttime = uptime;
                step_init = false;
            }
            break;

        case TEST_STATE_CALCULATE:
            if (elapsed_time >= step_duration)
            {
                teststate = TEST_STATE_IDLE;
                step_index = 0;
                step_init = false;
            }

        case TEST_STATE_IDLE:
            break;  
    }
}


static void moxy_data_parse(const uint8_t * p_data, uint16_t len)
{
    // Moxy D804 Packet typically starts with a bitmask, then data.
    // Standard Moxy SmO2 is at byte offset 2-3
    // Standard Moxy tHb is at byte offset 6-7
    
    if (len >= 6)
    {
        // Reconstruct 16-bit values from Little-Endian bytes
        smo2_raw = (p_data[3] << 8) | p_data[2];
        thb_raw  = (p_data[7] << 8) | p_data[6];

        // Moxy scales SmO2 by 10 (e.g., 750 = 75.0%)
        // Moxy scales tHb by 100 (e.g., 1250 = 12.50 g/dL)
        // Log it to the Debugger
        //NRF_LOG_INFO("DATA >> SmO2: %d.%d%% | tHb: %d.%02d g/dL", 
        //                      smo2_raw / 10, smo2_raw % 10,
        //                      thb_raw / 100, thb_raw % 100);

        // Output to UART (PC Terminal) ToDo: queue up in a message buffer and move to the main loop
        if (teststate == TEST_STATE_CUFF)
        {
            char msg[MSG_BUF_SIZE];
            int str_len = snprintf(msg, sizeof(msg), "T: %d, S: %d.%d, t: %d.%02d\r\n", 
                                   elapsed_time,
                                   smo2_raw / 10, smo2_raw % 10,
                                   thb_raw / 100, thb_raw % 100);
            enqueue_message(msg);
            if (elapsed_time > slope_delay && smo2_raw > 0)
            {
                smo2[smo2_index] = smo2_raw;
                time[smo2_index] = elapsed_time;
                smo2_index++;
            }
        }
    }
}


void moxy_uuid_init(void)
{
    ret_code_t err_code;
    ble_uuid128_t base_uuid = MOXY_BASE_UUID;

    // Register the 128-bit base. This returns a "type" (usually 0x02 or 0x03)
    err_code = sd_ble_uuid_vs_add(&base_uuid, &m_moxy_uuid_type);
    APP_ERROR_CHECK(err_code);
}

static void enable_moxy_notifications(uint16_t conn_handle, uint16_t cccd_handle)
{
    ret_code_t err_code;
    uint8_t   data[2] = {0x01, 0x00}; // Value to enable notifications

    ble_gattc_write_params_t write_params;
    memset(&write_params, 0, sizeof(write_params));

    write_params.handle   = cccd_handle;
    write_params.len      = 2;
    write_params.p_value  = data;
    write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    write_params.offset   = 0;

    err_code = sd_ble_gattc_write(conn_handle, &write_params);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Moxy notifications enabled!");
    }
}

/**
 * @brief Fits y = A * e^(B * x) to negative slope data.
 * @param p_A Output parameter for the amplitude (will be negative)
 * @param p_B Output parameter for the rate (per second)
 * @param count Number of data points
 */
void fit_negative_exponential(float * p_A, float * p_B, uint8_t count) {
    float sumX  = 0;
    float sumY  = 0; // sum of ln(|slope|)
    float sumXX = 0;
    float sumXY = 0;
    uint8_t n   = 0;

    if (count < 2) return;

    // Time offset in milliseconds
    uint32_t t0 = slope_time[0];

    for (uint8_t i = 0; i < count; i++) {
        // 1. Handle Negative Slopes: 
        // Log is only defined for positive numbers. We fit the magnitude.
        float absolute_y = (slope[i] < 0) ? -slope[i] : slope[i];
        
        // Safety: skip if slope is exactly 0
        if (absolute_y < 0.000001f) continue;

        // 2. Time Scaling: Convert ms to seconds for a readable B rate
        float x = (float)(slope_time[i] - t0) / 1000.0f;
        float y_ln = logf(absolute_y);

        sumX  += x;
        sumXX += x * x;
        sumXY += x * y_ln;
        sumY  += y_ln;
        n++;
    }

    if (n < 2) {
        *p_A = 0; *p_B = 0;
        return;
    }

    // 3. Linear Regression
    float denominator = (n * sumXX - sumX * sumX);
    if (fabsf(denominator) < 0.000001f) {
        *p_A = 0; *p_B = 0;
        return;
    }

    float m = (n * sumXY - sumX * sumY) / denominator; // This is B
    float c = (sumY - m * sumX) / n;                   // This is ln(A)

    // 4. Conversion and Sign Re-application
    *p_B = m;            // The rate of change
    *p_A = -expf(c);     // We make A negative because the original data was negative
}

/**
 * @brief Logic to add inside fit_negative_exponential to calculate R-Squared
 * @param m The calculated slope (B)
 * @param c The calculated intercept (ln(A))
 * @return float The R-squared value (0.0 to 1.0)
 */
float calculate_r_squared(float m, float c, uint8_t n) {
    float ss_res = 0;
    float ss_tot = 0;
    float mean_y = 0;
    float y_vals[50]; // Temporary array for ln values

    // 1. Calculate the mean of the linearized Y values
    for (uint8_t i = 0; i < n; i++) {
        y_vals[i] = logf(fabsf(slope[i]));
        mean_y += y_vals[i];
    }
    mean_y /= n;

    // 2. Calculate Sum of Squares
    uint32_t t0 = slope_time[0];
    for (uint8_t i = 0; i < n; i++) {
        float x = (float)(slope_time[i] - t0) / 1000.0f;
        float y_pred = m * x + c;
        
        ss_res += powf(y_vals[i] - y_pred, 2);
        ss_tot += powf(y_vals[i] - mean_y, 2);
    }

    if (ss_tot == 0) return 0;
    return 1.0f - (ss_res / ss_tot);
}

int main(void)
{    
    // Initialize.
    log_init();
    timer_init();
    uart_init();
    buttons_leds_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    moxy_uuid_init();
    db_discovery_init();
    scan_init();

    // Start execution.
    enqueue_message("Moxy Vital started.\r\n\r\n");
    NRF_LOG_INFO("Moxy Vital started.");
    scan_start();

    loadtime[0] = 3;
    resttime[0] = 5;

    // Enter main loop.
    for (;;)
    {
        // Get current time
        uptime = get_now_ms();
        
        // Process any Commands that have been recieved
        if (m_cmd_ready)
        {
            GUI_handle_uart_cmds();
        }

        // Calculate elapsed_time in the current step
        if (starttime != 0) 
        {
            elapsed_time = uptime - starttime;
        }

        // Send 1 message from the queue
        if (m_msg_ready)
        {
            app_uart_put_string((uint8_t *)m_msg_queue[m_msg_tail]);
            m_msg_tail = (m_msg_tail + 1) % MSG_BUF_COUNT;
            if (m_msg_tail == m_msg_head)
            {
              m_msg_ready = false;
            }
        }
        
        // Check if we need to turn off the pedal pulse
        if (pedal_state && elapsed_time > 500)
        {
            pedal_reset();
        }
        
        
        if (teststate != TEST_STATE_IDLE)
        {
            // Check if we need to take a new action because it's a start of a new step
            test_start_step();
            // Check if we have reached the end of the step and advance
            test_check_end();
        }
        else
        {
            idle_state_handle();
        }
    }
}
