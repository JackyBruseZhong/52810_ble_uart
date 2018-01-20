/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */



#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "OP_main.h"
#include "nrf_gpio.h"
#include "ble_dis.h"
#include "het_open_platform.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "lrd_at.h"
#include "fds.h"
#include "fds_internal_defs.h"		
#include "nrf_drv_gpiote.h"
#include "ble_dfu.h"
#include "nrf_dfu_svci.h"
#include "AT_ComHandler.h"
#include "nrf_delay.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */


#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_OPEN_PLATFORM_SERVICE, BLE_UUID_TYPE_BLE}
};

uart_data_t 							 OP_uart_data;
uart_data_recv_t  				 uart_data_recv;
open_platform_data_recv_t  ble_data_recv;
void dtm_mode_check(void);

bool PowerUp_uart_rec = false;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void set_tx_power(void)
{
		int8_t 	tx_power_set;	
		switch(at_com_data.tx_power)
		{
			case 0:
//				 printf("change tx_power +4dBm\r\n");	
				 tx_power_set = 4;					
				break;
			
			case 1:
//				 printf("change tx_power 0dBm\r\n");	
				 tx_power_set = 0;						
				break;
			
			case 2:
//				 printf("change tx_power -4dBm\r\n");	
				 tx_power_set = -4;					
				break;
			
			case 3:
//				 printf("change tx_power -8dBm\r\n");	
				 tx_power_set = -8;					
				break;
			
			case 4:
//				 printf("change tx_power -12dBm\r\n");	
				 tx_power_set = -12;					
				break;			

			case 5:
//				 printf("change tx_power -16dBm\r\n");	
				 tx_power_set = -16;					
				break;

			case 6:
//				 printf("change tx_power -20dBm\r\n");	
				 tx_power_set = -20;					
				break;	

			case 7:
//				 printf("change tx_power -30dBm\r\n");	
				 tx_power_set = -30;					
				break;			

			case 8:
//				 printf("change tx_power -40dBm\r\n");	
				 tx_power_set = -40;					
				break;		
      default:
	//			 printf("DEFALUT tx_power 0dBm\r\n");	
				 tx_power_set = 0;					
				break;						
		}
	  uint32_t err_code = sd_ble_gap_tx_power_set(tx_power_set);
	  APP_ERROR_CHECK(err_code);			
}
/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
	  
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

		if((at_com_data.device_name[0]>0)&&(at_com_data.device_name[0]<=21))				// Device name has been saved in the flash.
		{
		//	printf("Change device name %s len=%d\r\n",&at_com_data.device_name[1],at_com_data.device_name[0]);	
			err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)&at_com_data.device_name[1],
                                          at_com_data.device_name[0]);	
			APP_ERROR_CHECK(err_code);																		
																																				
		}		 
		else																	// Device name isn't in the flash.
		{    
				err_code = sd_ble_gap_device_name_set(&sec_mode,
																						(const uint8_t *) DEVICE_NAME,
																						strlen(DEVICE_NAME));
				APP_ERROR_CHECK(err_code);
		}
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

		if((at_com_data.con_interval.conInterval>=16)&&(at_com_data.con_interval.conInterval<=3200))
		{
//			printf("Change conInterval %x\r\n",at_com_data.con_interval.conInterval);
			if(at_com_data.con_interval.conInterval>MIN_CONN_INTERVAL)
			{
					 gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
					
				 gap_conn_params.max_conn_interval = MSEC_TO_UNITS(at_com_data.con_interval.conInterval, UNIT_1_25_MS);;
			}
			else if(at_com_data.con_interval.conInterval<MAX_CONN_INTERVAL)
			{
				 gap_conn_params.min_conn_interval = MSEC_TO_UNITS(at_com_data.con_interval.conInterval, UNIT_1_25_MS);
				 gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL; 		
			}
		}	
		else
		{	
			gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
			gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
		}
		if((at_com_data.con_interval.latency >0)&&(at_com_data.con_interval.latency <0xFFFF))
		{
			gap_conn_params.slave_latency	  = at_com_data.con_interval.latency;
	//		printf("change latency\r\n");
		}
		else
		{
			gap_conn_params.slave_latency	  = SLAVE_LATENCY;
		}
		gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
			
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
		
		set_tx_power();
	
}


APP_TIMER_DEF(m_send_mcu_id);


static void send_mcu_timeout_handler(void * p_context)
{			
//		printf("send_mcu_timeout\r\n");
			for(uint8_t i=0;i<ble_data_recv.total_len;i++)
			{
				app_uart_put(ble_data_recv.rece_data[i]);
			}
			nrf_gpio_pin_clear(UART_DATA_SEND_PIN);	
			memset(ble_data_recv.rece_data,0,sizeof(ble_data_recv.rece_data));
}

void send_data_to_mcu_timer_init(void)
{
		app_timer_create(&m_send_mcu_id,
										 APP_TIMER_MODE_SINGLE_SHOT,
										 send_mcu_timeout_handler);
				
}
void send_data_to_mcu_timer_start(uint16_t time)
{
		app_timer_start(m_send_mcu_id, time,NULL);
}
/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
//        uint32_t err_code;

       // printf("Received data from BLE NUS. Writing data on UART len=%d\r\n.",p_evt->params.rx_data.length);
//        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    }

}
static void dis_service_init(void)
{
		
//		uint8_t  switc_;
    ble_dis_init_t 								dis_init;
		ble_dis_sys_id_t  						sys_id;
		ble_dis_pnp_id_t  						pnp_id;
		ble_dis_reg_cert_data_list_t  reg_cert_data;	
		ble_gap_addr_t 								mac_addr; 
		uint8_t 											reg_cert_data_arry[20]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};		
		
		memset(&mac_addr,0,sizeof(mac_addr));	
		uint32_t error_code = sd_ble_gap_addr_get(&mac_addr);

// Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
		
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUM);
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)FIRM_WARE_REVERSION);
		ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
	  ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)SERIAL_NUM);
		ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HARD_WARE_REVERSION);
		ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, (char *)SOFT_WARE_REVERSION);
			
		sys_id.manufacturer_id            = (mac_addr.addr[3]<<16)|(mac_addr.addr[4]<<8)|mac_addr.addr[5];//ORG_UNIQUE_ID;
    sys_id.organizationally_unique_id = (mac_addr.addr[0]<<16)|(mac_addr.addr[1]<<8)|mac_addr.addr[2];//MANUFACTURER_ID;

		pnp_id.vendor_id_source						= PNP_VENDOR_ID_SOURCE;
		pnp_id.vendor_id									= PNP_VENDOR_ID;
		pnp_id.product_version						= PNP_PRODUCT_VERSION;
		pnp_id.product_id									= PNP_PRODUCT_ID;
		
		reg_cert_data.list_len						= strlen((char *)reg_cert_data_arry);
		reg_cert_data.p_list							= reg_cert_data_arry;
    dis_init.p_sys_id                 = &sys_id;
		dis_init.p_pnp_id 								= &pnp_id;		
		dis_init.p_reg_cert_data_list			= &reg_cert_data;
    
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    uint32_t err_code = ble_dis_init(&dis_init);
}
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
//            printf("Device is preparing to enter bootloader mode.");
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
 //           printf("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
//            printf("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
//            printf("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
 //           printf("Unknown event from ble_dfu_buttonless.");
            break;
    }
}
static void dfu_service_init(void)
{
    uint32_t       err_code;		
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    };

    // Initialize the async SVCI interface to bootloader.
//    err_code = ble_dfu_buttonless_async_svci_init();
//    APP_ERROR_CHECK(err_code);
    
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);	
}	
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
	
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
	
		// Initialize Device information Service.
    dis_service_init();	
	
  	dfu_service_init();

}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            printf("Connected");

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
//            printf("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

#ifndef S140
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
#endif

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
#if !defined (S112)
         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;
#endif //!defined (S112)
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
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
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


void app_uart_handle_ATCommand(uint8_t UartData)
{
	//	printf("UartData=0x%02x\r\n ",UartData);
		if((UartData==0x01)&&(uart_data_recv.receive_start_flag== false))
		{
		//	printf("Flag OK!\r\n");
			uart_data_recv.data_index = 0;	
			uart_data_recv.rece_data[uart_data_recv.data_index++] = UartData;		
			uart_data_recv.receive_start_flag = true;			
		}
		else if((uart_data_recv.rece_data[0]==0x01)&&(uart_data_recv.receive_start_flag == true)&&(UartData==0xFC))
		{
		//		printf("FC OK!\r\n");
				uart_data_recv.rece_data[uart_data_recv.data_index++] = UartData;
		}
		else if((uart_data_recv.rece_data[0]==0x01)&&(uart_data_recv.rece_data[1]==0xFC)&&(uart_data_recv.receive_start_flag == true))
		{
		//	printf("UartData=0x%02x\r\n ",UartData);
			uart_data_recv.rece_data[uart_data_recv.data_index++] = UartData;				
			if(uart_data_recv.data_index==0x04)
			{
  	//			printf("total_len=%d data_index=%d\r\n",UartData,uart_data_recv.data_index);
					uart_data_recv.total_len = UartData;
					uart_data_recv.receive_len = true;
			}	
											
			if(uart_data_recv.receive_len)
			{
		//		printf("data_len++ data=%02x\r\n",UartData);		
				uart_data_recv.data_len++;
			}
			
		}		
		else
		{
//				printf("other data\r\n");					
					memset(&uart_data_recv,0,sizeof(uart_data_recv));
		}

		if((uart_data_recv.rece_data[0]==0x01)&&(uart_data_recv.rece_data[1]==0xFC)
			  &&(uart_data_recv.receive_start_flag == true)&&(uart_data_recv.receive_len)&&((uart_data_recv.data_len-1) >=uart_data_recv.total_len))										//receive data end					
		{				
//			printf("Recive Finish data_len=%d data_index=%d\r\n",uart_data_recv.data_len,uart_data_recv.data_index);
//			for(uint8_t i=0;i<uart_data_recv.data_index;i++)
//				printf("0x%02x ",uart_data_recv.rece_data[i]);
//			printf("\r\n");	
			PowerUp_uart_rec = true;	
			La_atCheck(uart_data_recv.rece_data,uart_data_recv.data_index);
			memset(&uart_data_recv,0,sizeof(uart_data_recv));
		}

}


APP_TIMER_DEF(m_uart_receive_id);

#define UART_RECEIVE_TIMEOUT_INTERVAL 	APP_TIMER_TICKS(40)

void uart_receive_timer_start(void)
{
	app_timer_start(m_uart_receive_id, UART_RECEIVE_TIMEOUT_INTERVAL,NULL);
}

static void uart_receive_timeout_handler(void * p_context)
{
	//printf("timeout len=%d\r\n",OP_uart_data.len);
	Sender_SetData(0,OP_uart_data.data_array,OP_uart_data.len);	
	app_timer_stop(m_uart_receive_id);	
	OP_uart_data.len = 0;
}
void uart_receive_timer_init(void)
{
	app_timer_create(&m_uart_receive_id,
                   APP_TIMER_MODE_SINGLE_SHOT,//APP_TIMER_MODE_REPEATED,
									 uart_receive_timeout_handler);

}

void app_uart_handle_data(uint8_t UartData)
{	
//		printf("flag=%x\r\n",UartData);
//		printf("flag=%d %d\r\n",OP_uart_data.buffer1_empty,OP_uart_data.buffer2_receive_complete);	
			app_timer_stop(m_uart_receive_id);
		  OP_uart_data.data_array[OP_uart_data.len++] = UartData;					
			uart_receive_timer_start();
			if(OP_uart_data.len==20)
			{
//				printf("REC OK\r\n");
//				printf("data_len:%d\r\n",OP_uart_data.len);
//				for(uint8_t i=0;i<(OP_uart_data.len);i++)
//					printf("%02x ",OP_uart_data.data_array[i]);	
//				printf("\r\n");		
				OP_uart_data.len = 0;
			  Sender_SetData(0,OP_uart_data.data_array,20);						
			}					
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t uart_data = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_data));
					printf("rec=%x\r\n",uart_data);	
					if(!nrf_gpio_pin_read(UART_DATA_COMMAND_MODE_PIN))
					{
						
						app_uart_handle_ATCommand(uart_data);
					}
					else
					{
						app_uart_handle_data(uart_data);
						
					}

            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t  comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UARTE_BAUDRATE_BAUDRATE_Baud9600
    };
		if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud9600)
		{			
				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud9600;
		}
		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud19200)
		{			
				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud19200;
		}
		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud38400)
		{			
				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud38400;
		}
		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud57600)
		{			
				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud57600;
		}
		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud115200)
		{			
				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud115200;
		}
		else
		{
				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud9600;
		}	
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_HIGH,//APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);

		nrf_gpio_cfg_input(UART_DATA_COMMAND_MODE_PIN, NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_output(UART_DATA_SEND_PIN);
		nrf_gpio_cfg_output(BLE_STATUS_INDICATE_PIN);
		nrf_gpio_cfg_output(BLE_MODULE_WAKED_UP_REPLY_PIN);		
		nrf_gpio_cfg_output(9);
		
		OP_uart_data.len = 0;
		OP_uart_data.send_index = 0;
		OP_uart_data.send_len = 0;
		OP_uart_data.send_index_rf = 0;
		
		nrf_gpio_pin_clear(UART_DATA_SEND_PIN);		
  	nrf_gpio_pin_clear(BLE_STATUS_INDICATE_PIN);
		nrf_gpio_pin_set(BLE_MODULE_WAKED_UP_REPLY_PIN);
		nrf_gpio_pin_set(9);
//		app_uart_close();
		
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(uint32_t adv_inerval)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
		uint8_t m_addl_adv_manuf_data[17] = {0x00,0x00,0xC4,0xF1,0x00,0x0b,0x03,0x01};			//  0x01 0x03 ¡§o??-?¨¤?|¨¬?		

		ble_advdata_manuf_data_t   manuf_data;	
	 
		get_mac_addr(&m_addl_adv_manuf_data[11]);
	  manuf_data.company_identifier = 0x2211;
    manuf_data.data.size          = sizeof(m_addl_adv_manuf_data);
    manuf_data.data.p_data        = m_addl_adv_manuf_data;

				
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	  init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

		
		init.srdata.p_manuf_specific_data = &manuf_data;
		
		init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
		if((at_com_data.adver_interval>=32)&&(at_com_data.adver_interval<=16834))
		{
			init.config.ble_adv_fast_interval = at_com_data.adver_interval;
		}
		else	
		{	
			init.config.ble_adv_fast_interval = adv_inerval;
		}		
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;


    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void Mac_Addr_Set(void)
{
	 uint8_t i = 0;
	 uint8_t mac_addr[8];
	 uint32_t err_code;
	 ble_gap_addr_t gap_addr;
	 const uint8_t *addr = (uint8_t *)0x100010f8;
	
	 for(i=0;i<8;i++)
	 {
			mac_addr[i] = (uint8_t)*(addr);
			addr++;
	 }	
	 if((mac_addr[0]==0)&&(mac_addr[1]==0)&&(mac_addr[2]==0)&&(mac_addr[3]==0)&&(mac_addr[4]==0)&&(mac_addr[5]==0))
	 {
	    ;
	 }
	 else if((mac_addr[0]==0xFF)&&(mac_addr[1]==0xFF)&&(mac_addr[2]==0xFF)&&(mac_addr[3]==0xFF)&&(mac_addr[4]==0xFF)&&(mac_addr[5]==0xFF))
	 {
			;
	 }
	 else
	 {
			gap_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
			memcpy(gap_addr.addr, mac_addr, 6); 			
	 
			err_code = sd_ble_gap_addr_set(&gap_addr);
			APP_ERROR_CHECK(err_code);				        
	 }
}
#define BLE_REPLY_MCU_WAKE_UP_TIMEOUT_INTERVAL 					APP_TIMER_TICKS(10)
APP_TIMER_DEF(m_exter_wake_up_ble_delay_id);	
/**@brief Callback function for handling events from the button module.
 *
*/

/**@brief wake_up_timeout_handler     
 *
 * @param[in]   p_context
 *
 * @return      void
 */
static void wake_up_reply_timeout_handler(void * p_context)
{		
//		printf("wake_up_reply_timeout\r\n");
		nrf_gpio_pin_clear(BLE_MODULE_WAKED_UP_REPLY_PIN);			
}
/**@brief wake_up_timer_init     
 *
 * @param[in]   void
 *
 * @return      void
 */
void wake_up_reply_timer_init(void)
{
	app_timer_create(&m_exter_wake_up_ble_delay_id,
                   APP_TIMER_MODE_SINGLE_SHOT,
									 wake_up_reply_timeout_handler);
}
void app_uart_open(void)
{
    uint32_t                     err_code;
	//	flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));
    app_uart_comm_params_t  comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UARTE_BAUDRATE_BAUDRATE_Baud9600
    };
//		if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud9600)
//		{			
//				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud9600;
//		}
//		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud19200)
//		{			
//				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud19200;
//		}
//		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud38400)
//		{			
//				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud38400;
//		}
//		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud57600)
//		{			
//				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud57600;
//		}
//		else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud115200)
//		{			
//				comm_params.baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud115200;
//		}
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_HIGH,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief wake_up_timer_start     
 *
 *	Function for starting the wake_up timer.
 *
 * @param[in]   void
 *
 * @return      void
 */
void wake_up_reply_timer_start(void)
{
	app_timer_start(m_exter_wake_up_ble_delay_id, BLE_REPLY_MCU_WAKE_UP_TIMEOUT_INTERVAL,NULL);		
}
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
			if(nrf_gpio_pin_read(EXT_WAKE_UP_BLE_MODULE_PIN))
			{
						//printf("High!\r\n");	
						nrf_delay_ms(30);
						if(nrf_gpio_pin_read(EXT_WAKE_UP_BLE_MODULE_PIN))
						{	
//							app_uart_close();
							nrf_gpio_pin_set(BLE_MODULE_WAKED_UP_REPLY_PIN);
						}
			//			
			}	
			else if(!nrf_gpio_pin_read(EXT_WAKE_UP_BLE_MODULE_PIN))
			{
					// printf("Low!\r\n");	
						nrf_delay_ms(30);
						if(!nrf_gpio_pin_read(EXT_WAKE_UP_BLE_MODULE_PIN))
						{	
//							app_uart_open();	
							nrf_gpio_pin_clear(9);		
							
							//wake_up_reply_timer_start();
						}				

			}			
}
void in_mcu_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
			if(!nrf_gpio_pin_read(BLE_MODULE_ENABLE))
			{
				//	printf("pin low\r\n");
					uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
					APP_ERROR_CHECK(err_code);
			}			
}
/**
 * @brief Function for configuring: PIN_IN pin for input and configures GPIOTE to give an interrupt on pin change.

*/
#include "bsp_btn_ble.h"

#define BIN_ID_UART_SLEEP      	0
#define BIN_ID_UART_WORK      	0
#define BIN_ID_ADV_WORK	      	1
#define BIN_ACTION_UART_WORK   	BSP_BUTTON_ACTION_PUSH
#define BIN_ACTION_ADV_WORK   	BSP_BUTTON_ACTION_PUSH
#define BIN_ACTION_UART_SLEEP   BSP_BUTTON_ACTION_RELEASE


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
volatile uint8_t uart_status = 0;
void handle_uart_sta(void)
{
		switch(uart_status)
		{
			case 1:
							nrf_gpio_pin_set(9);
							uart_status = 0;
			        NRF_LOG_FLUSH();
							NRF_UARTE0->ENABLE = 0;

			break;
			case 2:
				  		nrf_gpio_pin_clear(9);
							uart_status = 0;
			        NRF_LOG_FLUSH();
							NRF_UARTE0->ENABLE = 8;
							app_uart_put(0x55);
							app_uart_put(0x66);
						//	printf("UART Start\r\n");
			break;				
		}	
	
}	
void bsp_event_handler(bsp_event_t event)
{
		uint32_t err_code ;
    switch (event)
    {
        case BSP_EVENT_UART_SLEEP:
					//		printf("uart_sleep\r\n");
						//	nrf_gpio_pin_set(9);
						//	NRF_LOG_FLUSH();
						//	NRF_UARTE0->ENABLE = 0;
							//app_uart_close();		
							uart_status = 1;
							nrf_gpio_pin_set(BLE_MODULE_WAKED_UP_REPLY_PIN);				
            break;

        case BSP_EVENT_UART_WORK:
						//	NRF_LOG_FLUSH();
						//	NRF_UARTE0->ENABLE = 8;
				  //    nrf_gpio_pin_clear(9);
							
			       // app_uart_open();
				     	// printf("uart_work....\r\n");	
				      uart_status = 2;	
							wake_up_reply_timer_start();																								
            break;
        case BSP_EVENT_ADV_WORK:
							//printf("m_advertising\r\n");	
							err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
							APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


static uint32_t uart_buttons_configure()
{
		uint32_t err_code;
    err_code = bsp_event_to_button_action_assign(BIN_ID_UART_SLEEP,
                                                 BIN_ACTION_UART_SLEEP,
                                                 BSP_EVENT_UART_SLEEP);
		APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(BIN_ID_UART_WORK,
                                                 BIN_ACTION_UART_WORK,
                                                 BSP_EVENT_UART_WORK);
		APP_ERROR_CHECK(err_code);	
	
    err_code = bsp_event_to_button_action_assign(BIN_ID_ADV_WORK,
                                                 BIN_ACTION_ADV_WORK,
                                                 BSP_EVENT_ADV_WORK);
		APP_ERROR_CHECK(err_code);		
	
    return NRF_SUCCESS;
}
static void button_init(void)
{

    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

		uart_buttons_configure();
		
}
uint32_t Send_Notify(uint8_t *send_data, uint16_t data_len)
{	
  	uint32_t err_code;
		ble_gatts_hvx_params_t hvx_params;
		memset(&hvx_params, 0, sizeof(hvx_params));	

//		hvx_params.handle = m_nus.tx_handles_indic.value_handle;
//		hvx_params.type   = BLE_GATT_HVX_INDICATION;
		hvx_params.handle = m_nus.tx_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.p_data = send_data;			
		hvx_params.p_len  = &data_len;	
		
//		printf("data_len:%d\r\n",*hvx_params.p_len);	
//		for(uint8_t i=0;i<(*hvx_params.p_len);i++)
//			printf("%02x ",hvx_params.p_data[i]);	
//		printf("\r\n");	
//		printf("Send_Notify\r\n");
		err_code = sd_ble_gatts_hvx(m_nus.conn_handle, &hvx_params);


		APP_ERROR_CHECK(err_code);
		return err_code;
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
static void timers_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);	
	  log_init();	
  	uart_receive_timer_init();
		PowerUp_write_timer_init();
		send_data_to_mcu_timer_init();
		wake_up_reply_timer_init();
}
/**@brief Function for placing the application in low power state while waiting for events.
 */
void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;

    // Initialize.
	  dtm_mode_check();	  
		timers_init();			 
		Mac_Addr_Set();
				 
	  ble_stack_init();
	 	
	  flash_init();
	  uart_init();
	  button_init();
    gap_params_init();

    gatt_init();	
    services_init(); 
    advertising_init(APP_ADV_INTERVAL);	
    conn_params_init();
		
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		printf("Application Start!\r\n");	  

    // Enter main loop.
    for (;;)
    {
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        power_manage();
				flash_operate(fs_event_num);
				handle_uart_sta();
    }
}


/**
 * @}
 */
