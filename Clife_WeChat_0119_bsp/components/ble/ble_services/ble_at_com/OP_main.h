#ifndef OP_MAIN_H__
#define OP_MAIN_H__

#include "ble_atHandler.h"
//#include "ble_wechat_service.h"
//#include "het_open_platform.h"

#define UART_DATA_COMMAND_MODE_PIN					6
#define UART_DATA_SEND_PIN									20
#define EXT_WAKE_UP_BLE_MODULE_PIN  				31
#define BLE_MODULE_WAKED_UP_REPLY_PIN       12
#define BLE_MODULE_ENABLE										16//21
#define BLE_STATUS_INDICATE_PIN             11

#define DTM_MODE_PIN 												20
#define DTM_GREEN_LED            						10


#define RX_PIN_NUMBER  4
#define TX_PIN_NUMBER  3
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           true

#define DEVICE_NAME                     "Ble-Device"                           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "HET-Intelligent-Control"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM          							"H&T-BT52810"
#define SERIAL_NUM				 							"123456789"
#define HARD_WARE_REVERSION  						"H&T-V1.0.0"
#define SOFT_WARE_REVERSION  						"V1.0.0"
#define FIRM_WARE_REVERSION  						"V1.0.0-180119"

#define MANUFACTURER_ID                 0x5544332211                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x887766                                   /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */


#define APP_ADV_INTERVAL                160                                    /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                      /**< The advertising timeout (in units of seconds). */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection i nterval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (40 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0       
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define PNP_VENDOR_ID_SOURCE						0x01
#define PNP_VENDOR_ID										0x0302
#define PNP_PRODUCT_ID									0x0504
#define PNP_PRODUCT_VERSION							0x0706

#define IEE_11073_DATA									0x102030405060708090													/** IEEE 11073-20601 Regulatory Certification Data */
#define MTU_SIZE												251

void OP_sys_init(void);
void OP_flash_init(void);
void OP_uart_init(void);	
void OP_ble_init(void);
void main_func(void);
void wechat_main_func(void);
void uart_init(void);
void flash_operate(uint8_t event);



//extern ble_wechat_t 		m_ble_wechat;
extern bool   uart_enable;	
extern bool PowerUp_uart_rec;

#endif 
