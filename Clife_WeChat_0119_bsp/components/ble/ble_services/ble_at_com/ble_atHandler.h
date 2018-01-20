#include "nrf52810_bitfields.h"
#include "stdint.h"
#include "stdbool.h"
#include "nrf52810.h"
//#include "pstorage.h"

#ifndef BLE_ATHANDLER_H
#define BLE_ATHANDLER_H

#define ROLE_SERVICE

#define TRUE   			1
#define FALSE       0
	
#define SUCCESS (TRUE)
#define FAILURE (FALSE)
	
#define MAC_LENGTH                                                6
#define UART_BR_LENGTH	                                          20
#define B_ADDR_LEN                               				  				6
	
	 /* MAX name len(20) + name len(1)*/
#define BLE_DEVICE_NAME_MAX_LENGTH 								  							20 
	
	   
#define UART_LOG_LEVEL 											 											LEVEL3_LOGS 
	
	
#define toupper(c) 			((c)>='a' && (c)<='z' ? (c)+('A'-'a'):(c))

	
	 /* uart rate*/
	typedef enum
	{
		UART_RATE_2K4	  = UARTE_BAUDRATE_BAUDRATE_Baud2400,
		UART_RATE_9K6	  = UARTE_BAUDRATE_BAUDRATE_Baud9600,
		UART_RATE_19K2	= UARTE_BAUDRATE_BAUDRATE_Baud19200,
		UART_RATE_38K4  = UARTE_BAUDRATE_BAUDRATE_Baud38400,
		UART_RATE_57K6	= UARTE_BAUDRATE_BAUDRATE_Baud57600,
		UART_RATE_115K2 = UARTE_BAUDRATE_BAUDRATE_Baud115200,
	}uart_rate_t; 
	/*********************************************************************
	 * 
	 *CONSTRUCT.
	 *
	 */
	
	typedef struct _uartData_t
	{ 
	  uint8_t src[28];	 
	} uartData_t;
	
	
	typedef struct _uartConfig_t
	{ 
	  uint32_t br;
	  uint16_t stop_bit;
	  uint16_t parity;
	  uint16_t flow_control;
	  uint16_t flow_control_threshold;
	} uartConfig_t;
	
	
	typedef struct	_conInterval_t
	{ 
	  uint16_t conInterval;
	  uint16_t latency;
	} conInterval_t;
	

typedef  struct 
{	
		
	uartConfig_t 		uart_cfg;																				/* uart config information*/		
	uint16_t				adver_interval;																	/* advering interval time*/
	conInterval_t 	con_interval ;																	/* connect interval time*/	
	uint16_t 				tx_power;																			/* tx power */	


	uint8_t         delay_time;	
	uint8_t		 			device_name[BLE_DEVICE_NAME_MAX_LENGTH+4];				/* device name */	
	
//	uint16_t 				adv_enable;																			/* advering switch flag*/		
//	uint16_t        bond_mode;
//	uint8_t 				passkey[8];																				/* pass key*/	
//	uint8_t 				btAddr[MAC_LENGTH];  														/* mac addr */	
//	uint8_t					DeviceType[24];
//	uint8_t					manufac_name[22];
//	uint8_t					serial_num[22];	
//	uint8_t					hardware_rev[22];
//	uint8_t					software_rev[22];
//	uint8_t					IEE_11073[22];
//	uint8_t 				encry_type[4];
//	uint8_t					pnp_data[22];		
}at_com_data_t;


//typedef  struct 
//{	
//	uint8_t					manufac_name[22];
//	uint8_t					serial_num[22];	
//	uint8_t					hardware_rev[22];
//	uint8_t					software_rev[22];
//	uint8_t					IEE_11073[22];
//	uint8_t 				encry_type[3];
//	uint8_t					pnp_data[22];
//}device_data_t;	


	
	/*********************************************************************
	 * 
	 *PUBLIC MACRO.
	 *
	 */
	
	//DEFAULT VALUES 
#define DEFAULT_BR				       								UARTE_BAUDRATE_BAUDRATE_Baud115200
#define DEFAULT_STP							  							1
#define DEFAULT_PRT							  							0
#define DEFAULT_FC							  							0
#define DEFAULT_FCT							  							0

#define DEFAULT_TX_POWER				  							0
#define DEFAULT_DELAY_TIME											10	
	
#define IOCAP_DEFAULT_MODE					  					3
#define DEFAULT_ADV_INTERBVAL				  					64
#define DEFAULT_ADV_ENABLE					  					1
#define DEFAULT_CONN_INTERVAL				  					75
#define DEFAULT_LATENCY						  						0
#define DEFAULT_DEVICE_NAME											DEVICE_NAME
#define DEFAULT_PASSKEY						  						0



#define DEFAULT_MANUFACTURER_NAME               MANUFACTURER_NAME                
#define DEFAULT_SERIAL_NUM				 							SERIAL_NUM
#define DEFAULT_HARD_WARE_REVERSION  						HARD_WARE_REVERSION
#define DEFAULT_SOFT_WARE_REVERSION  						SOFT_WARE_REVERSION
#define DEFAULT_PNP_VENDOR_ID_SOURCE						PNP_VENDOR_ID_SOURCE
#define DEFAULT_PNP_VENDOR_ID										PNP_VENDOR_ID
#define DEFAULT_PNP_PRODUCT_ID									PNP_PRODUCT_ID
#define DEFAULT_PNP_PRODUCT_VERSION							PNP_PRODUCT_VERSION

	

	
	/* toal nvid len for at cmd*/
#define BLE_NVID_USER_LEN					  (BLE_NVID_PASSKEY+(sizeof(uint32_t)))
	 
	
	//UART BRD
#define STR_UART_BR_9600 	 	"9600"
#define STR_UART_BR_19200	 	"19200"
#define STR_UART_BR_38400	 	"38400"
#define STR_UART_BR_57600		 "57600"
#define STR_UART_BR_115200   "115200"



//(accepted values are  -30, -20, -16, -12, -8, -4, 0, and 4 dBm)
#define TX_POWER_4dBm									"4"
#define TX_POWER_0dBm									"0"
#define TX_POWER_N4dBm								"-4"
#define TX_POWER_N8dBm								"-8"
#define TX_POWER_N12dBm								"-12"
#define TX_POWER_N16dBm								"-16"
#define TX_POWER_N20dBm								"-20"
#define TX_POWER_N30dBm								"-30"
#define TX_POWER_N40dBm								"-40"



typedef enum
{
		FS_IDLE,
    FS_STORE_EVT,   //!< Event for @ref fs_store.
    FS_ERASE_EVT    //!< Event for @ref fs_erase.
} fs_evt;


/* PARM ENUM*/
	typedef enum
	{
		CT_NVM_STATR_ADDR = 1,
			
	}ct_param_type_t;
	
extern at_com_data_t					at_com_data;
extern uint8_t App_state;

	
extern bool check_string(uint8_t *str);	
extern void Delayms(uint32_t t);
extern void StrToHex(uint8_t *pDest, uint8_t *pSrc, uint16_t pLen);
extern char *bdAddr2Str( uint8_t *pAddr );	
extern bool isBrCorrect_u16(uint32_t br);
extern bool isIncStr(uint8_t *buf);
extern bool isBrCorrect(uint8_t *brBuf);	
extern uint8_t str_cmp(uint8_t *p1,uint8_t *p2,uint8_t len);	
extern void IntToStr(uint8_t *des,uint32_t pData,uint8_t *len);
//extern void write_at_default_value(void);
extern uint32_t _atoi(char *str);	
extern uartData_t* getUartParamValue(uint8_t *buffer, uint16_t len, uint16_t* pNums);
extern void atCmddisponse(uint8_t *rx_data,uint16_t len);	
extern void  uart_put_string(char  *str);
extern uint8_t hexAssiVarilfy(uint8_t str);
	
extern uint8_t BLE_SetUartConfigHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetDeviceNameHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_SetTxPowerHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_SetBondModeHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_SetPassKeyHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_DisConnectHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_GetMacAddressHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_GetPeripheralRoleHandler(uint8_t *buffer,  uint16_t len);

extern uint8_t BLE_QueryUartConfigHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_QueryDeviceNameHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_QueryAdvIntervalHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_QueryConnIntervalHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_QueryTxPowerHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_QueryBondModeHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_QueryPassKeyHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_ModeSelectHandler(uint8_t *buffer, uint16_t len);


extern uint8_t BLE_ConnectHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_SetAdvIntervalHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetConnIntervalHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetAdvEnableHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_DisconConHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SystemResetHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_CentralRecoveryHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_PeriPheralRecoveryHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_QueryDeviceStateHandler(uint8_t *buffer, uint16_t len);
extern uint8_t BLE_PeriRssiHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QueryPeriPheralStateHandler(uint8_t *buffer, uint16_t len);


extern uint8_t BLE_SetDeviceTypeHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QueryDeviceTypeHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetManufactureNameHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetSerialNumberHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetHardwarRevHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetSoftwareRevHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetIEEE11073Handler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetPnpIdHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SetEncryTypeHandler( uint8_t *buffer, uint16_t len );
extern uint8_t BLE_QueryManufactureNameHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QuerySerialNumberHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QueryHardwarRevHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QuerySoftwareRevHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QueryPnpIdHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QueryIEEE11073Handler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_QueryEncryTypeHandler(uint8_t *buffer,  uint16_t len);
extern uint8_t BLE_SwitchToBootloaderHandler(uint8_t *buffer,uint16_t len);
extern uint8_t BLE_UartResponseHandler( uint8_t *buffer, uint16_t len );
extern uint8_t BLE_SetUartDelayTime( uint8_t *buffer, uint16_t len );
extern uint8_t BLE_GetUartDelayTime( uint8_t *buffer, uint16_t len );



extern void delay_us(uint32_t volatile number_of_us);
extern uint8_t Str_turn_to_16(char str);
extern char *arrayToStr( uint8_t *pAddr );;
extern void get_mac_addr(uint8_t *p_mac_addr);


#endif


