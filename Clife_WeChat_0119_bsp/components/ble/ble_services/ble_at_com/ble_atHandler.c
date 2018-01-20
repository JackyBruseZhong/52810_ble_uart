#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "ble_atHandler.h"
#include "stdio.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_error.h"
#include "ble_gap.h"
#include "app_error.h"
#include "ble_hci.h"
#include "nrf52810.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "nrf_delay.h"
#include "OP_main.h"
#include "app_uart.h"
#include "AT_ComHandler.h"

#define   PSTORAGE_BLOCK_SIZE            160
#define   PSTORAGE_BLOCK_NUM							2

#define   PSTORAGE_BLOCK_0								0
#define   PSTORAGE_BLOCK_1								1
#define   PSTORAGE_BLOCK_2								2
#define   PSTORAGE_BLOCK_3								3



uint32_t ble_passkey = 0;

typedef uint8_t( *atcmd_callback)(uint8_t *buffer, uint16_t len);
//custom struct format use to create at commond and compare.
typedef struct _AT_CMD_T{
	 char commond[9];
     atcmd_callback atcmd_CB;
}AT_CMD_T;


const AT_CMD_T ui_atcmd[] = { 
	{"AT+CA=", 			BLE_SetUartConfigHandler				},	
	{"AT+CB=", 			BLE_SetDeviceNameHandler				},
	{"AT+CC=", 			BLE_SetAdvIntervalHandler				}, 
	{"AT+CD=", 			BLE_SetConnIntervalHandler			},
	{"AT+CE=", 			BLE_SetTxPowerHandler						},
	{"AT+CF=", 			BLE_SetBondModeHandler					},
	{"AT+CG=", 			BLE_SetPassKeyHandler						},
	{"AT+CH=", 			BLE_SetAdvEnableHandler					},
	{"AT+CI=", 			BLE_PeriRssiHandler							},
	{"AT+CX=",     	BLE_SetDeviceTypeHandler         },
	
	{ "AT+MA=",     BLE_SetManufactureNameHandler   },
	{ "AT+MB=",     BLE_SetSerialNumberHandler      },
	{ "AT+MC=",     BLE_SetHardwarRevHandler        },
	{ "AT+MD=",     BLE_SetSoftwareRevHandler       },
	{ "AT+MF=",     BLE_SetIEEE11073Handler         },
	{ "AT+MG=",     BLE_SetPnpIdHandler             },
	{ "AT+MH=",			BLE_SetEncryTypeHandler					},	
	{ "AT+ME=", 		BLE_SetUartDelayTime						},
	
	{ "AT+MA?\r\n", BLE_QueryManufactureNameHandler },
	{ "AT+MB?\r\n", BLE_QuerySerialNumberHandler    },
	{ "AT+MC?\r\n", BLE_QueryHardwarRevHandler      },
	{ "AT+MD?\r\n", BLE_QuerySoftwareRevHandler     },
	{ "AT+MG?\r\n", BLE_QueryPnpIdHandler           },
	{ "AT+MF?\r\n", BLE_QueryIEEE11073Handler       },
	{	"AT+MH?\r\n",	BLE_QueryEncryTypeHandler				},
	{ "AT+ME?\r\n", BLE_GetUartDelayTime},
	
	{"AT+CK?\r\n", BLE_GetMacAddressHandler					},
	{"AT+CL?\r\n", BLE_GetPeripheralRoleHandler			},
	{"AT+CA?\r\n", BLE_QueryUartConfigHandler				},	
	{"AT+CB?\r\n", BLE_QueryDeviceNameHandler				},
	{"AT+CC?\r\n", BLE_QueryAdvIntervalHandler			},
	{"AT+CD?\r\n", BLE_QueryConnIntervalHandler			},
	{"AT+CE?\r\n", BLE_QueryTxPowerHandler					},
	{"AT+CF?\r\n", BLE_QueryBondModeHandler					},
	{"AT+CG?\r\n", BLE_QueryPassKeyHandler					},
	{"AT+CO\r\n",  BLE_SystemResetHandler						},	
	{"AT+CJ\r\n",  BLE_DisconConHandler							},	
	{"AT+CP\r\n",	 BLE_PeriPheralRecoveryHandler		},
	{"AT+CQ?\r\n", BLE_QueryPeriPheralStateHandler 	},	
	{"AT+CX?\r\n", BLE_QueryDeviceTypeHandler},
	{"AT\r\n", 		 BLE_UartResponseHandler} 	
};


extern uint16_t  m_conn_handle;		


/* at cmd numbers */
#define AT_NUMBERS 					(sizeof(ui_atcmd)/sizeof(ui_atcmd[0]))

//static  pstorage_handle_t     m_flash_handle;
//static  pstorage_handle_t	  	block_0_handle;
//static  pstorage_handle_t	  	block_1_handle;

//static uint8_t pstorage_wait_flag = 0;
//static pstorage_block_t pstorage_wait_handle = 0;	
static bool rssi_start = false;	

at_com_data_t			at_com_data;
//device_data_t			at_com_data;
	
//uint8_t DeviceNameLen = 0;
uint8_t App_state = 0;

void delay_us(uint32_t volatile number_of_us)
{
    while(number_of_us != 0)
    {
        number_of_us--;       
    }
}



void atCmddisponse(uint8_t *rx_data,uint16_t len)
{ 
	uint8_t atNumber=0;
  uint16_t acCmdCnt = 0; 
	
	for(atNumber=0;atNumber<AT_NUMBERS;atNumber++)
	{
		acCmdCnt = strlen(ui_atcmd[atNumber].commond); 

 		if(len >= acCmdCnt)
		{
			if(memcmp(rx_data,ui_atcmd[atNumber].commond,acCmdCnt)==0)
			{
				((ui_atcmd[atNumber]).atcmd_CB)(rx_data,acCmdCnt);

				break;
			}
		}
	}
}


/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      isContain
 *
 * @brief  judge whether including "\r\n"
 *
 * @param  buffer
 *
 * @return  bool-TRUE/FALSE
 */
bool isContain(uint8_t *buffer)
{
	uint8_t i;
	uint8_t cnt = strlen((const char*)buffer); 
	for(i=0; i < cnt ; i++)
	{
		if(buffer[i]=='\r' && buffer[i+1]=='\n')
		{
			return TRUE;
		}
	}

	return FALSE;
}



/*********************************************************************
 * @fn      getUartParamValue
 *
 * @brief  get UART Param Value.
 *
 * @param   buffer-len-pNums
 *
 * @return  uartData_t
 */ 
uartData_t* getUartParamValue(uint8_t *buffer, uint16_t len, uint16_t* pNums)
{
	uint16_t i,k = 0;
	int index = 0;
	uint8_t tmpBuf[41]={0};
	uartData_t uartData[6];
  uartData_t* pt = uartData;
	
	if(!isContain(buffer))
	{
		return NULL;
	}	
	for(i=len; i<strlen((const char*)buffer); i++)
	{
		if(buffer[i] == ',' || (buffer[i]=='\r'&&buffer[i+1]=='\n'))
		{
			tmpBuf[k] = '\0';
			memcpy(uartData[index++].src,tmpBuf,k+1);
			k = 0;
			continue;
		}
		tmpBuf[k++] = buffer[i];
	}
	*pNums = index;

	return pt;
	
}

uint8_t Str_turn_to_16(char str)
{

  uint8_t t;

  if (str>='A' && str <='F')
   t = str-55;
	else if (str>='a' && str <='f')
   t = str-87;
  else if(str>='0' && str <='9')
   t = str-48;

  return t;
} 

/*********************************************************************
 * @fn      isBrCorrect
 *
 * @brief  judge whether UART BR is  Legal
 *
 * @param  brBuf
 *
 * @return  bool-TRUE/FALSE
 */

bool isBrCorrect(uint8_t *brBuf)
{
	bool isCorrect = TRUE;
	
//	printf("compare %s",brBuf);
//	printf("\r\n");
	if(memcmp(brBuf,STR_UART_BR_9600,4)!=0 && 
		memcmp(brBuf,STR_UART_BR_19200,5)!=0 && 
		memcmp(brBuf,STR_UART_BR_38400,5)!=0 && 
		memcmp(brBuf,STR_UART_BR_57600,5)!=0&& 
		memcmp(brBuf,STR_UART_BR_115200,6)!=0)
	{
		isCorrect = FALSE;
	}

	return isCorrect;
}

/*********************************************************************
 * @fn      isBrCorrect
 *
 * @brief  judge whether UART BR is  Legal
 *
 * @param  brBuf
 *
 * @return  bool-TRUE/FALSE
 */
bool isBrCorrect_u16(uint32_t br)
{
	bool isCorrect = TRUE;
	
	if(br!=UART_RATE_2K4&&
		br!=UART_RATE_9K6&&
		br!=UART_RATE_19K2&&
		br!=UART_RATE_38K4&&
		br!=UART_RATE_57K6&&
		br!=UART_RATE_115K2)
	{
		isCorrect = FALSE;
	}

	return isCorrect;
}

/*********************************************************************
 * @fn      getCurrentBr
 *
 * @brief  get UARTBaud Rate
 *
 * @param   brBuf
 *
 * @return  uint8_t
 */

uint32_t ConvertBr(uint8_t *brBuf)
{
	uint32_t rt = 0;  
  if(memcmp(brBuf, STR_UART_BR_9600,4)==0){
		rt = UART_RATE_9K6;
	 
	}else if(memcmp(brBuf,STR_UART_BR_19200,5)==0){
		rt = UART_RATE_19K2;
	 
	}else if(memcmp(brBuf,STR_UART_BR_38400,5)==0){
		rt = UART_RATE_38K4;
	 
	}else if(memcmp(brBuf,STR_UART_BR_57600,5)==0){
		rt = UART_RATE_57K6;
	 
	}else if(memcmp(brBuf,STR_UART_BR_115200,6)==0){
		rt = UART_RATE_115K2;
	 
	}
	return rt;
}
int8_t ConvertTxPow(uint8_t *TxPowBuf)
{
	int8_t rt = 0;  
	if(memcmp(TxPowBuf, TX_POWER_4dBm,1)==0){
		rt = 4;
	 
	}else if(memcmp(TxPowBuf, TX_POWER_0dBm,1)==0){
		rt = 0;
	 
	}else if(memcmp(TxPowBuf,TX_POWER_N40dBm,3)==0){
		rt = -40;		
	}
	else if(memcmp(TxPowBuf,TX_POWER_N4dBm,2)==0){
		rt = -4;
	}else if(memcmp(TxPowBuf,TX_POWER_N8dBm,2)==0){
		rt = -8;
	 
	}else if(memcmp(TxPowBuf,TX_POWER_N12dBm,3)==0){
		rt = -12;
	 
	}else if(memcmp(TxPowBuf,TX_POWER_N16dBm,3)==0){
		rt = -16;
	}else if(memcmp(TxPowBuf,TX_POWER_N20dBm,3)==0){
		rt = -20;
	}else if(memcmp(TxPowBuf,TX_POWER_N30dBm,3)==0){
		rt = -30;		
	}
	return rt;
}



/*********************************************************************
 * @fn      _atoi
 *
 * @brief   Convert String into interger
 *
 * @param   str - point a String
 *
 * @return  none
 */ 
uint32_t _atoi(char *str)
{
    uint32_t value = 0;
    while(*str>='0' && *str<='9')
    {
        value *= 10;
        value += *str - '0';
        str++;
    }
    return value;
}


/*********************************************************************
 * @fn      str_cmp
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */ 
 uint8_t str_cmp(uint8_t *p1, uint8_t *p2,uint8_t len)
 { 
   uint8_t i=0;
   while(i<len){
	 if(p1[i]!=p2[i])
	   return 1;
	 i++;
   }
   return 0;
 }


/*********************************************************************
 * @fn		isIncStr
 *
 * @brief 	check if the target buf including "".
 *
 * @param	buf - target buf
 *
 * @return	TRUE/FALSE
 */

bool isIncStr(uint8_t *buf)
{
	uint8_t i;
	for(i=0; i<strlen((const char*)buf); i++)
	{
		if(buf[i] == '"')
		{
			return TRUE;
		}
	}

	return FALSE;
}


/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8_t *pAddr )
{
  uint8_t       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[15];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += (B_ADDR_LEN);

  for ( i = B_ADDR_LEN; i > 0; i-- )
  { 
  	*pStr++ = hex[((uint8_t)(*--pAddr) >> 4) &0x0F];
    *pStr++ = hex[(uint8_t)*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
char *arrayToStr( uint8_t *pAddr )
{
  uint8_t       i,len;
  char        hex[] = "0123456789ABCDEF";
  static char str[50];
  char        *pStr = str;
  *pStr++ = '0';
  *pStr++ = 'x';
	 len = strlen((char *)pAddr);

  for ( i = 0; i < len; i++ )
  { 
  	*pStr++ = hex[((uint8_t)(*pAddr) >> 4) &0x0F];
    *pStr++ = hex[(uint8_t)*pAddr & 0x0F];
		pAddr++;
	}

  *pStr = 0;

  return str;
}
char *u8ToStr( uint8_t p_des_data )
{
  char        hex[] = "0123456789ABCDEF";
  static char str[10];
  char        *pStr = str;

	*pStr++ = hex[((uint8_t)(p_des_data) >> 4) &0x0F];
	*pStr++ = hex[(uint8_t)p_des_data & 0x0F];

  return str;
}

uint8_t hexAssiVarilfy(uint8_t str)
{
	static const  char  hex[] = "0123456789ABCDEF";
	uint8_t retValue = 0xff;
	uint16_t cnt = 0;
  	for(cnt = 0; cnt <16; cnt++){
	  	if(str == hex[cnt]){
			retValue = cnt;
			break;
		}
	}
	return retValue;
}
bool check_string(uint8_t *str)
{
	uint8_t i;
	uint8_t cnt = strlen((const char*)str); 
	for(i=0; i < cnt+1; i++)
	{
		//printf("i=%d\r\n",i);
		if(str[i]=='\0' )
		{
			//printf("find it\r\n");
			return TRUE;
		}
	}
	return FALSE;
}


/*********************************************************************
 * @fn      xStrToHex
 *
 * @brief   Convert string to hex
 *
 *  pLen --- 
 * @return  none
 */
uint8_t xStrToHex(uint8_t *pHex, uint8_t *pStr, uint16_t strLen)
{
  uint16_t  i;
  uint8_t  value = 0;  
  strLen /=2;
  for ( i = 0; i < strLen; i++ )
  { 
		value = hexAssiVarilfy(*pStr++);
		if(value == 0xff){

			return 0;
		} 
		*pHex = (value<<4);
		value = hexAssiVarilfy(*pStr++);
		if(value == 0xff){

			return 0;
		}  
		*pHex |= value; 
		pHex++;
  }  
  return 1;
}
/*********************************************************************
 * @fn      HexToStr
 *
 * @brief   Convert het to string
 *
 *  pLen --- 
 * @return  none
 */
void xHexToStr(uint8_t *pStr, uint8_t *pHex, uint16_t pLen)
{
	static const  char  hex[] = "0123456789ABCDEF";
  uint8_t       i; 
  for ( i = 0; i < pLen; i++ )
  { 
  	*pStr = hex[((*pHex) >> 4) &0x0F];
			pStr++;
         *pStr = hex[(*pHex) & 0x0F];
		pStr++;
	pHex++;
  }  
}



/*********************************************************************
 * @fn      StrToHex
 *
 * @brief   Convert String to Hex
 *
 * @return  none
 */
void StrToHex(uint8_t *pDest, uint8_t *pSrc, uint16_t pLen)
{
	char h1,h2;
	uint8_t s1,s2;
	uint16_t i;

	for (i=0; i<pLen; i++)
	{
		h1 = pSrc[2*i];
		h2 = pSrc[2*i+1];

		s1 = toupper(h1) - 0x30;
		if (s1 > 9) 
			s1 -= 7;
		s2 = toupper(h2) - 0x30;
		if (s2 > 9) 
			s2 -= 7;
		pDest[5-i] = s1*16 + s2;
	}
}

/*********************************************************************
 * @fn      StrToHex
 *
 * @brief   Convert interger to String
 *
 * @return  none
 */
void IntToStr(uint8_t *des,uint32_t pData,uint8_t *len)
 {
 //	static uint8_t str[16];
	uint8_t strTmp[16];
	uint8_t cnt=0;
	uint8_t i=0;
	*len = 0;
     
	do{
		strTmp[cnt] = pData%10;
		strTmp[cnt++] += '0'; 
		pData=pData/10;
	}while(pData); 
	*len = cnt;

	for(i = 0; i<cnt;i++){
		des[i] = strTmp[cnt-i-1];
	}
	
}
void  uart_put_string(char  *str)
{
	char  *p;

	p = str;
	while(*p)
	{
		app_uart_put(*p++);	
	}
}

uint8_t BLE_SetUartConfigHandler(uint8_t *buffer, uint16_t len)
{

	uint32_t rt;
	uint16_t num;
	uartData_t *brData = NULL; 
	uartConfig_t uConfig;
	
	
	brData = getUartParamValue(buffer, len, &num);
	if(brData == NULL)
	{  
		return FAILURE;
	}

	if(!isBrCorrect(brData[0].src))
	{
		return FAILURE;
	}
	
	uConfig.flow_control_threshold = _atoi((char*)brData[4].src);	
	uConfig.flow_control = _atoi((char*)brData[3].src);	
	uConfig.parity = _atoi((char*)brData[2].src);	
	uConfig.stop_bit = _atoi((char*)brData[1].src);		
		//Convert UART BR
	rt = ConvertBr(brData[0].src);
	if(rt==0)
	{
		uart_put_string("ERROR\r\n");
		return FAILURE;
	}
	uConfig.br = rt;	
	
	at_com_data.uart_cfg.br											= uConfig.br;
	at_com_data.uart_cfg.stop_bit									= uConfig.stop_bit;		
	at_com_data.uart_cfg.parity										= uConfig.parity;
	at_com_data.uart_cfg.flow_control							= uConfig.flow_control;
	at_com_data.uart_cfg.flow_control_threshold		= uConfig.flow_control_threshold;

//	printf("REC uartConfig: 0x%08x 0x%x 0x%x 0x%x 0x%x",at_com_data.uart_cfg.baud,
//																				at_com_data.uart_cfg.stop_bit,
//																				at_com_data.uart_cfg.parity,
//																				at_com_data.uart_cfg.flow_control,
//																				at_com_data.uart_cfg.flow_control_threshold);
//		printf("\r\n");	
	uart_put_string("OK\r\n");	

	return SUCCESS;
}
uint8_t BLE_SetDeviceNameHandler(uint8_t *buffer,  uint16_t len)
{
	uint8_t  name[25];
	uint8_t  DeviceNameLen;
	uint16_t num;
	uartData_t *dnData = NULL;

	
	dnData = getUartParamValue(buffer, len, &num);
	if(dnData == NULL)
	{
		return FAILURE;
	}
	if(num != 1)
	{
		return FAILURE;
	}
	
	DeviceNameLen = strlen((char*)dnData[0].src);
	memcpy(name,dnData[0].src,strlen((char*)dnData[0].src));
	
	if(DeviceNameLen > 20)
	{
		uart_put_string("ERROR\r\n");		
		return FAILURE;
	}
	else
	{	
		memcpy((uint8_t *)&at_com_data.device_name[1],name,DeviceNameLen);
		
		at_com_data.device_name[DeviceNameLen+1] = '\0';
		at_com_data.device_name[0] = DeviceNameLen;

		uart_put_string("OK\r\n");	

		return SUCCESS;																						
	}
}

uint8_t BLE_SetTxPowerHandler(uint8_t *buffer,  uint16_t len)
{
	int8_t txPower;
	uint16_t num;
	uartData_t *tpData = NULL;
	
	
	tpData = getUartParamValue(buffer, len, &num);
	if(tpData == NULL)
	{
		return FAILURE;
	}
	if((num == 0)||(num >3))
	{
		return FAILURE;
	}
	txPower = ConvertTxPow(tpData[0].src);
	if(txPower <=4 )															/*  max power is 4 */
	{
		at_com_data.tx_power = txPower;
		uart_put_string("OK\r\n");	

		return SUCCESS;			
	}
	else
	{
			uart_put_string("ERROR\r\n");
			return FAILURE;	
	}
}


uint8_t BLE_SetBondModeHandler(uint8_t *buffer,  uint16_t len)
{
//	uint16_t num;
//	uartData_t *tpData = NULL;
//	
//	tpData = getUartParamValue(buffer, len, &num);
//	if(tpData == NULL)
//	{

//		return FAILURE;
//	}
//	if(num != 1)
//	{
//		return FAILURE;
//	}
//	uint8_t caplity = _atoi((char*)tpData[0].src);
//	if(((caplity==0))||((caplity>0)&&(caplity<5)))
//	{
//		at_com_data.bond_mode = caplity;
//		uart_put_string("OK\r\n");	

		return SUCCESS;	
//	}
//	else
//	{
//			uart_put_string("ERROR\r\n");
//			return FAILURE;	
//	}
}

uint8_t BLE_SetAdvEnableHandler(uint8_t *buffer,  uint16_t len)
{
//	uint16_t  num;
//	uint32_t  err_code;	
//	uartData_t *advData = NULL;
////	ble_gap_adv_params_t adv_params;
//	
//	advData = getUartParamValue(buffer, len, &num);
//	if(advData == NULL)
//	{
//		return FAILURE;
//	}
//	if(num != 1)
//	{
//		return FAILURE;
//	}
//	uint8_t advStatus = _atoi((char*)advData[0].src);
//	if(advStatus >1)
//	{
//		return FAILURE;
//	}

//	at_com_data.adv_enable = advStatus;

//		if(at_com_data.adv_enable==1&&App_state==0)
//		{
////    		err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
//    		APP_ERROR_CHECK(err_code);			
//			  uart_put_string("+IS:1\r\n");
//					
//		}
//		else if(at_com_data.adv_enable==0&&App_state==1)
//		{
//    		err_code = sd_ble_gap_adv_stop();
//				App_state = 0;
//    		APP_ERROR_CHECK(err_code);		
//			  uart_put_string("+IS:0\r\n");
//					
//		}
//		else
//		{
//			uart_put_string("+IS:");
//			app_uart_put(App_state+'0');
//			uart_put_string("\r\n");
//		}
//		uart_put_string("OK\r\n");	

		return SUCCESS;	
}

uint8_t BLE_DisconConHandler(uint8_t *buffer,  uint16_t len)
{
	uint32_t  err_code;	
     
	err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	APP_ERROR_CHECK(err_code);
	if(err_code == NRF_SUCCESS)
	{
			uart_put_string("+IS:0\r\n");
			return SUCCESS;
	}		

	return FAILURE;	
}

void get_mac_addr(uint8_t *p_mac_addr)
{
		uint32_t error_code;
		ble_gap_addr_t *p_mac_addr_t = (ble_gap_addr_t*)malloc(sizeof(ble_gap_addr_t));
		error_code = sd_ble_gap_addr_get(p_mac_addr_t);
		APP_ERROR_CHECK(error_code);
		uint8_t *d = p_mac_addr_t->addr;
	//	printf("Get addr:");
		for ( uint8_t i = 6; i >0;)
		{	
			i--;
			p_mac_addr[5-i]= d[i];
		//	printf("0x%02x ",p_mac_addr[5-i]);
		}
		free(p_mac_addr_t);
		p_mac_addr_t = NULL;
	//	printf("\r\n");
}
uint8_t BLE_GetMacAddressHandler(uint8_t *buffer,  uint16_t len)
{
	char *string;
	ble_gap_addr_t mac_addr; 
	
	memset(&mac_addr,0,sizeof(mac_addr));
	get_mac_addr(mac_addr.addr);
	uart_put_string("+CK:");
	string = bdAddr2Str(mac_addr.addr);
	uart_put_string(string);
	uart_put_string("\r\n");

  return SUCCESS;	
}

uint8_t BLE_PeriRssiHandler(uint8_t *buffer,  uint16_t len)
{
	int8_t  rssi_value_N = 0;
	uint8_t string[50],str_len,rssi_value;
	memset(string,0,sizeof(string));
	uint16_t num;
	uint32_t err_code;
	uartData_t *pData = NULL;

	pData = getUartParamValue(buffer, len, &num);
	if(pData == NULL)
	{
		return FAILURE;
	}
	if(num != 1)
	{
		return FAILURE;
	}
	uint8_t cmd = (uint8_t)_atoi((char*)pData[0].src);

	if( cmd > 1)
	{
		uart_put_string("ERROR\r\n");
		return FAILURE;
	}
	if(cmd == 1)
	{
		if(!rssi_start)
		{
			 err_code = sd_ble_gap_rssi_start(m_conn_handle,2,5);

			 if(err_code == NRF_SUCCESS)
			 {				
					nrf_delay_ms(100);
					err_code = sd_ble_gap_rssi_get(m_conn_handle,&rssi_value_N);
					if(err_code == NRF_SUCCESS)
					{	
							rssi_start = true;
							uart_put_string("+CI:");
							rssi_value = 0-rssi_value_N;
							string[0]='-';
							IntToStr(&string[1],rssi_value,&str_len);
							uart_put_string((char *)string);
							uart_put_string("\r\n");
						  return SUCCESS;
					}							 
		   }
		}
		else
		{
				err_code = sd_ble_gap_rssi_get(m_conn_handle,&rssi_value_N);
				if(err_code == NRF_SUCCESS)
				{	
						
						uart_put_string("+CI:");
						rssi_value = 0-rssi_value_N;
						string[0]='-';
						IntToStr(&string[1],rssi_value,&str_len);
						uart_put_string((char *)string);
						uart_put_string("\r\n");
						return SUCCESS;
				}	

		}	

	}
	else if(cmd == 0)
	{
		err_code = sd_ble_gap_rssi_stop(m_conn_handle);
		if(err_code == NRF_SUCCESS)
		{
				rssi_start = false;
				uart_put_string("OK\r\n");
				return SUCCESS;
		}	
	}
	else
	{
		uart_put_string("ERROR\r\n");
		return FAILURE;	
	}
	return FAILURE;
}

uint8_t BLE_SetAdvIntervalHandler(uint8_t *buffer,  uint16_t len)
{
	uint16_t num;
	uartData_t *pData = NULL;
	
	pData = getUartParamValue(buffer, len, &num);
	if(pData == NULL)
	{
		return FAILURE;
	}
	uint16_t advInterval = _atoi((char*)pData[0].src);
	
	if((advInterval>=32)&&(advInterval<=16384))
	{
		at_com_data.adver_interval = advInterval;
		uart_put_string("OK\r\n");	
	
		return SUCCESS;				
	}
	else
	{
				uart_put_string("ERROR\r\n");
				return FAILURE;	
	}
}


uint8_t BLE_SetConnIntervalHandler(uint8_t *buffer,  uint16_t len)
{
//	uint16_t num;
//	uint16_t tmpValue;
//	conInterval_t pConData;
//	uartData_t *connData = NULL;
//		

//	connData = getUartParamValue(buffer, len, &num);
//	if(connData == NULL)
//	{
//		return FAILURE;
//	}
//	
//	pConData.latency = _atoi((char*)connData[1].src);
//	pConData.conInterval = _atoi((char*)connData[0].src);
//	if(tmpValue == 0)
//	{
//		return FAILURE;
//	}
//	if((pConData.conInterval>=16)&&(pConData.conInterval<=3200))
//	{
//		at_com_data.con_interval.conInterval = pConData.conInterval;
//		at_com_data.con_interval.latency = pConData.latency;
//		uart_put_string("OK\r\n");	
//	
//		return SUCCESS;				
//	}
//	else
//	{
//		uart_put_string("ERROR\r\n");
		return FAILURE;	
//	}
}

uint8_t BLE_SetPassKeyHandler(uint8_t *buffer,  uint16_t len)
{
//	uint8_t  passkey[15],buffer_len=0;
//	uint16_t num;
//	uartData_t *pData = NULL;
//	
//	pData = getUartParamValue(buffer, len, &num);
//	if(pData == NULL)
//	{
//		return FAILURE;
//	}
//	if(num != 1)
//	{
//		return FAILURE;
//	}

//	buffer_len = strlen((char*)pData[0].src);
//	if(( num != 1 )||(buffer_len>6)||(memcmp(pData[0].src, "000000",buffer_len)==0))
//	{
//		uart_put_string("ERROR\r\n");
//		return FAILURE;
//	}
//	memcpy(passkey,pData[0].src,buffer_len);

//	at_com_data.passkey[0] = buffer_len;
//	for(uint8_t i=1;i<7;i++)
//	{
//		at_com_data.passkey[i]=Str_turn_to_16(passkey[i-1]);
//	}

//	uart_put_string("OK\r\n");	

	return SUCCESS;				

}
uint8_t BLE_GetPeripheralRoleHandler(uint8_t *buffer,  uint16_t len)
{
	uart_put_string("+CL:0\r\n");	
	return SUCCESS;
}


uint8_t BLE_SystemResetHandler(uint8_t *buffer, uint16_t len)
{
		uint32_t err_code;
		err_code = sd_nvic_SystemReset();
		if(err_code == NRF_SUCCESS)
		{
			;
		}	
	  return SUCCESS;
}
uint8_t BLE_PeriPheralRecoveryHandler(uint8_t *buffer, uint16_t len)
{
//	uint8_t deviceName[] = DEFAULT_DEVICE_NAME;
//	uint8_t manu_name[] = DEFAULT_MANUFACTURER_NAME;
//	uint8_t seria_num[] = DEFAULT_SERIAL_NUM;
//	uint8_t hard_ver[] = DEFAULT_HARD_WARE_REVERSION;
//	uint8_t soft_ver[] = DEFAULT_SOFT_WARE_REVERSION;

//	memset(&at_com_data,0,sizeof(at_com_data_t));

//	at_com_data.uart_cfg.br											= DEFAULT_BR;
//	at_com_data.uart_cfg.stop_bit									= DEFAULT_STP;		
//	at_com_data.uart_cfg.parity										= DEFAULT_PRT;
//	at_com_data.uart_cfg.flow_control							= DEFAULT_FC;
//	at_com_data.uart_cfg.flow_control_threshold		= DEFAULT_FCT;
//	
//	at_com_data.device_name[0] = strlen(DEFAULT_DEVICE_NAME);
//	memcpy(&at_com_data.device_name[1],deviceName,strlen((char*)deviceName));
//	
//	at_com_data.tx_power = DEFAULT_TX_POWER;
//	
//	at_com_data.adver_interval = DEFAULT_ADV_INTERBVAL;
//	at_com_data.adv_enable = DEFAULT_ADV_ENABLE;
//	
//	at_com_data.con_interval.conInterval = DEFAULT_CONN_INTERVAL;
//	at_com_data.con_interval.latency = DEFAULT_LATENCY;
//		
//	flash_clear();		
	
	return SUCCESS;	
}

uint8_t BLE_QueryPeriPheralStateHandler(uint8_t *buffer, uint16_t len)
{
	
	if(App_state == 0)
	{
		uart_put_string("+IS:0\r\n");
		return SUCCESS;
	}
	else 	if(App_state == 1)
	{
		uart_put_string("+IS:1\r\n");
		return SUCCESS;
	}
	else 	if(App_state == 2)
	{
		uart_put_string("+IS:2\r\n");
		return SUCCESS;
	}
	return FAILURE;
}
uint8_t BLE_QueryDeviceNameHandler(uint8_t *buffer, uint16_t len)
{
	char device_name[50],*string;
	uint8_t device_addr[6];
	
	memset(device_name,0,sizeof(device_name));
	memset(device_addr,0,sizeof(device_addr));	
	memset(at_com_data.device_name,0,sizeof(at_com_data.device_name));
//	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
	if((at_com_data.device_name[0]>0)&&(at_com_data.device_name[0]<22))
	{		
		uart_put_string((char *)&at_com_data.device_name[1]);
		uart_put_string("\r\n");	
		uart_put_string("OK\r\n");	
	}
	else if(at_com_data.device_name[0]==0xFF)
	{

		get_mac_addr(device_addr);
		memcpy(device_name,DEVICE_NAME,strlen(DEVICE_NAME));
		device_name[strlen(DEVICE_NAME)] = '-';
		
		string = arrayToStr(&device_addr[4]);
		if(string!=NULL)
			memcpy(&device_name[strlen(DEVICE_NAME)+1],&string[2],4);		
		uart_put_string(device_name);
		uart_put_string("\r\n");	
		uart_put_string("OK\r\n");		
	}
	else
	{
		return FAILURE;
	}	
	return SUCCESS;
}
uint8_t BLE_QueryUartConfigHandler(uint8_t *buffer, uint16_t len)
{
	uint8_t string[50],str_len;
	memset(string,0,sizeof(string));
	memset(&at_com_data.uart_cfg,0,sizeof(at_com_data.uart_cfg));	
//	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
	uart_put_string("+CA:");
	
	if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud9600)
	{			
			uart_put_string("9600,");
	}
	else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud19200)
	{			
			uart_put_string("19200,");
	}
	else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud38400)
	{			
			uart_put_string("38400,");
	}
	else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud57600)
	{			
			uart_put_string("57600,");
	}
	else if(at_com_data.uart_cfg.br == UARTE_BAUDRATE_BAUDRATE_Baud115200)
	{			
			uart_put_string("115200,");
	}
	else
	{
		uart_put_string("9600,0,0,1,0\r\n");	
		uart_put_string("OK\r\n");		
		return SUCCESS;
	}
	app_uart_put(at_com_data.uart_cfg.stop_bit+0x30);
	app_uart_put(',');	
	app_uart_put(at_com_data.uart_cfg.parity+0x30);
	app_uart_put(',');	
	app_uart_put(at_com_data.uart_cfg.flow_control+0x30);
	app_uart_put(',');	
//	app_uart_put(at_com_data.uart_cfg.flow_control_threshold+0x30);
	IntToStr(string,at_com_data.uart_cfg.flow_control_threshold,&str_len);
	uart_put_string((char *)string);
	uart_put_string("\r\n");	
	uart_put_string("OK\r\n");	
	return SUCCESS;
}
uint8_t BLE_QueryAdvIntervalHandler(uint8_t *buffer, uint16_t len)
{
	uint16_t  adv_interv = APP_ADV_INTERVAL;
	uint8_t string[50],str_len;
	memset(string,0,sizeof(string));
	memset(&at_com_data.adver_interval,0,sizeof(at_com_data.adver_interval));	
//	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data_t));
	if((at_com_data.adver_interval>=32)&&(at_com_data.adver_interval<=16384))
	{	
		uart_put_string("+CC:");
    IntToStr(string,at_com_data.adver_interval,&str_len);
		uart_put_string((char *)string);
		uart_put_string("\r\n");
		uart_put_string("OK\r\n");		
	}
	else if(at_com_data.adver_interval==0xFFFF)
	{
		uart_put_string("+CC:");
		IntToStr(string,adv_interv,&str_len);
		uart_put_string((char *)string);
		uart_put_string("\r\n");
		uart_put_string("OK\r\n");
	}
	else
	{
		return FAILURE;
	}	
	return SUCCESS;
}
uint8_t BLE_QueryConnIntervalHandler(uint8_t *buffer, uint16_t len)
{
//	uint16_t con_intver = MIN_CONN_INTERVAL;
//	uint8_t latency = SLAVE_LATENCY;
//	uint8_t string[50],str_len;
//	memset(string,0,sizeof(string));	
//	memset(&at_com_data.con_interval,0,sizeof(at_com_data.con_interval));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.con_interval.conInterval>15)&&(at_com_data.con_interval.conInterval<3201))
//	{	
//		uart_put_string("+CD:");
//    IntToStr(string,at_com_data.con_interval.conInterval,&str_len);
//		string[str_len] = ',';
//		IntToStr(&string[str_len+1],at_com_data.con_interval.latency,&str_len);
//		uart_put_string((char *)string);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n");
//	}
//	else if(at_com_data.con_interval.conInterval==0xFFFF)
//	{	
//		uart_put_string("+CD:");
//    IntToStr(string,con_intver,&str_len);
//		string[str_len] = ',';
//		IntToStr(&string[str_len+1],latency,&str_len);
//		uart_put_string((char *)string);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n");	
//	}	
//	else
//	{
//		return FAILURE;
//	}	
	return SUCCESS;
}
uint8_t BLE_QueryTxPowerHandler(uint8_t *buffer, uint16_t len)
{
//	uint8_t txPower = 0;
//	uint8_t string[10],str_len;
//	memset(string,0,sizeof(string));		
//	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if(at_com_data.tx_power == (-40)||
//		(at_com_data.tx_power == (-30))||
//		(at_com_data.tx_power == (-20))||
//		(at_com_data.tx_power == (-16))||
//		(at_com_data.tx_power == (-12))||
//		(at_com_data.tx_power == (-8))||
//		(at_com_data.tx_power == (-4))||
//		(at_com_data.tx_power ==0 )||
//		(at_com_data.tx_power ==4 ))
//	{	
//		uart_put_string("+CE:");
//		if(at_com_data.tx_power>=0)
//		{
//				IntToStr(string,at_com_data.tx_power,&str_len);
//		}
//		else
//		{
//				txPower = 0-at_com_data.tx_power;
//				string[0]='-';
//				IntToStr(&string[1],txPower,&str_len);
//		}
//		uart_put_string((char *)string);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n");	
//	}
//	else
//	{
//		uart_put_string("+CE:");
//		uart_put_string("0\r\n");
//		uart_put_string("OK\r\n");	
//		return FAILURE;
//	}
	return SUCCESS;
}
uint8_t BLE_QueryBondModeHandler(uint8_t *buffer, uint16_t len)
{	
//	uint8_t string[10],str_len;
//	memset(string,0,sizeof(string));		
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if(((at_com_data.bond_mode==0))||((at_com_data.bond_mode>0)&&(at_com_data.bond_mode<0x05)))
//	{	
//		uart_put_string("+CF:");
//		IntToStr(string,at_com_data.bond_mode,&str_len);
//		uart_put_string((char *)string);
//		uart_put_string("\r\n");
//	}
//	else if(at_com_data.bond_mode==0xFF)
//	{
//		uart_put_string("+CF:3\r\n");	
//	}
//	else
//	{
//		return FAILURE;
//	}	
	return SUCCESS;
}

uint8_t BLE_QueryPassKeyHandler(uint8_t *buffer, uint16_t len)
{		
//	uint8_t string[10],str_len;
//	memset(string,0,sizeof(string));	
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.passkey[0]>0)&&(at_com_data.passkey[0]<0x7))
//	{	
//		ble_passkey = at_com_data.passkey[1]*100000+at_com_data.passkey[2]*10000+at_com_data.passkey[3]*1000+at_com_data.passkey[4]*100+at_com_data.passkey[5]*10+at_com_data.passkey[6];
//		uart_put_string("+CG:");
//		IntToStr(string,ble_passkey,&str_len);
//		uart_put_string((char *)string);
//		uart_put_string("\r\nOK\r\n");
//	}
//	else if(at_com_data.passkey[0]==0xFF)
//	{
//		uart_put_string("+CG:");
//		uart_put_string("000000\r\n");	
//		uart_put_string("OK\r\n");			
//	}
//	else
//	{
//		return FAILURE;
//	}	
	return SUCCESS;
}
/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_SetDeviceTypeHandler( uint8_t *buffer, uint16_t len )
{
//	uint8_t     buffer_len,DeviceType[15];
//	uint16_t		num;
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}
//	buffer_len = strlen((char*)uart_data[0].src);
//	if(( num != 1 )||(buffer_len>22))
//	{
//		return FAILURE;
//	}
//	
//	memcpy(DeviceType,uart_data[0].src,buffer_len);
//	DeviceType[buffer_len] = '\0';

//	memset(at_com_data.DeviceType,0,sizeof(at_com_data.DeviceType));
//	xStrToHex(&at_com_data.DeviceType[1],DeviceType,buffer_len);
//	at_com_data.DeviceType[0] = buffer_len;

//	uart_put_string("OK\r\n");	

	return SUCCESS;	
}
uint8_t BLE_SwitchToBootloaderHandler(uint8_t *buffer, uint16_t len)
{	
//	uint32_t res;	

//	sd_power_gpregret_get(0,&res);
//	printf("res=0x%08x\r\n",res);
	sd_power_gpregret_set(0,0xB1);
//	sd_power_gpregret_get(0,&res);
//	printf("res=0x%08x\r\n",res);
//	sd_power_gpregret_clr(0,1);
//	sd_power_gpregret_get(0,&res);
//	printf("res=0x%08x\r\n",res);	
//	uart_put_string("OK\r\n");
	NVIC_SystemReset();
	return SUCCESS;
}
/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QueryDeviceTypeHandler( uint8_t *buffer, uint16_t len )
{
//	char *string = NULL;
//	memset(at_com_data.DeviceType,0,sizeof(at_com_data.DeviceType));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	
//	if((at_com_data.DeviceType[0]>0)&&(at_com_data.DeviceType[0]<23))
//	{
//		uart_put_string( "+CX:" );
//		string = arrayToStr((uint8_t *)&at_com_data.DeviceType[1]);
//	//	xHexToStr((uint8_t *)string,(uint8_t *)&at_com_data.DeviceType[1],at_com_data.DeviceType[0]/2);
//		uart_put_string(string);
//		uart_put_string("\r\nOK\r\n" );
//	}
//	else if(at_com_data.DeviceType[0]==0xFF)
//	{
//			uart_put_string( "0000000000000000000000\r\n" );
//			uart_put_string( "OK\r\n" );	
//	}
	return SUCCESS;	
}

/*************************************************************************
 * ??????
 */
uint8_t BLE_SetManufactureNameHandler( uint8_t *buffer, uint16_t len )
{
//	uint8_t 	buffer_len;
//	uint8_t   mau_name[25];
//	uint16_t	num;
//	
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}	
//	buffer_len = strlen((char*)uart_data[0].src);
//	if(( num != 1 )||(buffer_len>20))
//	{
//		return FAILURE;
//	}
//	
//	memcpy(mau_name,uart_data[0].src,buffer_len);
//	memset(at_com_data.manufac_name,0,sizeof(at_com_data.manufac_name));
//	memcpy(&at_com_data.manufac_name[1],mau_name,buffer_len);	
//	at_com_data.manufac_name[buffer_len+1] = '\0';
//	at_com_data.manufac_name[0] = buffer_len;

	uart_put_string("OK\r\n");	
	
	return SUCCESS;	
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QueryManufactureNameHandler( uint8_t *buffer, uint16_t len )
{
//	memset(at_com_data.manufac_name,0,sizeof(at_com_data.manufac_name));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.manufac_name[0]>0)&&(at_com_data.manufac_name[0]<0xFF))
//	{
//		uart_put_string( "+MA:" );
//		uart_put_string((char *)&at_com_data.manufac_name[1]);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}
//	else if(at_com_data.manufac_name[0]==0xFF)
//	{
//		uart_put_string( "+MA:" );
////		uart_put_string(MANUFACTURER_NAME);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}
//	else
//	{
//		return FAILURE;
//	}	
	return SUCCESS;
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_SetSerialNumberHandler( uint8_t *buffer, uint16_t len )
{
//	uint8_t   buffer_len;
//	uint8_t 	seial_num[25];
//	uint16_t	num;
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}	
//	buffer_len = strlen((char*)uart_data[0].src);
//	if(( num != 1 )||(buffer_len>20))
//	{
//		return FAILURE;
//	}

//	memcpy(seial_num,uart_data[0].src,buffer_len);

//	memset(at_com_data.serial_num,0,sizeof(at_com_data.serial_num));
//	memcpy(&at_com_data.serial_num[1],seial_num,buffer_len);
//	at_com_data.serial_num[buffer_len+1] = '\0';
//	at_com_data.serial_num[0] = buffer_len;
	
	uart_put_string("OK\r\n");	

	return SUCCESS;	
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QuerySerialNumberHandler( uint8_t *buffer, uint16_t len )
{
//	memset(at_com_data.serial_num,0,sizeof(at_com_data.serial_num));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.serial_num[0]>0)&&(at_com_data.serial_num[0]<0xFF))
//	{	
//		uart_put_string( "+MB:" );
//		uart_put_string((char *)&at_com_data.serial_num[1]);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}
//	else if(at_com_data.serial_num[0]==0xFF)
//	{
//		uart_put_string( "+MB:" );
////		uart_put_string(SERIAL_NUM);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}
	return SUCCESS;
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_SetHardwarRevHandler( uint8_t *buffer, uint16_t len )
{
//	uint8_t buffer_len;
//	uint8_t hard_ver[20];
//	uint16_t	num;
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}
//	buffer_len = strlen((char*)uart_data[0].src);	
//	if(( num != 1 )||(buffer_len>20))
//	{
//		return FAILURE;
//	}

//	memcpy(hard_ver,uart_data[0].src,buffer_len);

//	memset(at_com_data.hardware_rev,0,sizeof(at_com_data.hardware_rev));
//	memcpy(&at_com_data.hardware_rev[1],hard_ver,buffer_len);
//	at_com_data.hardware_rev[buffer_len+1]='\0';
//	at_com_data.hardware_rev[0] = buffer_len;

//	uart_put_string("OK\r\n");	

	return SUCCESS;	
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QueryHardwarRevHandler( uint8_t *buffer, uint16_t len )
{
//	memset(at_com_data.hardware_rev,0,sizeof(at_com_data.hardware_rev));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.hardware_rev[0]>0)&&(at_com_data.hardware_rev[0]<0xFF))
//	{
//		uart_put_string( "+MC:" );
//		uart_put_string((char *)&at_com_data.hardware_rev[1]);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}
//	else if(at_com_data.hardware_rev[0]==0xFF)
//	{
//		uart_put_string( "+MC:" );
////		uart_put_string(HARD_WARE_REVERSION);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}	
	return SUCCESS;
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_SetSoftwareRevHandler( uint8_t *buffer, uint16_t len )
{
//	uint8_t     buffer_len,soft_ver[20];
//	uint16_t		num;
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}
//	buffer_len = strlen((char*)uart_data[0].src);	
//	if(( num != 1 )||(buffer_len>20))
//	{
//		return FAILURE;
//	}
//	
//	memcpy(soft_ver,uart_data[0].src,buffer_len);

//	memset(at_com_data.software_rev,0,sizeof(at_com_data.software_rev));
//	memcpy(&at_com_data.software_rev[1],soft_ver,buffer_len);
//	at_com_data.software_rev[buffer_len+1] = '\0';
//	at_com_data.software_rev[0] = buffer_len;
	
	uart_put_string("OK\r\n");	

	return SUCCESS;	
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QuerySoftwareRevHandler( uint8_t *buffer, uint16_t len )
{
//	memset(at_com_data.software_rev,0,sizeof(at_com_data.software_rev));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.software_rev[0]>0)&&(at_com_data.software_rev[0]<0xFF))
//	{
//		uart_put_string( "+MD:" );
//		uart_put_string((char *)&at_com_data.software_rev[1]);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}
//	else if(at_com_data.software_rev[0]==0xFF)
//	{
//		uart_put_string( "+MD:" );
////		uart_put_string(SOFT_WARE_REVERSION);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}		
	return SUCCESS;
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_SetIEEE11073Handler( uint8_t *buffer, uint16_t len )
{
//	uint8_t     buffer_len,iee_data[25];
//	uint16_t		num;
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}
//	buffer_len = strlen((char*)uart_data[0].src);
//	if(( num != 1 )||(buffer_len>20))
//	{
//		return FAILURE;
//	}
//	
//	memcpy(iee_data,uart_data[0].src,buffer_len);
//	
//	memset(at_com_data.IEE_11073,0,sizeof(at_com_data.IEE_11073));
//	xStrToHex(&at_com_data.IEE_11073[1],iee_data,buffer_len);
//	at_com_data.IEE_11073[0] = buffer_len;
	
	uart_put_string("OK\r\n");	

	return SUCCESS;	
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QueryIEEE11073Handler( uint8_t *buffer, uint16_t len )
{
//	char *string = NULL;
//	uint8_t reg_cert_data_arry[20]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
//	memset(at_com_data.IEE_11073,0,sizeof(at_com_data.IEE_11073));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.IEE_11073[0]>0)&&(at_com_data.IEE_11073[0]<0xFF))
//	{
//		uart_put_string( "+MF:" );
//		string = arrayToStr((uint8_t *)&at_com_data.IEE_11073[1]);
//		uart_put_string(string);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}
//	else if(at_com_data.IEE_11073[0]==0xFF)
//	{
//		uart_put_string( "+MF:" );
//		string = arrayToStr(reg_cert_data_arry);
//		uart_put_string(string);
//		uart_put_string("\r\n");
//		uart_put_string("OK\r\n" );
//	}			
	return SUCCESS;
}

uint8_t BLE_SetEncryTypeHandler( uint8_t *buffer, uint16_t len )
{
//	uint8_t     buffer_len,encry_type_data[25];
//	uint16_t		num;
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}
//	buffer_len = strlen((char*)uart_data[0].src);
//	if(( num != 1 )||(buffer_len>1))
//	{
//		return FAILURE;
//	}
//	
//	memcpy(encry_type_data,uart_data[0].src,buffer_len);
//	
//	memset(at_com_data.encry_type,0,sizeof(at_com_data.encry_type));
//	memcpy(&at_com_data.encry_type[1],encry_type_data,buffer_len);
//	at_com_data.encry_type[buffer_len+1] = '\0';
//	at_com_data.encry_type[0] = buffer_len;
//	
//	uart_put_string("OK\r\n");	

	return SUCCESS;	
}
uint8_t BLE_SetUartDelayTime( uint8_t *buffer, uint16_t len )
{
//	uint8_t     buffer_len,encry_type_data[25];
	

	return SUCCESS;	
}

uint8_t BLE_GetUartDelayTime( uint8_t *buffer, uint16_t len )
{
	
	return SUCCESS;
}
/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QueryEncryTypeHandler( uint8_t *buffer, uint16_t len )
{
//	memset(at_com_data.encry_type,0,sizeof(at_com_data.encry_type));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	if((at_com_data.encry_type[0]>0)&&(at_com_data.encry_type[0]<0xFF))
//	{
//		if(memcmp(&at_com_data.encry_type[1], "0",1)==0)
//			at_com_data.encry_type[1] = 0x0;
//		else if(memcmp(&at_com_data.encry_type[1], "1",1)==0)
//			at_com_data.encry_type[1] = 0x01;
//		uart_put_string( "+MH:" );	
//		app_uart_put(at_com_data.encry_type[1]+0x30);
//		uart_put_string( "\r\nOK\r\n" );
//	}
//	else if(at_com_data.encry_type[0]==0xFF)
//	{
//		uart_put_string( "+MH:" );
//		uart_put_string("0\r\n");
//		uart_put_string( "OK\r\n" );	
//	}			
	return SUCCESS;
}

uint8_t BLE_SetPnpIdHandler( uint8_t *buffer, uint16_t len )
{
//	uint8_t     iee_data[25];
//	uint16_t		num;
//	uartData_t	*uart_data = NULL;

//	uart_data = getUartParamValue( buffer, len, &num );
//	if( uart_data == NULL )
//	{
//		return FAILURE;
//	}

//	memcpy(iee_data,uart_data[0].src,2);
//	memcpy(&iee_data[2],uart_data[1].src,4);
//	memcpy(&iee_data[6],uart_data[2].src,4);
//	memcpy(&iee_data[10],uart_data[3].src,4);

//	memset(at_com_data.pnp_data,0,sizeof(at_com_data.pnp_data));
//	memcpy(&at_com_data.pnp_data[1],iee_data,14);
//	at_com_data.pnp_data[15] = '\0';
//	at_com_data.pnp_data[0] = 15;
//		
//	uart_put_string("OK\r\n");	

	return SUCCESS;	
}

/***********************************************************
* Function:       // ????
* Description:    // ???????????
* Input:          // 1.????1,??,????????????????????
* Input:          // 2.????2,??,????????????????????
* Output:         // 1.????1,??
* Return:         // ????????
* Others:         // ????
***********************************************************/
uint8_t BLE_QueryPnpIdHandler( uint8_t *buffer, uint16_t len )
{	
//	memset(at_com_data.pnp_data,0,sizeof(at_com_data.pnp_data));
////	flash_read((uint32_t *)&at_com_data,sizeof(at_com_data));
//	
//	if((at_com_data.pnp_data[0]>0)&&(at_com_data.pnp_data[0]<0xFF))
//	{	
//		uart_put_string( "+MG:" );	
//		app_uart_put(at_com_data.pnp_data[1]);
//		app_uart_put(at_com_data.pnp_data[2]);
//		app_uart_put(',');
//		app_uart_put(at_com_data.pnp_data[3]);
//		app_uart_put(at_com_data.pnp_data[4]);
//		app_uart_put(at_com_data.pnp_data[5]);
//		app_uart_put(at_com_data.pnp_data[6]);		
//		app_uart_put(',');
//		app_uart_put(at_com_data.pnp_data[7]);
//		app_uart_put(at_com_data.pnp_data[8]);
//		app_uart_put(at_com_data.pnp_data[9]);
//		app_uart_put(at_com_data.pnp_data[10]);		
//		app_uart_put(',');
//		app_uart_put(at_com_data.pnp_data[11]);
//		app_uart_put(at_com_data.pnp_data[12]);
//		app_uart_put(at_com_data.pnp_data[13]);
//		app_uart_put(at_com_data.pnp_data[14]);		
//	
//		uart_put_string( "\r\nOK\r\n" );
//	}
//	else if(at_com_data.pnp_data[0]==0xFF)
//	{	
//		uart_put_string( "+MG:" );	
//		uart_put_string(u8ToStr(PNP_VENDOR_ID_SOURCE));	
//		app_uart_put(',');
//		uart_put_string(u8ToStr((PNP_VENDOR_ID>>8)&0xFF));	
//		uart_put_string(u8ToStr(PNP_VENDOR_ID&0x00FF));			
//		app_uart_put(',');
//		uart_put_string(u8ToStr((PNP_PRODUCT_ID>>8)&0xFF));	
//		uart_put_string(u8ToStr(PNP_PRODUCT_ID&0x00FF));				
//		app_uart_put(',');
//		uart_put_string(u8ToStr((PNP_PRODUCT_VERSION>>8)&0xFF));	
//		uart_put_string(u8ToStr(PNP_PRODUCT_VERSION&0x00FF));				
//		uart_put_string( "\r\nOK\r\n" );	
//	}
	return SUCCESS;
}
uint8_t BLE_UartResponseHandler( uint8_t *buffer, uint16_t len )
{
	uart_put_string("OK\r\n");	
	return SUCCESS;
}

//void write_at_default_value(void)
//{
//	uint8_t DeviceNameLen;
//	uint32_t err_code;
//	
//	uint8_t deviceName[21] = DEFAULT_DEVICE_NAME;

//	memset(&at_com_data,0,sizeof(at_com_data_t));
//	
//	at_com_data.uart_cfg.baud											= DEFAULT_BR;
//	at_com_data.uart_cfg.stop_bit									= DEFAULT_STP;		
//	at_com_data.uart_cfg.parity										= DEFAULT_PRT;
//	at_com_data.uart_cfg.flow_control							= DEFAULT_FC;
//	at_com_data.uart_cfg.flow_control_threshold		= DEFAULT_FCT;
//	
//	DeviceNameLen = strlen((char *)deviceName);
//	at_com_data.device_name[0] = DeviceNameLen;
//	memcpy(&at_com_data.device_name[1],deviceName,DeviceNameLen);
//	at_com_data.device_name[DeviceNameLen+1] = '\0';
//	
//	at_com_data.tx_power = DEFAULT_TX_POWER;
//	
//	at_com_data.adver_interval = DEFAULT_ADV_INTERBVAL;
//	at_com_data.adv_enable = DEFAULT_ADV_ENABLE;
//	
//	at_com_data.con_interval.conInterval = DEFAULT_CONN_INTERVAL;
//	at_com_data.con_interval.latency = DEFAULT_LATENCY;
//	
//		

//	at_com_data.encry_type[0] = '1';
//	at_com_data.encry_type[1] = '0';
//	at_com_data.encry_type[2] = '\0';
//	at_com_data.adv_enable = 1;
//		
//	flash_clear();	
//	
//	err_code = write_at_com_data(0);
//	APP_ERROR_CHECK(err_code);
//	err_code = write_at_com_data(1);
//	APP_ERROR_CHECK(err_code);	
//	
//	if(err_code== NRF_SUCCESS)
//	{
//		nrf_delay_ms(300);
//	
//	}	

//	
//}


/************************END***************************************************/

