/******************************************************************************
 *  FILE
 *      lrd_at.c
 *
 *  DESCRIPTION
 *      This file keeps information related to Discovered Battery service
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *===========================================================================*/
#include <string.h>

#include "lrd_at.h"
#include "ble_atHandler.h"
#include "OP_main.h"
#include "nrf_nvic.h"
#include "app_uart.h"
#include "ble_gap.h"
#include "AT_ComHandler.h"

/*============================================================================*
 *  Private Definitions
 *===========================================================================*/

#define LRD_AT_SET          0x01
#define LRD_AT_GET          0x04
#define LRD_AT_HEAD         0xFC
#define LRD_OUT_MAX_LEN     32
#define PRINT_SUCCE_LEN     4
#define PRINT_FAIL_LEN      4
/*============================================================================*
 *  Private Data Types
 *===========================================================================*/
typedef uint8_t(* krd_at_handler)(uint8_t *buffer, uint16_t len);


typedef struct _LRD_AT_CMD {
    uint8_t atId;
    krd_at_handler atHandler;
} LRD_AT_CMD;

typedef enum {
    LRD_AT_SET_BR = 1,                      // 波特率设定
    LRD_AT_GET_BR,                          // 获取波特率
    LRD_AT_SET_CONNINTERVAL ,               // 连接间隔设定
    LRD_AT_GET_CONNINTERVAL,                // 查询连接间隔
    LRD_AT_SET_PAIR_KEY,                    // 设置配对密码
    LRD_AT_GET_PAIR_KEY,                    // 获取配对密码
    LRD_AT_GET_MAC_ADDR,                    // 获取mac 地址
    LRD_AT_SET_DEVICE_NAME,                 // 设置设备名称
    LRD_AT_GET_DEVICE_NAME,                 // 获取设备名称
    LRD_AT_SET_ENABLE_PAIR,                 // 配对使能设置
    LRD_AT_GET_ENABLE_PAIR,                 // 查询配对使能
    LRD_AT_SET_ADV_INTERVAL,                // 设置广播间隔
    LRD_AT_GET_ADV_INTERVAL,                // 获取广播间隔
    LRD_AT_SET_TX_POWER,                    // 设置发送功率
    LRD_AT_GET_TX_POWER,                    // 获取发送功率
    LRD_AT_SET_UART_DELAY,                  // 设置UART 延时
    LRD_AT_GET_UART_DELAY,                  // 获取uart 延时

    LRD_AT_FACTORY = 0x80,                  // 恢复出厂
    LRD_AT_RESET = 0x81,                    // 模块复位
    LRD_AT_GET_FW_VERSION = 0xF0,           // 获取FW 版本号

} lrd_at_t;


/*============================================================================*
 *  Private Data
 *============================================================================*/

static LRD_AT_CMD lrd_atCmd[] = {

    {LRD_AT_SET_BR,           La_SetBr },
    {LRD_AT_GET_BR,           La_GetBr },
    {LRD_AT_SET_CONNINTERVAL, La_SetConnInterval },
    {LRD_AT_GET_CONNINTERVAL, La_GetConnInterval },
//      {LRD_AT_SET_PAIR_KEY,     La_SetPairKey },
//      {LRD_AT_GET_PAIR_KEY,     La_GetPairKey },
    {LRD_AT_GET_MAC_ADDR,     La_GetMacAddr },
    {LRD_AT_SET_DEVICE_NAME,  La_SetDeviceName },
    {LRD_AT_GET_DEVICE_NAME,  La_GetDeviceName },
//      {LRD_AT_SET_ENABLE_PAIR,  La_SetEnablePair },
//      {LRD_AT_GET_ENABLE_PAIR,  La_GetEnablePair },
    {LRD_AT_SET_ADV_INTERVAL, La_SetAdvInterval },
    {LRD_AT_GET_ADV_INTERVAL, La_GetAdvInterval },
    {LRD_AT_SET_TX_POWER,     La_SetTxPower },
    {LRD_AT_GET_TX_POWER,     La_GetTxPower },
    {LRD_AT_SET_UART_DELAY,   La_SetUartDelay },
    {LRD_AT_GET_UART_DELAY,   La_GetUartDelay },
    {LRD_AT_FACTORY,          La_Factory },
    {LRD_AT_RESET,            La_Reset },
    {LRD_AT_GET_FW_VERSION,   La_GetFwVersion },

};
/* at cmd numbers */
#define LRD_AT_NUMBERS      (sizeof(lrd_atCmd)/sizeof(lrd_atCmd[0]))


static uint8_t printBuffer[LRD_OUT_MAX_LEN];
static uint8_t PRINT_SUCCE[PRINT_SUCCE_LEN] = {LRD_AT_GET, LRD_AT_HEAD, 0x01, 0x00};
static uint8_t PRINT_FAIL[PRINT_FAIL_LEN] = {LRD_AT_GET, LRD_AT_HEAD, 0x02, 0x00};

#define LR_PRINTF_SUCCE()  SerialPrintHex(PRINT_SUCCE,PRINT_SUCCE_LEN)
#define LR_PRINTF_FAIL()   SerialPrintHex(PRINT_FAIL,PRINT_FAIL_LEN)

/*============================================================================*
 *  Public Data
 *============================================================================*/

/*============================================================================*
 *  Private Function Prototypes
 *===========================================================================*/
static bool la_chekCmd(uint8_t cmd, uint8_t* atIndex);


/*============================================================================*
 *  Private Function Implementations
 *===========================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *    SerialPrintHex
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/

void SerialPrintHex(uint8_t *data, uint8_t len)
{
		for(uint8_t i=0;i<len;i++)
			app_uart_put(data[i]);
}	

/*----------------------------------------------------------------------------*
 *  NAME
 *    la_chekCmd
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
static bool la_chekCmd(uint8_t cmd, uint8_t* atIndex)
{
    uint8_t atNumber = 0;

    for(atNumber = 0; atNumber < LRD_AT_NUMBERS; atNumber++) {
        if(lrd_atCmd[atNumber].atId == cmd) {
            *atIndex = atNumber;
            return TRUE;
        }
    }
    return FALSE;
}


/*============================================================================*
 *  Public Function Implementations
 *===========================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *     La_SetBr
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetBr(uint8_t *buffer, uint16_t len)
{
    uartConfig_t pConfig = {DEFAULT_BR, DEFAULT_STP, DEFAULT_PRT};

    // 判断命令字长度
    if(buffer[0] != 1 || len < 2) {
        return LRD_FAIL;
    }

    // 判断命令字
    switch(buffer[1]) {
        case 0:
            pConfig.br = UART_RATE_2K4;
            break;
#if 0
        case 1:
            pConfig.br = UART_RATE_4K8;
            break;
#endif
        case 2:
            pConfig.br = UART_RATE_9K6;
            break;

        case 3:
            pConfig.br = UART_RATE_19K2;
            break;
			
        case 4:
						pConfig.br = UART_RATE_38K4;
						break;

        case 5:
            pConfig.br = UART_RATE_57K6;
            break;

        case 6:
            pConfig.br = UART_RATE_115K2;
            break;

        default:
            pConfig.br = 0;
            break;

    }

    if(pConfig.br == 0) {
        return LRD_FAIL;
    }

    saveUartConfig(&pConfig);
//		printf("Set UART OK\r\n");
    LR_PRINTF_SUCCE();
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *     La_GetBr
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetBr(uint8_t *buffer, uint16_t len)
{
//    uartConfig_t* pConfig;
    if(buffer[0] != 0 ) {
        return LRD_FAIL;
    }

    getUartConfig();

    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_BR;
    printBuffer[3] = 1;             //数据长度


    switch(at_com_data.uart_cfg.br) {
        case UART_RATE_2K4:
            printBuffer[4] = 0;
            break;
#if 0
        case UART_RATE_4K8:
            printBuffer[4] = 1;
            break;
#endif
        case UART_RATE_9K6:
            printBuffer[4] = 2;
            break;

        case UART_RATE_19K2:
            printBuffer[4] = 3;
            break;

        case UART_RATE_38K4:
			printBuffer[4] = 4;
			break;
			
        case UART_RATE_57K6:
            printBuffer[4] = 5;
            break;

        case UART_RATE_115K2:
            printBuffer[4] = 6;
            break;

        default:
            printBuffer[4] = 0xFF;
            break;

    }
    if(printBuffer[4] == 0xFF) {

        return LRD_FAIL;
    }

    SerialPrintHex(printBuffer, 5);
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *   La_SetConnInterval
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetConnInterval(uint8_t *buffer, uint16_t len)
{
    conInterval_t  connInterval;
    if(buffer[0] != 2 || len < 3) {
        return LRD_FAIL;
    }
    connInterval.latency = 0;
    connInterval.conInterval = buffer[2] << 8 | (buffer[1] & 0xff);
    saveConnInterval(&connInterval);

    LR_PRINTF_SUCCE();
    return LRD_SUCCE;

}

/*----------------------------------------------------------------------------*
 *  NAME
 *   La_GetConnInterval
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetConnInterval(uint8_t *buffer, uint16_t len)
{
    if(buffer[0] != 0 ) {
        return LRD_FAIL;
    }
    getConnInterval();
    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_CONNINTERVAL;
    printBuffer[3] = 2;             //数据长度
    printBuffer[4] = (uint8_t)(at_com_data.con_interval.conInterval & 0x00ff);
    printBuffer[5] = (uint8_t)((at_com_data.con_interval.conInterval >> 8) & 0xff);
    SerialPrintHex(printBuffer, 6);
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *   La_SetPairKey
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetPairKey(uint8_t *buffer, uint16_t len)
{
    if(buffer[0] != 6 || len < 7) {
        return LRD_FAIL;
    }
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *   La_GetPairKey
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetPairKey(uint8_t *buffer, uint16_t len)
{
    if(buffer[0] != 0 ) {
        return LRD_FAIL;
    }
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *    La_GetMacAddr
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetMacAddr(uint8_t *buffer, uint16_t len)
{
		ble_gap_addr_t mac_addr; 

    if(buffer[0] != 0  ) {
        return LRD_FAIL;
    }
		memset(&mac_addr,0,sizeof(mac_addr));

    if(sd_ble_gap_addr_get(&mac_addr)!= NRF_SUCCESS) {
        return FAILURE;
    }
//		printf("addr=%08x",NRF_FICR->DEVICEADDR[0]);
//		printf("addr=%08x",NRF_FICR->DEVICEADDR[1]);
		
    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_MAC_ADDR;
    printBuffer[3] = 6;

    printBuffer[4] = mac_addr.addr[0];
    printBuffer[5] = mac_addr.addr[1];
    printBuffer[6] = mac_addr.addr[2];
    printBuffer[7] = mac_addr.addr[3];
    printBuffer[8] = mac_addr.addr[4];
    printBuffer[9] = mac_addr.addr[5];

    SerialPrintHex(printBuffer, 10);

    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *    La_SetDeviceName
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetDeviceName(uint8_t *buffer, uint16_t len)
{
    uint8_t deviceName[20];

    if( len < (buffer[0] + 1) || buffer[0] > 20) {
        return LRD_FAIL;
    }

    memcpy(deviceName,&buffer[1],buffer[0]);
		
//		deviceName[buffer[0]] = '\0';
//		for(uint8_t i=0;i<buffer[0];i++)		
//				printf("%x ",deviceName[i]);
//		printf("\r\n");
		
    saveDeviceName(deviceName);
    LR_PRINTF_SUCCE();
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *    La_GetDeviceName
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetDeviceName(uint8_t *buffer, uint16_t len)
{
    uint16_t nameLen = 0;
		uint8_t* ptr;
	
    if( buffer[0] != 0 ) {
        return LRD_FAIL;
    }
		
		memset(printBuffer,0,sizeof(printBuffer));
    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_DEVICE_NAME;

		ptr = getDeviceName();
		
	// 求名字长度
		if((at_com_data.device_name[0]>0)&&(at_com_data.device_name[0]<22))	
		{	
				printBuffer[3] = ptr[0];
				if(printBuffer[3] == 0) 
				{
					return LRD_FAIL;
				}	
				memcpy(&printBuffer[4],&ptr[1],printBuffer[3]);
		}
		else
		{
				printBuffer[3] = strlen((char*)ptr); 
				if(printBuffer[3] == 0) 
				{
					return LRD_FAIL;
				}	
				memcpy(&printBuffer[4],ptr,printBuffer[3]);			
		}
			
    nameLen = printBuffer[3]+4;
//    
//		printf("nameLen=%d\r\n",nameLen);
//		for(uint8_t i=0;i<nameLen+4;i++)
//			printf("%02x ",printBuffer[i]);
//		printf("\r\n");
    SerialPrintHex(printBuffer, nameLen);
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *    La_SetEnablePair
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetEnablePair(uint8_t *buffer, uint16_t len)
{
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *    La_GetEnablePair
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetEnablePair(uint8_t *buffer, uint16_t len)
{
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *    La_SetAdvInterval
 *  DESCRIPTION
 *   低字节在前
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetAdvInterval(uint8_t *buffer, uint16_t len)
{

    if( buffer[0] != 2 || len <= 2) {
        return LRD_FAIL;
    }

    saveAdvInterval(((uint16_t)buffer[2] << 8) | (buffer[1] & 0xff));
    LR_PRINTF_SUCCE();
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *    La_GetAdvInterval
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetAdvInterval(uint8_t *buffer, uint16_t len)
{
    uint16_t advInterval = 0;
    if( buffer[0] != 0) {
        return LRD_FAIL;
    }
    advInterval = getAdvInterval();
    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_ADV_INTERVAL;
    printBuffer[3] = 2;
    printBuffer[4] = (uint8_t)(advInterval) & 0xff;
    printBuffer[5] = (uint8_t)(advInterval >> 8) & 0xff;
    SerialPrintHex(printBuffer, 6);
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *    La_SetTxPower
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetTxPower(uint8_t *buffer, uint16_t len)
{
    if((buffer[0] != 0x01) || (len <= 1)) {
        return LRD_FAIL;
    }
		
    saveTxPower(buffer[1]);
    LR_PRINTF_SUCCE();
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *    La_GetTxPower
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetTxPower(uint8_t *buffer, uint16_t len)
{
    if( buffer[0] != 0) {
        return LRD_FAIL;
    }
    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_TX_POWER;
    printBuffer[3] = 1;
    printBuffer[4] = getTxPower();
    SerialPrintHex(printBuffer, 5);
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *    La_SetUartDelay
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_SetUartDelay(uint8_t *buffer, uint16_t len)
{
    if((buffer[0] != 0x01) || (len <= 1)) {
        return LRD_FAIL;
    }
	
    saveUartDelayTime(buffer[1]);
    LR_PRINTF_SUCCE();
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *   La_GetUartDelay
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_GetUartDelay(uint8_t *buffer, uint16_t len)
{
    if( buffer[0] != 0) {
        return LRD_FAIL;
    }
    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_UART_DELAY;
    printBuffer[3] = 1;
    printBuffer[4] = getUartDelayTime();
    SerialPrintHex(printBuffer, 5);
    return LRD_SUCCE; 
}


/*----------------------------------------------------------------------------*
 *  NAME
 *   La_Factory
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_Factory(uint8_t *buffer, uint16_t len)
{
    if( buffer[0] != 0) {
        return LRD_FAIL;
    }
    Appcfg_RecoveryHandler();	
    return LRD_SUCCE;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
uint8_t La_Reset(uint8_t *buffer, uint16_t len)
{
    if( buffer[0] != 0) {
        return LRD_FAIL;
    }

		uint32_t err_code = sd_nvic_SystemReset();
		if(err_code == NRF_SUCCESS)
		{
			;
		}	
    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *    La_GetFwVersion
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
#define FW_VERSION_LEN   6

uint8_t La_GetFwVersion(uint8_t *buffer, uint16_t len)
{
    uint8_t tmp[10];
    uint16_t fwStrLen = 0;
	if(buffer[0] != 0) {
        return LRD_FAIL;
    }

    fwStrLen = strlen(FIRM_WARE_REVERSION);
	if(fwStrLen < FW_VERSION_LEN){
	   // Version字符长度不符合要求
       return LRD_FAIL;
	}
	memcpy(tmp,FIRM_WARE_REVERSION,fwStrLen);  
		
    printBuffer[0] = LRD_AT_GET;
    printBuffer[1] = LRD_AT_HEAD;
    printBuffer[2] = LRD_AT_GET_FW_VERSION;
    printBuffer[3] = 0x02;
    printBuffer[4] = hexAssiVarilfy(tmp[1]);
    printBuffer[5] = (hexAssiVarilfy(tmp[3])<<4);	 
		printBuffer[5] |=hexAssiVarilfy(tmp[5]);
	
    SerialPrintHex(printBuffer, 6);

    return LRD_SUCCE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *    La_atCheck
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *---------------------------------------------------------------------------*/
lrd_at_handler_sta La_atCheck(uint8_t *rx_data, uint16_t len)
{
    uint8_t at_index = 0;

    lrd_at_handler_sta sta = LRD_FAIL;
    if(rx_data[0] != LRD_AT_SET || rx_data[1] != LRD_AT_HEAD) {
        return LRD_UNKOWN;
    }

    if(la_chekCmd(rx_data[2], &at_index)) {
        sta = ((lrd_atCmd[at_index]).atHandler)(rx_data + 3, len - 3);
    } else {
        return LRD_UNKOWN;
    }

    if(sta == LRD_FAIL) {
        LR_PRINTF_FAIL();
    }
    return sta;
}

