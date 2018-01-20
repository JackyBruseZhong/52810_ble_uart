/******************************************************************************
 *  FILE
 *      lrd_at.h
 *
 *  DESCRIPTION
 *      Header file for discovered Device Info Service data and its prototypes
 *
 ******************************************************************************/

#ifndef __LRD_AT_H__
#define __LRD_AT_H__


/*============================================================================*
 *  Local Header File
 *============================================================================*/
#include "OP_main.h"

/*============================================================================*
 *  Public Data Types
 *============================================================================*/
typedef enum
{
   LRD_SUCCE = 0,
   LRD_FAIL  = 1,
   LRD_UNKOWN  = 2,
}lrd_at_handler_sta;
/*============================================================================*
 *  Public Definitions
 *============================================================================*/
uint8_t La_SetBr(uint8_t *buffer, uint16_t len);
uint8_t La_GetBr(uint8_t *buffer, uint16_t len);
uint8_t La_SetConnInterval(uint8_t *buffer, uint16_t len);
uint8_t La_GetConnInterval(uint8_t *buffer, uint16_t len);
uint8_t La_SetPairKey(uint8_t *buffer, uint16_t len);
uint8_t La_GetPairKey(uint8_t *buffer, uint16_t len);
uint8_t La_GetMacAddr(uint8_t *buffer, uint16_t len);
uint8_t La_SetDeviceName(uint8_t *buffer, uint16_t len);
uint8_t La_GetDeviceName(uint8_t *buffer, uint16_t len);
uint8_t La_SetEnablePair(uint8_t *buffer, uint16_t len);
uint8_t La_GetEnablePair(uint8_t *buffer, uint16_t len);
uint8_t La_SetAdvInterval(uint8_t *buffer, uint16_t len);
uint8_t La_GetAdvInterval(uint8_t *buffer, uint16_t len);
uint8_t La_SetTxPower(uint8_t *buffer, uint16_t len);
uint8_t La_GetTxPower(uint8_t *buffer, uint16_t len);
uint8_t La_SetUartDelay(uint8_t *buffer, uint16_t len);
uint8_t La_GetUartDelay(uint8_t *buffer, uint16_t len);
uint8_t La_Factory(uint8_t *buffer, uint16_t len);
uint8_t La_Reset(uint8_t *buffer, uint16_t len);
uint8_t La_GetFwVersion(uint8_t *buffer, uint16_t len);
lrd_at_handler_sta La_atCheck(uint8_t *rx_data, uint16_t len);


#endif /* __LRD_AT_H__ */

