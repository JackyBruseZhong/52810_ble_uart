#ifndef __AT_COMHANDLER_H__
#define __AT_COMHANDLER_H__

#include "OP_main.h"
extern uint8_t   fs_event_num;	

extern void getUartConfig(void);
extern void getConnInterval(void);
extern uint8_t* getDeviceName(void);
extern uint16_t getAdvInterval(void);
extern uint8_t getTxPower(void);
extern uint8_t getUartDelayTime(void);
extern void Appcfg_RecoveryHandler(void);

extern void saveUartConfig(uartConfig_t* pConfig);
extern void saveConnInterval(conInterval_t *conInterval);
extern void saveDeviceName(uint8_t * name_arry);
extern void saveAdvInterval(uint16_t AdvInterval);
extern void saveTxPower(uint8_t TxPower);
extern void saveUartDelayTime(uint8_t DelayTime);

extern void flash_init(void);
extern void flash_clear(void);
extern void read_clear_data(void);
extern void flash_read(uint8_t *dat,uint16_t len);
extern void flash_write(uint8_t *dat,uint16_t len);
extern void PowerUp_write_timer_init(void);























#endif 

