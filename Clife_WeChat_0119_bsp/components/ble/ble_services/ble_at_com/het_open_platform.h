#ifndef HET_OPEN_PLATFORM_H__
#define HET_OPEN_PLATFORM_H__


#include <stdint.h>
#include <string.h>
#include "ble_nus.h"

#define 	MAX_SENDER_QUEUE_NUMBERS   						2
#define   PKT_LENGTH 								 					  255

typedef struct 
{	
		uint8_t 	buf[PKT_LENGTH]; 
		uint16_t  offset;
		uint16_t  len; 

}data_pkt_t;
typedef struct 
{
    data_pkt_t trans[MAX_SENDER_QUEUE_NUMBERS];
		uint8_t current_index;
		uint8_t unused_index; 
		uint8_t valid_numbers; 
}sender_queue_t;

typedef struct
{
		uint8_t *data;
		uint16_t len;
		uint16_t offset;
} data_info;

typedef struct
{
		bool      timer_start_flag;
		uint16_t  send_index;
		uint16_t 	len;
		uint16_t  send_len;
		uint16_t  send_index_rf;	
		uint8_t 	data_array[512];
}uart_data_t;

typedef struct
{
	bool 	  	receive_start_flag;
	bool      receive_len;
	uint8_t   data_len;
	uint8_t   data_index;
	uint16_t  total_len;
	uint8_t   rece_data[60];
}uart_data_recv_t;

typedef struct
{
	bool 	  	data_ready;
	uint16_t  total_len;
	uint8_t   rece_data[255];
}open_platform_data_recv_t;



extern uart_data_t 			 OP_uart_data;
extern uart_data_recv_t  uart_data_recv;
extern open_platform_data_recv_t  ble_data_recv;

extern open_platform_data_recv_t open_platform_data_recv;
extern void uart_send_data_to_mcu(uint8_t *src, uint8_t len);

extern void send_data_to_mcu_timer_start(uint16_t time);
extern void wake_up_timer_init(void);
extern void wake_up_timer_start(void);
extern void indica_info_timer_init(void);
extern void encryption_timer_init(void);
extern void encryption_timer_start(void);
extern void handle_clife_data(uint8_t *p_data,uint16_t len);
extern uint32_t Send_Notify(uint8_t *send_data, uint16_t data_len);
extern void Sender_SetData(uint16_t offset,uint8_t* buff,uint16_t len);
extern void C_Send(void);


#endif



