#include "het_open_platform.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "ble_hci.h"
#include "ble_atHandler.h"
#include "OP_main.h"
#include "stdlib.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "nrf52810_bitfields.h"
#include "nrf_delay.h"

#define WAKEUP_MCU_TIMEOUT_INTERVAL					  APP_TIMER_TICKS(5)


static sender_queue_t 	senderQ;

extern uint16_t  m_conn_handle;	


/**@brief C_Send     
 *
 * @param[in]   void
 *
 * @return      void
 */
void C_Send(void)
{
  uint16_t sendSize = 0;
	data_pkt_t* pTran = &senderQ.trans[senderQ.current_index];
	
	if(senderQ.valid_numbers)
	{
      NEXT_CIRCLE:		
      if(pTran->offset < pTran->len)
			{
	        if( pTran->len - pTran->offset > MTU_SIZE ) 
					{
	            sendSize = MTU_SIZE;
	        } 
					else 
					{
	            sendSize =  pTran->len - pTran->offset;
	        }
					if(Send_Notify((&pTran->buf[pTran->offset]), sendSize) == NRF_SUCCESS)
					{
						pTran->offset += sendSize;
					}	
			}
			else
			{
        pTran->len = pTran->offset = 0;  
				if(senderQ.valid_numbers)
				{
            senderQ.valid_numbers--;
				}
		    if(senderQ.valid_numbers)
				{
          	senderQ.current_index ^= 0x01;
			   		senderQ.unused_index = (senderQ.current_index ==0 )? 1:0;
			   
			   		pTran = &senderQ.trans[senderQ.current_index];
                
          	goto NEXT_CIRCLE;
				} 

				senderQ.current_index = senderQ.unused_index = 0;
		}
			
	} 
	
}

/**@brief Sender_SetData     
 *
 * @param[in]   offset   to determine which buffer to be send
 * @param[in]   buff 		 point to the buffer to be send
 * @param[in]   len 		 data length to be send 
 *	
 * @return      void
 */
void Sender_SetData(uint16_t offset,uint8_t* buff,uint16_t len)
{
    // ?????????? 
    if(len > PKT_LENGTH)
    {
       	return;
    }
	
    senderQ.trans[senderQ.unused_index].len = len; 
    senderQ.trans[senderQ.unused_index].offset = offset;
    memcpy(senderQ.trans[senderQ.unused_index].buf,buff,len);
	
    if(senderQ.valid_numbers < MAX_SENDER_QUEUE_NUMBERS)
    {
       	senderQ.unused_index = (senderQ.current_index == 0)? 1:0;
       	senderQ.valid_numbers ++;
    } 

    if(senderQ.valid_numbers == 1)
    {
       	C_Send();
    }
}
/**@brief Send_Notify   
 *
 *	Function for sending data to app and the data_len should not more than 20 bytes.
 *
 * @param[in]   send_data   Point to the data struct need to send.
 * @param[in]   data_len    data length
 *
 * @return      void
 */


void handle_clife_data(uint8_t *p_data,uint16_t len)
{
}



