/******************************************************************************
 *  FILE
 *      DTM_App.c
 *
 *  DESCRIPTION
 *
 *
 *****************************************************************************/

#include "ble_dtm.h"
#include "nrf_gpio.h"
#include "OP_main.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "ble_gap.h"
#include "app_uart.h"

#define DTM_RX_PIN_NUMBER 	 								26
#define DTM_TX_PIN_NUMBER  									27
// Configuration parameters.
#define BITRATE  UART_BAUDRATE_BAUDRATE_Baud19200  /**< Serial bitrate on the UART */

// @note: The BLE DTM 2-wire UART standard specifies 8 data bits, 1 stop bit, no flow control.
//        These parameters are not configurable in the BLE standard.

/**@details Maximum iterations needed in the main loop between stop bit 1st byte and start bit 2nd
 * byte. DTM standard allows 5000us delay between stop bit 1st byte and start bit 2nd byte. 
 * As the time is only known when a byte is received, then the time between between stop bit 1st 
 * byte and stop bit 2nd byte becomes: 
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration): 
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations. 
 *
 * This is rounded down to 21. 
 *
 * @note If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE ((5000 + 2 * UART_POLL_CYCLE) / UART_POLL_CYCLE)

bool 		 cmd_reset = false;
uint8_t  mac_addr[6];

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

// Error handler for UART
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

static void dtm_uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          DTM_RX_PIN_NUMBER,
          DTM_TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UARTE_BAUDRATE_BAUDRATE_Baud19200
      };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);	
	
}
/**@brief Function for splitting UART command bit fields into separate command parameters for the DTM library.
*
 * @param[in]   command   The packed UART command.
 * @return      result status from dtmlib.
 */
static uint32_t dtm_cmd_put(uint16_t command)
{
    dtm_cmd_t      command_code = (command >> 14) & 0x03;
    dtm_freq_t     freq         = (command >> 8) & 0x3F;
    uint32_t       length       = (command >> 2) & 0x3F;
    dtm_pkt_type_t payload      = command & 0x03;
  
    // Check for Vendor Specific payload.
    if (payload == 0x03) 
    {
        /* Note that in a HCI adaption layer, as well as in the DTM PDU format,
           the value 0x03 is a distinct bit pattern (PRBS15). Even though BLE does not
           support PRBS15, this implementation re-maps 0x03 to DTM_PKT_VENDORSPECIFIC,
           to avoid the risk of confusion, should the code be extended to greater coverage. 
        */
        payload = DTM_PKT_VENDORSPECIFIC;
    }
    return dtm_cmd(command_code, freq, length, payload);
}
	
static void dtm_mode(void)
{
		uint32_t err_code;
    uint32_t    current_time;
    uint32_t    msb_time          = 0;     // Time when MSB of the DTM command was read. Used to catch stray bytes from "misbehaving" testers.
    bool        is_msb_read       = false; // True when MSB of the DTM command has been read and the application is waiting for LSB.
    uint16_t    dtm_cmd_from_uart = 0;     // Packed command containing command_code:freqency:length:payload in 2:6:6:2 bits.
    uint8_t     rx_byte;                   // Last byte read from UART.
    dtm_event_t result;                    // Result of a DTM operation.
	
    
		err_code = dtm_init();
		APP_ERROR_CHECK(err_code);
	
		for (;;)
		{
        // Will return every timeout, 625 us.
        current_time = dtm_wait();

        if (app_uart_get(&rx_byte) != NRF_SUCCESS)
        {
            // Nothing read from the UART.
            continue;
        }

        if (!is_msb_read)
        {
            // This is first byte of two-byte command.
            is_msb_read       = true;
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;

            // Go back and wait for 2nd byte of command word.
            continue;
        }

        // This is the second byte read; combine it with the first and process command
        if (current_time > (msb_time + MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE))
        {
            // More than ~5mS after msb: Drop old byte, take the new byte as MSB.
            // The variable is_msb_read will remains true.
            // Go back and wait for 2nd byte of the command word.
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;
            continue;
        }

        // 2-byte UART command received.
        is_msb_read        = false;
        dtm_cmd_from_uart |= (dtm_cmd_t)rx_byte;

        if (dtm_cmd_put(dtm_cmd_from_uart) != DTM_SUCCESS)
        {
            // Extended error handling may be put here.
            // Default behavior is to return the event on the UART (see below);
            // the event report will reflect any lack of success.
        }

        // Retrieve result of the operation. This implementation will busy-loop
        // for the duration of the byte transmissions on the UART.
        if (dtm_event_get(&result))
        {
            // Report command status on the UART.
            // Transmit MSB of the result.
            while (app_uart_put((result >> 8) & 0xFF));
            // Transmit LSB of the result.
            while (app_uart_put(result & 0xFF));
        }
	
				if(cmd_reset)
				{
					 cmd_reset = false;

					 for(uint8_t i=2;i<6;i++)
					 {
							app_uart_put(mac_addr[i]);
					 }	
					 app_uart_put(mac_addr[0]);
					 app_uart_put(mac_addr[1]);	
				}		

			}
		
}

uint8_t PCBA_keyio_test(void)
{
	uint8_t out_pin[7]={NRF_GPIO_PIN_MAP(1,11),NRF_GPIO_PIN_MAP(1,12),NRF_GPIO_PIN_MAP(1,15),NRF_GPIO_PIN_MAP(0,12),NRF_GPIO_PIN_MAP(0,13),NRF_GPIO_PIN_MAP(0,15),NRF_GPIO_PIN_MAP(0,14)};
	uint8_t in_pin[7] ={NRF_GPIO_PIN_MAP(1,13),NRF_GPIO_PIN_MAP(1,10),NRF_GPIO_PIN_MAP(0,4),NRF_GPIO_PIN_MAP(1,14),NRF_GPIO_PIN_MAP(0,16),NRF_GPIO_PIN_MAP(1,2),NRF_GPIO_PIN_MAP(0,22)};
	
	for(uint8_t i=0;i<sizeof(out_pin);i++)																		// input output setting
	{
			nrf_gpio_cfg_output(out_pin[i]);
			nrf_gpio_cfg_input(in_pin[i],NRF_GPIO_PIN_PULLUP);
	}
	nrf_delay_ms(3);
	
	for(uint8_t i=0;i<sizeof(out_pin);i++)																	// all output low			
	{
		 nrf_gpio_pin_clear(out_pin[i]);
	}
	nrf_delay_ms(3);
	
	if((nrf_gpio_pin_read(in_pin[0]))											   // check if high, if one pin is high,fail		
		||(nrf_gpio_pin_read(in_pin[1]))
		||(nrf_gpio_pin_read(in_pin[2]))
		||(nrf_gpio_pin_read(in_pin[3]))
		||(nrf_gpio_pin_read(in_pin[4]))
		||(nrf_gpio_pin_read(in_pin[5]))
		||(nrf_gpio_pin_read(in_pin[6]))
		)		
		{
//			printf("aaa\r\n");
			return 0;
		}
	
	for(uint8_t i=0;i<sizeof(out_pin);i++)																//  all output high	
	{
		 nrf_gpio_pin_set(out_pin[i]);
	}	 	
	nrf_delay_ms(3);
	if((!(nrf_gpio_pin_read(in_pin[0])))									// check if low, if one pin is low,fail		
		||(!(nrf_gpio_pin_read(in_pin[1])))
		||(!(nrf_gpio_pin_read(in_pin[2])))
		||(!(nrf_gpio_pin_read(in_pin[3])))
		||(!(nrf_gpio_pin_read(in_pin[4])))
		||(!(nrf_gpio_pin_read(in_pin[5])))
		||(!(nrf_gpio_pin_read(in_pin[6])))
		)	
		{
//			printf("bbb\r\n");
			return 0;
		}

	for(uint8_t i=0;i<sizeof(out_pin);i++)	
	{
		nrf_gpio_pin_clear(out_pin[i]);
		nrf_delay_ms(3);
		if(nrf_gpio_pin_read(in_pin[i]))
		{
//			printf("ccc=%d\r\n",i);	
			return 0;
		}
		nrf_gpio_pin_set(out_pin[i]);
		nrf_delay_ms(3);
		if(!(nrf_gpio_pin_read(in_pin[i])))
		{	
//			printf("ddd=%d\r\n",i);
			return 0;
		}
	}
	return 1;
}
/**@brief     Function to deal with the event in the BLE DTM mode
 *
 * @param[in] void
 *
 * @return    void
 */
void dtm_mode_check(void)
{
	  uint8_t flag,test_try,res;	
		nrf_gpio_cfg_input(DTM_MODE_PIN, NRF_GPIO_PIN_PULLUP);		


	
		if(!nrf_gpio_pin_read(DTM_MODE_PIN))
		{

				nrf_gpio_cfg_output(DTM_GREEN_LED);
				nrf_gpio_pin_set(DTM_GREEN_LED);
			

				flag = 0;
				test_try = 20;
				while(test_try--) {
					res = PCBA_keyio_test();
					flag <<= 1;
					flag |= res;
					if(flag == 0xff)
						break;
				}
				if(flag == 0xff)
				{
					 nrf_gpio_pin_clear(DTM_GREEN_LED);
				}
				else
				{
					 nrf_gpio_pin_set(DTM_GREEN_LED);			
				}

				dtm_uart_init();
				dtm_mode();
		}		
}

