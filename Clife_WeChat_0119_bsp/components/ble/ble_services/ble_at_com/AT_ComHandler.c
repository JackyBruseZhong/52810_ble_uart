#include "AT_ComHandler.h"
#include "app_util.h"
#include "stdio.h"
#include "string.h"
#include "OP_main.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "nrf_nvic.h"


#define PAGRE_TO_ERASE									1
#define FLASH_START_ADDR  							0x27000
#define FLASH_STOP_ADDR  								0x28000

#define FLASH_START_WRITE_ADDR					0x27000


#define POWER_UP_WRITE_TIMEOUT_INTERVAL 	APP_TIMER_TICKS(6000)

APP_TIMER_DEF(m_PowerUp_write_id);

uint8_t fs_event_num = 0;

static bool reset_write_para = false;

void power_manage(void);

void getUartConfig(void)
{
	flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));
	if(at_com_data.uart_cfg.br==0xFFFFFFFF)
	{
			at_com_data.uart_cfg.br = UARTE_BAUDRATE_BAUDRATE_Baud9600;
			at_com_data.uart_cfg.flow_control = 0;
			at_com_data.uart_cfg.flow_control_threshold = 0;
		  at_com_data.uart_cfg.parity = 0;
		  at_com_data.uart_cfg.stop_bit = 1;
	}	
  else
	{
			
			at_com_data.uart_cfg.br = at_com_data.uart_cfg.br;
			at_com_data.uart_cfg.flow_control = at_com_data.uart_cfg.flow_control;
			at_com_data.uart_cfg.flow_control_threshold = at_com_data.uart_cfg.flow_control_threshold;
		  at_com_data.uart_cfg.parity = at_com_data.uart_cfg.parity;
		  at_com_data.uart_cfg.stop_bit = at_com_data.uart_cfg.stop_bit;	
	}	
}	
void getConnInterval(void)
{
		uint8_t con_h, con_l;
		flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));	
		
		if(at_com_data.con_interval.conInterval==0xFFFF)
		{		
				at_com_data.con_interval.conInterval = MAX_CONN_INTERVAL;
			  at_com_data.con_interval.latency 		 = 0;
		}
		else
		{
				con_h = at_com_data.con_interval.conInterval&0x00FF;
				con_l = (at_com_data.con_interval.conInterval>>8)&0xFF;
				at_com_data.con_interval.conInterval = con_h;
				at_com_data.con_interval.conInterval = (at_com_data.con_interval.conInterval<<8);
				at_com_data.con_interval.conInterval |= con_l;
		}					

}

uint8_t* getDeviceName(void)
{
		uint8_t *device_name;
	
		memset(at_com_data.device_name,0,sizeof(at_com_data.device_name));
		flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));
		
		if((at_com_data.device_name[0]>0)&&(at_com_data.device_name[0]<22))
		{				
			  device_name = at_com_data.device_name;
				//printf("%s\r\n",device_name);
		}
		else
    {		
					device_name = (uint8_t *)&DEVICE_NAME;
			//	printf("%s\r\n",device_name);
		}
		return device_name;
}	
uint16_t getAdvInterval(void)
{
		uint16_t AdvInterval = 0;
		flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));
		if(at_com_data.adver_interval==0xFFFF)
		{		
				AdvInterval = APP_ADV_INTERVAL;
		}
		else
		{
				AdvInterval = at_com_data.adver_interval;
		}
		return AdvInterval;
}	
uint8_t getTxPower(void)
{
		int8_t TxPower = 0;
		flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));
		if(at_com_data.tx_power > 4)
		{		
				TxPower = 1;
		}
		else
		{
				TxPower = at_com_data.tx_power;
		}
		return TxPower;
}	
uint8_t getUartDelayTime(void)
{
		uint8_t DelayTime = 0;
		flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));
		if(at_com_data.delay_time==0xFF)
		{			
				DelayTime = DEFAULT_DELAY_TIME;
		}
		else
		{
				DelayTime = at_com_data.delay_time;
		}
		return DelayTime;
}	
void Appcfg_RecoveryHandler(void)
{
	flash_clear();
}	
void saveUartConfig(uartConfig_t* pConfig)
{	
	if(!reset_write_para)
	{
		read_clear_data();
	}
  at_com_data.uart_cfg.br 										= pConfig->br;
	at_com_data.uart_cfg.flow_control   				= pConfig->flow_control;
	at_com_data.uart_cfg.flow_control_threshold = pConfig->flow_control_threshold;
	at_com_data.uart_cfg.parity									= pConfig->parity;
	at_com_data.uart_cfg.stop_bit								= pConfig->stop_bit;
		
	if(!reset_write_para)
	{
		fs_event_num = FS_ERASE_EVT;	
	}	
//	printf("baud rate=%x\r\n",at_com_data.uart_cfg.br );	
}
void saveDeviceName(uint8_t *name_arry)
{
	if(!reset_write_para)
	{
		read_clear_data();
	}
	at_com_data.device_name[0] = strlen((char *)name_arry);
	at_com_data.device_name[at_com_data.device_name[0]] = '\0';

	memcpy(&at_com_data.device_name[1],name_arry,strlen((char *)name_arry));
	if(!reset_write_para)
	{
		fs_event_num = FS_ERASE_EVT;	
	}	

//	printf("name_len=%d\r\n",at_com_data.device_name[0] );
//	for(uint8_t i=1;i<at_com_data.device_name[0]+1;i++)
//		 printf("%x ",at_com_data.device_name[i]);		
//	printf("\r\n");
}
void saveConnInterval(conInterval_t *conInteval)
{
//	uint8_t con_h, con_l;

	if(!reset_write_para)
	{
		read_clear_data();
	}

//	con_h = conInteval->conInterval&0x00FF;
//	con_l = (conInteval->conInterval>>8)&0xFF;
//	at_com_data.con_interval.conInterval = con_h;
//	at_com_data.con_interval.conInterval = (at_com_data.con_interval.conInterval<<8);
//	at_com_data.con_interval.conInterval |= con_l;
	at_com_data.con_interval.conInterval = 	conInteval->conInterval;
	at_com_data.con_interval.latency = conInteval->latency;
	if(!reset_write_para)
	{
		fs_event_num = FS_ERASE_EVT;	
	}	
//	printf("conInterval=%x\r\n",at_com_data.con_interval.conInterval );
}
void saveAdvInterval(uint16_t AdvInterval)
{
	if(!reset_write_para)
	{
		read_clear_data();
	}
	at_com_data.adver_interval = AdvInterval;
	if(!reset_write_para)
	{
		fs_event_num = FS_ERASE_EVT;	
	}
//	printf("adver_interval=%x\r\n",at_com_data.adver_interval);	
}	
void saveTxPower(uint8_t TxPower)
{
	if(!reset_write_para)
	{
		read_clear_data();
	}
	at_com_data.tx_power = TxPower;
	if(!reset_write_para)
	{
		fs_event_num = FS_ERASE_EVT;	
	}		
//	printf("TxPower=%x\r\n",at_com_data.tx_power );
}	
void saveUartDelayTime(uint8_t DelayTime)
{
	if(!reset_write_para)
	{
		read_clear_data();
	}
	at_com_data.delay_time = DelayTime;
	if(!reset_write_para)
	{
		fs_event_num = FS_ERASE_EVT;	
	}		
}	





void fs_evt_handler(nrf_fstorage_evt_t * evt);


void fs_evt_handler(nrf_fstorage_evt_t * p_evt)	
{	

    if (p_evt->result != NRF_SUCCESS)
    {
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
//            printf("wrote %d bytes at address 0x%x.\r\n",
//                        p_evt->len, p_evt->addr);

//					uint32_t err_code = sd_nvic_SystemReset();
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
//            printf("erased %d page from address 0x%x.\r\n",
//                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }		
}
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    .evt_handler    = fs_evt_handler,
    .start_addr     = FLASH_START_ADDR,
    .end_addr       = FLASH_STOP_ADDR,
};
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}

void flash_clear(void)
{
		ret_code_t rc = nrf_fstorage_erase(
									&fstorage,   						/* The instance to use. */
									FLASH_START_WRITE_ADDR,     /* The address of the flash pages to erase. */
									PAGRE_TO_ERASE, 						/* The number of pages to erase. */
									NULL            						/* Optional parameter, backend-dependent. */
		);
//	  printf("clear rc=0x%02x \r\n",rc);
		if (rc == NRF_SUCCESS)
		{
//			 printf("Clear OK\r\n");	
				/* The operation was accepted.
					 Upon completion, the NRF_FSTORAGE_ERASE_RESULT event
					 is sent to the callback function registered by the instance. */
		}
}
void flash_write(uint8_t *dat,uint16_t len)
{
		ret_code_t rc = nrf_fstorage_write(
								&fstorage,   /* The instance to use. */
								FLASH_START_WRITE_ADDR,     /* The address in flash where to store the data. */
								dat,        		/* A pointer to the data. */
								len, /* Lenght of the data, in bytes. */
								NULL            /* Optional parameter, backend-dependent. */
		);
//	  printf("write rc=0x%02x\r\n",rc);
		if (rc == NRF_SUCCESS)
		{
//				printf("Write OK\r\n");
				/* The operation was accepted.
					 Upon completion, the NRF_FSTORAGE_WRITE_RESULT event
					 is sent to the callback function registered by the instance. */
		}
}

void flash_read(uint8_t *dat,uint16_t len)
{
		ret_code_t rc = nrf_fstorage_read(
				&fstorage,   /* The instance to use. */
				FLASH_START_WRITE_ADDR,     /* The address in flash where to read data from. */
				dat,        /* A buffer to copy the data into. */
				len  /* Lenght of the data, in bytes. */
		);
//		printf("read rc=0x%02x\r\n",rc);
		if (rc == NRF_SUCCESS)
		{
//				printf("Read OK\r\n");
				/* The operation was accepted.
					 Upon completion, the NRF_FSTORAGE_READ_RESULT event
					 is sent to the callback function registered by the instance.
					 Once the event is received, it is possible to read the contents of 'number'. */
		}
}	
void read_clear_data(void)
{
	flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));
	flash_clear();
}

void flash_init(void)
{
			nrf_fstorage_init(
					&fstorage,       															/* You fstorage instance, previously defined. */
					&nrf_fstorage_sd,   													/* Name of the backend. */
					NULL                													/* Optional parameter, backend-dependant. */
			);
	//	read_clear_data();
		memset(&at_com_data,0,sizeof(at_com_data));
//		printf("power=%d\r\n",at_com_data.tx_power);
//		at_com_data.tx_power = 8;
//		flash_write((uint8_t *)&at_com_data,sizeof(at_com_data));
//		memset(&at_com_data,0,sizeof(at_com_data));
		flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));	
	//	printf("aa_power=%d\r\n",at_com_data.tx_power);
}



void flash_operate(uint8_t event)
{		
		if(fs_event_num == FS_ERASE_EVT)
		{
			fs_event_num = FS_IDLE;
		//	at_com_data.tx_power = 0x33;
//			printf("power=%x\r\n",at_com_data.tx_power);	
//			printf("name=%s\r\n",&at_com_data.device_name[1]);	
//			printf("con_interval=%x\r\n",at_com_data.con_interval.conInterval);	
//			printf("adv_interval=%x\r\n",at_com_data.adver_interval);	
//			printf("uart_cfg=%x\r\n",at_com_data.uart_cfg.br);	
			
			flash_write((uint8_t *)&at_com_data,sizeof(at_com_data));	
			wait_for_flash_ready(&fstorage);	
			reset_write_para = false;	
			PowerUp_uart_rec = false;
//			printf("init power=%x\r\n",at_com_data.tx_power);

//			flash_read((uint8_t *)&at_com_data,sizeof(at_com_data));		
			
		}
}



	


static void PowerUp_write_timeout_handler(void * p_context)
{			
		
//		printf("name=%s len=%d\r\n",&at_com_data.device_name[1],at_com_data.device_name[0]);	
//		printf("con_interval=%x\r\n",at_com_data.con_interval.conInterval);	
//		printf("adv_interval=%x\r\n",at_com_data.adver_interval);	
		if((reset_write_para)&&(PowerUp_uart_rec))
		{
//			printf("Timeout write\r\n");	
			fs_event_num = FS_ERASE_EVT;
		}
	
}

void PowerUp_write_timer_init(void)
{
		app_timer_create(&m_PowerUp_write_id,
										 APP_TIMER_MODE_SINGLE_SHOT,
										 PowerUp_write_timeout_handler);
		
		app_timer_start(m_PowerUp_write_id,POWER_UP_WRITE_TIMEOUT_INTERVAL,NULL);
		reset_write_para = true;
}



























































































