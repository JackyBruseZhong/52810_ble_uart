#ifndef BLE_AT_H__
#define BLE_AT_H__



typedef struct
{
	char *string;
	AT_Command_Fun_t AT_Command_Fun;
}uart_com_t;

uart_com_t uart_com[10];

typedef void (AT_Command_Fun_t *) (char *p)


#endif