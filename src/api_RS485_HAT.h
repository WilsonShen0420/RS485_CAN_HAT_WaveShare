#ifndef APILTE_H_
#define APILTE_H_

#define UARTDEVICE_RS485HAT    "/dev/ttyAMA0"
#define MAX_COMMAND_SIZE     16
#define MAX_INFO_SIZE        128
#define TIMES_RETRY          5
#define TIME_SEC             2
#define TIME_MICROSEC        0

const char MBOS_COMMAND[8] = {0x01,0x06,0x00,0x00,0x00,0x08,0x88,0x0c};


int api_lte_init(void);
int api_Send_MbosCommand(const char *Command);
int api_Read_Command(const char *Command,char *Passer_ori);

#endif
