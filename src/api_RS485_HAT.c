/*
 * GEMALTO LTE module mPLAS9-W driver
 * AT command document : PLAS9-W AT_Command_Set_Specification_v01005.pdf
 *
 */
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include "api_RS485_HAT.h"

int lte_uart_fd;

/**
 *  @brief api_lte_init : to set the uart parameter for this LTE module
 *  @param  : no
 *  @return : 0 as open successful ; -1 as open fail
 */
int api_lte_init(void)
{
    struct termios toptions = {0};
printf("debug-5 \n");
    lte_uart_fd = open(UARTDEVICE_RS485HAT, O_RDWR | O_NOCTTY);
    if(-1 == lte_uart_fd)
        return -1;
printf("debug-4 \n");
    /* get current serial port settings */
    tcgetattr(lte_uart_fd, &toptions);
    cfsetispeed(&toptions, B9600);
    cfsetospeed(&toptions, B9600);
    /* 8 bits, no parity, no stop bits */
    //toptions.c_cflag &= ~PARENB;
    toptions.c_cflag |= PARENB;
    toptions.c_cflag &= ~PARODD;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;


    toptions.c_cflag |= CS8;

    /* no hardware flow control */
    toptions.c_cflag &= ~CRTSCTS;

    /* disable canonical input, disable echo,
    disable visually erase chars,
    disable terminal-generated signals */
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* disable input/output flow control, disable restart chars */
    toptions.c_iflag &= ~(ICRNL | IXON | IXOFF | IXANY);

    toptions.c_oflag &= ~OPOST;

    /* commit the serial port settings */
    tcsetattr(lte_uart_fd, TCSANOW, &toptions);

    return 0;
}


/**
 *  @brief api_Send_Command : to send AT command to LTE module
 *  @param Command          : the required command
 *  @return                 : return -1 if uart write fail
 */
int api_Send_MbosCommand(const char *Command)
{

    printf("command_length = %d \n",sizeof(MBOS_COMMAND));

    if(0 > write(lte_uart_fd, MBOS_COMMAND, 8));//sizeof(Command)))
        return -1;
}

/**
 *  @brief api_Send_Command : to read the return data from LTE module after sending the readable AT command
 *  @param Command          : the required command
 *  @param Passer_ori       : the return data from LTE module
 *  @return                 : return -1 if uart write fail ; return -3 if retry error
 */
int api_Read_Command(const char *Command,char *Passer_ori)
{   
    char command[MAX_COMMAND_SIZE] ={0};
    char buffer[1];
    char p_index = 0;
    bool smoniout_start = 0;
    fd_set rdfs;
    struct timeval tv; 
    char retry = 0;
    unsigned int time_period = 0;
    unsigned int pre_time = 0;
    unsigned int current_time = 0;
    time_t t;

    t = time(NULL);

    pre_time = current_time = time(&t);

    strcpy(command,Command);
    strcat(command,"\r");
     
    if(0 > write(lte_uart_fd, command, sizeof(command)))
       return -1;

    while(1)
    { 
        current_time = time(&t);
        time_period = current_time - pre_time;
        pre_time = current_time;

        if(TIMES_RETRY <= retry)
            return -3;      
        
        if(1 <= time_period)            //  blocking time bigger than monitoring time
            retry++;
  
        tv.tv_sec = TIME_SEC;
        tv.tv_usec = TIME_MICROSEC;
        FD_ZERO(&rdfs);
        FD_SET(lte_uart_fd,&rdfs);
        select(lte_uart_fd+1,&rdfs,NULL,NULL,&tv);  
      
        if(FD_ISSET(lte_uart_fd,&rdfs))
        {
            read(lte_uart_fd, buffer, 1);

            if(buffer[0] == ':')
                smoniout_start = 1;

            if(1 == smoniout_start)
            {
            	 // just leaving the data and comma, so the format seems like (xxx,xxx,xxx,xxxx)
            	 // this action is prepare for later process to capture desired element using strtok( ) function
                 if((':' != buffer[0]) && ('\b' != buffer[0]) && (' ' != buffer[0]) && ('"' != buffer[0]) && ('\r' != buffer[0]) && ('\n' != buffer[0]))
                 {
                     Passer_ori[p_index] =  buffer[0];
                     p_index++;                      
                 }                
                
                if(buffer[0] == '\n')           // the element at the end of return data
                    break;
            }
        }
        
    }   
 
    FD_CLR(lte_uart_fd,&rdfs);

    return 0;
}

int main(int argc, char *argv[]){
bool flag_skip = 0;
char passer_ori[128];
int reeoe_code = 0;
printf("debug-2 \n");
printf("debug-3 \n");
if(0 > (api_lte_init()))
    printf("open fail \n");

printf("debug-1 \n");

api_Send_MbosCommand(MBOS_COMMAND);

while(0)
{

usleep(1000000);
}
  return 0 ;
}

