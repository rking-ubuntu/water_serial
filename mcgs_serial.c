#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "serial_opt.h"
#include "crc.h"

int mcgs_serial(int channel_num,int value[])
{
    unsigned char serial_buff[20];
    unsigned char *p;
    int fd=0;
    int i;
    p=&serial_buff[2];
    fd = open_port(fd,1);
    set_opt(fd,9600,8,'N',1);

    serial_buff[0]=0x01;
    serial_buff[1]=0x03;
    serial_buff[2]=channel_num*2;
    for(i=0;i<channel_num;i++)
    {
        *(++p)=value[i]>>8;
        *(++p)=value[i];
    }

    crc(13,serial_buff);

    for(i=0;i<=10;i++)
    {
        write(fd,serial_buff,15);
        usleep(500);
    }
    close(fd);
    return 0;
}


