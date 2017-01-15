/**
 * @file water.cpp
 * @brief 270 处理器
 * @author Shao Xianhui
 * @version 1.0
 * @date 2009-11-11
 */

#include "common.h"
#include "CircleArray.h"
#include "spi_common.h"
#include "slot.h"
#include "ain.h"
#include "aout.h"
#include "din.h"
#include "dout.h"
#include "INIFileOP.h"
#include "Lqueue.h"

#include "serial_opt.h"
#include "crc.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/time.h>

#define QLEN 5
#define BUFFERSIZE 2048 //!< 接收缓冲区大小
#define LSB 0.00025
#define SC_POLLTIME_MIN 5

// 函数声明
int	errexit(const char *format, ...);
int	connectTCP(const char *host, const char *service); //!< TCP
int	passiveTCP(const char *service, int qlen); //!< 本地服务器Socket
void timer_func(int n); //!< 定时器函数
void* thread_func(void* data); //!< 线程函数
void InitSlot(); //!< 初始化子板
void ProcessCmd(int ssocket,unsigned char *buff,int len); //!< 处理命令
void PollSlot(Client_Data *p); //!< 轮询子板
void DetectSlotType(int slot); //!< 探测子板类型
void SigIO(int n);
void SendSlotOffLine(unsigned char slot);
void SendSlotOnLine(unsigned char slot);
int ReadIniFile(char *filename);

// 全局变量声明
CTS m_cts; //!< 子站向服务器发送的数据包
STC m_stc; //!< 服务器向子站发送的数据包
CCircleArray Array; //!< 环形数组队列
Client_Data Current; //!< 子站当前快照
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; //!< 互斥量
pthread_mutex_t flag = PTHREAD_MUTEX_INITIALIZER; //!< 互斥量
slot_descriptor_t sd[SLOT_MAX]; //!< SLOT_MAX个子板设备描述符
board_t slot_type[SLOT_MAX]; //!< SLOT_MAX个子板类型
struct itimerval interval; //!< 服务器轮询子站周期
int WriteArrayCount; //!< 写环形数组
//Lqueue Q; //!< 发送失败队列
char SERVERIP[20];
char SERVERPORT[6];
char LISTENPORT[6];
int SLOTNUMBER;
int changes_count;

/** 
 * @brief Main 主函数
 * 
 * @param argc 参数个数
 * @param argv[] 参数
 * 
 * @return 
 */
int main(int argc, char *argv[])
{
    //变量定义,初始化
    struct sockaddr_in fsin; //!< socket地址
    int	msock,ssock; //!< 服务器与子板的socket
    unsigned int alen = sizeof(fsin);
    unsigned char buffer[BUFFERSIZE]; //!< socket缓冲区
    int len = 0;
    pthread_attr_t tattr; //!< 线程属性
    pthread_t tid; //!< 线程ID
    struct sched_param param; //!< 线程优先级
    pthread_attr_init(&tattr); //!< 初始化线程属性
    int newprio = 20; //!< 线程优先级
    WriteArrayCount = 0;
    //InitQueue(Q);

    printf("chanes_buffer = %d\n",sizeof(struct Message));
    printf("Message = %d\n",sizeof(m_cts.m_Changes));
    ReadIniFile("water.ini");

    pthread_attr_getschedparam(&tattr,&param);
    param.sched_priority = newprio; //!< 优先级
    pthread_attr_setschedparam(&tattr, &param);

    for(int i = 0;i < SLOTNUMBER;i++)
    {
        slot_type[i] = BOARD_UNKNOWN;
        sd[i] = -1;
    }


    printf("Message = %d\n",sizeof(m_cts.m_Changes));
//    int success_init = 0;
//    while(success_init == 0)
//    {
//        int s = connectTCP(SERVERIP, SERVERPORT);
//    printf("Message = %d\n",sizeof(m_cts.m_Changes));
//        while(s < 0)
//        {
//            sleep(2);
//            s = connectTCP(SERVERIP, SERVERPORT);
//    printf("Mesiiiiiiiisage = %d\n",sizeof(m_cts.m_Changes));
//        }
//        m_cts.m_Reset.time = time(NULL);
//        write(s,&m_cts.m_Reset,m_cts.m_Reset.GetSize());
//        memset(buffer,0,BUFFERSIZE);
//        int len = 0,templen = 0;
//        unsigned char *tempbuffer = buffer;
//        //len = read(s,buffer,BUFFERSIZE);
//        while((templen = read(s,tempbuffer,BUFFERSIZE - len)) > 0)
//        {
//            len += templen;
//            tempbuffer += templen;
//        }
//        close(s);
//        if(len == sizeof(Client_Data) + sizeof(CTS_Return_Client_Para))
//        {
//            success_init = 1;
//            memcpy(&Current,buffer,sizeof(Client_Data));
//            memcpy(&m_cts.m_ReturnClientPara,buffer + sizeof(Client_Data),sizeof(CTS_Return_Client_Para));
//            m_cts.m_ReturnClientPara.type = 0x08;
//            printf("0---0:%d\n",Current.data[0].value[0].DValue);
//            stime(&m_cts.m_ReturnClientPara.count);
//            if(m_cts.m_ReturnClientPara.sc_polltime < SC_POLLTIME_MIN)
//                m_cts.m_ReturnClientPara.sc_polltime = SC_POLLTIME_MIN;
//            interval.it_interval.tv_sec = m_cts.m_ReturnClientPara.sc_polltime;
//            interval.it_interval.tv_usec = 0;
//            interval.it_value = interval.it_interval;
//        }
//    }
//
    // 探测子板类型
    for(int i = 0;i < SLOTNUMBER;i++)
    {
        DetectSlotType(i);
    }

    signal(SIGALRM, &timer_func);
    signal(SIGIO, &SigIO);

    //启动线程
    pthread_create(&tid,&tattr,thread_func,NULL);
    //启动定时器
    setitimer(ITIMER_REAL, &interval, NULL);

    //启动Socket监听
  //  msock = passiveTCP(LISTENPORT, QLEN);
    while(1) 
    {
   //     ssock = accept(msock, (struct sockaddr *)&fsin, &alen);
    //    if(ssock < 0)
     //       errexit("accept failed: %s\n", strerror(errno));
      //  len = read(ssock,buffer,BUFFERSIZE);
       // if(len != 0)
        //{
         //   ProcessCmd(ssock,buffer,len);
       // }
    }
    //printf("%d\n",m_cts.m_ReturnBuffer.GetSize());
    //结束
    return 0;
}

/** 
 * @brief 处理服务器命令
 * 
 * @param ssocket 服务器端Socket
 * @param buff 读取的数据
 */
void ProcessCmd(int ssocket,unsigned char *buff,int len)
{
    switch(buff[0])
    {
        case 0x01:
            if(len == m_stc.m_SetDDevice.GetSize())
            {
                memcpy(&m_stc.m_SetDDevice,buff,len);
                unsigned char slot,channel,value;
                bool success = true;
                state_t state;
                //pthread_mutex_lock(&flag);
                pthread_mutex_lock(&mutex);
                for(int i = 0;i < m_stc.m_SetDDevice.valid_num;i++)
                {
                    slot = m_stc.m_SetDDevice.data[i].slot;
                    channel = m_stc.m_SetDDevice.data[i].channel;
                    value = m_stc.m_SetDDevice.data[i].value;
                    if(slot < SLOTNUMBER && channel < DIGITALOUT_CHANNEL_NUM)
                    {
                        if(slot_type[slot] == BOARD_DOUT && sd[slot] > 0)
                        {
                            Current.data[slot].value[channel].DValue = value;
                        }
                        else
                        {
                            success = false;
                        }
                    }
                    else
                    {
                        success = false;
                    }
                }
                pthread_mutex_unlock(&mutex);
                //pthread_mutex_unlock(&flag);
                if(success)
                {
                    m_cts.m_ReturnSuccess.data = SetSuccess;
                    write(ssocket,&m_cts.m_ReturnSuccess,m_cts.m_ReturnSuccess.GetSize());
                }
                else
                {
                    m_cts.m_ReturnError.data = SetError;
                    write(ssocket,&m_cts.m_ReturnError,m_cts.m_ReturnError.GetSize());
                }
            }
            break;
        case 0x03:
            if(len == m_stc.m_SetADevice.GetSize())
            {
                memcpy(&m_stc.m_SetADevice,buff,len);
                unsigned char slot,channel;
                float value;
                bool success = true;
                state_t state;
                //pthread_mutex_lock(&flag);
                pthread_mutex_lock(&mutex);
                for(int i = 0;i < m_stc.m_SetADevice.valid_num;i++)
                {
                    slot = m_stc.m_SetADevice.data[i].slot;
                    channel = m_stc.m_SetADevice.data[i].channel;
                    value = m_stc.m_SetADevice.data[i].value;
                    if(slot < SLOTNUMBER && channel < ANALOGYOUT_CHANNEL_NUM)
                    {
                        if(slot_type[slot] == BOARD_AOUT && sd[slot] > 0)
                        {
                            int d = int(((value + 10) * 16384) / 20);
                            Current.data[slot].value[channel].AValue = value;
                        }
                        else
                        {
                            success = false;
                        }
                    }
                    else
                    {
                        success = false;
                    }
                }
                pthread_mutex_unlock(&mutex);
                //pthread_mutex_unlock(&flag);
                if(success)
                {
                    m_cts.m_ReturnSuccess.data = SetSuccess;
                    write(ssocket,&m_cts.m_ReturnSuccess,m_cts.m_ReturnSuccess.GetSize());
                }
                else
                {
                    m_cts.m_ReturnError.data = SetError;
                    write(ssocket,&m_cts.m_ReturnError,m_cts.m_ReturnError.GetSize());
                }
            }
            break;
        case 0x05:
            if(len == m_stc.m_GetClientData.GetSize())
            {
                memcpy(&m_stc.m_GetClientData,buff,len);

                pthread_mutex_lock(&mutex);
                memcpy(&m_cts.m_ReturnClientData.data,&Current,sizeof(Client_Data));
                pthread_mutex_unlock(&mutex);

                write(ssocket,&m_cts.m_ReturnClientData,m_cts.m_ReturnClientData.GetSize());
            }
            break;
        case 0x07:
            if(len == m_stc.m_GetClientPara.GetSize())
            {
                pthread_mutex_lock(&mutex);
                memcpy(&m_stc.m_GetClientPara,buff,len);
                m_cts.m_ReturnClientPara.count = time(NULL);
                pthread_mutex_unlock(&mutex);
                write(ssocket,&m_cts.m_ReturnClientPara,m_cts.m_ReturnClientPara.GetSize());
            }
            break;
        case 0x09:
            if(len == m_stc.m_SetClientPara.GetSize())
            {
                memcpy(&m_stc.m_SetClientPara,buff,len);

                //pthread_mutex_lock(&flag);
                pthread_mutex_lock(&mutex);
                unsigned char type = m_cts.m_ReturnClientPara.type;
                memcpy(&m_cts.m_ReturnClientPara,buff,len);
                m_cts.m_ReturnClientPara.type = type;
                if(m_cts.m_ReturnClientPara.sc_polltime < SC_POLLTIME_MIN)
                    m_cts.m_ReturnClientPara.sc_polltime = SC_POLLTIME_MIN; 
                //stime(&m_cts.m_ReturnClientPara.count);
                interval.it_interval.tv_sec = m_cts.m_ReturnClientPara.sc_polltime;
                interval.it_interval.tv_usec = 0;
                interval.it_value = interval.it_interval;
                setitimer(ITIMER_REAL, &interval, NULL);
                pthread_mutex_unlock(&mutex);
                //pthread_mutex_unlock(&flag);

                m_cts.m_ReturnSuccess.data = SetSuccess;
                write(ssocket,&m_cts.m_ReturnSuccess,m_cts.m_ReturnSuccess.GetSize());
            }
            break;
        case 0x0B:
            if(len == m_stc.m_GetChannelPara.GetSize())
            {
                memcpy(&m_stc.m_GetChannelPara,buff,len);
                unsigned char slot,channel;
                slot = m_stc.m_GetChannelPara.slot;
                channel = m_stc.m_GetChannelPara.channel;
                bool success = true;
                if(slot < SLOTNUMBER && channel < CHANNEL_MAX)
                {
                    if(slot_type[slot] == BOARD_AIN)
                    {
                        pthread_mutex_lock(&mutex);
                        m_cts.m_ReturnChannelPara.data.threshold = m_cts.m_ReturnClientPara.data[slot].data[channel].threshold;
                        m_cts.m_ReturnChannelPara.data.gain = m_cts.m_ReturnClientPara.data[slot].data[channel].gain;
                        pthread_mutex_unlock(&mutex);
                    }
                    else
                    {
                        success = false;
                    }
                }
                else
                {
                    success = false;
                }
                if(success)
                {
                    write(ssocket,&m_cts.m_ReturnChannelPara,m_cts.m_ReturnChannelPara.GetSize());
                }
                else
                {
                    m_cts.m_ReturnError.data = GetError;
                    write(ssocket,&m_cts.m_ReturnError,m_cts.m_ReturnError.GetSize());
                }
            }
            break;
        case 0x0D:
            if(len == m_stc.m_SetChannelPara.GetSize())
            {
                memcpy(&m_stc.m_SetChannelPara,buff,len);
                unsigned char slot,channel;
                slot = m_stc.m_SetChannelPara.slot;
                channel = m_stc.m_SetChannelPara.channel;
                pthread_mutex_lock(&flag);
                pthread_mutex_lock(&mutex);
                if(slot < SLOTNUMBER && channel < CHANNEL_MAX)
                {
                    m_cts.m_ReturnClientPara.data[slot].data[channel].threshold = m_stc.m_SetChannelPara.data.threshold;
                    m_cts.m_ReturnClientPara.data[slot].data[channel].gain = m_stc.m_SetChannelPara.data.gain;
                    m_cts.m_ReturnSuccess.data = SetSuccess;
                    write(ssocket,&m_cts.m_ReturnSuccess,m_cts.m_ReturnSuccess.GetSize());
                }
                else
                {
                    m_cts.m_ReturnError.data = ParaError;
                    write(ssocket,&m_cts.m_ReturnError,m_cts.m_ReturnError.GetSize());
                }
                pthread_mutex_unlock(&mutex);
                pthread_mutex_unlock(&flag);
            }
            break;
        case 0x0F:
            if(len = m_stc.m_GetBuffer.GetSize())
            {
                pthread_mutex_lock(&mutex);
                Array.CopyTo(&m_cts.m_ReturnBuffer);
                pthread_mutex_unlock(&mutex);

                write(ssocket,&m_cts.m_ReturnBuffer,m_cts.m_ReturnBuffer.GetSize());
            }
            break;
        case 0x11:
            if(len = m_stc.m_SetPollTime.GetSize())
            {
                memcpy(&m_stc.m_SetPollTime,buff,len);
                if(m_stc.m_SetPollTime.sc_polltime < SC_POLLTIME_MIN)
                    m_stc.m_SetPollTime.sc_polltime = SC_POLLTIME_MIN; 
                m_cts.m_ReturnClientPara.sc_polltime = m_stc.m_SetPollTime.sc_polltime;
                m_cts.m_ReturnClientPara.cs_polltime = m_stc.m_SetPollTime.cs_polltime;
                interval.it_interval.tv_sec = m_cts.m_ReturnClientPara.sc_polltime;
                interval.it_interval.tv_usec = 0;
                interval.it_value = interval.it_interval;
                setitimer(ITIMER_REAL, &interval, NULL);
                m_cts.m_ReturnSuccess.data = SetSuccess;
                write(ssocket,&m_cts.m_ReturnSuccess,m_cts.m_ReturnSuccess.GetSize());
            }
            break;
        case 0x13:
            if(len = m_stc.m_GetPollTime.GetSize())
            {
                m_cts.m_ReturnPollTime.sc_polltime = m_cts.m_ReturnClientPara.sc_polltime;
                m_cts.m_ReturnPollTime.cs_polltime = m_cts.m_ReturnClientPara.cs_polltime;
                write(ssocket,&m_cts.m_ReturnPollTime,m_cts.m_ReturnPollTime.GetSize());
            }
            break;
        case 0x15:
            if(len = m_stc.m_ResetClock.GetSize())
            {
                memcpy(&m_stc.m_ResetClock,buff,len);
                stime(&m_stc.m_ResetClock.count);
                m_cts.m_ReturnSuccess.data = SetSuccess;
                write(ssocket,&m_cts.m_ReturnSuccess,m_cts.m_ReturnSuccess.GetSize());
            }
            break;
        default:
            break;
    }
    (void)close(ssocket);
}

/** 
 * @brief 探测子板类型
 * 
 * @param slot 子板插槽编号(0-SLOTNUMBER - 1)
 */
void DetectSlotType(int slot)
{
    if(slot < 0 || slot > SLOTNUMBER - 1)
    {
        return;
    }
    //board_t slot_get_board_type(slot_descriptor_t sd);
    sd[slot] = slot_open(slot);
    if(sd[slot] < 0)
    {
        slot_type[slot] = BOARD_UNKNOWN;
    }
    else
    {
        slot_type[slot] = slot_get_board_type(sd[slot]);
        if(slot_type[slot] == BOARD_UNKNOWN)
        {
            if(slot_resync(sd[slot]) == 0)
            {
                slot_type[slot] = slot_get_board_type(sd[slot]);
            }
        }
    }

    switch(slot_type[slot])
    {
        case BOARD_DIN:
            sd[slot] = din_open(sd[slot]);
            m_cts.m_ReturnClientData.data.data[slot].type = DigitalDeviceIN;
	    Current.data[slot].type = DigitalDeviceIN;
            m_cts.m_ReturnClientPara.data[slot].type = DigitalDeviceIN;
            break;
        case BOARD_DOUT:
            sd[slot] = dout_open(sd[slot]);
            m_cts.m_ReturnClientData.data.data[slot].type = DigitalDeviceOUT;
	    Current.data[slot].type = DigitalDeviceOUT;
            m_cts.m_ReturnClientPara.data[slot].type = DigitalDeviceOUT;
            break;
        case BOARD_AIN:
            sd[slot] = ain_open(sd[slot]);
            m_cts.m_ReturnClientData.data.data[slot].type = AnalogyDeviceIN;
            Current.data[slot].type = AnalogyDeviceIN;
            m_cts.m_ReturnClientPara.data[slot].type = AnalogyDeviceIN;
            break;
        case BOARD_AOUT:
            sd[slot] = aout_open(sd[slot]);
            m_cts.m_ReturnClientData.data.data[slot].type = AnalogyDeviceOUT;
            Current.data[slot].type = AnalogyDeviceOUT;
            m_cts.m_ReturnClientPara.data[slot].type = AnalogyDeviceOUT;
            break;
        case BOARD_UNKNOWN:
            m_cts.m_ReturnClientData.data.data[slot].type = NoDevice;
            Current.data[slot].type = NoDevice;
            m_cts.m_ReturnClientPara.data[slot].type = NoDevice;
            break;
        default:
            break;
    }
}
/** 
 * @brief 初始化子板
 */
void InitSlot()
{
    //printf("Init Slot!\n");
    //for(int i = 0;i < SLOTNUMBER;i++)
    //{
    //if(slot_type[i] == BOARD_AIN && sd[i] > 0)
    //{
    //for(int j = 0;j < ANALOGY_CHANNEL_NUM;j++)
    //{
    //unsigned gain = m_cts.m_ReturnClientPara.data[i].data[j].gain;
    //ain_set_channel_gain(sd[i], j, gain);
    //}
    //}
    //}
}

/** 
 * @brief 轮询子板
 * 
 * @param p 子站临时数据
 */
void PollSlot(Client_Data *p)
{
    int s,c;
    p->t = time(NULL);
    Client_Data temp;
    int x;
    int value = 0;
    for(int i = 0;i < SLOTNUMBER;i++)
    {
        if(slot_type[i] != BOARD_UNKNOWN && sd[i] > 0)
        {
            switch(slot_type[i])
            {
                case BOARD_DIN:
                    value = din_read_all(sd[i]);
                    if(value < 0)
                    {
                        if(value == ERROR_BOARD_OFFLINE)
                        {
                            slot_type[i] = BOARD_UNKNOWN;
                            p->data[i].type = NoDevice;
                            SendSlotOffLine(i);
                            break;
                        }
                        slot_resync(sd[i]);
                        value = din_read_all(sd[i]);
                    }
                    if(value >= 0)
                    {
                        x = 1;
                        for(int j = 0;j < DIGITALIN_CHANNEL_NUM;j++) 
                        {
                            if(value & x)
                            {
                                p->data[i].value[j].DValue = 1;
                            }
                            else
                            {
                                p->data[i].value[j].DValue = 0;
                            }
                            if(Current.data[i].value[j].DValue != p->data[i].value[j].DValue)
                            {
                                    m_cts.m_Changes.changes[changes_count].type = DigitalDeviceIN;
                                    m_cts.m_Changes.changes[changes_count].slot = i;
                                    m_cts.m_Changes.changes[changes_count].channel = j;
                                    m_cts.m_Changes.changes[changes_count].value.DValue = p->data[i].value[j].DValue;
                                    printf("DDDDDD=%f    %f\n",m_cts.m_Changes.changes[changes_count].value.AValue,p->data[i].value[j].AValue);
                                    changes_count++;
                            }
                            x = x << 1;
                        }
                    }
                    break;
                case BOARD_AIN:
                    for(int j = 0;j < ANALOGYIN_CHANNEL_NUM;j++)
                    {
                        unsigned gain = m_cts.m_ReturnClientPara.data[i].data[j].gain;
                        state_t state = ain_set_channel_gain(sd[i],j,0);
                        if(state < 0)
                        {
                            if(state == ERROR_BOARD_OFFLINE)
                            {
                                slot_type[i] = BOARD_UNKNOWN;
                                p->data[i].type = NoDevice;
                                SendSlotOffLine(i);
                                break;
                            }
                            slot_resync(sd[i]);
                        }
                        if(state >= 0)
                        {
                            int count = 0;
                            while((value = ain_get_value(sd[i])) == ERROR_BOARD_BUSY)
                            {
                                if(count++ > 10)
                                    break;
                            }
                            if(value >= 0)
                            {
                                char *p1 = (char*)&value;
                                short int newd;
                                memcpy(&newd,p1,2);
                                float t = (newd / 4.0) * LSB;
                                printf("channel = %d;value = %X;float = %f\n",j,value,t);
                                p->data[i].value[j].AValue = t;
                                if(fabs(Current.data[i].value[j].AValue - p->data[i].value[j].AValue) > m_cts.m_ReturnClientPara.data[i].data[j].threshold)
                                {
                                    m_cts.m_Changes.changes[changes_count].type = AnalogyDeviceIN;
                                    m_cts.m_Changes.changes[changes_count].slot = i;
                                    m_cts.m_Changes.changes[changes_count].channel = j;
                                    m_cts.m_Changes.changes[changes_count].value.AValue = p->data[i].value[j].AValue;
                                    printf("AAAAAA=%d     %d    %f    %f\n",i,j,m_cts.m_Changes.changes[changes_count].value.AValue,p->data[i].value[j].AValue);
                                    changes_count++;
                                }
                            }
                            else
                            {
                                slot_type[i] = BOARD_UNKNOWN;
                                p->data[i].type = NoDevice;
                                break;
                            }
                        }
                    }
                    break;
                case BOARD_DOUT:
                    value = 0;
                    x = 1;
                    for(int j = 0;j < DIGITALOUT_CHANNEL_NUM;j++)
                    {
                        if(p->data[i].value[j].DValue == 1)
                        {
                            value = value | x;
                        }
                        x = x << 1;
                    }
                    state_t state = dout_write_all(sd[i],value);
                    c = 0;
                    if(state < 0)
                    {
                        if(state == ERROR_BOARD_OFFLINE)
                        {
                            slot_type[i] = BOARD_UNKNOWN;
                            p->data[i].type = NoDevice;
                            SendSlotOffLine(i);
                            break;
                        }
                        slot_resync(sd[i]);
                        state = dout_write_all(sd[i],value);
                    }
                    break;
                case BOARD_AOUT:
                    float value;
                    int d;
                    for(int j = 0;j < ANALOGYOUT_CHANNEL_NUM;j++)
                    {
                        value = p->data[i].value[j].AValue;
#ifdef DEBUG
                        printf("sd = %X,slot = %d,channel = %d,value = %f\n",sd[i],i,j,value);
#endif
                        d = int(value * 500);

                        state_t state;
                        while((state = aout_set_channel_value(sd[i],j,d)) == ERROR_BOARD_BUSY)
                        {
#ifdef DEBUG
                            printf(".");
#endif
                        }
#ifdef DEBUG
                        printf("\n");
#endif

                        c = 0;
                        if(state < 0)
                        {
                            if(state == ERROR_BOARD_OFFLINE)
                            {
#ifdef DEBUG
                                printf("**********************************Board %d Offline!\n",i);
#endif
                                slot_type[i] = BOARD_UNKNOWN;
                                p->data[i].type = NoDevice;
                                SendSlotOffLine(i);
                                c = -1;
                                break;
                            }
                            if(c == 1)
                            {
#ifdef DEBUG
                                printf("Analogy Slot %d(0-7) Write Error=%d SD=%X!\n",i,state,sd[i]);
#endif
                                c = -1;
                                break;
                            }
                            c++;
#ifdef DEBUG
                            printf("Resync!\n");
#endif
                            slot_resync(sd[i]);
                            while((state = aout_set_channel_value(sd[i],j,d)) == ERROR_BOARD_BUSY)
                            {
#ifdef DEBUG
                                printf(".");
#endif
                            }
#ifdef DEBUG
                            printf("\n");
#endif
                        }
                        if(c == -1)
                            break;
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

/** 
 * @brief 线程函数，处理轮询子板任务
 * 
 * @param data 参数 无
 * 
 * @return 无
 */
void* thread_func(void* data)
{
    Client_Data temp;
    while(1)
    {
        pthread_mutex_lock(&flag);

        pthread_mutex_lock(&mutex);
        memcpy(&temp,&Current,sizeof(Client_Data));
        pthread_mutex_unlock(&mutex);

        ///////////////////////////////////////////////////////////////////////////////
        //int Qsize = Sizequeue(Q);
        //for(int i = 0;i < Qsize;i++)
        //{
            //int s;
            //ElemType e;
            //s = connectTCP(SERVERIP, SERVERPORT);
            //if(s > 0)
            //{
                //Dequeue(Q,e);
                //unsigned char slot = e.slot;
                //unsigned char channel = e.channel;
                //if(slot < SLOTNUMBER)
                //{
                    //if(slot_type[slot] == BOARD_DIN)
                    //{
                        //m_cts.m_ReturnDDevice.slot = slot;
                        //m_cts.m_ReturnDDevice.channel = channel;
                        //m_cts.m_ReturnDDevice.data = e.value.DValue;
                        //write(s,&m_cts.m_ReturnDDevice,m_cts.m_ReturnDDevice.GetSize());
                    //} else if(slot_type[slot] == BOARD_AIN)
                    //{
                        //m_cts.m_ReturnADevice.slot = slot;
                        //m_cts.m_ReturnADevice.channel = channel;
                        //m_cts.m_ReturnADevice.data = e.value.AValue;
                        //write(s,&m_cts.m_ReturnADevice,m_cts.m_ReturnADevice.GetSize());
                    //}
                //}
                //close(s);
            //}
            //else
            //{
                //break;
            //}
        //}
        ///////////////////////////////////////////////////////////////////////////////

        changes_count = 0;

        PollSlot(&temp);
        unsigned char serial_buff[8];
        int fd=0;
        fd=open_port(fd,1);
        set_opt(fd,9600,8,'N',1);

        printf("\n\n");
        for(int j=0;j<=0;j++)
        {
            int value=0;
            state_t state = ain_set_channel_gain(sd[0],j,0);
            value=ain_get_value(sd[0]);
            printf("ddddddddddddddddd%x  %d\n",value,j);
            serial_buff[0]=0x01;
            serial_buff[1]=0x03;
            serial_buff[2]=0x02;
            serial_buff[3]=value>>8;
            serial_buff[4]=value;
            crc(5,serial_buff);
            write(fd,serial_buff,7);
        }
        close(fd);
        printf("\n\n");
        if(changes_count)
        {
            int s;
            s = connectTCP(SERVERIP, SERVERPORT);
            if(s > 0)
            {
                m_cts.m_Changes.time = time(NULL);
                write(s,&m_cts.m_Changes,1 + sizeof(time_t) + changes_count * sizeof(struct Message));
            }
            close(s);
        }

        pthread_mutex_lock(&mutex);
        for(int i = 0;i < SLOTNUMBER;i++)
        {
            switch(slot_type[i])
            {
                case BOARD_DOUT:
		    Current.data[i].type = DigitalDeviceOUT;
		    break;
                case BOARD_AOUT:
		    Current.data[i].type = AnalogyDeviceOUT;
		    break;
                case BOARD_DIN:
                    Current.data[i] = temp.data[i];
	            Current.data[i].type = DigitalDeviceIN;
		    break;
                case BOARD_AIN:
                    Current.data[i] = temp.data[i];
		    Current.data[i].type = AnalogyDeviceIN;
		    break;
                case BOARD_UNKNOWN:
		    Current.data[i].type = NoDevice;
		    break;
		default:
		    Current.data[i].type = NoDevice;

            }
        }
        Current.t = time(NULL);
        if(WriteArrayCount > 0)
        {
            WriteArrayCount--;
            Array.AddData(&Current);
        }
        pthread_mutex_unlock(&mutex);

        pthread_mutex_unlock(&flag);
//#ifdef DEBUG
        timeval ccc;
        gettimeofday(&ccc,NULL);
        //printf("time1=%u   time2=%u\n",ccc.tv_sec,ccc.tv_usec);
        //printf("time=%u\n",Current.t);
//#endif
        usleep(5000);
    }
}

/** 
 * @brief 定时器
 * 
 * @param n 信号
 */
void timer_func(int n)
{
    //pthread_mutex_lock(&mutex);
    WriteArrayCount++;
    printf("time");
    //pthread_mutex_unlock(&mutex);
}


/** 
 * @brief 读取配置文件
 * 
 * @param filename 文件名
 * 
 * @return 
 */
int ReadIniFile(char *filename)
{
    int ret = -1;
    char temp[4];
    ret = ConfigGetKey(filename, "SOCKET", "ServerIP", SERVERIP);
    ret = ConfigGetKey(filename, "SOCKET", "ServerPort", SERVERPORT);
    ret = ConfigGetKey(filename, "SOCKET", "ListenPort", LISTENPORT);
    ret = ConfigGetKey(filename, "INFO", "SlotNumber", temp);
    SLOTNUMBER = atoi(temp);
    return ret;
}

/** 
 * @brief SIGIO信号处理函数
 * 
 * @param n SIGIO信号
 */
void SigIO(int n)
{
    pthread_mutex_lock(&flag);
    pthread_mutex_lock(&mutex);
    for(int i = 0;i < SLOTNUMBER;i++)
    {
        if(slot_type[i] == BOARD_UNKNOWN && sd[i] > 0)
        {
            slot_close(sd[i]);
            DetectSlotType(i);
            if(slot_type[i] != BOARD_UNKNOWN)
            {
                SendSlotOnLine(i);
            }
        }
    }
    pthread_mutex_unlock(&mutex);
    pthread_mutex_unlock(&flag);
}

/** 
 * @brief 发送子板离线消息
 * 
 * @param slot 子板编号
 */
void SendSlotOffLine(unsigned char slot)
{
    int s;
    s = connectTCP(SERVERIP, SERVERPORT);
    if(s > 0)
    {
        m_cts.m_ReturnOffLine.time = time(NULL);
        m_cts.m_ReturnOffLine.slot = slot;
        write(s,&m_cts.m_ReturnOffLine,m_cts.m_ReturnOffLine.GetSize());
        close(s);
    }
}

/** 
 * @brief 发送子板上线消息
 * 
 * @param slot 子板编号
 */
void SendSlotOnLine(unsigned char slot)
{
    int s;
    s = connectTCP(SERVERIP, SERVERPORT);
    if(s > 0)
    {
        m_cts.m_ReturnOnLine.time = time(NULL);
        m_cts.m_ReturnOnLine.slot = slot;
        write(s,&m_cts.m_ReturnOnLine,m_cts.m_ReturnOnLine.GetSize());
        close(s);
    }
}
