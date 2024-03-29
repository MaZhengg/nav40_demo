//创作者：尚彬彬 2019.6.24
//如有疑问请加交流qq群：903013799
//程序主要功能：
//1、linux下串口读取 2、循环队列的构造 3、帧结构用结构体存储
//此代码配合  nav40_demo.h使用
#include <fcntl.h>      //open函数的头文件
#include <termios.h>    //串口驱动函数
#include <unistd.h>
#include <errno.h>    
#include <stdio.h>      //标准输入输出头文件
#include <iostream>
#include <string.h>
#include "nav40_demo.h"
#include <stdint.h>


using namespace std;

#define NAV_DL     122    //数据长度
#define NAV_DH1    0xEB   //帧头
#define NAV_DH2    0x90   //帧头
#define MAXSIZE    1024   //缓冲区长度

typedef struct
{
	unsigned char Recbuf[MAXSIZE];
	int tail;              //尾指针
	int head;              //头指针
}Suqueue;

Suqueue queue_cycle;
APM_Datatype APM;
unsigned int checksum = 0;  //校验和
unsigned int checkRes_L, checkRes_H; //4个字节
unsigned char temp_buf[122]={0};


//设置波特率，初始化串口
int set_uart_baudrate(const int _fd, unsigned int baud)
{
	int speed;
	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	tcgetattr(_fd, &uart_config);

	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~CRTSCTS;

	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	uart_config.c_iflag = 0;

	uart_config.c_oflag = 0;

	uart_config.c_cc[VTIME] = 0;
	uart_config.c_cc[VMIN] = 1;

	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		return 0;
	}

	return 1;
}


int main(int argc, char ** argv)
{
	int fd = open("/dev/ttyUSB0", O_RDWR);
	//Suqueue queue_cycle;
	memset(queue_cycle.Recbuf, 0, MAXSIZE);            //初始化缓冲数组
	queue_cycle.tail = 0;                           //初始化缓冲数组指针
	queue_cycle.head = 0;
	if (fd == -1)
	{
		printf("open error.\n");
		return 0;
	}
	set_uart_baudrate(fd, 115200);
        int a=0;
        
	while (1)
	{
		unsigned char buf[1];
		int len = read(fd, buf, 1);
                memcpy(queue_cycle.Recbuf + queue_cycle.tail, buf, len);
                queue_cycle.tail=(queue_cycle.tail+1)%MAXSIZE;     

		while ((queue_cycle.tail>queue_cycle.head && queue_cycle.tail - queue_cycle.head >=2*NAV_DL) || (queue_cycle.tail<queue_cycle.head && (MAXSIZE -queue_cycle.head + queue_cycle.tail )>= 2*NAV_DL))
		{
				if (queue_cycle.Recbuf[queue_cycle.head] == NAV_DH1)   //校验帧头
				{
 					queue_cycle.head = (queue_cycle.head + 1 )% MAXSIZE;
					if (queue_cycle.Recbuf[queue_cycle.head ] == NAV_DH2)   //校验帧头
					{
						queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
						for (int k = 0; k <= 117; k++)
						{
							checksum += queue_cycle.Recbuf[queue_cycle.head];
							queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
						}
						checksum = checksum + 0xEB + 0x90;
						checkRes_L = checksum & 0x00ff;
						checkRes_H = (checksum >> 8) & 0x00ff;
						checksum = 0;       //必须清零
                                                
						if (queue_cycle.Recbuf[queue_cycle.head] == checkRes_L && queue_cycle.Recbuf[(queue_cycle.head + 1) % MAXSIZE] == checkRes_H)
						{
                                                          for (int j = 121; j>=0; j--)
							{
                                                                temp_buf[121-j]= queue_cycle.Recbuf[(queue_cycle.head + MAXSIZE - j+1) % MAXSIZE];
							}
                                                        memcpy(&APM, temp_buf,122 );  //注入 缓冲区                                                     
                                                         //在这里访问结构体成员即可      
                                                      //printf("apm_counter:%d\r\n",APM.counter);
                                                      printf("----------------\n-------------\n");
                                                      //printf("apm_pitch2:%f\r\n",APM.pitch*180/3.1415926);
                                                      printf("apm_yaw2:%f\r\n",APM.yaw);
                                                      //cout<<APM.pitch*180/3.1415926<<endl;
						}
					}
				}
                                else queue_cycle.head = (queue_cycle.head + 1 )% MAXSIZE;			
		}

	}
}

