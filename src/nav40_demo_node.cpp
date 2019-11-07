/*//创作者：尚彬彬 2019.6.24
//如有疑问请加交流qq群：903013799
//本程序主要功能，
1、串口读取、2循环队列缓冲、3拷贝到帧结构体 4、ros发布 5、自定义消息类型
梳理代码逻辑的可以参考我下面的两篇博文
https://mp.weixin.qq.com/s/S134o9ofs843DG4PEGMppQ
https://mp.weixin.qq.com/s/a9DHDKsowaYarL1EcI8Hrw
nav30/40/50通用，使用时直接访问结构体成员即可
////此代码配合  nav40_demo.h和nav40_msg.msg使用
关于串口、读取

#### c_cflag 标志常量：Control mode ( 控制模式)

Control mode主要用于控制终端设备的硬件设置。利用termios结构的c_cflag的标志来加以控制。控制模式用在序列线连接到数据设备，也可以用在与终端设备的交谈。

一般来说，改变终端设备的组态要比使用termios的控制模式来改变行(lines)的行为来得容易。

CBAUD ：(不属于 POSIX) 波特率掩码 (4+1 位)。
CBAUDEX ：(不属于 POSIX) 扩展的波特率掩码 (1 位)，包含在 CBAUD 中。
(POSIX 规定波特率存储在 termios 结构中，并未精确指定它的位置，而是提供了函数 cfgetispeed() 和 cfsetispeed() 来存取它。一些系统使用 c_cflag 中 CBAUD 选择的位，其他系统使用单独的变量，例如 sg_ispeed 和 sg_ospeed 。)
CSIZE：字符长度掩码（传送或接收字元时用的位数）。取值为 CS5（传送或接收字元时用5bits）, CS6, CS7, 或 CS8。
CSTOPB ：设置两个停止位，而不是一个。
CREAD ：打开接受者。
PARENB ：允许输出产生奇偶信息以及输入的奇偶校验（启用同位产生与侦测）。
PARODD ：输入和输出是奇校验（使用奇同位而非偶同位）。
HUPCL ：在最后一个进程关闭设备后，降低 modem 控制线 (挂断)。(?)
CLOCAL ：忽略 modem 控制线。
LOBLK :(不属于 POSIX) 从非当前 shell 层阻塞输出(用于 shl )。(?)
CIBAUD :(不属于 POSIX) 输入速度的掩码。CIBAUD 各位的值与 CBAUD 各位相同，左移了 IBSHIFT 位。
CRTSCTS :(不属于 POSIX) 启用 RTS/CTS (硬件) 流控制。
*/
#include <ros/ros.h>
#include <fcntl.h>      //open函数的头文件
#include <termios.h>    //串口驱动函数
#include <unistd.h>
#include <errno.h>    
#include <stdio.h>      //标准输入输出头文件
#include <string.h>
#include "std_msgs/Float32.h"
#include "nav40_demo.h"
#include "nav40_demo/nav40_msg.h"  //自定义消息文件

#include <sensor_msgs/Imu.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>



using namespace std;

#define NAV_DL     122    //数据长度
#define NAV_DH1    0xEB   //帧头
#define NAV_DH2    0x90   //帧头
#define MAXSIZE    1024   //缓冲区长度

typedef struct
{
	unsigned char Recbuf[MAXSIZE];  //缓冲数组
	int tail;              //尾指针
	int head;              //头指针
}Suqueue;

Suqueue queue_cycle;          //创建缓冲数组            
APM_Datatype APM;            //创建帧结构体
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

	tcgetattr(_fd, &uart_config); //获取终端参数

  uart_config.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
  uart_config.c_cflag &= ~PARENB;  /* no parity bit */
  uart_config.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
  uart_config.c_cflag &= ~CSIZE;
  uart_config.c_cflag |= CS8;    /* 8-bit characters */
  uart_config.c_cflag &= ~CRTSCTS;/* no hardware flowcontrol */

  /* setup for non-canonical mode */
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	uart_config.c_iflag = 0;
	uart_config.c_oflag = 0;

  /* fetch bytes as they become available */
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

#define epsilon 0.00001
#define COUNT 100
double fun(double x)
{
    return x*x;
}

//Romberg法积分
double Romberg(double last_time,double current_time)
{
    int m ,n;
    double h,x,s,q,ep;
    double p,*R =new double[COUNT];

            h=current_time-last_time;
            R[0]= h*(fun(last_time)+ fun(current_time))/2.0;
            m=1;
            n=1;
            ep=epsilon+1.0;
            while ((ep >= epsilon)&& (m <COUNT))
            {
                p = 0.0;
            {
                for(int i=0;i<n;i++)
                {
                    x = last_time+ (i+0.5)*h ;
                    p= p + fun(x);
                }
                    p= (R[0]+ h*p)/2.0;
                    s = 1.0;
                    for(int k=1;k<=m;k++)
                    {
                        s = 4.0*s;
                        q= (s*p-R[k-1])/(s-1.0);
                            R[k-1]= p;
                            p =q;
                    }
                    p=fabs(q -R[m-1]);
                    m =m + 1;
                    R[m-1]= q;
                    n = n + n;
                    h = h/2.0;
                }
                return (q);
            }
}

int main(int argc, char **argv)
{
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
    //初始化
    ros::init(argc, argv, "nav40_node");
    ros::NodeHandle nh;
    nh.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    nh.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
    nh.param<std::string>("frame_id", frame_id, "imu_link");

    int fd = open("/dev/ttyUSB0", O_RDWR);              //打开串口
    memset(queue_cycle.Recbuf, 0, MAXSIZE);            //初始化缓冲数组
    queue_cycle.tail = 0;                           //初始化缓冲数组指针
    queue_cycle.head = 0;
    if (fd == -1)
    {
       printf("open error.\n");
       return 0;
     }
     set_uart_baudrate(fd, 115200);                 //串口初始化
    //公告消息
    ros::Publisher nav40_pitch = nh.advertise<std_msgs::Float32>
            ("nav40/pitch", 10);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu> ("imu_data",10);
    //设置循环频率
    ros::Rate rate(30.0);

    cout << "open success"<< endl;
	
    while(ros::ok())
    {
        //nav40_demo::nav40_msg Pitch;  //如需自定义消息类型可仿造
        std_msgs::Float32  Pitch;
	
	//循环队列读取串口数据
        unsigned char buf[1];
        int len = read(fd, buf, 1);
	memcpy(queue_cycle.Recbuf + queue_cycle.tail, buf, len);
	queue_cycle.tail = (queue_cycle.tail + 1) % MAXSIZE;
        
       //进入帧结构判断
       //循环队列大于等于2倍的长度，才进入帧结构的判断
        while ((queue_cycle.tail>queue_cycle.head && queue_cycle.tail - queue_cycle.head >= 2 * NAV_DL) || (queue_cycle.tail<queue_cycle.head && (MAXSIZE - queue_cycle.head + queue_cycle.tail) >= 2 * NAV_DL))
		{
			if (queue_cycle.Recbuf[queue_cycle.head] == NAV_DH1)   //校验帧头
			{
				queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
				if (queue_cycle.Recbuf[queue_cycle.head] == NAV_DH2)   //校验帧头
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
					//检验和
					if (queue_cycle.Recbuf[queue_cycle.head] == checkRes_L && queue_cycle.Recbuf[(queue_cycle.head + 1) % MAXSIZE] == checkRes_H)
					{   //校验和通过
					    for (int j = 121; j>=0; j--)
              {
                temp_buf[121-j]= queue_cycle.Recbuf[(queue_cycle.head + MAXSIZE - j+1) % MAXSIZE];
              }
              memcpy(&APM, temp_buf,122 );  // 将一帧完整的数据帧拷贝到结构体
              //在这里访问结构体成员即可
              printf("apm_counter:%d\r\n",APM.counter);
              Pitch.data=APM.pitch*180/3.1415926;
              nav40_pitch.publish(Pitch);//发布消息

              // calculate measurement time
              sensor_msgs::Imu imu_data;
              imu_data.header.stamp = ros::Time::now();
              imu_data.header.frame_id = "imu_link";
              tf::Quaternion q;
              q = tf::createQuaternionFromRPY(APM.roll,APM.pitch,APM.yaw);
              imu_data.orientation.x = q.x();
              imu_data.orientation.y = q.y();
              imu_data.orientation.z = q.z();
              imu_data.orientation.w = q.w();
              imu_data.linear_acceleration.x = APM.accel_x;
              imu_data.linear_acceleration.y = APM.accel_y;
              imu_data.linear_acceleration.z = APM.accel_z;
              imu_data.angular_velocity.x = APM.roll_rate;
              imu_data.angular_velocity.y = APM.pitch_rate;
              imu_data.angular_velocity.z = APM.yaw_rate;

              imu_pub.publish(imu_data);

              geometry_msgs::Pose pose_temp;
              //pose_temp.position.x =
					}
				}
			}
			else queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
		}
    }
    ros::spinOnce();
    rate.sleep();
    return 0;
}
