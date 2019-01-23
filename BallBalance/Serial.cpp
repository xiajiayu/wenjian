//
// Created by xiajiayu on 19-1-21.
//
//串口相关的头文件
//首先定义无符号字节型
#include"Serial.h"
#ifndef BYTE
#define BYTE unsigned char
#endif
BYTE sendata[30];
int Serial_port::setOpt(int fd)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    { return -1; }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;//终端读使能
    newtio.c_cflag &= ~CSIZE;

    switch (nBits)//设置数据位数
    {
        case 7:
            newtio.c_cflag |= CS7;//7位
            break;
        case 8:
            newtio.c_cflag |= CS8;//8位
            break;
    }

    switch (nEvent)//设置校验位
    {
        case 'O':                     //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':                     //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':                    //无校验
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch (nSpeed)//设置波特率
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 57600:
            cfsetispeed(&newtio, B57600);
            cfsetospeed(&newtio, B57600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }
    if (nStop == 1)//设置停止位
    {                newtio.c_cflag &= ~CSTOPB;        }
    else if (nStop == 2)
    {                newtio.c_cflag |= CSTOPB;        }
    newtio.c_cc[VTIME] = 0;//设置缓冲区字符读取等待时间
    newtio.c_cc[VMIN] = 0;//设置读取缓冲区字节数限制
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
        return -1;
    else
        return 0;
}
//******将double转成BYTE类型，这里只使用前四位**************
void double2byte(BYTE *hexdata, double ddata)
{
    unsigned char str[128];
    sprintf((char*)str, "%f", ddata);
    hexdata[0] = str[0];
    hexdata[1] = str[1];
    hexdata[2] = str[2];
    hexdata[3] = str[3];
}


//在main()函数中添加以下内容：
//检查串口是否连接并设置串口属性
void Serial_port::Test() {
    int fd, fcom;
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) printf("Cannot find tty0 !!\n");
    fcom = FCcom.setOpt(fd);
    if (fcom == -1) printf("Cannot set option !!\n");

    sendata[0] = 0x3A;//数据头1
    sendata[1] = 0xA3;//数据头2
    sendata[2] = 0x13;//数据长度
    sendata[3] = 0x1A;//命令ID

//将double x,y,z依次转成byte类型
    double2byte(&sendata[4], X);
    double2byte(&sendata[8], Y);
    double2byte(&sendata[12],speed);


//发送到串口并检查是否发送成功
int nr;
    nr = write(fd, sendata, 19);
    if (nr > 0) printf("send data to uart success! nr=%d\n", nr);
    else printf("\n send data to uart fail!");
}