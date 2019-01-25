//
// Created by xiajiayu on 19-1-20.
//

#include"SerialPort.h"


SerialCom::SerialCom()
{
//    m_frame_flag_pc = 0;
   // m_buff_mode = 0;
    //m_stable = 0;
   // m_frame_flag = 0;
}

SerialCom::~SerialCom()
{
    close(fd);
    std::cout << "-- SerialCom Destructor successfully!" << std::endl;
}
int SerialCom::set_interface_attribs(int fd, int speed, int parity)//波特率校验,不用看
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        // error_message ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;     // no remapping, no delays
    tty.c_cc[VMIN] = 0;  // read doesn't block
    tty.c_cc[VTIME] = 1; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= parity;
    // tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CSTOPB;

    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        // error_message ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void SerialCom::set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        //        error_message ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 1; // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        ;
    //("error %d setting term attributes", errno);
}
int SerialCom::init()
{
    // int fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
    int found_flag = false;
    int fd;
    std::string serial_path = "/dev/ttyUSB0" ;//打开串口
    fd = open(serial_path.c_str(), O_RDWR | O_NOCTTY);
    std::cout << "fd is: " << fd << std::endl;
    set_interface_attribs(fd, B115200, 0); // set speed to 115,200 bps,ot 8n1 (no parity)
    set_blocking(fd, 0); // set no blocking
    return fd;
}
bool SerialCom::open_port()
{
    fd = init();
    if (fd > 0)
    {
        std::cout<<"T\n";
        return true;
    }
    else
    {   std::cout<<"F\n";
        return false;
    }
}

int SerialCom::send(unsigned char *str, int len)
{
    int n = write(this->fd, str, len);
    return n;
}
void SerialCom::getdata(){

    sendata[0] = '!';//数据头1

//将double x,y,z依次转成byte类型
    double2byte(&sendata[1], X);
    double2byte(&sendata[5], Y);
   // double2byte(&sendata[9],speed);


//发送到串口并检查是否发送成功
    int nr;
    //open_port();
    nr = send(sendata,9);
    std:: cout<<sendata;
    if (nr > 0) printf("send data to uart success! nr=%d\n", nr);
    else printf("\n send data to uart fail!");
}
void SerialCom::double2byte(BYTE *hexdata, double ddata)
{
    unsigned char str[255];
    sprintf((char*)str, "%f", ddata);
    hexdata[0] = str[0];
    hexdata[1] = str[1];
    hexdata[2] = str[2];
    hexdata[3] = str[3];
}


