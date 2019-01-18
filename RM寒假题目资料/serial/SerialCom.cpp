//
// Created by loop on 2/12/17.
//
#include "SerialCom.h"

SerialCom::SerialCom()
{
    m_frame_flag_pc = 0;
    m_buff_mode = 0;
    m_stable = 0;
    m_frame_flag = 0;
}

SerialCom::~SerialCom()
{
    close(fd);
    std::cout << "-- SerialCom Destructor successfully!" << std::endl;
}
int SerialCom::set_interface_attribs(int fd, int speed, int parity)
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
    for(int i=0; i<4; i++)
    {
        std::string serial_path = "/dev/ttyUSB" + std::to_string(i);
        fd = open(serial_path.c_str(), O_RDWR | O_NOCTTY);
        lg::run_log <<"fd: " <<fd <<"\n";
        if (fd < 0)
        {
            continue;
        }else{
            found_flag = true;
            break;
        }
    }
    // int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if(!found_flag)
    {
        std::string error = strerror(errno);
        lg::run_log << "--   error " << errno << " opening "
                    << "/dev/ttyS0" << error << "\n";
        return -1;
    }

    set_interface_attribs(fd, B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
    lg::run_log << "--   speed : 115200bps"
                << "\n";
    set_blocking(fd, 0); // set no blocking
    return fd;
}

int SerialCom::init(int num)
{
    char *portname = strdup("/dev/ttyUSB");
    // char *portname = strdup(().c_str());
    portname[11] = num + '0';
    int fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        std::string error = strerror(errno);
        lg::run_log << "--   error " << errno << " opening "
                    << "/dev/ttyS0" << error << "\n";
        return -1;
    }
    set_interface_attribs(fd, B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking(fd, 0);                   // set no blocking
    return fd;
}

bool SerialCom::open_port()
{
    fd = init();
    if (fd > 0)
    {
        lg::run_log << "--   init serial port:" << fd << " successfully!"
                    << "\n";
        return true;
    }
    else
    {
        return false;
    }
}

int SerialCom::receive()
{
    // unsigned char recieve_data[RXPackSize] = {};
    int fd = this->fd;
    int n = read(fd, recvBuffer, sizeof(recvBuffer));

    // std::cout <<"receive size in receive:" <<n <<std::endl;
    // std::cout <<"receive data" <<recvBuffer <<std::endl;

    if ((n < RXPackSize) || !Verify_CRC8_Check_Sum(recvBuffer, RXPackSize))
        return -1;
    else
        return n;
}

int SerialCom::send(unsigned char *str, int len)
{
    int n = write(this->fd, str, len);
    return n;
}

bool SerialCom::serialLoop()
{
    sleep(5);
    lg::run_log << "-- serial port thread begin running!"
                << "\n";

    while (1)
    {
        //延时函数
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
        //获取数据
        int success = decodeData();
    }

    return 0;
}

unsigned long long int recvCounts = 0;
int SerialCom::decodeData()
{
    unsigned char data_raw[RXPackSize] = {0};

    int receive_len = receive();

    // std::cout <<"receive len: " <<receive_len <<std::endl;

    if (receive_len != RXPackSize)
        return -1;

    for (int i = 0; i < RXPackSize; i++)
        data_raw[i] = recvBuffer[i];

    if (data_raw[0] == '!')
    {
        mutex_recieve.lock();
#if defined TX2
        m_rec_data.source = data_raw[1] & 0x0f;
        m_rec_data.destination = (data_raw[1] & 0xf0) >> 4;
        m_rec_data.platform_mode = data_raw[2] & 0x0f;
        m_rec_data.cloud_mode = (data_raw[2] & 0xf0) >> 4;
        m_rec_data.position = data_raw[3] & 0x0f;
        m_rec_data.shoot_mode = (data_raw[3] & 0xf0) >> 4;
        //memcpy(&m_rec_data.pitch, &data_raw[4], 2);
        //memcpy(&m_rec_data.yaw, &data_raw[6], 2);
        m_rec_data.pitch = ((short)(data_raw[4] << 8) | data_raw[5]) / 100.0;
        m_rec_data.yaw = ((short)(data_raw[6] << 8) | data_raw[7]) / 100.0;
        printf("%lf, %lf\n", m_rec_data.pitch, m_rec_data.yaw);
        if (m_rec_data.source == 2)
        {
            m_rec_data.blood = data_raw[8] & 0x0f;
            m_rec_data.attack_side = (data_raw[8] & 0xf0) >> 4;
        }
        else
        {
            m_rec_data.aim_side = data_raw[8] & 0x0f;
            m_rec_data.aim_kind = (data_raw[8] & 0xf0) >> 4;
        }
#endif
        m_rec_data.mode=(data_raw[1]&data_raw[2])&0xff;

        //heat stable
        unsigned char stable_state = data_raw[2];
        if (stable_state == 0x84)
            m_stable = -4;                            //已发射子弹
        else if (stable_state == 0x85)
            m_stable = -5;                            //热量不允许
        else if (stable_state == 0x83)
            m_stable = -3;                      //热量no允许
        else if( stable_state ==0)
            m_stable = 0;
        //buff
        unsigned char buff_state = data_raw[1];
        if (buff_state == 0x02)
            m_buff_mode =2;                               //
        else if (buff_state == 0x82)
            m_buff_mode = -2;                               //大符模式
        else if (buff_state == 0x81)
            m_buff_mode = -1;                               //小符模式
        else if (buff_state == 0)
            m_buff_mode = 0;//不进行操作
        //frame flag
        unsigned char flag_pre = data_raw[5];
        unsigned char flag_aft = data_raw[6];
        m_frame_flag =  (short)((flag_pre << 8) | (flag_aft));   //帧标志位

        unsigned char flag_pre1 = data_raw[3];
        unsigned char flag_aft1 = data_raw[4];
        m_received_flag=  (short)((flag_pre1 << 8) | (flag_aft1));

        // for shoot mode
        pitch_value=float((data_raw[3]<<8)|(data_raw[4]&0xff))/100;
        yaw_value=float((data_raw[5]<<8)|(data_raw[6]&0xff))/100;
        speed=float((data_raw[7]<<8)|(data_raw[8]&0xff))/100;

        mutex_recieve.unlock();

        // std::cout <<"mode: " <<m_buff_mode <<"stable: " <<m_stable <<std::endl;

    }
    else
    {
        return -1;
    }
#if ((defined _RM_DEC_DEBUG) && (defined _RM_DEC_RECIEVE_PRINT))
    std::cout << "recieve data[" << RXPackSize << "]: "; //<< data_raw;
    for (int i = 0; i < RXPackSize; i++)
    {
        printf("%4d", (unsigned char)data_raw[i]);
    }
    std::cout << std::endl;
    // printf("%lld\n", recvCounts++);
#endif
#if (defined _RM_DEC_WRITE_LOGFILE) && (defined _RM_DEC_SERIALLOG)
    int n = RXPackSize;
    lg::serial_recieve_log << "recieve data[" << n << "]: "; //<< data_raw;
    for (int i = 0; i < n; i++)
    {
        int data = (int)data_raw[i];
        lg::serial_recieve_log << data << " ";
    }
    lg::serial_recieve_log << "\n";
#endif
    return 0;

}

bool SerialCom::getReciveData(REC_DATA &rec_data)
{
    //TODO:if return the same data, return false;
    mutex_recieve.lock();
    rec_data = m_rec_data;
    mutex_recieve.unlock();
    return true;
}

bool SerialCom::sendAttack(short yaw,short pitch,short freq,char shoot_mode)
{
    std::cout<<"send Freq: "<<1/((cv::getTickCount()-send_time)/cv::getTickFrequency())<<std::endl;

    if((abs(yaw)<0.5)&&(abs(pitch)<0.5)&&(freq>0))
    {
        shoot_mode=0;
        std::cout<<"shoot mode: "<<int(shoot_mode)<<std::endl;
    }
    else if((abs(yaw)<0.9)&&(abs(pitch)<0.5)&&(freq>0))
        shoot_mode=0;  
    else
        shoot_mode=0;

    send_time=cv::getTickCount();
    // std::getc(stdin);
    sendBuffer[0] = '!';
    sendBuffer[1] =(pitch >> 8) & 0xff; //source, destination
    sendBuffer[2] = pitch & 0xff;
    sendBuffer[3] = (yaw >> 8) & 0xff;
    sendBuffer[4] = yaw & 0xff;
    sendBuffer[5] = (freq>>8)&0xff;     // positive for shoot, negetive for buff
    sendBuffer[6] = freq&0xff;
    sendBuffer[7] = shoot_mode&0xff;
    sendBuffer[8] = 0;
    Append_CRC8_Check_Sum(sendBuffer, TXPackSize);

    mutex_send.lock();
    int n = write(this->fd, sendBuffer, sizeof(sendBuffer));
    mutex_send.unlock();


#if (/*(defined _RM_DEC_DEBUG) && */(defined _RM_DEC_SEND_PRINT))
    std::cout << "send data[" << TXPackSize << "]: "; //<< data_raw;
    for (int i = 0; i < TXPackSize; i++)
    {
        printf("%02x ", sendBuffer[i]);
    }
    std::cout << " [pitch = " << (double)pitch / 100.0 << ", yaw = " << (double)yaw / 100.0 << "]";
    printf("\n");
#endif
#if (defined _RM_DEC_WRITE_LOGFILE) && (defined _RM_DEC_SERIALLOG)
    for (int i = 0; i < TXPackSize; i++)
    {
        int data = (int)sendBuffer[i];
        lg::serial_send_log << data << " ";
    }
    double serial_send_pitch = (double)pitch / 100.0;
    double serial_send_yaw = (double)yaw / 100.0;
    lg::serial_send_log << " [pitch = " << serial_send_pitch << ", yaw = " << serial_send_yaw << "]\n";
#endif

    return true;
}
