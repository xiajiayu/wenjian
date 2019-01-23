//
// Created by xiajiayu on 19-1-20.
//

#ifndef BIGBUFFER_SERIAL_H
#define BIGBUFFER_SERIAL_H
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <vector>
// #define TXPackSize 11
#define TXPackSize 20
#ifndef BYTE
#define BYTE unsigned char
#endif
// #ifdef _RM_DEC_TK1
// #define RXPackSize 21
// #else
// #define RXPackSize 20
// #endif
#define RXPackSize 10
#define RX_BUFFER_SIZE RXPackSize + 2

class SerialCom
{
public:
    //short m_frame_flag_pc;
    double X;//x
    double Y;//y
    double speed=0;
    //short m_frame_flag;  //帧标志位
    double send_time=0;
    BYTE sendata[TXPackSize];
private:
    int fd;


public:
    SerialCom();
    ~SerialCom();
    int init();
    bool open_port();
    void double2byte(BYTE *hexdata, double ddata);

    //
    //inline int getBuffMode(){return m_buff_mode;}
    //inline short getFrameFlag(){return m_frame_flag;}
   // inline void setFrameFlag(short flag){m_frame_flag = flag;}
    int send(unsigned char *str,int );
    void getdata();


private:
    //int decodeData();
    int set_interface_attribs(int fd, int speed, int parity);
    void set_blocking(int fd, int should_block);
    template <typename T>
    static inline char combData(T data_l, T data_h)
    {
        return (data_l & 0x0f) | ((data_h << 4) & 0xf0);
    }

};
#endif //BIGBUFFER_SERIAL_H


