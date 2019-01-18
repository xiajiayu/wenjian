//
// Created by loop on 2/12/17.
//
#ifndef BIGBUFFER_SERIAL_H
#define BIGBUFFER_SERIAL_H
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <boost/thread/mutex.hpp>
//#include <stdio.h>
#include <iostream>
#include <vector>

#include "algorithmOfCRC.h"
#include "../tool/define.h"
#include "../tool/Log.h"
#include "../para/Parameter.h"

// #define TXPackSize 11
#define TXPackSize 10

// #ifdef _RM_DEC_TK1
// #define RXPackSize 21
// #else
// #define RXPackSize 20
// #endif
#define RXPackSize 10
#define RX_BUFFER_SIZE RXPackSize + 2

struct REC_DATA
{
    int pitch, yaw;
    char source;
    char destination;
    char platform_mode;
    char cloud_mode;
    char position;
    char shoot_mode;
    char aim_side;
    char aim_kind;
    char blood;
    char attack_side;
    char mode;
};

class SerialCom
{
public:
    short m_frame_flag_pc;
    short m_received_flag;  //嵌软收到己方发的帧序号

    float pitch_value;
    float yaw_value;
    float speed;

  private:
    int fd;
    char buf[RX_BUFFER_SIZE];
    unsigned char sendBuffer[TXPackSize];
    unsigned char recvBuffer[RXPackSize];

    boost::mutex mutex_send;
    boost::mutex mutex_recieve;

    REC_DATA m_rec_data;

    //buff
    int m_buff_mode;
    int m_stable; //热量情况
    short m_frame_flag;  //帧标志位

    double send_time=0;
    double time_c=0;

  public:
    SerialCom();
    ~SerialCom();
    int init();
    int init(int num);
    bool open_port();

    bool serialLoop();
    // bool sendAttack(int yaw, int pitch, int platform, int cloud, int position, int shoot);
    bool sendAttack(short yaw,short pitch,short freq,char shoot_mode);
    bool getReciveData(REC_DATA &rec_data);

    inline int getBuffMode(){return m_buff_mode;}
    inline int getHeatStable(){return m_stable;}
    inline short getFrameFlag(){return m_frame_flag;}
    inline void setFrameFlag(short flag){m_frame_flag = flag;}
    int send(unsigned char *str, int n);

  private:
    int set_interface_attribs(int fd, int speed, int parity);
    void set_blocking(int fd, int should_block);
    template <typename T>
    static inline char combData(T data_l, T data_h)
    {
        return (data_l & 0x0f) | ((data_h << 4) & 0xf0);
    }
    int receive();
    int decodeData();

   };
#endif //BIGBUFFER_SERIAL_H
