//
// Created by xiajiayu on 19-1-21.
//

#ifndef BALLBALANCE_SERIAL_H
#define BALLBALANCE_SERIAL_H
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#ifndef BYTE
#define BYTE unsigned char
#endif
BYTE sendata[30];

using std::cout;
using std::endl;
class Serial_port{
public:
    Serial_port(int NSpeed, int NBits, char NEvent, int NStop):nSpeed(NSpeed),nBits(NBits),nEvent(NEvent),nStop(NStop){}//寻找设备
    int setOpt(int fd);     //串口属性设置
    ~Serial_port(){}
    void Test();
public:
    double X;
    double Y;
    double speed;
private:
    int  nSpeed;
    int  nBits;
    char nEvent;
    int  nStop;
};
//}FCcom(19200, 8, 'N', 1);

#endif