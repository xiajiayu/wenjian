//
// Created by xiajiayu on 19-1-19.
//

#include"Header.h"
#include"BallCenter.h"
#include"SerialPort.h"

using namespace std;
int main() {

    VideoCapture cap1;
    cap1.open(0);

    int key;
    double t=0;
    //double fps;
    //char string[10];
    while (1) {
        Mat src;
        t = (double)cv::getTickCount();
        key=waitKey(1);
        BallCenter ballcenter;
        float LX=ballcenter.centerPoint.x;
        float LY=ballcenter.centerPoint.y;
        cap1 >> src;
        ballcenter.process(src);
        //speed
        //x,y
        //zhenllv
         // t为该处代码执行所耗的时间,单位为秒,fps为其倒数
           t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

            double  nx=ballcenter.centerPoint.x;
           SerialCom serial;
            serial.X=nx;
            double ny=ballcenter.centerPoint.y;
            serial.Y=ny;
            serial.speed= (double)(sqrt((LX - nx) * (LX - nx) + (LY - ny) * (LY - ny)) / t);
            serial.getdata();
           //char data[30]='!'+x+y+char(t)+0;
           //cout<<data;
           //for(int i=0;i<data.length();i++){
           //serial.sendBuffer[i]=data[i];}
           // serial.init();
          // cout<<serial.sendBuffer[0];
           // serial.send(serial.sendBuffer, sizeof(serial.sendBuffer));
          //serial.getdata();
          //serial.Usage();

        if(key=='q')
            break;
    }
    return 0;

}