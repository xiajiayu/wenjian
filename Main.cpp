//
// Created by xiajiayu on 19-1-19.
//

#include"Header.h"
#include"BallCenter.h"
#include"SerialPort.h"
using namespace std;
double fc1, fc2, cc1, cc2, kc1, kc2, kc3, kc4;
int main() {
    fc1 =547.5113;
    fc2 = 550.0285;
    cc1 = 307.9806;
    cc2 = 233.0172;
    kc1 = -0.4451;
    kc2 = 0.2077;
    kc3 =0;
    kc4 =0;
    VideoCapture cap1;
    cap1.open(0);
    int key;
    double t=0;
    //double fps;
    //char string[10];
                Mat src;
                Mat Rsrc;
                Mat intrinsic_matrix(3,3,CV_64F);
                intrinsic_matrix.at<double>(0,0)=fc1;
                intrinsic_matrix.at<double>(0,1)= 0;
                intrinsic_matrix.at<double>(0,2)=cc1;
                intrinsic_matrix.at<double>(1,0)=0;
                intrinsic_matrix.at<double>(1,1)= fc2;
                intrinsic_matrix.at<double>(1,2)=cc2;
                intrinsic_matrix.at<double>(2,0)=0;
                intrinsic_matrix.at<double>(2,1)= 0;
                intrinsic_matrix.at<double>(2,2)= 1;
                cv::Mat distortion_coeffs(1,4,CV_64F);
                distortion_coeffs.at<double>(0,0)=kc1;
                distortion_coeffs.at<double>(0,1)=kc2;
                distortion_coeffs.at<double>(0,2)=kc3;
                distortion_coeffs.at<double>(0,3)=kc4;
    while (1) {
        t = (double)cv::getTickCount();
        key=waitKey(1);
        BallCenter ballcenter;
        float LX=ballcenter.centerPoint.x;
        float LY=ballcenter.centerPoint.y;
        cap1 >> src;
        undistort(src,Rsrc,intrinsic_matrix,distortion_coeffs);
        ballcenter.process(Rsrc);
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
