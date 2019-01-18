#pragma once

#include <iostream>
#include <inttypes.h>

#include "SerialCom.h"
#include "../detection/ArmRecognition.h"
#include "../camera/Calibration.h"
#include "../tool/define.h"
#include "../tool/Log.h"

#define ANGLE_STORAGE 30

#define _RM_DEC_SERIAL_MODE_SEARCH 1
#define _RM_DEC_SERIAL_MODE_SHOOT 2
#define _RM_DEC_SERIAL_MODE_RUNAWAY 3
#define _RM_DEC_SERIAL_MODE_TK1 4

#define _RM_DEC_SERIAL_MDOE_SHOOT_SHOOT 0
#define _RM_DEC_SERIAL_MDOE_SHOOT_CIRCLE 1
#define _RM_DEC_SERIAL_MODE_TK1_IN 0
#define _RM_DEC_SERIAL_MODE_TK1_OUT 1

class Policy
{
  private:
    // SerialCom m_com;                            //���ڣ�����λ��ͨ�ţ�������̨���ݣ����Ϳ���ָ��
    boost::thread m_com_thread;
    Calibration m_cam_calibration;
    int m_mode_change_count;
    int m_camera_index;    
    
    std::vector<double> m_pitch_storage;
    std::vector<double> m_yaw_storage;

    //stores are used for store the angle data for analysing the changing rate
    double m_pitch_store;
    double m_yaw_store;
    bool m_first_store_flag;
    bool m_camera_checkout_flag;
    REC_DATA m_rec_data;
    bool m_aim_flag;

    static const int m_shoot2search_upper = 10;
    static const int m_search2shoot_upper = 20;

    void calAimAngle(detection::rec::AIM &, double &, double &);
    void performSendData(double &yaw, double &pitch, int &platform, int &cloud, int &position, int &shoot);
    void decision();

public:
    SerialCom m_com;
    Policy();
    ~Policy();
    inline void setCalibration(Calibration &calibration) { m_cam_calibration = calibration; };  //��������ڲ�
    bool initSerial();
    char getMode();
    bool sendAttack(detection::rec::AIM &aim, detection::rec::AIMTYPE);
    inline void loadCameraIndex(int index) {m_camera_index = index;}
    inline bool getCameraCheckoutFlag(){return m_camera_checkout_flag;}
    void ReadMode(int &mode){ mode = m_com.getBuffMode();}
    void ReadStable(int &mode, int &stable, short &flag);
    void sendShoot(double yaw, double pitch, int shoot_cnt);

};

