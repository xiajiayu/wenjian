#include "Policy.h"
#include <chrono>

//coefficients are used for recovering the error-may sending angle
static const double coefficient_pitch[5] = {0.5, 0.3, 0.1, 0.05, 0.05};
static const double coefficient_yaw[5] = {0.6, 0.2, 0.1, 0.05, 0.05};

//they are used to debug
static short angle_change = -1;
static short mode_change = 1;

//the last mode
static short cloud_mode_store = 0;
static short shoot_mode_store = 0;
static short position_mode_store = 0;
static short platform_mode_store = 1;

Policy::Policy()
{
    m_mode_change_count = 0;
    m_first_store_flag = true;
    m_camera_checkout_flag = false;
}

Policy::~Policy()
{
    std::cout << "-- Serial Destructor successfully!" << std::endl;
}

bool Policy::initSerial()
{
    lg::run_log << "-- begin opening serial port ..."
                << "\n";
    if (!m_com.open_port())
    {
        lg::run_log << "--   init serial failed"
                    << "\n";
        return false;
    }
    else
    {
        m_com_thread = boost::thread(&SerialCom::serialLoop, &m_com);
        // m_com.serialLoop();
        lg::run_log << "--   init serial successfully!"
                    << "\n";
        return true;
    }
}

char Policy::getMode()
{
    REC_DATA mode;
    m_com.getReciveData(mode);
    printf("mode = %d\n", mode.mode);
    return mode.mode;
}
//decision part
void Policy::decision()
{
    //TODO:decide the current mode, by reciving data
    m_com.getReciveData(m_rec_data);
}

void Policy::calAimAngle(detection::rec::AIM &aim, double &angle_yaw, double &angle_pitch)
{
    cv::Mat_<cv::Point2d> arm_center(1, 1);
    cv::Mat_<cv::Point2d> arm_center_ds(1, 1);
    arm_center(0) = cv::Point2d(aim.position.x, aim.position.y);

    cv::Mat cameraMatrix = m_cam_calibration.getcameramatrix();
    cv::Mat distCoffes = m_cam_calibration.getdistCoeffs();
    undistortPoints(arm_center, arm_center_ds, cameraMatrix, distCoffes);

    double Fx = m_cam_calibration.getfx();
    double Fy = m_cam_calibration.getfy();
    double Cx = m_cam_calibration.getcx();
    double Cy = m_cam_calibration.getcy();

    // std::cout << Fx << ", " << Fy << ", " << Cx << ", " << Cy << ", " << std::endl;
    angle_yaw = -atan((arm_center(0).x - Cx-para::DELTA_CX) / (Fx)) * 180 / PI; //calculate the shoot angle
    angle_pitch = atan((arm_center(0).y - Cy-para::DELTA_CY) / (Fy)) * 180 / PI;

    float pitch=m_com.pitch_value;
    float yaw=m_com.yaw_value;
    float speed=m_com.speed;
    float depth=aim.depth;
    // correct shoot trace
}

/**
 * control the sending data,
 * as they are not as so good as we think
 *
 * */
void Policy::performSendData(double &yaw, double &pitch, int &platform, int &cloud, int &position, int &shoot)
{
    //load the last angle, and correct the current angle
    yaw = yaw /*- m_pre_yaw + m_cur_yaw*/;
    pitch = pitch /*- m_pre_pitch + m_cur_pitch*/;
    switch (cloud)
    {
    case 1: //shoot or prepare to shoot
    case 2:
        switch (m_camera_index) //2 cameras
        {
        case 0:
            pitch -= para::PITCH_0;
            yaw -= para::YAW_0;
            //TODO:correct yaw and pitch on camera 0
            break;
        case 1:
            pitch -= para::PITCH_1;
            yaw -= para::YAW_1;
            //TODO:correct yaw and pitch on camera 1
            break;
        default:
            break;
        }
        //TODO:more effecient method to narrow sending-angle
        // pitch /= 4;
        // yaw = yaw >= 0 ? log2(abs(yaw) + 1) : -log2(abs(yaw) + 1);
        // if (abs(yaw) < 1.2)
        //     yaw /= 10;
        // else
        //     yaw /= 5;
        break;
    default: //do not shoot
        break;
    }

    //store the first-sending angle data
    if (m_first_store_flag)
    {
        m_pitch_store = pitch;
        m_yaw_store = yaw;
        m_first_store_flag = false;
    }
    else
    {
        //if the angle changed rapidly and the angle is not so small
        //narrow its
        if ((abs(pitch) > 1.2 * abs(m_pitch_store)) && (pitch > 1))
        {
            pitch = pitch * 0.7;
        }
        if ((abs(yaw) > 1.2 * abs(m_yaw_store)) && (yaw > 1))
        {
            yaw = yaw * 0.7;
        }
        m_pitch_store = pitch;
        m_yaw_store = yaw;
    }

    // //muliple a large integer
    // //more detail data will be saved
    // short send_yaw = (short)(yaw * 100);
    // short send_pitch = (short)(pitch * 100);

    //while serial sending (pitch, yaw) = (0, 0), both of them equal to zero
    //it may be a no-such-good data
    //try to ignore it, if the formal data have some exciting laws
    // (abs(yaw) < 1) && (abs(pitch) < 1) &&
    if ((cloud <= 1))
    {
        int storage_size = m_pitch_storage.size();
        if (storage_size > 0)
        {
            bool coefficient_flag = false;
            for (int i = storage_size - 1; (i >= 0) && (i > storage_size - 5); i--)
            {
                if ((abs(m_yaw_storage[i]) > 0) || abs((m_pitch_storage[i]) > 0))
                {
                    coefficient_flag = true;
                    break;
                }
            }
            if (coefficient_flag)
            {
                int index = 0;
                double recalculate_pitch = 0;
                double recalculate_yaw = 0;
                //combination a new data by multipling differrent coefficient
                for (int i = storage_size - 1; (i >= 0) && (i > storage_size - 5); i--, index++)
                {
                    recalculate_pitch += m_pitch_storage[i] * coefficient_pitch[index];
                    recalculate_yaw += m_yaw_storage[i] * coefficient_yaw[index];
                }
                yaw = recalculate_yaw;
                pitch = recalculate_pitch;
                platform = platform_mode_store;
                shoot = shoot_mode_store;
                cloud = cloud_mode_store;
            }
        }
    }

    //save what i had send to the bottom machine
    //so, it is able to be analysed
    //which may help smooth the sending data
    //or get rid of some no-such-good data
    if (m_pitch_storage.size() > ANGLE_STORAGE)
    {
        m_pitch_storage.erase(m_pitch_storage.begin());
        m_yaw_storage.erase(m_yaw_storage.begin());
    }
    m_pitch_storage.push_back(pitch);
    m_yaw_storage.push_back(yaw);

    //restore the formal mode, in order to smooth the changing rate of the mode
    shoot_mode_store = shoot;
    cloud_mode_store = cloud;
    platform_mode_store = platform;
    position_mode_store = position;

    //if the aim has been the center of the camera,
    //the method of camera checking out should be changed as well, use only one camera
    if ((abs(pitch) <= 1) && (abs(yaw) <= 1))
    {
        m_camera_checkout_flag = true;
    }
    else
    {
        m_camera_checkout_flag = false;
    }

#ifdef _RM_DEC_TK1
    if ((send_yaw == 0) && (send_pitch == 0))
    {
        mutex_send.lock();
        if (m_control_flag == 0)
        {
            send_mode = _RM_DEC_SERIAL_MODE_TK1;
            send_mode_detail = m_tk1_position;
        }
        mutex_send.unlock();
    }
#endif
#if ((defined _RM_DEC_DEBUG) && (defined _RM_DEC_SEND_HANDLE))
    printf("input angle:\n");
    std::cin >> send_yaw;
    std::cin >> send_pitch;
    send_yaw *= 100;
    send_pitch *= 100;
    std::cout << send_yaw << ", " << send_pitch << std::endl;
    send_mode = 2;
    send_mode_detail = 0;
    getchar();
#endif
#if ((defined _RM_DEC_DEBUG) && (defined _RM_DEC_SEND_AUTO))
    send_yaw = _RM_DEC_SEND_DEFINE_YAW;
    send_pitch = angle_change * _RM_DEC_SEND_DEFINE_PITCH;
    angle_change *= -1;
    send_yaw *= 100;
    send_pitch *= 100;
    if (mode_change < 10)
    {
        mode_change++;
        send_mode = 1;
    }
    else
        send_mode = _RM_DEC_SEND_DEFINE_MODE;
    send_mode_detail = _RM_DEC_SEND_DEFINE_MODE_DETAIL;
#endif
}

/**
 * mesage send function
 * @parameter :
 *      aim   aim's position and it's depth
 *      detected_mode   the aim's pattarn (armor/worse armor/lightbar)
 * **/
bool Policy::sendAttack(detection::rec::AIM &aim, detection::rec::AIMTYPE aim_type)
{
    int cloud = 0;
    int platform = 0;
    int shoot = 0;
    int position = 0;
    int send_pitch = 0;
    int send_yaw = 0;
    //which mode to be chosen
    if (aim_type > 0) //armor, good or bad
    {
        double angle_yaw, angle_pitch;
        calAimAngle(aim, angle_yaw, angle_pitch);
        
        if((aim.depth>3500)>3500&&(aim.position.y<486))
        {
            std::cout<<"depth of arm:"<<aim.depth<<"\t worse aim"<<std::endl;
            aim_type=detection::rec::WORSE_ARMOR;
        }
        //may be useful in the future
        // chrono::steady_clock::time_point time_aim = chrono::steady_clock::now();
        switch (aim_type)
        {
        case detection::rec::GOOD_ARMOR:
        {
            m_mode_change_count = 0;
            cloud = 2;
            shoot = 1;
            // printf("-------------------------angle_yaw = %lf\n", angle_yaw);
            performSendData(angle_yaw, angle_pitch, platform, cloud, position, shoot);
            // printf("-------------------------angle_yaw = %lf\n", angle_yaw);
            send_pitch = (short)(angle_pitch * 100);
            send_yaw = (short)(angle_yaw * 100);
            m_com.sendAttack(send_yaw, send_pitch, 1,2);
            break;
        }
        case detection::rec::VERYGOOD_ARMOR:
        {
            cloud = 2;
            shoot = 2;
            performSendData(angle_yaw, angle_pitch, platform, cloud, position, shoot);
            send_pitch = (short)(angle_pitch * 100);
            send_yaw = (short)(angle_yaw * 100);
            m_com.sendAttack(send_yaw, send_pitch, 1,2);
            break;
        }
        case detection::rec::WORSE_ARMOR:
        {
            m_com.sendAttack(0,0,1,0);
            break;
        }
        default:
            lg::run_log << "-- error detected mode!"
                        << "\n";
            break;
        }
    }
    else //lightbars, good or bad
    {
        // m_mode_change_count++;
        // switch (aim_type)
        // {
        // case -1:
        //     if (m_mode_change_count >= m_shoot2search_upper)
        //     {
        //         decision();
        //         if(m_aim_flag)
        //         {
        //             switch(m_rec_data.aim_side)
        //             {
        //             case 0:
        //                 cloud = 3;
        //                 break;
        //             case 1:
        //                 cloud = 4;
        //                 break;
        //             default:
        //                 cloud = 0;
        //                 break;
        //             }
        //             // send_pitch = (short)(m_rec_data.pitch * 100);
        //             // send_yaw = (short)(m_rec_data.yaw * 100);
        //             // m_com.sendAttack(send_yaw, send_pitch, 2);
        //             m_com.sendAttack(0, 0, 1);
        //         }//which mode to be chosen
        //         else
        //             m_com.sendAttack(0, 0, 1);
        //     }
        //     else
        //     {
        //         double angle_yaw, angle_pitch;
        //         calAimAngle(aim, angle_yaw, angle_pitch);
        //         cloud = 0;//1;
        //         shoot = -1;//-1;
        //         performSendData(angle_yaw, angle_pitch, platform, cloud, position, shoot);
        //         send_pitch = (short)(angle_pitch * 100);
        //         send_yaw = (short)(angle_yaw * 100);
        //         m_com.sendAttack(send_yaw, send_pitch, 1);
        //     }
        // default:
        //     m_com.sendAttack(0,0,1);
        // }
        m_com.sendAttack(0,0,1,0);
    }   
}
void Policy::ReadStable(int &mode, int &stable, short &flag)
{
    mode = m_com.getBuffMode();
    stable = m_com.getHeatStable();
    flag = m_com.getFrameFlag();
}
void Policy::sendShoot(double yaw, double pitch, int shoot_cnt)
{
    unsigned char sendBuffer[TXPackSize];

    //控制 发送串口数据开始打击大符
    short send_x = (short)(yaw * 100);
    short send_y = (short)(pitch * 100);
    short cnt = -1 - shoot_cnt;

    //  short speed = -2;
    short frame_flag = m_com.m_frame_flag_pc;
    sendBuffer[0] = '!';
    sendBuffer[1] = (send_y >> 8) & 0xff;
    sendBuffer[2] = send_y & 0xff;
    sendBuffer[3] = (send_x >> 8) & 0xff;
    sendBuffer[4] = send_x & 0xff;
    sendBuffer[5] = (cnt >> 8) & 0xff;
    sendBuffer[6] = cnt & 0xff;
    sendBuffer[7] = (frame_flag >> 8) & 0xff;
    sendBuffer[8] = frame_flag & 0xff;
    Append_CRC8_Check_Sum(sendBuffer, TXPackSize);

    lg::buff_log << "senddata" << "\n";//std::endl;
    lg::buff_log << "crc: "<<std::hex << sendBuffer[9] << "\n";//std::endl;

    short frm_flag_rcv = 0;
    int stable = 0;
    int mode;
    int send_times = 0;
    bool change = true;

    double t0 = cv::getTickCount();

    while (1)
    {
        if(send_times %2 ==0)
        {

            if(change)
            {
                send_x = send_x +1;
                change = false;
            }else
            {
                send_x = send_x -1;
                change = true;
            }
            sendBuffer[3] = (send_x >> 8) & 0xff;
            sendBuffer[4] = send_x & 0xff;
            Append_CRC8_Check_Sum(sendBuffer, TXPackSize);

            m_com.send(sendBuffer, sizeof(sendBuffer));
        }

        // lg::buff_log << "writeData" << "\n";//std::endl;
        // lg::buff_log << "frame_flag: " << frame_flag << "\n";//std::endl;
        ReadStable(mode, stable, frm_flag_rcv);
        // lg::buff_log << "frm_flag_rcv: " << frm_flag_rcv << "\n";//std::endl;

        double t1 = cv::getTickCount();
        double time = (t1 - t0) / cv::getTickFrequency();
        send_times ++;

        if (((frm_flag_rcv == frame_flag) && (stable == -4 || stable == -3)) || (mode != -1 && mode != -2))
        {
            float send_frequency = send_times/time;
            lg::buff_log << "break by control" << "\n";//std::endl;
            lg::buff_log << "wait time: " << time << "\n";//std::endl;
            lg::buff_log <<" send frequency: " <<send_frequency <<"\n" ;
            break;
        }


        if (time > 0.6)
        {
            lg::buff_log << "wait time > 1" << "\n";//std::endl;
            lg::buff_log <<" frame flag rcv by me " <<frm_flag_rcv <<"frame flag: " <<frame_flag <<"\n";
            lg::buff_log <<" should be received by Embedded platform frame flag " <<m_com.m_received_flag;

            break;
        }
    }

    m_com.m_frame_flag_pc ++;
    if (m_com.m_frame_flag_pc >= 65500)   //2的16次方 25536 超过会溢出
        m_com.m_frame_flag_pc  = 1;
    m_com.setFrameFlag(frame_flag);
}

