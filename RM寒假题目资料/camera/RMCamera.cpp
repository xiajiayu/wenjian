//
// Created by nvidia on 2/12/16.
//

#include "RMCamera.h"

RMCamera::RMCamera(const int camera_count)
{
  m_camera_offline[0] = true;
  m_camera_offline[1] = true;
  m_camera_offline[2] = true;
  m_camera_offline[3] = true;
  m_camera_type[0] = NORMAL;
  m_camera_type[1] = NORMAL;
  m_camera_type[2] = NORMAL;
  m_camera_type[3] = NORMAL;
  m_check_same_buffer = true;
  m_camera_count = camera_count;
  m_is_video = false;
  m_camera_index_bias = 0;
  m_camera_index_store = -1;
  m_camera_index = 0;
  m_reload_calibration_flag = true;
  m_same_frame_count = 0;
  m_same_frame_count_upper = 100;
  for (int i = 0; i < 4; i++)
    m_camera_real_index[i] = -1;
  m_device_path = para::CAMERA_PATH;
  m_camera_index = 0;
}

void RMCamera::setCameraType(int index, CAMERA_TYPE type)
{
  if((index < 0) || (index > 3))
    {
      printf("camera index error! [0, 1, 2, 3]\n");
      exit(-1);
    }
  m_camera_type[index] = type;
}

void RMCamera::init()
{
  switch (m_camera_count)
    {
    case 4:
      initCamera(m_camera_3, 3);
      if (!m_camera_calibration_3.read(para::FILEPATH_CALIBRATION_3))
        {
          lg::run_log << "-- load calibration file 3 failed! \n";
          exit(-113);
        }
    case 3:
      initCamera(m_camera_2, 2);
      if (!m_camera_calibration_2.read(para::FILEPATH_CALIBRATION_2))
        {
          lg::run_log << "-- load calibration file 2 failed! \n";
          exit(-112);
        }
    case 2:
      initCamera(m_camera_1, 1);
      if (!m_camera_calibration_1.read(para::FILEPATH_CALIBRATION_1))
        {
          lg::run_log << "-- load calibration file 1 failed! \n";
          exit(-111);
        }
    case 1:
      initCamera(m_camera_0, 0);
      if (!m_camera_calibration_0.read(para::FILEPATH_CALIBRATION_0))
        {
          lg::run_log << "-- load calibration file 0 failed! \n";
          exit(-110);
        }
      break;
    default:
      printf("up to 4 cameras only!");
      exit(-1);
      break;
    }
  //while all camera init falied, exit!
  if(m_camera_offline[0] &&
     m_camera_offline[1] &&
     m_camera_offline[2] &&
     m_camera_offline[3])
    {
      lg::run_log << "-- all camera offline! exit soon!\n";
      exit(-1);
    }
  lg::run_log << "-- waiting for data, please waiting ...\n"; //std::endl;
  sleep(2);
  cameraLoop();
  for (int i = 1; i <= 2; i++)
    {
      sleep(1);
      std::cout << "-- wait " << i << "/2 s ..." << std::endl;
    }
  //test after initialization
  sleep(1);
  setCheckSameBuffer(true);
  cv::Mat test;
  bool error_direction = true;
  for (int i = 0; i < m_camera_count; i++)
  {
    int camera_index = i;
    lg::run_log << "-- testing camera " << i << "...\n";
    getFrame(test, camera_index, error_direction);
    error_direction = false;
    if (camera_index == i)
    {
      lg::run_log << "-- testing camera " << i << " succesfully!\n";
      std::cout<<"-- testing camera " << i << " succesfully!\n";
    }
    else
    {
      lg::run_log << "-- testing camera " << i << " failed!\n";
      std::cout<<"-- testing camera " << i << " failed!\n";

    }
  }
  setCheckSameBuffer(false);
}

// void RMCamera::searchPara()
// {
//   int low=1000;
//   int high=5000;
//   int step=10; // search for 
// }

RMCamera::RMCamera(const std::string video_path)
{
  m_reload_calibration_flag = true;
  m_is_video = true;
  m_camera_count = 1;
  m_video_cap.open(video_path);
  if (!m_video_cap.isOpened() || !m_camera_calibration_0.read(para::FILEPATH_CALIBRATION_0))
    {
      lg::run_log << "-- open video " << video_path << " failed!\n";
      exit(-110);
    }
  else
    {
      lg::run_log << "-- open video " << video_path << " successfully!\n";
    }
}

RMCamera::~RMCamera()
{
  if (!m_is_video)
    {
      m_camera_0.killCamera();
      m_camera_1.killCamera();
      m_camera_2.killCamera();
      m_camera_3.killCamera();
      lg::run_log << "-- kill all camera successfully! \n";
    }
  std::cout << "-- RMCamera Destructor successfully!" << std::endl;
}

void RMCamera::setMVIndex(int camera_index, int device_index)
{
  switch(camera_index)
    {
    case 0:
      m_camera_0.setMVIndex(device_index);
      break;
    case 1:
      m_camera_1.setMVIndex(device_index);
      break;
    case 2:
      m_camera_2.setMVIndex(device_index);
      break;
    case 3:
      m_camera_3.setMVIndex(device_index);
      break;
    }
}

bool RMCamera::initCamera(RMVideoCapture &camera, const int camera_index)
{
  CAMERA_TYPE camera_type = m_camera_type[camera_index];
  bool camera_device_return = false;
  switch(camera_type)
    {
    case MV:
      camera_device_return = camera.init(MV);
      break;
    default:
    case NORMAL:
      std::string device_path = m_device_path + std::to_string(camera_index);
      lg::run_log << "-- begin opening [camera] " << device_path << " ...\n";
      camera_device_return = camera.init(device_path.c_str());
      break;
    }
  //add configurations after camera_device_return, but befor camera.startStream()
  if(camera_device_return &&
     camera.setVideoFormat(para::FRAME_WIDTH_SET[camera_index], para::FRAME_HEIGHT_SET[camera_index]) &&
     camera.setPixelFormat(para::MV_PIXEL_TYPE) &&
     camera.setExposureTime(false, para::EXPOSURE[camera_index]) &&
     camera.setGainMode(para::MV_GAIN_MODE) &&
     camera.setWhiteBalance(MV_BALANCEWHITE_AUTO_CONTINUOUS) &&
     camera.startStream())
    {
      lg::run_log << "--   open camera " << camera_index << " successfully!"
                  << "\n";
      m_camera_offline[camera_index] = false;
      return true;
    }
  else
    {
      lg::run_log << "--   open camera " << camera_index << " failed!"
                  << "\n";
      m_camera_offline[camera_index] = true;
      return false;
    }

}

bool RMCamera::cameraLoop()
{
  switch (m_camera_count)
    {
    case 4:
      if(!m_camera_offline[3])
        m_camera_thread_3 = boost::thread(&RMVideoCapture::getFrameLoop, &m_camera_3, 3);
    case 3:
      if(!m_camera_offline[2])
        m_camera_thread_2 = boost::thread(&RMVideoCapture::getFrameLoop, &m_camera_2, 2);
    case 2:
      if(!m_camera_offline[1])
        m_camera_thread_1 = boost::thread(&RMVideoCapture::getFrameLoop, &m_camera_1, 1);
    case 1:
      if(!m_camera_offline[0])
        m_camera_thread_0 = boost::thread(&RMVideoCapture::getFrameLoop, &m_camera_0, 0);
      break;
    default:
      break;
    }

}

char RMCamera::getFrame(cv::Mat &frame, int camera_index_input, const bool error_direction)
{
#if ((defined _RM_DEC_DEBUG) && (defined _RM_DEC_DEBUG_BY_VIDEO))
  if (m_is_video)
    {
      cv::Mat test;
      m_video_cap >> test;
      if (test.empty())
        return -1;
      cv::resize(test, test, cv::Size(para::FRAME_WIDTH_SET[0], para::FRAME_HEIGHT_SET[0]));
      test.copyTo(frame);
      return 1;
    }
#else
  cv::Mat frame_temp;
  int camera_index = camera_index_input;
  if (m_camera_real_index[camera_index_input] >= 0)
    camera_index = m_camera_real_index[camera_index_input];
  //while the user changed the camera
  //it should be done to make sure he/she will get fulled-image
  if (m_camera_index_store >= 0)
    {
      if (camera_index != m_camera_index_store)
        {
          m_camera_index_bias = 0;
          m_reload_calibration_flag = true; //camera changed, calibration file changed
          // printf("1\n");
        }
      else
        {
          m_reload_calibration_flag = false;
          // printf("2\n");
        }
    }

  m_camera_index = camera_index + m_camera_index_bias;
  m_camera_index_store = camera_index;
  // std::cout << m_camera_index << std::endl;
  if ((m_camera_index >= m_camera_count) || (m_camera_index < 0))
    return -1;
  // printf("camera index : %d\n", m_camera_index);

  switch (m_camera_index)
    {
    case 3:
      if (loadFrame(frame, m_camera_3, 3, error_direction) < 0)
        return -103;
      break;
    case 2:
      if (loadFrame(frame, m_camera_2, 2, error_direction) < 0)
        return -102;
      break;
    case 1:
      if (loadFrame(frame, m_camera_1, 1, error_direction) < 0)
        return -101;
      break;
    case 0:
      if (loadFrame(frame, m_camera_0, 0, error_direction) < 0)
        return -100;
      break;
    default:
      return reboot();
    }
  m_camera_real_index[camera_index_input] = m_camera_index;

  camera_index_input = m_camera_index;
  //make show the camera return you a different image
  if(m_check_same_buffer)
    if (!m_frame_store.empty())
      {
        long long int diff_counts = 0;
        int frame_element_size = m_frame_store.total();
        const int diff_counts_upper = 10;
        for (int i = 0; i < frame_element_size; i++)
          {
            if (m_frame_store.data[i] != frame.data[i])
              {
                diff_counts++;
                if (diff_counts > diff_counts_upper)
                  break;
              }
          }
        if (diff_counts > diff_counts_upper)
          {
            frame.copyTo(m_frame_store);
            m_same_frame_count = 0;
            return 1;
          }
        else
          {
            m_same_frame_count++;
            if (m_same_frame_count > m_same_frame_count_upper)
              restartCamera();
            // return 2;
            return getFrame(frame, camera_index_input, error_direction);
          }
      }
    else
      {
        frame.copyTo(m_frame_store);
        m_same_frame_count = 0;
        return 1;
      }
#endif
}

void RMCamera::setGetFrameIndex(int index)
{
}

char RMCamera::loadFrame(cv::Mat &frame, RMVideoCapture &camera, const int camera_index, const bool error_direction)
{
  m_data_mutex.lock();
  camera.m_frame.copyTo(frame);
  m_data_mutex.unlock();
  if (frame.empty())
    {
      for (int i = 1; (i <= 5) && (frame.empty()) && !m_camera_offline[camera_index] && (!camera.isCameraOffline()); i++)
        {
          m_data_mutex.lock();
          camera.m_frame.copyTo(frame);
          m_data_mutex.unlock();
          lg::run_log << "-- [" << i << "/5]trying to wait data from camera " << camera_index << " ..."
                      << "\n";
          usleep(100000);
        }
    }
  //decide if the camera is good ro not
  //if it is still offline or nothing recived, try the next camera
  if (frame.empty() || camera.isCameraOffline() || m_camera_offline[camera_index])
    {
      m_camera_offline[camera_index] = true;
      if (error_direction)
        {
          //the bias between the reality camera and the camera you wish
          m_camera_index_bias++;
        }
      else
        {
          m_camera_index_bias--;
        }
      for (int i = 1; i <= 3; i++)
        {
          if (!error_direction)
            getFrame(frame, camera_index - m_camera_index_bias - 1, error_direction);
          else
            getFrame(frame, camera_index - m_camera_index_bias + 1, error_direction);
          // printf("camera_index fuck %d", camera_index - m_camera_index_bias - 1);
          // printf("camera_index_bias %d\n", m_camera_index_bias);
          if ((m_camera_index < 0) || (m_camera_index >= m_camera_count))
            break;
          lg::run_log << "-- [" << i << "/3]trying to wait data from camera " << m_camera_index << " ..."
                      << "\n";
          if (!frame.empty())
            break;
        }
      // printf("camera %d no data\n", m_camera_index);

      //no camera return correct data
      if (frame.empty())
        {
          m_camera_offline[m_camera_index] = true;
          return -1;
        }
    }
  return 1;
}

Calibration &RMCamera::getCalibration()
{
  m_reload_calibration_flag = false;
#if ((defined _RM_DEC_DEBUG) && (defined _RM_DEC_DEBUG_BY_VIDEO))
  return m_camera_calibration_0;
#endif
  int file_index = m_camera_index;
  if (file_index < 0)
    file_index = 0;
  else if (file_index >= m_camera_count)
    file_index = m_camera_count - 1;
  // printf("%d\n", file_index);
  switch (file_index)
    {
    case 0:
      return m_camera_calibration_0;
    case 1:
      return m_camera_calibration_1;
    case 2:
      return m_camera_calibration_2;
    case 3:
      return m_camera_calibration_3;
    default:
      return m_camera_calibration_0;
    }
}

void RMCamera::whiteBalance(cv::Mat &frame)
{
  std::vector<cv::Mat> rgb;
  cv::split(frame, rgb);
  double r, g, b;
  b = mean(rgb[0])[0];
  g = mean(rgb[1])[0];
  r = mean(rgb[2])[0];

  double kr, kg, kb;
  kb = (r + g + b) / (3 * b);
  kg = (r + g + b) / (3 * g);
  kr = (r + g + b) / (3 * r);

  rgb[0] = rgb[0] * kb;
  rgb[1] = rgb[1] * kg;
  rgb[2] = rgb[2] * kr;

  cv::merge(rgb, frame);
}

char RMCamera::reboot()
{
  lg::run_log << "-- no camera online, restart now! \n";
  exit(-1);
  //todo reboot the programe while all camera offline
}

void RMCamera::restartCamera()
{
  lg::run_log << "waitting data for a very long time, restart soon!\n";
  exit(-1);
}
