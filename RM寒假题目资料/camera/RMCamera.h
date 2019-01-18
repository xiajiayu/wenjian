//
// Created by nvidia on 2/12/16.
//

#ifndef RM2018_RMCAMERA_H
#define RM2018_RMCAMERA_H

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <inttypes.h>
#include <boost/thread.hpp>

#include "RMVideoCapture.hpp"
#include "Calibration.h"
#include "../para/Parameter.h"
#include "../tool/Log.h"

class RMCamera
{
 private:
  //No.0 means to the most important camera
  
  CAMERA_TYPE m_camera_type[4];
  int m_camera_real_index[4];
  //each calibration file reltated to the camera with the same number
  Calibration m_camera_calibration_0, m_camera_calibration_1, m_camera_calibration_2, m_camera_calibration_3;
  bool m_reload_calibration_flag;

  bool m_camera_offline[4];

  char m_error_flag;
  std::string m_device_path;

  int m_camera_count;
  int m_camera_index;
  int m_camera_index_bias;
  int m_camera_index_store;
  bool initCamera(RMVideoCapture &, const int);

  cv::Mat m_frame_0, m_frame_1, m_frame_2, m_frame_3;
  cv::Mat m_frame_store;
  int m_same_frame_count;
  int m_same_frame_count_upper;
  bool m_check_same_buffer;

  char reboot();
  bool cameraLoop();
  char loadFrame(cv::Mat &, RMVideoCapture &, const int, const bool);
  void restartCamera();

  boost::thread m_camera_thread_0, m_camera_thread_1, m_camera_thread_2, m_camera_thread_3;
  boost::mutex m_data_mutex;

  bool m_is_video;
  cv::VideoCapture m_video_cap;

 public:
  RMVideoCapture m_camera_0, m_camera_1, m_camera_2, m_camera_3;
  RMCamera(const int camera_count = 1);
  RMCamera(const std::string);
  ~RMCamera();
  void init();
  // void searchPara();
  void setCameraType(int, CAMERA_TYPE);
  char getFrame(cv::Mat &frame, int camera_index = 0, const bool error_direction = true);
  void setGetFrameIndex(int index = 0);
  inline bool getReloadCalibrationFlag() { return m_reload_calibration_flag; }
  inline void setReloadCalibrationFlag(bool flag) { m_reload_calibration_flag = flag; }
  Calibration &getCalibration();
  void whiteBalance(cv::Mat &frame);
  //set the index of the mv camera, and the index of the real device
  void setMVIndex(int, int);
  void setCheckSameBuffer(bool check){
    m_check_same_buffer = check;
  }

  void captureVideo(bool flag = false)
  {
    switch (m_camera_index)
    {
      case 0:
        m_camera_0.setCaptureVideo(flag);
            break;
      case 1:
        m_camera_1.setCaptureVideo(flag);
            break;
      case 2:
        m_camera_2.setCaptureVideo(flag);
            break;
      case 3:
        m_camera_3.setCaptureVideo(flag);
            break;
    }
  }
  inline int currentIndex()
  {
#if ((defined _RM_DEC_DEBUG) && (defined _RM_DEC_DEBUG_BY_VIDEO))
    return 0;
#else
    return m_camera_index;
#endif

  }
};

#endif //RM2018_RMCAMERA_H
