/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and
to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#pragma once
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/thread/mutex.hpp>

#include "../para/Parameter.h"
#include "../tool/define.h"
#include "./include/MvCameraControl.h"

#define MAX_IMAGE_DATA_SIZE (30*1280*1024)

enum CAMERA_TYPE{NORMAL = 0, MV = 1};

class RMVideoCapture {
public:
  RMVideoCapture(const char * device, int size_buffer = 1);
  RMVideoCapture();
  ~RMVideoCapture();
  //  void initCamera(const char * device, int size_buffer = 1);
  bool startStream();
  bool closeStream();
  bool setExposureTime(bool auto_exp, int t);
  bool setVideoFormat(int width, int height, bool mjpg = 1);
  bool setGainMode(_MV_CAM_GAIN_MODE_);
  bool setPixelFormat(MvGvspPixelType);
  bool setWhiteBalance(_MV_CAM_BALANCEWHITE_AUTO_);
  bool changeVideoFormat(int width, int height, bool mjpg = 1);
  bool getVideoSize(int & width, int & height);

  bool setVideoFPS(int fps);
  bool setBufferSize(int bsize);
  void restartCapture();
  int getFrameCount(){
    return cur_frame;
  }

  void info();

  char getNewFrame(cv::Mat &);
  RMVideoCapture& operator >> (cv::Mat & image);
  bool getFrameLoop(const int);
  inline void setCaptureVideo(bool flag = false)
  {
    m_shoot_frame_mutex.lock();
    m_capture_video_flag = flag;
    m_shoot_frame_mutex.unlock();
  }
  inline bool isCameraOffline()
  {
    //        std::cout << m_camera_offline;
    m_offline_mutex.lock();
    bool camera_offline = m_camera_offline;
    m_offline_mutex.unlock();
    return camera_offline;
  }
  inline void killCamera()
  {
    m_kill_mutex.lock();
    m_camera_kill = true;
    m_kill_mutex.unlock();
  }
  cv::Mat m_frame;

  bool init(CAMERA_TYPE camera_type = MV);
  bool init(const char *device, int size_buffer = 1, CAMERA_TYPE camera_type = NORMAL);
  void setMVIndex(int);

private:
  bool m_capture_video_flag=false;
  void cvtRaw2Mat(const void * data, cv::Mat & image);
  bool refreshVideoFormat();
  bool initMMap();
  int xioctl(int fd, int request, void *arg);
  boost::mutex m_frame_mutex;
  boost::mutex m_kill_mutex;
  boost::mutex m_offline_mutex;
  boost::mutex m_raw2mat_mutex;
  boost::mutex m_shoot_frame_mutex;

private:
  struct MapBuffer {
    void * ptr;
    unsigned int size;
  };
  unsigned int capture_width;
  unsigned int capture_height;
  unsigned int format;
  int fd;
  unsigned int buffer_size;
  unsigned int buffr_idx;
  unsigned long long int cur_frame;
  MapBuffer * mb;
  const char * video_path;
  bool m_camera_offline;
  bool m_camera_kill;
  bool m_no_new_mb;
  int m_camera_index;

  CAMERA_TYPE m_camera_type;

  //the next var are used for mv camera
  void *m_mv_p_handle;
  MV_CC_DEVICE_INFO_LIST m_mv_device_list;
  MV_CC_DEVICE_INFO * m_mv_p_device_list;
  int m_mv_return;
  int m_mv_camera_index;
  MV_FRAME_OUT_INFO_EX m_mv_image_info;
  unsigned char* m_mv_p_data;
  unsigned int m_mv_data_size;
};