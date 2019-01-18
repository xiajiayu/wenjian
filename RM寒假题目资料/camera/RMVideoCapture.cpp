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

#include "RMVideoCapture.hpp"
#include "linux/videodev2.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

RMVideoCapture::RMVideoCapture(const char * device, int size_buffer) : video_path(device) {
  fd = open(device, O_RDWR);
  buffer_size = size_buffer;
  buffr_idx = 0;
  cur_frame = 0;
  capture_width = 0;
  capture_height = 0;
  m_offline_mutex.lock();
  m_camera_offline = false;
  m_offline_mutex.unlock();
  mb = new MapBuffer[buffer_size];
  m_no_new_mb = false;
  m_camera_index = 0;
}

RMVideoCapture::RMVideoCapture() {}

void RMVideoCapture::setMVIndex(int device_index)
{
  m_mv_camera_index = device_index;
}

bool RMVideoCapture::init(CAMERA_TYPE camera_type)
{
  m_camera_type = camera_type;
  m_mv_p_handle = NULL;
  memset(&m_mv_device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  m_mv_return = MV_CC_EnumDevices(MV_USB_DEVICE, &m_mv_device_list);
  if(m_mv_return != MV_OK)
    return false;
  if(m_mv_device_list.nDeviceNum <= 0)
    return false;
  m_mv_return = MV_CC_CreateHandle(&m_mv_p_handle, m_mv_device_list.pDeviceInfo[m_mv_camera_index]);
  if(m_mv_return != MV_OK)
    return false;
  m_mv_return = MV_CC_OpenDevice(m_mv_p_handle);
  if(m_mv_return != MV_OK)
    return false;
  m_mv_image_info = {0};
  memset(&m_mv_image_info, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  m_mv_p_data = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
  m_mv_data_size = MAX_IMAGE_DATA_SIZE;
  m_offline_mutex.lock();
  m_camera_offline = false;
  m_offline_mutex.unlock();
  return true;
}

bool RMVideoCapture::init(const char *device, int size_buffer, CAMERA_TYPE camera_type) {
  m_camera_type = camera_type;
  fd = open(device, O_RDWR);
  if(fd == -1)
    return false;
  buffr_idx = 0;
  cur_frame = 0;
  capture_width = 0;
  capture_height = 0;
  m_offline_mutex.lock();
  m_camera_offline = false;
  m_offline_mutex.unlock();
  m_camera_index = 0;
  m_no_new_mb = true;
  buffer_size = size_buffer;
  mb = new MapBuffer[buffer_size];
  video_path = device;
  m_no_new_mb = false;
  return true;
}

void RMVideoCapture::restartCapture() {
  close(fd);
  fd = open(video_path, O_RDWR);
  buffr_idx = 0;
  cur_frame = 0;
}

RMVideoCapture::~RMVideoCapture() {
  close(fd);
  if (!m_no_new_mb)
    delete [] mb;
  std::cout << "-- RMVideoCapture " << m_camera_index << " Destructor successfully!" << std::endl;
}

void RMVideoCapture::cvtRaw2Mat(const void * data, cv::Mat & image) {
  if (format == V4L2_PIX_FMT_MJPEG) {
    m_raw2mat_mutex.lock();
    cv::Mat src(capture_height, capture_width, CV_8UC3, (void*) data);
    image = cv::imdecode(src, 1);
    m_raw2mat_mutex.unlock();
    // std::cout << "fuck";
  }
  else if (format == V4L2_PIX_FMT_YUYV) {
    cv::Mat yuyv(capture_height, capture_width, CV_8UC2, (void*) data);
    cv::cvtColor(yuyv, image, CV_YUV2BGR_YUYV);
  }
}

char RMVideoCapture::getNewFrame(cv::Mat &image)
{
  if(m_camera_type == MV)
    {
      m_mv_return = MV_CC_GetOneFrameTimeout(m_mv_p_handle, m_mv_p_data, m_mv_data_size, &m_mv_image_info, 1000);
      cv::Mat src(capture_height, capture_width, CV_8UC3, (unsigned char *)m_mv_p_data);
      // cv::flip(src, src, 0);
      m_raw2mat_mutex.lock();
      cv::cvtColor(src, image, CV_RGB2BGR);
      m_raw2mat_mutex.unlock();
      return 0;
    }

  struct v4l2_buffer bufferinfo = {0};
  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = buffr_idx;
  if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
    perror("-- VIDIOC_DQBUF Error");
    m_offline_mutex.lock();
    m_camera_offline = true;
    m_offline_mutex.unlock();
    return -10;
  }
  cvtRaw2Mat(mb[buffr_idx].ptr, image);
  memset(&bufferinfo, 0, sizeof(bufferinfo));
  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = buffr_idx;

  // Queue the next one.
  if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
    perror("-- VIDIOC_DQBUF Error");
    m_offline_mutex.lock();
    m_camera_offline = true;
    m_offline_mutex.unlock();
    return -10;
  }
  ++buffr_idx;
  buffr_idx = buffr_idx >= buffer_size ? buffr_idx - buffer_size : buffr_idx;
  ++cur_frame;
}

RMVideoCapture & RMVideoCapture::operator >> (cv::Mat & image) {
  // //    std::cout << "current buffr idx: " << buffr_idx << std::endl;
  // struct v4l2_buffer bufferinfo = {0};
  // bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // bufferinfo.memory = V4L2_MEMORY_MMAP;
  // bufferinfo.index = buffr_idx;
  // if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
  //   perror("-- VIDIOC_DQBUF Error");
  //   m_offline_mutex.lock();
  //   m_camera_offline = true;
  //   m_offline_mutex.unlock();
  //   exit(1);
  // }

  // //std::cout << "raw data armor_size: " << bufferinfo.bytesused << std::endl;
  // cvtRaw2Mat(mb[buffr_idx].ptr, image);

  // //memset(&bufferinfo, 0, sizeof(bufferinfo));
  // bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // bufferinfo.memory = V4L2_MEMORY_MMAP;
  // bufferinfo.index = buffr_idx;

  // // Queue the next one.
  // if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
  //   perror("-- VIDIOC_DQBUF Error");
  //   m_offline_mutex.lock();
  //   m_camera_offline = true;
  //   m_offline_mutex.unlock();
  //   exit(1);
  // }
  // ++buffr_idx;
  // buffr_idx = buffr_idx >= buffer_size ? buffr_idx - buffer_size : buffr_idx;
  // ++cur_frame;
  // return *this;
}


bool RMVideoCapture::getFrameLoop(const int camera_index) {
  m_kill_mutex.lock();
  m_camera_kill = false;
  m_kill_mutex.unlock();
  m_camera_index = camera_index;
#ifdef _RM_SHOOT_VIDEO
  cv::VideoWriter writer;
  system("mkdir $HOME/RM2018-data/video");
  struct timeval tv;
  gettimeofday(&tv, NULL);
  std::random_device rd;
  std::string home = getenv("HOME");
  std::default_random_engine e(rd());
  std::uniform_int_distribution<> u(0, 1000000);
  usleep(u(e));
  // sleep(2);
  std::string file_path = home + "/RM2018-data/video/" +
    std::to_string((tv.tv_sec * 1000) % 1000000 + tv.tv_usec / 1000) +
    std::to_string(u(e)) +
    "shoot.avi";
  writer.open(file_path, CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(para::FRAME_WIDTH_SET[camera_index], para::FRAME_HEIGHT_SET[camera_index]));

  if (!writer.isOpened())
    {
      std::cout << "-- [camera " << camera_index << "] writer can not open!" << std::endl;
    }

#endif
  while (1)
    {
      usleep(2000);
      m_kill_mutex.lock();
      bool kill_flag = m_camera_kill;
      m_kill_mutex.unlock();

      if (kill_flag)
        return 0;
      // printf("good data 1 --- %d\n", camera_index);

      m_frame_mutex.lock();
      char camera_health = getNewFrame(m_frame);          //lock have been add in this function
      m_frame_mutex.unlock();

      // printf("good data 2 --- %d\n", camera_index);

      if (camera_health == -10)
        {
#ifdef _RM_SHOOT_VIDEO
          writer.release();
#endif
          lg::run_log << "-- [camera " << camera_index << "] offline! \n";
          m_offline_mutex.lock();
          m_camera_offline = true;
          m_offline_mutex.unlock();
          m_frame.release();
          closeStream();
          return false;
        }
      m_frame_mutex.lock();
      bool frame_empty = m_frame.empty();
      m_frame_mutex.unlock();

      // printf("good data 3 --- %d\n", camera_index);

      if (frame_empty)
        {
#ifdef _RM_SHOOT_VIDEO
          writer.release();
#endif
          lg::run_log << "-- [camera " << camera_index << "] data is empty!\n";
          // std::cout << "-- [camera " << camera_index << "] data is empty!\n";
          m_offline_mutex.lock();
          m_camera_offline = true;
          m_offline_mutex.unlock();
          // sleep(20);
          m_frame.release();
          closeStream();
          return false;   //need to be improve
        }
#ifdef _RM_SHOOT_VIDEO
    m_shoot_frame_mutex.lock();
    if (m_capture_video_flag)
    {
      m_frame_mutex.lock();
      writer << m_frame;
      m_frame_mutex.unlock();
    }
    m_shoot_frame_mutex.unlock();
#endif
      // usleep(10000);
      // printf("good data\n");
    }
}


bool RMVideoCapture::initMMap() {
  struct v4l2_requestbuffers bufrequest = {0};
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = buffer_size;

  if (ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0) {
    perror("--   VIDIOC_REQBUFS");
    return false;
  }

  for (int i = 0; i < buffer_size; ++i) {
    struct v4l2_buffer bufferinfo = {0};
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = i; /* Queueing buffer index 0. */

    // Put the buffer in the incoming queue.
    if (ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0) {
      perror("--   VIDIOC_QUERYBUF");
      return false;
    }

    mb[i].ptr = mmap(
                     NULL,
                     bufferinfo.length,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     fd,
                     bufferinfo.m.offset);
    mb[i].size = bufferinfo.length;

    if (mb[i].ptr == MAP_FAILED) {
      perror("MAP_FAILED");
      return false;
    }
    memset(mb[i].ptr, 0, bufferinfo.length);

    // Put the buffer in the incoming queue.
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = i;
    if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
      perror("--   VIDIOC_QBUF");
      return false;
    }
  }
  return true;
}

bool RMVideoCapture::startStream() {
  switch(m_camera_type)
    {
    case MV:
      m_mv_return = MV_CC_SetEnumValue(m_mv_p_handle, "TriggerMode", 0);
      if(m_mv_return != MV_OK)
        return false;
      m_mv_return = MV_CC_StartGrabbing(m_mv_p_handle);
      if(m_mv_return != MV_OK)
        return false;
      return true;
      break;
    default:
    case NORMAL:
      cur_frame = 0;
      refreshVideoFormat();
      if (initMMap() == false)
        return false;

      __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("--   VIDIOC_STREAMON");
        return false;
      }
      return true;
      break;
    }
}

bool RMVideoCapture::closeStream() {
  cur_frame = 0;
  buffr_idx = 0;
  __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
    perror("--   VIDIOC_STREAMOFF");
    return false;
  }
  for (int i = 0; i < buffer_size; ++i) {
    munmap(mb[i].ptr, mb[i].size);
  }
  return true;
}

bool RMVideoCapture::setGainMode(_MV_CAM_GAIN_MODE_ gain_mode)
{
  switch(m_camera_type)
    {
    case MV:
      m_mv_return = MV_CC_SetGainMode(m_mv_p_handle, gain_mode);
      if(m_mv_return != MV_OK)
        return false;
      return true;
      break;
    default:
      return true;
      break;
    }
}

bool RMVideoCapture::setPixelFormat(MvGvspPixelType pixel_type)
{
  switch(m_camera_type)
    {
    case MV:
      m_mv_return = MV_CC_SetPixelFormat(m_mv_p_handle, pixel_type);
      if(m_mv_return != MV_OK)
        {
          printf("--   set pixel format failed!\n");
          return false;
        }
      return true;
      break;
    default:
      return true;
      break;
    }
}

bool RMVideoCapture::setWhiteBalance(_MV_CAM_BALANCEWHITE_AUTO_ balance_white)
{
  switch(m_camera_type)
    {
    case MV:
      m_mv_return = MV_CC_SetBalanceWhiteAuto(m_mv_p_handle, balance_white);
      if(m_mv_return != MV_OK)
        return false;
      return true;
    default:
    case NORMAL:
      return true;
      break;
    }
}

bool RMVideoCapture::setExposureTime(bool auto_exp, int t) {
  switch(m_camera_type)
    {
    case MV:
      if(auto_exp)
        {
          m_mv_return = MV_CC_SetExposureAutoMode(m_mv_p_handle, MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
          if(m_mv_return != MV_OK)
            return false;
        }
      else
        {
          m_mv_return = MV_CC_SetExposureAutoMode(m_mv_p_handle, MV_EXPOSURE_AUTO_MODE_OFF);
          if(m_mv_return != MV_OK)
            return false;
          m_mv_return = MV_CC_SetExposureTime(m_mv_p_handle, t);
          if(m_mv_return != MV_OK)
            return false;
        }
      return true;
      break;
    default:
    case NORMAL:
      if (auto_exp) {
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_AUTO;
        if ( xioctl(fd, VIDIOC_S_CTRL, &control_s) < 0) {
          printf("--   Set Auto Exposure error\n");
          return false;
        }
      }
      else {
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_MANUAL;
        if ( xioctl(fd, VIDIOC_S_CTRL, &control_s) < 0) {
          printf("--   Close MANUAL Exposure error\n");
          return false;
        }
        control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        control_s.value = t;
        if ( xioctl(fd, VIDIOC_S_CTRL, &control_s) < 0) {
          printf("--   Set Exposure Time error\n");
          return false;
        }
        printf("--   Set Exposure %d!\n", t);
      }
      return true;
      break;
    }
}

bool RMVideoCapture::changeVideoFormat(int width, int height, bool mjpg) {
  closeStream();
  restartCapture();
  setVideoFormat(width, height, mjpg);
  startStream();
  return true;
}

bool RMVideoCapture::setVideoFormat(int width, int height, bool mjpg) {
  switch(m_camera_type)
    {
    case MV:
      m_mv_return = MV_CC_SetIntValue(m_mv_p_handle, "Width", width);
      if(m_mv_return != MV_OK)
        return false;
      m_mv_return = MV_CC_SetIntValue(m_mv_p_handle, "Height", height);
      if(m_mv_return != MV_OK)
        return false;
      capture_width = width;
      capture_height = height;
      return true;
      break;
    default:
    case NORMAL:
      if (capture_width == width && capture_height == height)
        return true;
      capture_width = width;
      capture_height = height;
      cur_frame = 0;
      struct v4l2_format fmt = {0};
      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt.fmt.pix.width = width;
      fmt.fmt.pix.height = height;
      if (mjpg == true)
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      else
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.field = V4L2_FIELD_ANY;

      if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
        printf("--   Setting Pixel Format\n");
        return false;
      }
      return true;
      break;
    }
}

bool RMVideoCapture::refreshVideoFormat() {
  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
    perror("--   Querying Pixel Format\n");
    return false;
  }
  capture_width = fmt.fmt.pix.width;
  capture_height = fmt.fmt.pix.height;
  format = fmt.fmt.pix.pixelformat;
  return true;
}


bool RMVideoCapture::getVideoSize(int & width, int & height) {
  if (capture_width == 0 || capture_height == 0) {
    if (refreshVideoFormat() == false)
      return false;
  }
  width = capture_width;
  height = capture_height;
  return true;
}

bool RMVideoCapture::setVideoFPS(int fps) {
  struct v4l2_streamparm stream_param = {0};
  stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  stream_param.parm.capture.timeperframe.denominator = fps;
  stream_param.parm.capture.timeperframe.numerator = 1;

  if (-1 == xioctl(fd, VIDIOC_S_PARM, &stream_param)) {
    printf("--   Setting Frame Rate\n");
    return false;
  }
  return true;
}

bool RMVideoCapture::setBufferSize(int bsize) {
  if (buffer_size != bsize) {
    buffer_size = bsize;
    delete [] mb;
    mb = new MapBuffer[buffer_size];
  }
}

void RMVideoCapture::info() {
  struct v4l2_capability caps = {};
  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps)) {
    perror("Querying Capabilities\n");
    return;
  }

  printf( "Driver Caps:\n"
          "  Driver: \"%s\"\n"
          "  Card: \"%s\"\n"
          "  Bus: \"%s\"\n"
          "  Version: %d.%d\n"
          "  Capabilities: %08x\n",
          caps.driver,
          caps.card,
          caps.bus_info,
          (caps.version >> 16) && 0xff,
          (caps.version >> 24) && 0xff,
          caps.capabilities);


  struct v4l2_cropcap cropcap = {0};
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))  {
    perror("Querying Cropping Capabilities\n");
    return;
  }

  printf( "Camera Cropping:\n"
          "  Bounds: %dx%d+%d+%d\n"
          "  Default: %dx%d+%d+%d\n"
          "  Aspect: %d/%d\n",
          cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
          cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
          cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

  struct v4l2_fmtdesc fmtdesc = {0};
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  char fourcc[5] = {0};
  char c, e;
  printf("  FMT : CE Desc\n--------------------\n");
  while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
    strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
    c = fmtdesc.flags & 1 ? 'C' : ' ';
    e = fmtdesc.flags & 2 ? 'E' : ' ';
    printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
    fmtdesc.index++;
  }

  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
    perror("Querying Pixel Format\n");
    return;
  }
  strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
  printf( "Selected Camera Mode:\n"
          "  Width: %d\n"
          "  Height: %d\n"
          "  PixFmt: %s\n"
          "  Field: %d\n",
          fmt.fmt.pix.width,
          fmt.fmt.pix.height,
          fourcc,
          fmt.fmt.pix.field);

  struct v4l2_streamparm streamparm = {0};
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(fd, VIDIOC_G_PARM, &streamparm)) {
    perror("Querying Frame Rate\n");
    return;
  }
  printf( "Frame Rate:  %f\n====================\n",
          (float)streamparm.parm.capture.timeperframe.denominator /
          (float)streamparm.parm.capture.timeperframe.numerator);
}

int RMVideoCapture::xioctl(int fd, int request, void *arg) {
  int r;
  do r = ioctl (fd, request, arg);
  while (-1 == r && EINTR == errno);
  return r;
}
