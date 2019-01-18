#pragma once

#ifndef CALIBRATION_H_
#define CALIBRATION_H

// C++
#include <iostream>
#include <time.h>
// OpenCV
#include <opencv2/core/core.hpp>
//#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "../tool/Log.h"

using namespace std;

class Calibration
{
public:

	Calibration();
	~Calibration();

	bool read(const std::string path_file);
	
	inline double getfx() const { return fx_; }
	inline double getfy() const { return fy_; }
	inline double getcx() const { return cx_; }
	inline double getcy() const { return cy_; }

	inline double getp1() const { return p1_; }
	inline double getp2() const { return p2_; }
	inline double getp3() const { return p3_; }
	inline double getp4() const { return p4_; }
	inline double getp5() const { return p5_; }

	cv::Mat getcameramatrix() const{ return cameraMatrix_; }
	cv::Mat getdistCoeffs() const { return distCoeffs_; }

private:

	cv::Mat cameraMatrix_;
	cv::Mat distCoeffs_;

	double fx_;
	double fy_;
	double cx_;
	double cy_;
	double p1_;
	double p2_;
	double p3_;
	double p4_;
	double p5_;

};

#endif