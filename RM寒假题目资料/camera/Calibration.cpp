#include "Calibration.h"


Calibration::Calibration()
{
}


Calibration::~Calibration()
{
	std::cout << "-- Calibration Destructor successfully!" << std::endl;
}

bool Calibration::read(const std::string path_file)
{
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	lg::run_log << "-- begin opening [calibration] " << path_file << " ...\n";

	cv::FileStorage storage(path_file, cv::FileStorage::READ);
	if (!storage.isOpened())
	{
		lg::run_log << "--   open calibration file " << path_file << " failed!\n";
		return false;
	}
	else
	{
		lg::run_log << "--   open calibration file " << path_file << " successfully!\n";
	}
	storage["camera-matrix"] >> cameraMatrix;
	storage["distortion"] >> distCoeffs;

	cameraMatrix.copyTo(cameraMatrix_);
	distCoeffs.copyTo(distCoeffs_);

	//cout << cameraMatrix << endl;

	fx_ = cameraMatrix_.at<double>(0, 0);
	fy_ = cameraMatrix_.at<double>(1, 1);
	cx_ = cameraMatrix.at<double>(0, 2);
	cy_ = cameraMatrix_.at<double>(1, 2);

	p1_ = distCoeffs_.at<double>(0, 0);
	p2_ = distCoeffs_.at<double>(0, 1);
	p3_ = distCoeffs_.at<double>(0, 2);
	p4_ = distCoeffs_.at<double>(0, 3);
	p5_ = distCoeffs_.at<double>(0, 4);

	storage.release();

	return true;
}