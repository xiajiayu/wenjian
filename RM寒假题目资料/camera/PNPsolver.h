#pragma once

#include <opencv2/opencv.hpp>

#include <math.h>

#include <iostream>

using namespace std;

// �������ڿ��ٽ��PNP���⣬˳������ռ�������ת�Լ�ͼ��ϵ�����ϵ������ϵ��ϵ����ͶӰ����

// Ĭ��ʹ��Gao��P3P+��ͶӰ����Ҫ������4��������

// ����˳��

// 1.��ʼ������

// 2.����SetCameraMatrix(),SetDistortionMatrix()���ú�����ڲ����뾵ͷ�������

// 3.��Points3D��Points2D�����һһ��Ӧ���������

// 4.����Solve()�������м���

// 5.��RoteM, TransM, W2CTheta��������������



class PNPsolver
{
public:
	PNPsolver();
	PNPsolver(double fx, double fy, double u0, double v0, double k1, double k2, double p1, double p2);
	~PNPsolver();

//	enum METHOD
//	{
//		CV_ITERATIVE = CV_ITERATIVE,
//		CV_P3P = CV_P3P,
//		CV_EPNP = CV_EPNP
//	};

	/***********************λ�˹�������������**************************/
	vector<cv::Point3f> Points3D_big,Points3D_small,Points3D;//�洢�ĸ������������
	vector<cv::Point2f> Points2D;//�洢�ĸ����ͼ������
	double height, width, hw_ratio;
	/***********************λ�˹��ƽ��**************************/
	//����������ת������ƽ�ƾ���

	cv::Mat RoteM, TransM;

	//����ϵ�����ϵ��������תŷ���ǣ�����ϵ�մ���ת��������������ϵ��ȫƽ�С�

	//��ת˳��Ϊx��y��z

	cv::Point3d Theta_W2C;

	//���ϵ������ϵ��������תŷ���ǣ��������ϵ�մ���ת���������������ϵ��ȫƽ�С�

	//��ת˳��Ϊz��y��x

	cv::Point3d Theta_C2W;

	//�������ϵ�У���������ϵԭ��Ow������

	cv::Point3d Ow;

	//��������ϵ�У��������ϵԭ��Oc������

	cv::Point3d Oc;



	/*********************���з���*****************************/

	//��PNP���⣬���λ����Ϣ

	//���ú���RoteM, TransM, W2CTheta����������ȡ������������˵���μ�ע��

	//���������CV_ITERATIVE��CV_P3P��Ĭ�ϣ���CV_EPNP������μ�Opencv documentation.

	//ʵ��

	//CV_ITERATIVE�������ƺ�ֻ����4��������������⣬5�����ǹ���4��ⲻ����ȷ�Ľ�

	//CV_P3P��Gao�ķ�������ʹ�������ĸ������㣬������������������4Ҳ���ܶ���4

	//CV_EPNP��������ʵ����������>=4��������⣬����Ҫ4�㹲��

	//����ֵ��

	//0��ȷ

	//-1����ڲ�����������δ����

	//-2δ�ṩ�㹻�������㣬����������Ŀ��ƥ��

	//-3����ĵ������������printf��Ϣ

	int Solve(int method);

	//���ݼ�����Ľ��������������ͶӰ��ͼ�񣬷�����������㼯

	//ʹ��ǰ��Ҫ����Solve()������λ��

	//����Ϊ��������ϵ�ĵ����꼯��

	//���Ϊ��ͶӰ��ͼ���ϵ�ͼ�����꼯��

	vector<cv::Point2f> WordFrame2ImageFrame(vector<cv::Point3f> WorldPoints);



	//��������Ĳ�����ͼ������ת�������������

	//ʹ��ǰ��Ҫ����Solve()������λ��

	//����Ϊͼ���ϵĵ�����

	//���Ϊ����������ϵ����

	cv::Point2f ImageFrame2CameraFrame(cv::Point2f p);



	//��������ڲ�������

	cv::Mat SetCameraMatrix(double fx, double fy, double u0, double v0)

	{
		camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
		camera_matrix.ptr<double>(0)[0] = fx;
		camera_matrix.ptr<double>(0)[2] = u0;
		camera_matrix.ptr<double>(1)[1] = fy;
		camera_matrix.ptr<double>(1)[2] = v0;
		camera_matrix.ptr<double>(2)[2] = 1.0;
		return camera_matrix;
	}

	//���û���ϵ������

	cv::Mat SetDistortionMatrix(double k_1, double  k_2, double  p_1, double  p_2)

	{
		distortion = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
		distortion.ptr<double>(0)[0] = k_1;
		distortion.ptr<double>(1)[0] = k_2;
		distortion.ptr<double>(2)[0] = p_1;
		distortion.ptr<double>(3)[0] = p_2;
		return distortion;
	}

	//��������������ת������ϵ

	static cv::Point3d RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)

	{
		double r = theta * CV_PI / 180;
		double c = cos(r);
		double s = sin(r);
		double new_x = (vx*vx*(1 - c) + c) * old_x + (vx*vy*(1 - c) - vz*s) * old_y + (vx*vz*(1 - c) + vy*s) * old_z;
		double new_y = (vy*vx*(1 - c) + vz*s) * old_x + (vy*vy*(1 - c) + c) * old_y + (vy*vz*(1 - c) - vx*s) * old_z;
		double new_z = (vx*vz*(1 - c) - vy*s) * old_x + (vy*vz*(1 - c) + vx*s) * old_y + (vz*vz*(1 - c) + c) * old_z;
		return cv::Point3d(new_x, new_y, new_z);
	}

	//���ռ����Z����ת

	//������� x yΪ�ռ��ԭʼx y����

	//thetazΪ�ռ����Z����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180

	//outx outyΪ��ת��Ľ������

	static void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)

	{
		double x1 = x;//����������һ�Σ���֤&x == &outx���������Ҳ�ܼ�����ȷ
		double y1 = y;
		double rz = thetaz * CV_PI / 180;
		outx = cos(rz) * x1 - sin(rz) * y1;
		outy = sin(rz) * x1 + cos(rz) * y1;
	}

	//���ռ����Y����ת

	//������� x zΪ�ռ��ԭʼx z����

	//thetayΪ�ռ����Y����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180

	//outx outzΪ��ת��Ľ������

	static void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)

	{
		double x1 = x;
		double z1 = z;
		double ry = thetay * CV_PI / 180;
		outx = cos(ry) * x1 + sin(ry) * z1;
		outz = cos(ry) * z1 - sin(ry) * x1;
	}

	//���ռ����X����ת

	//������� y zΪ�ռ��ԭʼy z����

	//thetaxΪ�ռ����X����ת���ٶȣ��Ƕ��ƣ���Χ��-180��180

	//outy outzΪ��ת��Ľ������

	static void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)

	{
		double y1 = y;//����������һ�Σ���֤&y == &y���������Ҳ�ܼ�����ȷ
		double z1 = z;
		double rx = thetax * CV_PI / 180;
		outy = cos(rx) * y1 - sin(rx) * z1;
		outz = cos(rx) * z1 + sin(rx) * y1;
	}
private:
	cv::Mat camera_matrix;//�ڲ�������
	cv::Mat distortion;//����ϵ��

	cv::Mat rvec;//���������ת����
	cv::Mat tvec;//�������ƽ������
};