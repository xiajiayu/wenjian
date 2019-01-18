#include "PNPsolver.h"
#include <opencv2/opencv.hpp>

PNPsolver::PNPsolver()
{
	//��ʼ���������
	vector<double>rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);
}
PNPsolver::PNPsolver(double fx, double fy, double u0, double v0, double k1, double k2, double p1, double p2)
{
	//��ʼ���������
	vector<double>rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);
	SetCameraMatrix(fx, fy, u0, v0);
	SetDistortionMatrix(k1, k2, p1, p2);
}

PNPsolver::~PNPsolver()
{
    std::cout << "-- PNPsolver Destructor successfully!" << std::endl;
}

int PNPsolver::Solve(int method = CV_ITERATIVE)
{
	width = (sqrt((Points2D[0].x - Points2D[1].x) * (Points2D[0].x - Points2D[1].x) + (Points2D[0].y - Points2D[1].y) * (Points2D[0].y - Points2D[1].y)) + sqrt((Points2D[2].x - Points2D[3].x) * (Points2D[2].x - Points2D[3].x) + (Points2D[2].y - Points2D[3].y) * (Points2D[2].y - Points2D[3].y))) / 2;
	height = (sqrt((Points2D[0].x - Points2D[3].x) * (Points2D[0].x - Points2D[3].x) + (Points2D[0].y - Points2D[3].y) * (Points2D[0].y - Points2D[3].y)) + sqrt((Points2D[1].x - Points2D[2].x) * (Points2D[1].x - Points2D[2].x) + (Points2D[1].y - Points2D[2].y) * (Points2D[1].y - Points2D[2].y))) / 2;
	hw_ratio = width / height;
	if(hw_ratio > 3.5)
	{
		Points3D = Points3D_big;
	}
	else
	{
		Points3D = Points3D_small;
	}
	//����У��
	if (camera_matrix.cols == 0 || distortion.cols == 0)
	{
		printf("����ڲ�����������δ���ã�\r\n");
		return -1;
	}

	if (Points3D.size() != Points2D.size())
	{
		printf("3D��������2D��������һ�£�\r\n");
		return -2;
	}
	if (Points3D.size() != 4)
	{
		printf("ʹ��CV_ITERATIVE��CV_P3P����ʱ���������������ӦΪ4��\r\n");
		return -2;
	}
	
	/*******************���PNP����*********************/
	//�����ַ������
	cv::solvePnP(Points3D, Points2D, camera_matrix, distortion, rvec, tvec, false, CV_ITERATIVE);

	double rm[9];
	RoteM = cv::Mat(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, RoteM);
	double r11 = RoteM.ptr<double>(0)[0];
	double r12 = RoteM.ptr<double>(0)[1];
	double r13 = RoteM.ptr<double>(0)[2];
	double r21 = RoteM.ptr<double>(1)[0];
	double r22 = RoteM.ptr<double>(1)[1];
	double r23 = RoteM.ptr<double>(1)[2];
	double r31 = RoteM.ptr<double>(2)[0];
	double r32 = RoteM.ptr<double>(2)[1];
	double r33 = RoteM.ptr<double>(2)[2];
	TransM = tvec;

	//������������ϵ��������תŷ���ǣ���ת�����ת����������ϵ��
	//��ת˳��Ϊz��y��x
	double thetaz = atan2(r21, r11) / CV_PI * 180;
	double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
	double thetax = atan2(r32, r33) / CV_PI * 180;

	//���ϵ������ϵ��������תŷ���ǣ��������ϵ�մ���ת���������������ϵ��ȫƽ�С�
	//��ת˳��Ϊz��y��x
	Theta_C2W.z = thetaz;
	Theta_C2W.y = thetay;
	Theta_C2W.x = thetax;

	//���������ϵ�����ϵ��������תŷ���ǣ�����ϵ�մ���ת�����ת���������ϵ��
	//��ת˳��Ϊx��y��z
	Theta_W2C.x = -1 * thetax;
	Theta_W2C.y = -1 * thetay;
	Theta_W2C.z = -1 * thetaz;

	/*************************************�˴�������������ϵԭ��Oc����������ϵ�е�λ��**********************************************/

	/***********************************************************************************/
	/* ��ԭʼ����ϵ������תz��y��x������ת������������ϵƽ�У�����OcOw�������ת */
	/* ��������֪��������������ϵ��ȫƽ��ʱ��OcOw��ֵ */
	/* ��ˣ�ԭʼ����ϵÿ����ת��ɺ󣬶�����OcOw����һ�η�����ת�����տ��Եõ���������ϵ��ȫƽ��ʱ��OcOw */
	/* ����������-1������������ϵ����������� */
	/***********************************************************************************/

	//���ƽ�ƾ��󣬱�ʾ���������ϵԭ�㣬��������(x,y,z)�ߣ��͵�����������ϵԭ��
	double tx = tvec.ptr<double>(0)[0];
	double ty = tvec.ptr<double>(0)[1];
	double tz = tvec.ptr<double>(0)[2];

	//x y z ΪΨһ���������ԭʼ����ϵ�µ�����ֵ
	//Ҳ��������OcOw���������ϵ�µ�ֵ
	double x = tx, y = ty, z = tz;
	Ow.x = x;
	Ow.y = y;
	Ow.z = z;

	//�������η�����ת
	CodeRotateByZ(x, y, -1 * thetaz, x, y);
	CodeRotateByY(x, z, -1 * thetay, x, z);
	CodeRotateByX(y, z, -1 * thetax, y, z);

	//����������������ϵ�µ�λ������
	//������OcOw����������ϵ�µ�ֵ
	Oc.x = x*-1;
	Oc.y = y*-1;
	Oc.z = z*-1;

	return 0;
}

//���ݼ�����Ľ��������������ͶӰ��ͼ�񣬷�����������㼯
//����Ϊ��������ϵ�ĵ����꼯��
//���Ϊ��ͶӰ��ͼ���ϵ�ͼ�����꼯��
vector<cv::Point2f> PNPsolver::WordFrame2ImageFrame(vector<cv::Point3f> WorldPoints)
{
	vector<cv::Point2f> projectedPoints;
	cv::projectPoints(WorldPoints, rvec, tvec, camera_matrix, distortion, projectedPoints);
	return projectedPoints;
}

