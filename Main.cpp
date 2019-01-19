#include "opencv2/highgui.hpp"

#include "opencv2/imgproc.hpp"

#include<iostream>



using namespace std;

using namespace cv;



int lowH = 50, highH = 80, lowS = 43, highS = 255, lowV = 46, highV = 255;
float Rwidth=20.0 ,RHeight = 20.0;

void segmentImg(int, void*);

void FindContours(Mat&Image, Mat&ContourImage);

Mat src;

Mat RSrc;

Mat dst;

vector<vector<Point>> contours;

vector<Vec4i>hierarcy;

int main() {
	VideoCapture cap(0);
	while (cap.isOpened()) {

		cap >> src;
		//src = imread("C://ͼƬ1.png");

			//Mat splited[3];

			//split(src, splited);

			//imshow("a",splited[0]);

		RSrc = Mat::zeros(src.size(), src.type());

		Mat ContourImg = src.clone();

		//GaussianBlur(src,RSrc, Size(8, 8),0.0);//Ϊʲô����������Ҫͣ�٣�

		namedWindow("Hsv", 0);

		createTrackbar("LH", "Hsv", &lowH, 255, segmentImg);

		createTrackbar("HH", "Hsv", &highH, 255, segmentImg);

		createTrackbar("LS", "Hsv", &lowS, 255, segmentImg);

		createTrackbar("HS", "Hsv", &highS, 255, segmentImg);

		createTrackbar("LV", "Hsv", &lowV, 255, segmentImg);

		createTrackbar("HV", "Hsv", &highV, 255, segmentImg);

		segmentImg(0, 0);
		Point center;
		FindContours(dst, ContourImg);

		namedWindow("Contours", 1);

		imshow("Contours", ContourImg);
		waitKey(1);

	}	
	return 0;
}


	//HSV

	void segmentImg(int, void*){

		Mat srcImg = src.clone();

		Mat HsvImg;
		Mat ImgThreshold;

		//BGRתHSV

		cvtColor(srcImg, HsvImg, COLOR_BGR2HSV);

		vector<Mat>splitHsv;

		split(HsvImg, splitHsv);

		equalizeHist(splitHsv[2], splitHsv[2]);

		merge(splitHsv, HsvImg);



		//����HSV

		inRange(HsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), ImgThreshold);

		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

		morphologyEx(ImgThreshold, ImgThreshold, MORPH_OPEN, element);//������

		morphologyEx(ImgThreshold, ImgThreshold, MORPH_CLOSE, element);//�ղ���

		namedWindow("thresholdImage", 1);

		imshow("thresholdImage", ImgThreshold);

		dst = ImgThreshold;

	}





//��ȡ���������ĵ�



void FindContours(Mat &Image, Mat &ContourImage) {

	//��ȡ����

	findContours(Image, contours, hierarcy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);

	//drawContours(ContourImage, contours, -1, Scalar(255, 0, 0), 2, 8, hierarcy);

	if(contours.size()!=2)return  ;

	//������Ӿ���

	vector<Rect>boundRect(contours.size());
	
	vector<vector<Point2f>>RectPoint(contours.size());



	float width = 0;

	float height = 0;

	float CX = 0;

	float CY = 0;

	Point2f centerPoint;

	Point2f pointFirst;
	Point2f pointZero;

	for (int i = 0; i < contours.size(); i++)

	{
		float x0 = 0, y0 = 0, w0 = 0, h0 = 0;

		boundRect[i] = boundingRect((Mat)contours[i]); //����ÿ����������Ӿ���

		drawContours(ContourImage, contours, i, Scalar(0, 0, 255), 2, 8);  //��������

		x0 = boundRect[i].x;  //��õ�i����Ӿ��ε����Ͻǵ�x����

		y0 = boundRect[i].y; //��õ�i����Ӿ��ε����Ͻǵ�y����

		w0 = boundRect[i].width; //��õ�i����Ӿ��εĿ��

		h0 = boundRect[i].height; //��õ�i����Ӿ��εĸ߶�

		RectPoint[i].push_back(Point(x0, y0));

		RectPoint[i].push_back(Point(x0 + w0, y0));

		RectPoint[i].push_back(Point(x0 + w0, y0 + h0));

		RectPoint[i].push_back(Point(x0, y0 + h0));

		rectangle(ContourImage, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8); //���Ƶ�i����Ӿ���

	}



	if(boundRect[0].area()<boundRect[1].area()){

		 width=boundRect[1].width;

		 height=boundRect[1].height;

		 pointFirst=RectPoint[1][0];

		//�ж����ĵ�

		CX = (RectPoint[0][0].x+RectPoint[0][1].x) / 2.0;

		CY = (RectPoint[0][0].y+RectPoint[0][3].y) / 2.0;
		Point2f center = Point2f(CX, CY);
		circle(ContourImage, center, 1, Scalar(255,0,0), 1,8,0);

		//cout << "center:" << centerPoint;



	}

	else {

		 width=boundRect[0].width;

		 height=boundRect[0].height;

		 pointFirst=RectPoint[0][0];

		//�ж����ĵ�

		 CX= (RectPoint[1][0].x+RectPoint[1][1].x) / 2.0;

		 CY= (RectPoint[1][0].y+RectPoint[1][3].y) / 2.0;
		 Point2f center = Point2f(CX, CY);
		circle(ContourImage, center, 1, Scalar(255,0,0), 1,8,0);

	//cout << "center:" << centerPoint;



	}
	
	//�õ�С������

	float Xlength = fabs(width / 2.0 - CX + pointFirst.x);
	float Xrl = Rwidth * Xlength*60.0 / width;
	if ((CX - pointFirst.x) <=width / 2.0) {
		centerPoint.x = 30 - Xrl;
	}
	else {
		centerPoint.x = 30 +Xrl;

	}
	float Ylength = fabs(height/ 2.0 - CY + pointFirst.y);
	float Yrl = RHeight * Ylength*60.0 /height;
	if ((CY - pointFirst.y) <= height / 2.0) {
		centerPoint.y = 30 - Yrl;
	}
	else {
		centerPoint.y = 30 + Yrl;
	}
	cout << centerPoint;
	

}