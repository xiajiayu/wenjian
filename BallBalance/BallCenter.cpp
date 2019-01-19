#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include<iostream>

using namespace std;
using namespace cv;

int lowH =156, highH = 180, lowS = 43, highS = 255, lowV = 46, highV = 255;
void segmentImg(int,void*);
void FindContours(Mat&Image,Mat&ContourImage);
Mat src;
Mat RSrc;
Mat dst;
vector<vector<Point>> contours;
vector<Vec4i>hierarcy;
int main() {
	VideoCapture cap(1);
	while (cap.isOpened()) {
	   int key=waitKey(1);
        cap >> src;
        //Mat splited[3];
        //split(src, splited);
        //imshow("a",splited[0]);
        RSrc = Mat::zeros(src.size(), src.type());
        Mat ContourImg = src.clone();
        //GaussianBlur(src,RSrc, Size(8, 8),0.0);//为什么总是在这里要停顿？
        namedWindow("Hsv", 0);
        createTrackbar("LH", "Hsv", &lowH, 255, segmentImg);
        createTrackbar("HH", "Hsv", &highH, 255, segmentImg);
        createTrackbar("LS", "Hsv", &lowS, 255, segmentImg);
        createTrackbar("HS", "Hsv", &highS, 255, segmentImg);
        createTrackbar("LV", "Hsv", &lowV, 255, segmentImg);
        createTrackbar("HV", "Hsv", &highV, 255, segmentImg);
        segmentImg(0, 0);
        FindContours(dst, ContourImg);
        namedWindow("Contours", 1);
        imshow("Contours", ContourImg);
    }
	return 0;
}

//HSV
void segmentImg(int,void*) {
	Mat srcImg = src.clone();
	Mat HsvImg, ImgThreshold;
	//BGR转HSV
	cvtColor(srcImg, HsvImg, COLOR_BGR2HSV);
	vector<Mat>splitHsv;
	split(HsvImg, splitHsv);
	equalizeHist(splitHsv[2], splitHsv[2]);
	merge(splitHsv, HsvImg);
	
	//调节HSV
	inRange(HsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), ImgThreshold);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(ImgThreshold, ImgThreshold, MORPH_OPEN, element);//开操作
	morphologyEx(ImgThreshold, ImgThreshold, MORPH_CLOSE, element);//闭操作
	namedWindow("thresholdImage", 1);
	imshow("thresholdImage", ImgThreshold);
	dst=ImgThreshold;
}


//提取轮廓及中心点

void FindContours(Mat &Image, Mat &ContourImage) {
	//提取轮廓
	findContours(Image, contours, hierarcy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);
	//drawContours(ContourImage, contours, -1, Scalar(255, 0, 0), 2, 8, hierarcy);
	//if(contours.size()!=2)return ;
	//绘制外接矩形
	vector<Rect>boundRect(contours.size());
	vector<vector<Point2f>>RectPoint;

	float width=0;
	float height=0;
	float CX=0;
	float CY=0;
	Point2f centerPoint;
	Point2f pointzero;
	for (int i = 0; i < contours.size(); i++)
	{   float x0 = 0, y0 = 0, w0 = 0, h0 = 0;
		boundRect[i] = boundingRect((Mat)contours[i]); //查找每个轮廓的外接矩形
		drawContours(ContourImage, contours, i, Scalar(0, 0, 255), 2, 8);  //绘制轮廓
		x0 = boundRect[i].x;  //获得第i个外接矩形的左上角的x坐标
		y0 = boundRect[i].y; //获得第i个外接矩形的左上角的y坐标
		w0 = boundRect[i].width; //获得第i个外接矩形的宽度
		h0 = boundRect[i].height; //获得第i个外接矩形的高度
		RectPoint[i].push_back(Point(x0,y0));
		RectPoint[i].push_back(Point(x0+w0,y0));
		RectPoint[i].push_back(Point(x0+w0,y0+h0));
		RectPoint[i].push_back(Point(x0,y0+h0));
		rectangle(ContourImage, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8); //绘制第i个外接矩形
	}

	/*if(boundRect[0].area()<boundRect[1].area()){
		 width=boundRect[1].width;
		 height=boundRect[1].height;
		 pointzero=RectPoint[1][0];
		//判断中心点
		CX = (RectPoint[0][0].x+RectPoint[0][1].x) / 2.0;
		CY = (RectPoint[0][0].y+RectPoint[0][3].y) / 2.0;
		circle(ContourImage, centerPoint, 1, Scalar(255,0,0), 1,8,0);
		//cout << "center:" << centerPoint;

	}
	else{
		
		 width=boundRect[0].width;
		 height=boundRect[0].height;
		 pointzero=RectPoint[0][0];
		//判断中心点
	     CX= (RectPoint[1][0].x+RectPoint[1][1].x) / 2.0;
		 CY= (RectPoint[1][0].y+RectPoint[1][3].y) / 2.0;
		circle(ContourImage, centerPoint, 1, Scalar(255,0,0), 1,8,0);
	//cout << "center:" << centerPoint;

	}
	centerPoint.x=CX-pointzero.x;
	centerPoint.y=CY-pointzero.y;
	cout<<centerPoint;*/
}
