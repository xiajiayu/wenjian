#include "opencv2/highgui.hpp"
#include "opencv2/highgui.hpp"

#include "opencv2/imgproc.hpp"

#include<iostream>
#include"BallCenter.h"


using namespace std;

using namespace cv;


BallCenter::BallCenter() {};


 void BallCenter::process(Mat&src) {
		//src = imread("C://图片1.png");

			//Mat splited[3];

			//split(src, splited);

			//imshow("a",splited[0]);
		//Mat RSrc;
		Mat dst=Mat::zeros(src.size(),src.type());
		//RSrc = Mat::zeros(src.size(), src.type());

		Mat ContourImg = src.clone();
		/*namedWindow("Hsv", 0);
		createTrackbar("LH", "Hsv", &lowH, 255, segmentImg);
		createTrackbar("HH", "Hsv", &highH, 255, segmentImg);
		createTrackbar("LS", "Hsv", &lowS, 255, segmentImg);
		createTrackbar("HS", "Hsv", &highS, 255, segmentImg);
		createTrackbar("LV", "Hsv", &lowV, 255, segmentImg);
		createTrackbar("HV", "Hsv", &highV, 255, segmentImg);*/

		segmentImg(src,dst);
        vector<vector<Point> > contour;
        vector<Point2f> point_img;
         contours_filter(dst, contour);
         point_img = pick_point(contour);
         //visual_point(src,point_img);
     Point2f src_vertices[4];
     src_vertices[0] = point_img[0];
     src_vertices[1] = point_img[1];
     src_vertices[2] = point_img[2];
     src_vertices[3] = point_img[3];
     //透视变换
     Mat warpMatrix = getPerspectiveTransform(src_vertices, dst_vertices);
     Mat warped;
     warpPerspective(dst, warped, warpMatrix, warped.size(), INTER_LINEAR, BORDER_CONSTANT);
     imshow("warped",warped);
     FindContours(warped, ContourImg);

     namedWindow("Contours", 1);

     imshow("Contours", ContourImg);
     DrawCircle(warped);

}


	//HSV
	void BallCenter::segmentImg(Mat&src,Mat&dst){

		Mat srcImg = src.clone();

		Mat HsvImg;
		Mat ImgThreshold;

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

		dst = ImgThreshold;

	}





//提取轮廓及中心点



void BallCenter::FindContours(Mat &Image, Mat &ContourImage) {
    vector<vector<Point>> contours;
    vector<Vec4i>hierarcy;
	//提取轮廓

	findContours(Image, contours, hierarcy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);

	//drawContours(ContourImage, contours, -1, Scalar(255, 0, 0), 2, 8, hierarcy);

	if (contours.size() != 2){
	    update=false;
	    return;
	}

	//绘制外接矩形

	vector<Rect> boundRect(contours.size());

	vector<vector<Point2f>> RectPoint(contours.size());


	float width = 0;

	float height = 0;

	float CX = 0;

	float CY = 0;
	Point2f pointFirst;
	Point2f pointZero;

	for (int i = 0; i < contours.size(); i++) {
		float x0 = 0, y0 = 0, w0 = 0, h0 = 0;

		boundRect[i] = boundingRect((Mat) contours[i]); //查找每个轮廓的外接矩形

		drawContours(ContourImage, contours, i, Scalar(0, 0, 255), 2, 8);  //绘制轮廓

		x0 = boundRect[i].x;  //获得第i个外接矩形的左上角的x坐标

		y0 = boundRect[i].y; //获得第i个外接矩形的左上角的y坐标

		w0 = boundRect[i].width; //获得第i个外接矩形的宽度

		h0 = boundRect[i].height; //获得第i个外接矩形的高度

		RectPoint[i].push_back(Point(x0, y0));

		RectPoint[i].push_back(Point(x0 + w0, y0));

		RectPoint[i].push_back(Point(x0 + w0, y0 + h0));

		RectPoint[i].push_back(Point(x0, y0 + h0));

		rectangle(ContourImage, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8); //绘制第i个外接矩形

	}


	if (boundRect[0].area() < boundRect[1].area()) {

		width = boundRect[1].width;

		height = boundRect[1].height;

		pointFirst = RectPoint[1][0];

		//判断中心点

		CX = (RectPoint[0][0].x + RectPoint[0][1].x) / 2.0;

		CY = (RectPoint[0][0].y + RectPoint[0][3].y) / 2.0;
		Point2f center = Point2f(CX, CY);
		circle(ContourImage, center, 1, Scalar(255, 0, 0), 1, 8, 0);

		//cout << "center:" << centerPoint;



	} else {

		width = boundRect[0].width;

		height = boundRect[0].height;

		pointFirst = RectPoint[0][0];

		//判断中心点

		CX = (RectPoint[1][0].x + RectPoint[1][1].x) / 2.0;

		CY = (RectPoint[1][0].y + RectPoint[1][3].y) / 2.0;
		Point2f center = Point2f(CX, CY);
		circle(ContourImage, center, 1, Scalar(255, 0, 0), 1, 8, 0);

		//cout << "center:" << centerPoint;



	}


	centerPoint.x = (CX - pointFirst.x) * 65 / Rwidth;
	centerPoint.y = (CY - pointFirst.y) * 65 / RHeight;

}

void BallCenter::contours_filter(Mat &imgThresholded, vector<vector<Point> > &contours)
{
    vector<Vec4i> hierarchy;
    findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//求轮廓　
    //初步筛选符合要求的矩形
    Mat contour = Mat::zeros(imgThresholded.rows, imgThresholded.cols, CV_8SC3);
    //drawContours(contour, contours, -1, Scalar(200, 200, 200), 2);
    //imshow("contours", contour);

}

vector<Point2f> BallCenter::pick_point(vector<vector<Point>>&contours) {
    vector<Point>hull;
    vector<Point2f> point_img;
    convexHull(contours[0], hull);
    //cout << hull.size();
    vector<Point> squar;
    size_t num = hull.size();
    if (num >= 4) {
        float max_area;
        for (int m = 0;m < num - 3;m++) {
            for (int n = m + 1;n < num - 2;n++) {
                for (int j = n + 1;j < num - 1;j++) {
                    for (int k = j + 1;k < num;k++) {
                        vector<Point> squar_tmp;
                        squar_tmp.push_back(hull[m]);
                        squar_tmp.push_back(hull[n]);
                        squar_tmp.push_back(hull[j]);
                        squar_tmp.push_back(hull[k]);
                        if (m == 0 && n == 1 && j == 2 && k == 3) {
                            max_area = fabs(contourArea(Mat(squar_tmp)));
                            squar.clear();
                            squar = squar_tmp;
                        }
                        else {
                            float area = fabs(contourArea(Mat(squar_tmp)));
                            if (area > max_area) {
                                max_area = area;
                                squar.clear();
                                squar = squar_tmp;
                            }
                        }
                    }
                }
            }
        }
    }
    //给四点排序
    vector<Point> squar_sort = squar;
    for (int i = 0;i < squar_sort.size();i++) {
        point_img.clear();
        for (size_t num_p = 0;num_p < squar_sort.size();num_p++) {
            // point_img.push_back(squar_sort[num_p] * (1 / minifactor));
            point_img.push_back(squar_sort[num_p]);
        }
    }

    vector<Point2f> point_temp = point_img;
    point_img.clear();
    point_img.push_back(point_temp[1]);
    point_img.push_back(point_temp[2]);
    point_img.push_back(point_temp[3]);
    point_img.push_back(point_temp[0]);
   // cout << point_img;
    return point_img;
}

// 可视化角点
void BallCenter::visual_point(Mat src, vector<Point2f> point_image)
{
    for (int i = 0;i < point_image.size();i++)
    {
        circle(src, point_image[i], 5, Scalar(255, 200, 0), 3);
    }
    //imshow("point", src);
    //imwrite("point.png", src);
}


void BallCenter::DrawCircle(Mat&warped){
     float Rx=Rwidth/65;
     float Ry=RHeight/65;
     vector<Point2f>Point;
     Point.push_back(Point2f(12.5*Rx,12.5*Ry));
    Point.push_back(Point2f(32.5*Rx,12.5*Ry));
    Point.push_back(Point2f(52.5*Rx,12.5*Ry));
    Point.push_back(Point2f(12.5*Rx,32.5*Ry));
    Point.push_back(Point2f(32.5*Rx,32.5*Ry));
    Point.push_back(Point2f(52.5*Rx,32.5*Ry));
    Point.push_back(Point2f(12.5*Rx,52.5*Ry));
    Point.push_back(Point2f(32.5*Rx,52.5*Ry));
    Point.push_back(Point2f(52.5*Rx,52.5*Ry));
    for(int i=0;i<Point.size();i++){
       circle(warped,Point[i],3*Rx,Scalar(255,0,0));

    }

    namedWindow("Cirle",1);
    imshow("Cirle",warped);



}




