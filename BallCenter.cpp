#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace cv;
int lowH =155, highH = 185, lowS = 34, highS = 255, lowV = 42, highV = 255;
void segmentImg(int,void*);
void FindContours(Mat&Image,Mat&ContourImage);
Mat src;
Mat RSrc;
Mat dst;
vector<vector<Point>> contours;
vector<Vec4i>hierarcy;
int main() {
	VideoCapture cap(0);
	while (cap.isOpened()) {
		cap >> src;
	//src = imread("src3.png");
		//Mat splited[3];
		//split(src, splited);
		//imshow("a",splited[0]);
		RSrc = Mat::zeros(src.size(), src.type());
		Mat ContourImg = src.clone();
		//GaussianBlur(src,RSrc, Size(5, 5),0.0);
		namedWindow("Hsv", 0);
		createTrackbar("LH", "Hsv", &lowH, 255,segmentImg);
		createTrackbar("HH", "Hsv", &highH, 255, segmentImg);
		createTrackbar("LS", "Hsv", &lowS, 255, segmentImg);
		createTrackbar("HS", "Hsv", &highS, 255,segmentImg);
		createTrackbar("LV", "Hsv", &lowV, 255, segmentImg);
		createTrackbar("HV", "Hsv", &highV, 255, segmentImg);
		segmentImg(0,0);
		FindContours(dst, ContourImg);
		imshow("Contours", ContourImg);
		waitKey(1);
	}
	return 0;
}
//图像二值化
//Ostu法
/*int Ostu(Mat &image) {
	int width = image.cols;
	int height = image.rows;
	int x = 0, y = 0;
	int pixelCount[256];
	float pixelPro[256];
	int i, j, pixelSum = width * height, threshold = 0;

	uchar* data = (uchar*)image.data;

	//初始化  
	for (i = 0; i < 256; i++)
	{
		pixelCount[i] = 0;
		pixelPro[i] = 0;
	}

	//统计灰度级中每个像素在整幅图像中的个数  
	for (i = y; i < height; i++)
	{
		for (j = x; j < width; j++)
		{
			pixelCount[data[i * image.step + j]]++;
		}
	}


	//计算每个像素在整幅图像中的比例  
	for (i = 0; i < 256; i++)
	{
		pixelPro[i] = (float)(pixelCount[i]) / (float)(pixelSum);
	}

	//经典ostu算法,得到前景和背景的分割  
	//遍历灰度级[0,255],计算出方差最大的灰度值,为最佳阈值  
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (i = 0; i < 256; i++)
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

		for (j = 0; j < 256; j++)
		{
			if (j <= i) //背景部分  
			{
				//以i为阈值分类，第一类总的概率  
				w0 += pixelPro[j];
				u0tmp += j * pixelPro[j];
			}
			else       //前景部分  
			{
				//以i为阈值分类，第二类总的概率  
				w1 += pixelPro[j];
				u1tmp += j * pixelPro[j];
			}
		}

		u0 = u0tmp / w0;        //第一类的平均灰度  
		u1 = u1tmp / w1;        //第二类的平均灰度  
		u = u0tmp + u1tmp;      //整幅图像的平均灰度  
								//计算类间方差  
		deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
		//找出最大类间方差以及对应的阈值  
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = i;
		}
	}
	return threshold;
}*/
//HSV

void segmentImg(int, void*) {

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


void FindContours(Mat &Image, Mat &ContourImage) {

	//提取轮廓

	findContours(Image, contours, hierarcy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);

	//drawContours(ContourImage, contours, -1, Scalar(255, 0, 0), 2, 8, hierarcy);

	if (contours.size() != 2)return;

	//绘制外接矩形

	vector<Rect>boundRect(contours.size());

	vector<vector<Point2f>>RectPoint(contours.size());





	float CX = 0;

	float CY = 0;

	Point2f centerPoint;

	Point2f pointFirst;
	Point2f pointZero;

	for (int i = 0; i < contours.size(); i++)

	{
		float x0 = 0, y0 = 0, w0 = 0, h0 = 0;

		boundRect[i] = boundingRect((Mat)contours[i]); //查找每个轮廓的外接矩形

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

	float width = boundRect[0].width;
	float height = boundRect[0].height;

	for (int i = 0;i < boundRect.size() - 1;i++)
		for (int j = i + 1;j < boundRect.size();j++) {
			if (boundRect[i].area() < boundRect[j].area()) {

				width = boundRect[j].width;

				height = boundRect[j].height;
				pointFirst = RectPoint[j][0];
			}
			
			//判断中心点
		}

	cout << width << endl;
	cout << height << endl;
	cout << pointFirst;
	//得到小球坐标
	



}

