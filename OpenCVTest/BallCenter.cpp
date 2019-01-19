#include<opencv2/opencv.hpp>
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
		cap >> src;
		//Mat splited[3];
		//split(src, splited);
		//imshow("a",splited[0]);
		RSrc = Mat::zeros(src.size(), src.type());
		Mat ContourImg = src.clone();
		//GaussianBlur(src,RSrc, Size(8, 8),0.0);//Ϊʲô����������Ҫͣ�٣�
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
//ͼ���ֵ��
//Ostu��
/*int Ostu(Mat &image) {
	int width = image.cols;
	int height = image.rows;
	int x = 0, y = 0;
	int pixelCount[256];
	float pixelPro[256];
	int i, j, pixelSum = width * height, threshold = 0;

	uchar* data = (uchar*)image.data;

	//��ʼ��  
	for (i = 0; i < 256; i++)
	{
		pixelCount[i] = 0;
		pixelPro[i] = 0;
	}

	//ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���  
	for (i = y; i < height; i++)
	{
		for (j = x; j < width; j++)
		{
			pixelCount[data[i * image.step + j]]++;
		}
	}


	//����ÿ������������ͼ���еı���  
	for (i = 0; i < 256; i++)
	{
		pixelPro[i] = (float)(pixelCount[i]) / (float)(pixelSum);
	}

	//����ostu�㷨,�õ�ǰ���ͱ����ķָ�  
	//�����Ҷȼ�[0,255],������������ĻҶ�ֵ,Ϊ�����ֵ  
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (i = 0; i < 256; i++)
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

		for (j = 0; j < 256; j++)
		{
			if (j <= i) //��������  
			{
				//��iΪ��ֵ���࣬��һ���ܵĸ���  
				w0 += pixelPro[j];
				u0tmp += j * pixelPro[j];
			}
			else       //ǰ������  
			{
				//��iΪ��ֵ���࣬�ڶ����ܵĸ���  
				w1 += pixelPro[j];
				u1tmp += j * pixelPro[j];
			}
		}

		u0 = u0tmp / w0;        //��һ���ƽ���Ҷ�  
		u1 = u1tmp / w1;        //�ڶ����ƽ���Ҷ�  
		u = u0tmp + u1tmp;      //����ͼ���ƽ���Ҷ�  
								//������䷽��  
		deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
		//�ҳ������䷽���Լ���Ӧ����ֵ  
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = i;
		}
	}
	return threshold;
}*/
//HSV

void segmentImg(int,void*) {
	Mat srcImg = src.clone();
	Mat HsvImg, ImgThreshold;
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
	dst=ImgThreshold;
}


//��ȡ���������ĵ�

void FindContours(Mat &Image, Mat &ContourImage) {
	//��ȡ����
	findContours(Image, contours, hierarcy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);
	//drawContours(ContourImage, contours, -1, Scalar(255, 0, 0), 2, 8, hierarcy);
	//������Ӿ���
	vector<Rect>boundRect(contours.size());
	int x0 = 0, y0 = 0, w0 = 0, h0 = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		boundRect[i] = boundingRect((Mat)contours[i]); //����ÿ����������Ӿ���
		drawContours(ContourImage, contours, i, Scalar(0, 0, 255), 2, 8);  //��������
		x0 = boundRect[i].x;  //��õ�i����Ӿ��ε����Ͻǵ�x����
		y0 = boundRect[i].y; //��õ�i����Ӿ��ε����Ͻǵ�y����
		w0 = boundRect[i].width; //��õ�i����Ӿ��εĿ��
		h0 = boundRect[i].height; //��õ�i����Ӿ��εĸ߶�
		rectangle(ContourImage, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8); //���Ƶ�i����Ӿ���
	}
	//�ж����ĵ�
	Point2f centerPoint;
	centerPoint.x = (2*x0 + w0) / 2.0;
	centerPoint.y = (2*y0 + h0) / 2.0;
	circle(ContourImage, centerPoint, 1, Scalar(255,0,0), 1,8,0);
	//cout << "center:" << centerPoint;
}






