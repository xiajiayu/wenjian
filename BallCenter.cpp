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

void segmentImg(int, void*) {

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

	if (contours.size() != 2)return;

	//������Ӿ���

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

	float width = boundRect[0].width;
	float height = boundRect[0].height;

	for (int i = 0;i < boundRect.size() - 1;i++)
		for (int j = i + 1;j < boundRect.size();j++) {
			if (boundRect[i].area() < boundRect[j].area()) {

				width = boundRect[j].width;

				height = boundRect[j].height;
				pointFirst = RectPoint[j][0];
			}
			
			//�ж����ĵ�
		}

	cout << width << endl;
	cout << height << endl;
	cout << pointFirst;
	//�õ�С������
	



}

