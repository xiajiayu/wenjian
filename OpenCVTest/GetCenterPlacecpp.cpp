#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;

Mat segmentImg(Mat &src);
void contours_filter(Mat &imgThresholded, vector<vector<Point> > &contours);
vector<Point2f>pick_point(vector<vector<Point>>&contours);
void visual_point(Mat src, vector<Point2f> point_image);
/*int main() {
	VideoCapture cap(1);
	Mat frame;
	while (cap.isOpened()) {
		cap >> frame;
		imshow("frame", frame);
		Mat imgThresholded = segmentImg(frame);

		vector<vector<Point> > contours;

		vector<Point2f> point_img;

		contours_filter(imgThresholded, contours);

		point_img = pick_point(contours);

		visual_point(frame, point_img);
		waitKey(1);
	}
	return 0;

}
*/
/*
Mat segmentImg(Mat &src) {
	Mat srcImg = src.clone();
	Mat HsvImg, ImgThreshold;

	//BGRתHSV
	cvtColor(srcImg, HsvImg, COLOR_BGR2HSV);
	vector<Mat>splitHsv;
	split(HsvImg, splitHsv);
	equalizeHist(splitHsv[2], splitHsv[2]);
	merge(splitHsv, HsvImg);
	int lowH = 0, highH = 180, lowS = 0, highS = 43, lowV = 50, highV = 255;
	inRange(HsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), ImgThreshold);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(ImgThreshold, ImgThreshold, MORPH_OPEN, element);//������
	morphologyEx(ImgThreshold, ImgThreshold, MORPH_CLOSE, element);//�ղ���
	//namedWindow("thresholdImage", 0);
	//imshow("thresholdImage", ImgThreshold);
	return ImgThreshold;
}





//�������

void contours_filter(Mat &imgThresholded, vector<vector<Point> > &contours)
{
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//��������
	//����ɸѡ����Ҫ��ľ���
	Mat contour = Mat::zeros(imgThresholded.rows, imgThresholded.cols, CV_8SC3);
	drawContours(contour, contours, -1, Scalar(200, 200, 200), 2);
	imshow("contours", contour);
}



vector<Point2f>pick_point(vector<vector<Point>>&contours) {
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

	//���ĵ�����

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

	cout << point_img;

	return point_img;

}



// ���ӻ��ǵ�

void visual_point(Mat src, vector<Point2f> point_image)

{

	for (int i = 0;i < point_image.size();i++)
	{
		circle(src, point_image[i], 5, Scalar(255, 200, 0), 3);
	}
	imshow("point", src);
	imwrite("point.png", src);

}



*/