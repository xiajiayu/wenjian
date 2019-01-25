//
// Created by xiajiayu on 19-1-19.
//

#ifndef BALLBALANCE_BALLCENTER_H
#define BALLBALANCE_BALLCENTER_H
#include"Header.h"
using namespace cv;
using namespace std;
class BallCenter{
public:
    BallCenter();
    void process(Mat &src);

private:
    void contours_filter(Mat &imgThresholded, vector<vector<Point> > &contours);
    vector<Point2f>pick_point(vector<vector<Point>>&contours);
    void visual_point(Mat src, vector<Point2f> point_image);
    void segmentImg(Mat&src,Mat &dst);
    void FindContours(Mat&Image, Mat&ContourImage);
public:
    Point2f centerPoint=Point2f(32.5,32.5);
    bool update=true;
private:
    int lowH = 155, highH = 185, lowS = 34, highS = 255, lowV = 46, highV = 255;
    float Rwidth=396 ,RHeight = 397;
    Point2f dst_vertices[4]={Point2f(120,427),Point2f(120,30),Point2f(516,30),Point2f(516,427)};


};



#endif //BALLBALANCE_BALLCENTER_H
