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
    bool update=false;
private:
    int lowH = 151, highH = 183, lowS = 31, highS = 255, lowV = 50, highV = 255;
    float Rwidth=408 ,RHeight = 409;
    Point2f dst_vertices[4]={Point2f(116,454),Point2f(116,45),Point2f(524,45),Point2f(524,454)};


};



#endif //BALLBALANCE_BALLCENTER_H
