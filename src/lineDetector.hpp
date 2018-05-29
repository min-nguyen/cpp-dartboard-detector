#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sobel_operator.h"
#include "circleDetector.hpp"
#include <math.h>
#include <algorithm> 
#include <numeric> 
#include <algorithm> 
using namespace cv;
using namespace std;

class LineDetector{
    public:
        static Point trueIntersect(Vec4i t_line1, Vec4i t_line2);
        static bool isLineInBox(Vec4i box, Vec4i t_line);
        //Returns vector of intersection coordinates and associated lines
        //Vec3i         -> intersections :: [(central x, central y, num intersections)]
        //Vector<Vec4i> -> lines         :: [(x1, y1, x2, y2)]
        static vector<pair<Vec3i, vector<Vec4i> > > findLineIntersections(Vec4i boundingBox, Mat img, vector<Vec4i> lines);    
        static Mat houghLineSpace(Mat *grad_mag, Mat* grad_dir);
        static bool isEqual(const Vec4i& _l1, const Vec4i& _l2);
        static vector<pair<Vec3i, vector<Vec4i> > > houghLineIntersect(Vec4i boundingBox, Mat src, Mat *grad_mag, Mat* grad_dir);
        static vector<Vec4i> circlesToBoxes(vector<Circle> circles);
        static double cotan(double i) { return(1 / std::tan(i)); }
        static double cosec(double i) { return(1 / std::sin(i)); }
};


#endif