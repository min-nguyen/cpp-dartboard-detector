#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include "sobel_operator.h"
#include "circleDetector.hpp"
#include "lineDetector.hpp"
#include <math.h>
#include <algorithm> //std::for_each
#include <numeric> //std::transform, std::accumulate
#include <algorithm> 
#include <unordered_map>
using namespace cv;
using namespace std;


class Detector{
public:
    static std::pair<std::vector<Vec4i>, std::vector<Vec4i> > detect_VH_Intersections(String FILE);
    static std::pair<std::vector<Vec4i>, std::vector<Vec4i> > detect_VH(String FILE);
protected:
    static std::pair<std::vector<Vec4i>, std::vector<Vec4i> >  rectUnion(std::vector<Vec4i> viola, std::vector<Vec4i> hough);
    static float rectMedianDifference(Vec4i viola, Vec4i hough);
    static float rectOverlap(Vec4i violaa, Vec4i hough);
    static std::vector<Vec4i> violaRects(Mat frame);
    
};

#endif