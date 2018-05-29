#ifndef CIRCLEDETECTOR_H
#define CIRCLEDETECTOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sobel_operator.h"
#include <math.h>
#include <algorithm> 
#include <numeric> 
#include <algorithm> 
using namespace cv;
using namespace std;
#define MAXRAD 100
#define MINRAD 20
struct Circle {
    Point p;
    int radius;

    Circle(Point p, int radius){
        this->p = p;
        this->radius = radius;
    }
};
class CircleDetector{
public:
        static void displayImage(const std::string& window_name, const Mat& image);
        static vector<Circle> houghCircles(Mat *grad_mag, Mat* grad_dir);
    private:
        static void display_centers(int ***huff_space, int rows, int cols, int cells);
        static vector<Circle> draw_itersecting_circles(vector<Circle>* circles, int rows, int cols);
        static vector<Circle> display_circles(int ***huff_space, int rows, int cols, int cells);
        static void drawThresholdedHuffSpace(Mat_<uchar>* grad_mag, uchar threshold_S);
        static std::vector<std::vector<Rect> > parse(const char** argv);
		static char* shiftToNPastNextSpace(char* strp, int n);
		static void printError(char* errorMessage_strliteral);
};


#endif