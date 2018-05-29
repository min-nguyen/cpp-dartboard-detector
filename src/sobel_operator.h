#ifndef _SOBEL_OPERATOR_H
#define _SOBEL_OPERATOR_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector> //std::vector
#include <numeric> //std::accumulate
#include <algorithm> //std::transform, std::clamp
#include <cmath> //std::hypot, std::atan
#include <cstdlib> //std::abs (int)
using namespace cv;

struct sobel_output {
    Mat_<float>* gradient_x; 
    Mat_<float>* gradient_y; 
    Mat_<uchar>* gradient_mag;
    Mat_<float>* gradient_dir;
};

class Sobel{
private:
    static Mat_<int>* xkernel();
    static Mat_<int>* ykernel();
    static Mat_<float>* conv(const Mat* input, const Mat_<int>* kernel);
    static Mat_<uchar>* calc_gradient_mag(const Mat_<float>* gradient_x, const Mat_<float>* gradient_y, uchar (*func)(const float&, const float&));
    //static Mat_<float>* calc_gradient_mag(const Mat_<float>* gradient_x, const Mat_<float>* gradient_y, float (*func)(const float&, const float&));
    static Mat_<float>* calc_gradient_dir(const Mat_<float>* gradient_x, const Mat_<float>* gradient_y, float (*func)(const float&, const float&));

public:
    static struct sobel_output* sobel(const Mat* input);

};

#endif

