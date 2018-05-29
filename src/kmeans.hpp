#ifndef KMEANS_HPP
#define KMEANS_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdbool.h>
#include <numeric>
#include "detector.hpp"
using namespace std;
using namespace cv;
#define INVALID_ARG -1


class Kmeans{
public:
    static vector<Rect> getFinalResult(String filename);
private:
    static void averageRegion(Size size, Mat& labels, Mat& new_labels);
    static Mat translateImg(const Mat &img, Mat& trans_img, int offsetx, int offsety);
    static Mat* kmeans(Mat src);
    static bool templMatch(Mat test, Mat read_templ);
};

#endif 