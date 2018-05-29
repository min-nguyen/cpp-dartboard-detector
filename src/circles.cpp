#include "circleDetector.hpp"
#include "sobel_operator.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

int main(int argc, char** argv){
    Mat image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    sobel_output* sobel_output = Sobel::sobel(&image);
    std::vector<Circle> circles = CircleDetector::houghCircles(sobel_output->gradient_mag, sobel_output->gradient_dir);

    for(Circle circle : circles){
        cv::circle(image, circle.p, circle.radius, Scalar(255,255,0));
    }
    imshow("Hough Circles", image);
    waitKey(0);
}