#include "sobel_operator.h"
using namespace cv;

Mat_<int>* Sobel::xkernel(){ //int
    Mat* xkernel = new Mat();
    std::vector<int> row1 = { 1, 0, -1 };
    std::vector<int> row2 = { 2, 0, -2 };
    std::vector<int> row3 = { 1, 0, -1 };
    xkernel->push_back(Mat(row1).t());
    xkernel->push_back(Mat(row2).t());
    xkernel->push_back(Mat(row3).t());
    return (Mat_<int>*) xkernel;
}

Mat_<int>* Sobel::ykernel(){
    Mat* ykernel = new Mat();
    std::vector<int> row1 = { 1, 2, 1 };
    std::vector<int> row2 = { 0, 0, 0 };
    std::vector<int> row3 = { -1, -2, -1 };
    ykernel->push_back(Mat(row1).t());
    ykernel->push_back(Mat(row2).t());
    ykernel->push_back(Mat(row3).t());
    return (Mat_<int>*) ykernel; //Mat.type() -> CV_32SC1
}


Mat_<float>* Sobel::conv(const Mat* input, const Mat_<int>* kernel){
    Mat temp(kernel->rows, kernel->cols, kernel->type());
    int (*abs) (int) = &std::abs;
    std::transform((int*)kernel->datastart, (int*)kernel->dataend, (int*)temp.datastart, abs);
    int normalization_factor = cv::sum(temp)[0];
    
    Mat_<float>* output = (Mat_<float>*) new Mat(input->rows, input->cols, CV_32FC1);
    for(int iy = 0; iy < input->rows; iy++){ //image_y
        for(int ix = 0; ix < input->cols; ix++){ //image_x

            std::vector<float> cov_xy;
            for(int ky = -kernel->rows/2; ky <= kernel->rows/2; ky++){ //kernel_y
                for(int kx = -kernel->cols/2; kx <= kernel->cols/2 ; kx++){ //kernel_x
                    if(!(iy + ky < 0) && !(iy + ky >= input->rows) && !(ix + kx < 0) && !(ix + kx >= input->cols)){
                        cov_xy.push_back(input->at<uchar>(iy+ky, ix+kx) * (*kernel)(ky + kernel->rows/2, kx + kernel->cols/2));
                    }
                }
            }
            (*output)(iy, ix) = std::accumulate(cov_xy.begin(), cov_xy.end(), 0) / normalization_factor;
        }
    }
    return output;
}

Mat_<uchar>* Sobel::calc_gradient_mag(const Mat_<float>* gradient_x, const Mat_<float>* gradient_y, uchar (*func)(const float&, const float&)){
    assert(gradient_x->size == gradient_y->size);
    Mat_<uchar>* output = (Mat_<uchar>*) new Mat(gradient_x->rows, gradient_x->cols, CV_8UC1);
    std::transform((float*)gradient_x->datastart, (float*)gradient_x->dataend,
         (float*)gradient_y->datastart, (uchar*) output->datastart, func);
    return output;
}

// Mat_<float>* Sobel::calc_gradient_mag(const Mat_<float>* gradient_x, const Mat_<float>* gradient_y, float (*func)(const float&, const float&)){
//     assert(gradient_x->size == gradient_y->size);
//     Mat_<float>* output = (Mat_<float>*) new Mat(gradient_x->rows, gradient_x->cols, CV_32FC1);
//     std::transform((float*)gradient_x->datastart, (float*)gradient_x->dataend,
//          (float*)gradient_y->datastart, (float*) output->datastart, func);
//     return output;
// }


Mat_<float>* Sobel::calc_gradient_dir(const Mat_<float>* gradient_x, const Mat_<float>* gradient_y, float (*func)(const float&, const float&)){
    assert(gradient_x->size == gradient_y->size);
    Mat_<float>* output = (Mat_<float>*) new Mat(gradient_x->rows, gradient_x->cols, CV_32FC1);
    std::transform((float*)gradient_x->datastart, (float*)gradient_x->dataend,
         (float*)gradient_y->datastart, (float*) output->datastart, func);
    // for(int row = 0; row < output->rows; row++){
    //     for(int col = 0; col < output->cols; col++){
    //         output->at<float>(row, col) = std::atan(gradient_y->at<float>(row, col) / gradient_x->at<float>(row, col));
    //     }
    // }
    return output;
}

// template <typename Ret, typename Type>
// Ret my_hypot(const Type &x, const Type &y){
//     return std::hypot(x, y);
// }

// template <typename Ret, typename Type>
// Ret my_arctan(const Type &x, const Type &y){
//     return std::atan(y / x);
// }

struct sobel_output* Sobel::sobel(const Mat* input){ 
    struct sobel_output* sobel_out = new sobel_output();
    sobel_out->gradient_x = conv(input, xkernel());
    sobel_out->gradient_y = conv(input, ykernel());

    //lambdas that do not capture can be assigned to function pointers
    //std::function<float(int,int)> can be used instead if it did capture
    uchar (*hypot)(const float&, const float&) = [](const float& x, const float& y) -> uchar{
        return (uchar) std::hypot(x, y);
    };
    // float (*hypot)(const float&, const float&) = [](const float& x, const float& y) -> float{
    //     return (float) std::hypot(x, y);
    // };
    sobel_out->gradient_mag = calc_gradient_mag(sobel_out->gradient_x, sobel_out->gradient_y, hypot);
    //sobel_out->gradient_mag = gradient_info(sobel_out->gradient_x, sobel_out->gradient_y, &my_hypot<float,int>);

    float (*arctan)(const float&, const float&) = [](const float& x, const float& y) -> float{
        if(x == 0) return 0;
        else return std::atan(y / x);
    };
    sobel_out->gradient_dir = (Mat_<float>*) calc_gradient_dir(sobel_out->gradient_x, sobel_out->gradient_y, arctan);
    //sobel_out->gradient_dir = gradient_info(sobel_out->gradient_x, sobel_out->gradient_y, &my_arctan<float,int>);
    return sobel_out;
}



