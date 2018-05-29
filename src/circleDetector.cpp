#include "circleDetector.hpp"

void CircleDetector::displayImage(const std::string& window_name, const Mat& image){
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    imshow(window_name, image);
}

void CircleDetector::display_centers(int ***huff_space, int rows, int cols, int cells){
    Mat *circle_centers = new Mat(rows, cols, CV_8UC1);
    for(int row = 0; row < rows; row++){
        for(int col = 0; col < cols; col++){
            circle_centers->at<uchar>(row,col) = (uchar) std::accumulate(huff_space[row][col], huff_space[row][col] + cells, 0); 
        }
    }
   displayImage("Circle Centers", *circle_centers);
}

#define tolerance 30
vector<Circle> CircleDetector::draw_itersecting_circles(vector<Circle>* circles, int rows, int cols){

    int grid_rows = rows/tolerance + 1;
    int grid_cols = cols/tolerance + 1;
    vector<Circle> **reduced_image_grid = new vector<Circle>*[grid_rows];
    for(int grid_row = 0; grid_row < grid_rows; grid_row++){
        reduced_image_grid[grid_row] = new vector<Circle>[grid_cols];
    }

    for(Circle circle : *circles){
        int grid_row = circle.p.y/tolerance;
        int grid_col = circle.p.x/tolerance;
        reduced_image_grid[grid_row][grid_col].push_back(circle);
    }

    vector<vector<Circle>> groups;
    for(int grid_row = 0; grid_row < grid_rows; grid_row++){
        for(int grid_col = 0; grid_col < grid_cols; grid_col++){
            bool isGroup = !reduced_image_grid[grid_row][grid_col].empty();
            if(isGroup) groups.push_back(reduced_image_grid[grid_row][grid_col]);
        }
    }

    //reduction (reduce a vector of circles to a circle)
    auto toPoint = [](const Circle& c1) -> Point{ return c1.p; };
    auto toRadius = [](const Circle& c1) -> int{ return c1.radius; };
    auto max_length = [](const vector<Circle>& vec1, vector<Circle>& vec2) -> vector<Circle>{ 
        return vec1.size() > vec2.size() ? vec1 : vec2;
    };
    int maxsize = std::accumulate(groups.begin(), groups.end(), vector<Circle>(), max_length).size();
    

    vector<Circle> final_circles;
    for(vector<Circle> group : groups){
        if((float)group.size() >= (maxsize * 0.8)){
            vector<Point> points_in_group(group.size());
            //printf("Group size: %d\n", group.size());
            vector<int> radiuss_in_group(group.size());
            std::transform(group.begin(), group.end(), points_in_group.begin(), toPoint);
            std::transform(group.begin(), group.end(), radiuss_in_group.begin(), toRadius);
            auto maxRadius = std::max_element(radiuss_in_group.begin(), radiuss_in_group.end());
            //printf("size: %d\n", points_in_group.size());
            Point meanPoint = std::accumulate(points_in_group.begin(), points_in_group.end(), Point(0,0)) / (int)points_in_group.size();
            final_circles.push_back(Circle(meanPoint, *maxRadius));
        }
    }

    Mat circles_Mat(rows, cols, CV_8UC1);

    for(Circle c : final_circles){
        cv::circle(circles_Mat, c.p, c.radius, Scalar(255, 255 ,255));
    }
    displayImage("Concentric Circles", circles_Mat);
    return final_circles;
}

vector<Circle> CircleDetector::display_circles(int ***huff_space, int rows, int cols, int cells){
    int max = 0;
    for(int row = 0; row < rows; row++){
        for(int col = 0; col < cols; col++){
            for(int cell = 0; cell < cells; cell++){
                int val = huff_space[row][col][cell];
                if(val > max) max = val;
            }
        }
    }
    //thresholdH - oversensitive: 0.8 undersensitive: 0.9
    int threshold_H = 0.9 * max;

    Mat circles_Mat(rows, cols, CV_8UC1, Scalar(0));
    vector<Circle>* circles = new vector<Circle>();
    for(int row = 0; row < rows; row++){
        for(int col = 0; col < cols; col++){
            for(int cell = 0; cell < cells; cell++){
                if(huff_space[row][col][cell] >= threshold_H){
                    circle(circles_Mat, Point(col, row), cell, Scalar(255, 0, 0));
                    circles->push_back(Circle(Point(col, row), cell));
                }   
            }
        }
    }
   displayImage("Circles", circles_Mat);
    vector<Circle> final_circles = draw_itersecting_circles(circles, rows, cols);
    return final_circles;
}

void CircleDetector::drawThresholdedHuffSpace(Mat_<uchar>* grad_mag, uchar threshold_S){
    Mat_<uchar>* thresholded_gradmag = (Mat_<uchar>*) new Mat(grad_mag->rows, grad_mag->cols, grad_mag->type());
    for(int row = 0; row < grad_mag->rows; row++){
        for(int col = 0; col < grad_mag->cols; col++){
            if(((*grad_mag)(row, col)) >= threshold_S) (*thresholded_gradmag)(row, col) = (*grad_mag)(row, col); 
            else (*thresholded_gradmag)(row, col) = 0;
        }
    }
   displayImage("Thresholded Magnitude", *thresholded_gradmag);
}

vector<Circle> CircleDetector::houghCircles(Mat *grad_mag, Mat* grad_dir){
    //Scale
    for(int row = 0; row < grad_mag->rows; row++){
        for(int col = 0; col < grad_mag->cols; col++) grad_mag->at<uchar>(row,col) = grad_mag->at<uchar>(row,col) * 3;
    }

    uchar max = *std::max_element(grad_mag->datastart, grad_mag->dataend);
    printf("max threshold_S %d\n", max);
    //thresholdS - image1: 0.45
    uchar threshold_S = max * 0.45;
    printf("threshold_S %d\n", threshold_S);

    drawThresholdedHuffSpace((Mat_<uchar>*)grad_mag, threshold_S);

    int cells = MAXRAD;
    int ***huff_space = new int**[grad_mag->rows];
    for(int ***row = huff_space; row < huff_space + grad_mag->rows; row++){
        *row = new int*[grad_mag->cols];
        for(int **col = *row; col < *row + grad_mag->cols; col++){
            *col = new int[cells];
            for(int *cell = *col; cell < *col + cells; cell++) *cell = 0;
        }
    }

    for(int y = 0; y < grad_mag->rows; y++){
        for(int x = 0; x < grad_mag->cols; x++){
            if(grad_mag->at<uchar>(y, x) > threshold_S){
                float angle = grad_dir->at<float>(y, x);
                for(int r = MINRAD; r < MAXRAD; r++){
                    int x_0 = x + r * std::cos(angle);
                    int y_0 = y + r * std::sin(angle);
                    if(x_0 >= 0 && x_0 < grad_mag->cols && y_0 >= 0 && y_0 < grad_mag->rows){
                        huff_space[y_0][x_0][r] += 1;
                    }

                    x_0 = x - r * std::cos(angle);
                    y_0 = y - r * std::sin(angle);
                    if(x_0 >= 0 && x_0 < grad_mag->cols && y_0 >= 0 && y_0 < grad_mag->rows){
                        huff_space[y_0][x_0][r] += 1;
                    }
                }
            }
        }
    }
    display_centers(huff_space, grad_mag->rows, grad_mag->cols, cells);
    vector<Circle> circles = display_circles(huff_space, grad_mag->rows, grad_mag->cols, cells);
    return circles;
}
