#include "kmeans.hpp"

Mat* Kmeans::kmeans(Mat src){
  //Each pixel is assigned a row to store their R,G and B values
    Mat samples(src.rows * src.cols, 3, CV_32F);
    for( int y = 0; y < src.rows; y++ ){
        for( int x = 0; x < src.cols; x++ ){
            for( int z = 0; z < 3; z++){
            // if(z == 0 || z == 1){
            //     samples.at<float>(y + x*src.rows, z) = 0;
            // }
            //Each R,G and B value is stored from the image
            samples.at<float>(y + x*src.rows, z) = src.at<Vec3b>(y,x)[z];
        }   
        }     
    }

    int clusterCount = 2; //Looking only for black and white
    Mat labels; //Mat object that stores for each pixel (column) its cluster label (first row)
    int attempts = 5; //Runs of kmeans with different initializations
    Mat centers; //Cluster centroids
    cv::kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers );
    
    Mat new_labels = Mat(labels.size(), labels.type());
    averageRegion(src.size(), labels, new_labels);

    Mat new_image = Mat( src.size(), src.type() ); //Thresholded image
    for( int y = 0; y < src.rows; y++ )
    for( int x = 0; x < src.cols; x++ ){ 
        int cluster_idx = new_labels.at<int>(y + x*src.rows, 0);
        for(int z = 0; z < 3; z++){
            new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
            new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
            new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
        }
    }

    imshow( "Clustered Image", new_image );
    return new Mat(new_image);
}

void Kmeans::averageRegion(Size size, Mat& labels, Mat& new_labels){
        for(int row = 0; row < size.height; row++){
        for(int col = 0; col < size.width; col++){
            vector<int> neighbours;
            for(int row_offset = -size.height/40; row_offset < size.height/40; row_offset++){
                for(int col_offset = -size.width/40; col_offset < size.width/40; col_offset++){
                    if(row+row_offset > 0 && row+row_offset < size.height && col+col_offset > 0 && col+col_offset < size.width){
                        neighbours.push_back(labels.at<int>((row+row_offset)*size.height + (col+col_offset), 0));
                    }
                }
            }
            float mean = std::accumulate(neighbours.begin(), neighbours.end(), 0.0) / neighbours.size();
            if(mean > 0.5) new_labels.at<int>(row*size.height + col, 0) = 1;
            else new_labels.at<int>(row*size.height + col, 0) = 0;
        }
    } 
}

Mat Kmeans::translateImg(const Mat &img, Mat& trans_img, int offsetx, int offsety){
    Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img, trans_img , trans_mat, img.size());
    return trans_mat;
}

bool Kmeans::templMatch(Mat test, Mat read_templ){
    Mat templ(test.rows, test.cols, read_templ.type());
    resize(read_templ, templ, test.size());

    cvtColor(test, test, COLOR_BGR2GRAY);
    cvtColor(templ, templ, COLOR_BGR2GRAY);
    threshold(test, test, INVALID_ARG, 255, THRESH_BINARY | THRESH_OTSU);
    threshold(templ, templ, INVALID_ARG, 255, THRESH_BINARY | THRESH_OTSU);

    imshow("Test Image", test);
    imshow("Template", templ);

    resize(templ, templ, test.size());

    double max_hamming_dist = 0.0f;
    Mat trans_scaled_originalSize = templ.clone();
    // for(float scale = 0.4; (scale+=0.2) < 1.6; ){
    //     Mat trans_scaled_originalSize(templ.rows, templ.cols, templ.type());
    //     Mat trans_scaled(scale * templ.rows, scale * templ.cols, templ.type());
    //     resize(templ, trans_scaled, trans_scaled.size(), 0, 0, INTER_LINEAR);
    //     if(scale < 1){
    //         trans_scaled.copyTo(trans_scaled_originalSize(Rect((trans_scaled_originalSize.rows - trans_scaled.rows) / 2, (trans_scaled_originalSize.cols - trans_scaled.cols) / 2, trans_scaled.rows, trans_scaled.cols)));
    //     }
    //     if(scale > 1){
    //         trans_scaled_originalSize = trans_scaled(Rect((trans_scaled.rows - trans_scaled_originalSize.rows) / 2, (trans_scaled.cols - trans_scaled_originalSize.cols) / 2, trans_scaled_originalSize.rows, trans_scaled_originalSize.cols));
    //     }
        Mat trans_scaled_originalSize_shifted = Mat(templ.rows, templ.cols, templ.type());
        for(int trans_y = -80; (trans_y+=40) < 80; ){
            for(int trans_x = -80; (trans_x+=40) < 80; translateImg(trans_scaled_originalSize, trans_scaled_originalSize_shifted, trans_x, trans_y)){
                double hamming_dist = 0.0f;
                for(int row = 0; row < test.rows; row++){
                    for(int col = 0; col < test.cols; col++){
                        hamming_dist += norm(test.at<Vec3b>(row,col), trans_scaled_originalSize_shifted.at<Vec3b>(row,col));
                    }
                }
                max_hamming_dist = std::max(hamming_dist, max_hamming_dist);
            }
        }
    // }
    double percentage_match = max_hamming_dist / (test.rows * test.cols * 255) * 100;
    std::cout << percentage_match << std::endl;
    if(percentage_match > 80) return true;
    else false;
}

vector<Rect> Kmeans::getFinalResult(String filename){
    Mat color_image = imread(filename, CV_LOAD_IMAGE_COLOR );

    //CLIP Box from Image and use kmeans color segmentation on that
    std::pair<std::vector<Vec4i>, std::vector<Vec4i>> output = Detector::detect_VH(filename);
    
    std::vector<Vec4i> overlapped = output.first;
    std::vector<Vec4i> non = output.second;

    std::vector<Rect> overlapped_rects (overlapped.size());
    std::transform(overlapped.begin(), overlapped.end(), overlapped_rects.begin(), [](const Vec4i vec4i) -> Rect{
        return Rect(vec4i[0], vec4i[1], vec4i[2] - vec4i[0], vec4i[3] - vec4i[1]);
    });
    
    std::vector<Rect> non_rects (non.size());
    std::transform(non.begin(), non.end(), non_rects.begin(), [](const Vec4i vec4i) -> Rect{
        return Rect(vec4i[0], vec4i[1], vec4i[2] - vec4i[0], vec4i[3] - vec4i[1]);
    });
    for(Rect non_rect : non_rects){
        Mat src;
        try{
             src = color_image(non_rect);
        }
        catch(cv::Exception exc){
            continue;
        }
        cvtColor(src, src, COLOR_BGR2HSV);
        for(int row = 0; row < src.rows; row++){
            for(int col = 0; col < src.cols; col++){
                uchar Hue = src.at<Vec3b>(row, col)[0];
                //Set Red to Black
                if(Hue < 10 || (Hue > 175 && Hue < 180)){
                    src.at<Vec3b>(row, col)[2] = 0; //The absense of color
                }
                //Set Green to White
                if((Hue > 20 && Hue < 110) && !(src.at<Vec3b>(row, col)[2] < 40)) {
                    src.at<Vec3b>(row, col)[1] = 0; //The absense of color
                    src.at<Vec3b>(row, col)[2] = 255; //Presence of alot of light
                }
            }
        }
        cvtColor(src, src, COLOR_HSV2BGR);
        imshow( "Image", src );

        Mat* colsegIMG = Kmeans::kmeans(src);
        bool good_box = Kmeans::templMatch(*colsegIMG, imread("../templates/templ.jpg", CV_LOAD_IMAGE_COLOR));

        if(good_box) overlapped_rects.push_back(non_rect);
    }

    Mat image = imread(filename, CV_LOAD_IMAGE_COLOR);
    for(Rect final_box: overlapped_rects){
        rectangle(image, final_box, Scalar(255, 255,0));
    }
    imshow("Final Detection Results Using Color Segmentation", image);
    imwrite("detected.jpg", image);
    return overlapped_rects;
}

