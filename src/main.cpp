#include "detector.hpp"
#include "kmeans.hpp"



////////////
// --arg1 = image_path   --arg2 = detector type <- [line | colseg]
////////////

int main(int argc, char** argv){
    if(argc != 3) {
        printf("Usage: ./final_boss <image> <detector> \n");
        printf("    where <detector> = line | colseg\n");
        return EXIT_FAILURE;
    }
    if(!imread(argv[1], CV_LOAD_IMAGE_COLOR).data){
        printf("Invalid Image File\n");
        return EXIT_FAILURE;
    }
    bool line_detector = false;
    bool colseg_detector = false;
    if(!( (line_detector = (strcmp(argv[2], "line") == 0)) || (colseg_detector = (strcmp(argv[2], "colseg") == 0)) )){
        printf("Invalid Detector\n");
        return EXIT_FAILURE;
    }
    if(line_detector) Detector::detect_VH_Intersections(argv[1]);
    if(colseg_detector) Kmeans::getFinalResult(argv[1]);

    waitKey(0);
    return EXIT_SUCCESS;
}