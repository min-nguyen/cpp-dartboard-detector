/////////////////////////////////////////////////////////////////////////////
//
// COMS30121 - face.cpp
//
/////////////////////////////////////////////////////////////////////////////

// header inclusion
#include "parseAnnotations.hpp"

#define NUM_IMAGES 15
#define MAX_LINE_SIZE 300

void ParseAnnotations::printError(char* errorMessage_strliteral){
	cerr << errorMessage_strliteral << endl;
}

char* ParseAnnotations::shiftToNPastNextSpace(char* strp, int n){
	for(int i = 0; i < n; i++){
		strp = std::strchr(strp, (int)' '); 
		strp++;
	}
	return strp;
}

std::vector<std::vector<Rect>> ParseAnnotations::parse(const char** argv)
{
	std::vector<std::vector<cv::Rect> > imageAnnotations;
	cout << "\n" << argv[0] << "\n" << argv[1] << "\n" << argv[2];
	std::string annotationsPath = argv[2];
	FILE* annotationsIFP = fopen(annotationsPath.c_str(), "r");
	if(annotationsIFP == NULL) { 
		cout << "file could not be read" << endl;
		return imageAnnotations; 
	}

	char *buffer = new char[MAX_LINE_SIZE];
	int file_num, annotations_in_image;
	int buffer_positions_indicator = 0;
	while (fgets(buffer, MAX_LINE_SIZE, annotationsIFP)) {
		char* strp = buffer;
		if(sscanf(strp, "/Users/Charana/CS Documents/Programming/C++/Computer-Vision/CW1/images/dart%d.jpg %d", &file_num, &annotations_in_image) == 2){
			// Shift parameter to be altered
			strp = shiftToNPastNextSpace(strp, 3);
			printf("File Num: %d\n", file_num);
			printf("Annotations In Image: %d\n", annotations_in_image);
		}
		else {
			printError("First Matching failed");
			return imageAnnotations;
		}
		int x , y, width, height;
		std::vector<cv::Rect> bounding_boxes;
		for(int i = 0; i < annotations_in_image; i++){
			if(sscanf(strp, "%d %d %d %d", &x, &y, &width, &height) == 4){
				bounding_boxes.push_back(cv::Rect(x, y, width, height));
				printf("x, y, width, height: = %d, %d, %d, %d\n", x, y, width, height);
				strp = shiftToNPastNextSpace(strp, 4);
			}
			else { 
				printf("%d \n", sscanf(strp, "%d %d %d %d", &x, &y, &width, &height));
				printError("Second Matching failed");
				return imageAnnotations;
			}
		}
		imageAnnotations.push_back(bounding_boxes);
	}
	return imageAnnotations;
}
