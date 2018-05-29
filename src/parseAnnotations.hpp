#ifndef PARSEANNOTATIONS_H
#define PARSEANNOTATIONS_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"

using namespace std;
using namespace cv;

class ParseAnnotations{
	public:
		static std::vector<std::vector<Rect>> parse(const char** argv);
	private:
		static char* shiftToNPastNextSpace(char* strp, int n);
		static void printError(char* errorMessage_strliteral);
};



#endif
