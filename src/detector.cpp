#include "detector.hpp"

std::pair<std::vector<Vec4i>, std::vector<Vec4i>> Detector::detect_VH(String FILE){
	Mat image = imread(FILE, CV_LOAD_IMAGE_GRAYSCALE);
	Mat frame = imread(FILE, CV_LOAD_IMAGE_COLOR);
	sobel_output* sobel_output = Sobel::sobel(&image);
	Vec4i dart4rect(169, 85, 342, 262);


	CircleDetector::displayImage("Gradient Magnitude", *sobel_output->gradient_mag);
	vector<Vec4i> violaBoxes = violaRects(frame);
	vector<Circle> circles = CircleDetector::houghCircles(sobel_output->gradient_mag, sobel_output->gradient_dir);
	vector<Vec4i> boundingBoxes = LineDetector::circlesToBoxes(circles);
	Mat f = LineDetector::houghLineSpace(sobel_output->gradient_mag, sobel_output->gradient_dir);
	
	//////////////////////////////////////////////////////
	//** DRAW HOUGH BOXES AND VIOLA BOXES ONTO IMAGE
	//////////////////////////////////////////////////////
	Mat resultingBoxes = imread(FILE, CV_LOAD_IMAGE_COLOR); 
	for(int i = 0; i < violaBoxes.size(); i++){
		Vec4i t_Box = violaBoxes[i];
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(0,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(0,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(0,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(0,255,0), 2, CV_8UC3);
	}
	for(int i = 0; i < boundingBoxes.size(); i++){
		Vec4i t_Box = boundingBoxes[i];
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
	}
	imshow("Viola And Hough Boxes", resultingBoxes);

	//////////////////////////////////////////////////////
	//** UNION VIOLA AND HOUGH BOXES, FILTER REDUNDANT OVERLAPPING RECTS
	//////////////////////////////////////////////////////

	std::pair<std::vector<Vec4i>, std::vector<Vec4i>> rect_pair = rectUnion(violaBoxes, boundingBoxes);
	return rect_pair;
}
std::pair<std::vector<Vec4i>, std::vector<Vec4i>> Detector::detect_VH_Intersections(String FILE){
	Mat image = imread(FILE, CV_LOAD_IMAGE_GRAYSCALE);
	Mat frame = imread(FILE, CV_LOAD_IMAGE_COLOR);
	sobel_output* sobel_output = Sobel::sobel(&image);
	Vec4i dart4rect(169, 85, 342, 262);


	CircleDetector::displayImage("Gradient Magnitude", *sobel_output->gradient_mag);

	vector<Vec4i> violaBoxes = violaRects(frame);
	vector<Circle> circles = CircleDetector::houghCircles(sobel_output->gradient_mag, sobel_output->gradient_dir);
	vector<Vec4i> boundingBoxes = LineDetector::circlesToBoxes(circles);
	Mat f = LineDetector::houghLineSpace(sobel_output->gradient_mag, sobel_output->gradient_dir);
	
	//////////////////////////////////////////////////////
	//** DRAW HOUGH BOXES AND VIOLA BOXES ONTO IMAGE
	//////////////////////////////////////////////////////
	Mat resultingBoxes = imread(FILE, CV_LOAD_IMAGE_COLOR); 
	for(int i = 0; i < violaBoxes.size(); i++){
		Vec4i t_Box = violaBoxes[i];
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(0,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(0,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(0,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(0,255,0), 2, CV_8UC3);
	}
	for(int i = 0; i < boundingBoxes.size(); i++){
		Vec4i t_Box = boundingBoxes[i];
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(resultingBoxes, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
	}
	imshow("Viola And Hough Boxes", resultingBoxes);

	//////////////////////////////////////////////////////
	//** UNION VIOLA AND HOUGH BOXES, FILTER REDUNDANT OVERLAPPING RECTS
	//////////////////////////////////////////////////////

	std::pair<std::vector<Vec4i>, std::vector<Vec4i>> rect_pair = rectUnion(violaBoxes, boundingBoxes);
	std::vector<Vec4i> overlapped = rect_pair.first;
	std::vector<Vec4i> nonoverlapped = rect_pair.second;
	printf("OVERLAPPED SIZE %d NON %d\n", overlapped.size(), nonoverlapped.size());
	Mat unioned_mat = imread(FILE, CV_LOAD_IMAGE_COLOR); 
	for(int i = 0; i < overlapped.size(); i++){
		Vec4i t_Box = overlapped[i];
		line(unioned_mat, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(unioned_mat, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		line(unioned_mat, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(unioned_mat, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
	}
	for(int i = 0; i < nonoverlapped.size(); i++){
		Vec4i t_Box = nonoverlapped[i];
		line(unioned_mat, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(unioned_mat, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		line(unioned_mat, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(unioned_mat, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
	}
	imshow("Unioned Viola And Hough Boxes", unioned_mat);

	//////////////////////////////////////////////////////
	//** LINE DETECTOR - DETECT IF INTERSECTIONS OCCUR NEAR CENTRE
	/////////////////////////////////////////////////////

	std::vector<Vec4i> successful_detections;
	
	// for(int i = 0; i < overlapped.size(); i++){
	// 	successful_detections.push_back(overlapped[i]);
	// }


	//////////////////////////////////////////////////////
	//** OVERLAPPED RECTS - COMMENTED OUT AS ALL OVERLAPPED RECTS HAVE BEEN CONSIDERED AS TRUE
	/////////////////////////////////////////////////////
	Mat line_mat  = imread(FILE, CV_LOAD_IMAGE_COLOR); 
	for(int i = 0; i < overlapped.size(); i++){
		
		vector<pair<Vec3i, vector<Vec4i>>> successful_lines;
		Vec4i t_Box = overlapped[i];
		line(line_mat, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		vector<pair<Vec3i, vector<Vec4i>>> intersectsAndLines = LineDetector::houghLineIntersect(overlapped[i], image, sobel_output->gradient_mag, sobel_output->gradient_dir);
		Vec2i t_median;
		t_median[0] = (overlapped[i][2] + overlapped[i][0])/2;
		t_median[1] = (overlapped[i][1] + overlapped[i][3])/2;

		int successfulIntersections = 0;
		for(int j = 0; j < intersectsAndLines.size(); j++){
			pair<Vec3i, vector<Vec4i>> intersections = intersectsAndLines[j];
			int medianx = intersections.first[0];
			int mediany = intersections.first[1];
			int medianDiffx = abs(t_median[0] - medianx);
			int medianDiffy = abs(t_median[1] - mediany);
			float absDiff = sqrt(medianDiffx*medianDiffx + medianDiffy*medianDiffy);
			if(absDiff < 25){
				successful_lines.push_back(intersectsAndLines[j]);
				successfulIntersections++;
				successful_detections.push_back(overlapped[i]);
				
			}
		}

		
		pair<Vec3i, vector<Vec4i>> max_inter = make_pair(Vec3i(0, 0, 0), vector<Vec4i>());
		for(int j = 0; j < successful_lines.size(); j++){
			max_inter = successful_lines[j].first[2]  > max_inter.first[2]  ? successful_lines[j] : max_inter;
		}
		if(max_inter.first[2] > 2){
			vector<Vec4i> lines = max_inter.second;
			for(int jj = 0; jj < lines.size(); jj++){
				Vec4i t_line = lines[jj];
				
				line(line_mat, Point(t_line[0], t_line[1]), Point(t_line[2], t_line[3]), Scalar(255,0,0), 2, CV_8UC1);
			}
		}
	}
	imshow("Line Intersection Results For Overlapped Rects", line_mat);

	//////////////////////////////////////////////////////
	//** NON-OVERLAPPED RECTS
	//////////////////////////////////////////////////////
	Mat line_mat2 = imread(FILE, CV_LOAD_IMAGE_COLOR); 

	for(int i = 0; i < nonoverlapped.size(); i++){
		Vec4i t_Box = nonoverlapped[i];

		vector<pair<Vec3i, vector<Vec4i>>> successful_lines;
		line(line_mat2, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat2, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat2, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat2, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);

		vector<pair<Vec3i, vector<Vec4i>>> intersectsAndLines = LineDetector::houghLineIntersect(nonoverlapped[i], image, sobel_output->gradient_mag, sobel_output->gradient_dir);
		Vec2i t_median;
		t_median[0] = (nonoverlapped[i][2] + nonoverlapped[i][0])/2;
		t_median[1] = (nonoverlapped[i][1] + nonoverlapped[i][3])/2;
		
		int successfulIntersections = 0;
		for(int j = 0; j < intersectsAndLines.size(); j++){
			pair<Vec3i, vector<Vec4i>> intersections = intersectsAndLines[j];
			int medianx = intersections.first[0];
			int mediany = intersections.first[1];
			int medianDiffx = abs(t_median[0] - medianx);
			int medianDiffy = abs(t_median[1] - mediany);
			float absDiff = sqrt(medianDiffx*medianDiffx + medianDiffy*medianDiffy);
			if(absDiff < 25){
				successful_lines.push_back(intersectsAndLines[j]);
				successfulIntersections++;
			}
		}

		
		pair<Vec3i, vector<Vec4i>> max_inter = make_pair(Vec3i(0, 0, 0), vector<Vec4i>());
		for(int j = 0; j < successful_lines.size(); j++){
			max_inter = successful_lines[j].first[2]  > max_inter.first[2]  ? successful_lines[j] : max_inter;
			
		}
		if(max_inter.first[2] > 3){
			vector<Vec4i> lines = max_inter.second;
			for(int jj = 0; jj < lines.size(); jj++){
				Vec4i t_line = lines[jj];
				
				line(line_mat2, Point(t_line[0], t_line[1]), Point(t_line[2], t_line[3]), Scalar(255,0,0), 2, CV_8UC1);
			}
			successful_detections.push_back(nonoverlapped[i]);
		}
	}

	imshow("Line Intersection Results For Non-Overlapped Rects" + FILE, line_mat2);

	//////////////////////////////////////////////////////
	//** FINAL RECTS
	//////////////////////////////////////////////////////
	Mat line_mat3 = imread(FILE, CV_LOAD_IMAGE_COLOR); 

	for(int i = 0; i < successful_detections.size(); i++){
		Vec4i t_Box = successful_detections[i];
		line(line_mat3, Point(t_Box[0], t_Box[1]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat3, Point(t_Box[0], t_Box[1]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat3, Point(t_Box[2], t_Box[3]), Point(t_Box[2], t_Box[1]), Scalar(255,255,0), 2, CV_8UC3);
		line(line_mat3, Point(t_Box[2], t_Box[3]), Point(t_Box[0], t_Box[3]), Scalar(255,255,0), 2, CV_8UC3);
	}
	
	imshow("Final Detection Results Using Line Intersection" + FILE, line_mat3);
	imwrite("detected.jpg", line_mat3);
	return rect_pair;
}


std::pair<std::vector<Vec4i>, std::vector<Vec4i>>  Detector::rectUnion(std::vector<Vec4i> viola, std::vector<Vec4i> hough){
	float size = viola.size(); 

	std::unordered_map<int, bool> hough_taken;
	std::unordered_map<int, bool> viola_taken;
	std::vector<float> result;
	std::vector<Vec4i> overlappedRects;
	std::vector<Vec4i> nonoverlappedRects;
	for(int i = 0; i <  hough.size(); i++){
		for(int j = 0; j <  hough.size(); j++){
			if(j != i){
				float overlap = rectOverlap(hough[j], hough[i]);
				float medianDiff = rectMedianDifference(hough[j], hough[i]);
				if((overlap > 0.5 || medianDiff < 50)){
					hough.erase(hough.begin()+i);
					break;
				}
			}
		}
	}
	for(int i = 0; i <  viola.size(); i++){
		for(int j = 0; j <  viola.size(); j++){
			if(j != i){
				float overlap = rectOverlap(viola[j], viola[i]);
				float medianDiff = rectMedianDifference(viola[j], viola[i]);
				if((overlap > 0.5 || medianDiff < 50)){
					viola.erase(viola.begin()+i);
					break;
				}
			}
		}
	}
	for(int i = 0; i < viola.size(); i++){	
		bool viola_overlap = false;
		for(int j = 0; j < hough.size(); j++){
			float overlap = rectOverlap(viola[i], hough[j]);
			float medianDiff = rectMedianDifference(viola[i], hough[j]);
			if((overlap > 0.5 || medianDiff < 50)){
				viola_overlap = true;
				if(!(hough_taken[j])){
					hough_taken[j] = true;
					overlappedRects.push_back(hough[j]);
				}
			}
		}
		if(!viola_overlap){
			nonoverlappedRects.push_back(viola[i]);
			viola_taken[i] = true;
		}
	}
	

	for(int i = 0; i <  hough.size(); i++){	
		bool hough_overlap = false;
		if(!(hough_taken[i])){
			for(int j = 0; j <  viola.size(); j++){
				float overlap = rectOverlap(viola[j], hough[i]);
				float medianDiff = rectMedianDifference(viola[j], hough[i]);
				if((overlap > 0.5 || medianDiff < 50)){
					hough_overlap = true;
				}
			}
			if(!hough_overlap){
				nonoverlappedRects.push_back(hough[i]);
			}
		}
	}
	std::pair<std::vector<Vec4i>, std::vector<Vec4i>> res = std::make_pair(overlappedRects, nonoverlappedRects);
	return res;
}

float Detector::rectMedianDifference(Vec4i viola, Vec4i hough){
	int xmin_v, xmax_v, ymin_v, ymax_v, xmin_a, xmax_a, ymin_a, ymax_a;
	xmin_v = viola[0];
	xmax_v = viola[2];
	ymin_v = viola[1];
	ymax_v = viola[3];
	xmin_a = hough[0];
	xmax_a = hough[2];
	ymin_a = hough[1];
	ymax_a = hough[3];

	Vec2i medianHough;
	medianHough[0] = (hough[2] + hough[0])/2;
	medianHough[1] = (hough[1] + hough[3])/2;
	Vec2i medianViola;
	medianViola[0] = (viola[2] + viola[0])/2;
	medianViola[1] = (viola[1] + viola[3])/2;

	int medianDiffx = abs(medianHough[0] - medianViola[1]);

	int medianDiffy = abs(medianHough[0] - medianViola[1]);
	
	float absDiff = sqrt(medianDiffx*medianDiffx + medianDiffy*medianDiffy);
	return absDiff;
}

float Detector::rectOverlap(Vec4i violaa, Vec4i hough){
	int xmin_v, xmax_v, ymin_v, ymax_v, xmin_a, xmax_a, ymin_a, ymax_a;
 	xmin_v = violaa[0];
	xmax_v = violaa[2];
	ymin_v = violaa[1];
	ymax_v = violaa[3];
	xmin_a = hough[0];
	xmax_a = hough[2];
	ymin_a = hough[1];
	ymax_a = hough[3];


    float x_intersection = min(xmax_a, xmax_v) - max(xmin_a, xmin_v) + 1;
	float y_intersection = min(ymax_a, ymax_v) - max(ymin_a, ymin_v) + 1;
 	float overlap =( x_intersection <= 0 || y_intersection <=0) ? 0 : x_intersection*y_intersection;
	float percentageoverlap = max(overlap/((xmax_a-xmin_a)*(ymax_a-ymin_a)), overlap/((xmax_v-xmin_v)*(ymax_v-ymin_v)));

	return percentageoverlap;
}

std::vector<Vec4i> Detector::violaRects(Mat frame)
{
    std::vector<Rect> faces;
    std::vector<Vec4i> violaBoxes;
	Mat frame_gray;
	// 1. Prepare Image by turning it into Grayscale and normalising lighting
	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );

	// 2. Perform Viola-Jones Object Detection 
	String cascade_name = "../cascades/dart_cascade.xml";
	CascadeClassifier cascade;
	if( !cascade.load( cascade_name ) ){ 
		printf("--(!)Error loading cascade\n"); 
		abort();
    };
	cascade.detectMultiScale( frame_gray, 
		faces, 1.1, 1, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50), Size(500,500) );

    // 3. Print number of Faces found
	std::cout << faces.size() << std::endl;

    // 4. Draw box around faces found
	for( int i = 0; i < faces.size(); i++ ){
        int x = faces[i].x;
        int y = faces[i].y;
        int x2 = faces[i].x + faces[i].width;
        int y2 = faces[i].y + faces[i].height;
		violaBoxes.push_back(Vec4i(x, y, x2, y2));
	}
	return violaBoxes;
}