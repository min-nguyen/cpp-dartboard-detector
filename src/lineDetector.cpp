#include "lineDetector.hpp"


// 


Point LineDetector::trueIntersect(Vec4i t_line1, Vec4i t_line2){
    
        float m1 = (t_line1[0] - t_line1[2] == 0) ?  -1000
            : ((float) t_line1[1] - (float) t_line1[3]) / ((float) t_line1[0] - (float) t_line1[2]) ;
        float m2 = (t_line2[0] - t_line2[2] == 0) ?  -1000
            : ((float) t_line2[1] - (float)t_line2[3]) / ((float)t_line2[0] - (float)t_line2[2]) + 0.0;
        float c1 = t_line1[1] - t_line1[0]*m1;
        float c2 = t_line2[1] - t_line2[0]*m2;
        
       


        float intersect_x = (c2 - c1) / (m1 - m2);
        float intersect_y = intersect_x*m1 + c1;
        Point intersection_xy(intersect_x, intersect_y);
        float maxX1 = max(t_line1[0], t_line1[2]);
        float minX1 = min(t_line1[0], t_line1[2]);
        float maxY1 = max(t_line1[1], t_line1[3]);
        float minY1 = min(t_line1[1], t_line1[3]);
    
        float maxX2 = max(t_line2[0], t_line2[2]);
        float minX2 = min(t_line2[0], t_line2[2]);
        float maxY2 = max(t_line2[1], t_line2[3]);
        float minY2 = min(t_line2[1], t_line2[3]);


        //HANDLE PARALLEL TO Y - AXIS
        if(m1 == -1000 || m2 == -1000){
            if(m1 == -1000){
                if(minX1 > minX2  && minX1 < maxX2){
                    float intersection_at_y = m2*minX1 + c2;
                    Point intersection(minX1, intersection_at_y);
                    return intersection;
                }
            }
            if(m2 == -1000){
                if(minX2 > minX1  && minX2 < maxX1){
                    float intersection_at_y = m1*minX2 + c1;
                    Point intersection(minX2, intersection_at_y);
                    return intersection;
                }
            }
        }


        // waitKey(0);
        if(   intersection_xy.x <= maxX1 && intersection_xy.x >= minX1 
           && intersection_xy.y <= maxY1 && intersection_xy.y >= minY1 
           && intersection_xy.x <= maxX2 && intersection_xy.x >=minX2 
           && intersection_xy.y <= maxY2 && intersection_xy.y >= minY2 ){
             
               return intersection_xy;
        }
       return Point(-1, -1);
}


bool LineDetector::isLineInBox(Vec4i box, Vec4i t_line){

    float maxX_line = max(t_line[0], t_line[2]);
    float minX_line = min(t_line[0], t_line[2]);
    float maxY_line = max(t_line[1], t_line[3]);
    float minY_line = min(t_line[1], t_line[3]);

    float maxX_box = box[2]; 
    float minX_box = box[0];
    float maxY_box = box[3];
    float minY_box = box[1];

    if(   minX_box < minX_line && maxX_box > maxX_line 
       && minY_box < minY_line && maxY_box > maxY_line){
           return true;
    }

    float m1 = (t_line[0] - t_line[2] == 0) ?  1
             : ((float) t_line[1] - (float) t_line[3]) / ((float) t_line[0] - (float) t_line[2]) ;

    float c1 = t_line[1] - t_line[0]*m1;
    float y_cutLeft     = m1 * minX_box + c1;
    float y_cutRight    = m1 * maxX_box + c1;
    float x_cutTop      = (maxY_box - c1)/m1;
    float x_cutBottom   = (minY_box - c1)/m1;
    if(    ((y_cutLeft < maxY_box &&  y_cutLeft > minY_box) && (y_cutLeft < maxY_line &&  y_cutLeft > minY_line))  
        || ((y_cutRight < maxY_box &&  y_cutRight > minY_box) && (y_cutRight < maxY_line &&  y_cutRight > minY_line))  
        || ((x_cutTop < maxX_box &&  x_cutTop > minX_box) && (x_cutTop < maxX_line &&  x_cutTop > minX_line)) 
        || ((x_cutBottom < maxX_box &&  x_cutBottom > minX_box) && (x_cutTop < maxX_line &&  x_cutTop > minX_line))  ){
        return true;
    }

    return false;
}

//Returns vector of intersection coordinates and associated lines
//Vec3i         -> intersections :: [(central x, central y, num intersections)]
//Vector<Vec4i> -> lines         :: [(x1, y1, x2, y2)]
vector<pair<Vec3i, vector<Vec4i>>> LineDetector::findLineIntersections(Vec4i boundingBox, Mat img, vector<Vec4i> lines){
    vector<pair<Vec3i, vector<Vec4i>>> intersectionsAndLines;
    Mat linesMat = img.clone();

    for(int k = 0; k < lines.size(); k++){
        Vec4i l = lines[k];
            for(int ot = k; ot < lines.size(); ot++){
                if(ot != k){
                    Vec4i otherLine = lines[ot];
                    Point intersection = trueIntersect(l, otherLine);
                    int intersect_x = intersection.x;
                    int intersect_y = intersection.y;
                    
                    if(intersect_x != -1){
                        if(intersect_x < boundingBox[2] && intersect_x > boundingBox[0] 
                            && intersect_y < boundingBox[3] && intersect_x > boundingBox[1]){
                            bool found = false;
                            for(auto it = intersectionsAndLines.begin(); it != intersectionsAndLines.end(); it++){
                                int it_x = it->first[0];
                                int it_y = it->first[1];
                                float L2dist = sqrt( pow(intersect_x - it_x, 2) + pow(intersect_y - it_y, 2));
                                if(L2dist < 20){
                                    it->first[2] = it->first[2] + 1;
                                    it->second.push_back(otherLine);
                                    it->second.push_back(l);
                                    found = true;
                                }
                            }
                            if(!found){
                                Vec3i newIntersection(intersect_x, intersect_y, 1);
                                vector<Vec4i> newLines;
                                newLines.push_back(ot);
                                newLines.push_back(l);
                                pair<Vec3i, vector<Vec4i>> newIntersectionAndLine = make_pair(newIntersection, newLines);
                                intersectionsAndLines.push_back(newIntersectionAndLine);

                            }
                        }
                    }
                }
            }
    }
    
    return intersectionsAndLines;
}

Mat LineDetector::houghLineSpace(Mat *grad_mag, Mat* grad_dir){
    uchar max = *std::max_element(grad_mag->datastart, grad_mag->dataend);
    uchar threshold_S = (95/100) * max;
    uchar threshold_H = 130;
    int maxX = 360; 
    int maxY = 720;
    //imshow("", *grad_dir);
    
    Mat houghspace = Mat::zeros(2000, 361, CV_8UC1);
    //Read image into hough space
    for(int y = 0; y < grad_mag->rows; y++){
        for(int x = 0; x < grad_mag->cols; x++){
            if(grad_mag->at<uchar>(y, x) > threshold_S && grad_dir->at<uchar>(y,x) > threshold_H){
                for(int angle = 0; angle < 360; angle+=2){
                    int r = x * std::cos(angle*(M_PI / 180)) + y * std::sin(angle*(M_PI / 180)) + 500;
                        houghspace.at<uchar>(abs(r), angle) += 1;
                }
            }
        }
    }
    //imshow("Hough Line Space", houghspace);
    return houghspace;
}

bool LineDetector::isEqual(const Vec4i& _l1, const Vec4i& _l2)
{
    Vec4i l1(_l1), l2(_l2);

    float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));

    float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);

    if (fabs(product / (length1 * length2)) < cos(CV_PI / 30))
        return false;

    float mx1 = (l1[0] + l1[2]) * 0.5f;
    float mx2 = (l2[0] + l2[2]) * 0.5f;

    float my1 = (l1[1] + l1[3]) * 0.5f;
    float my2 = (l2[1] + l2[3]) * 0.5f;
    float dist = sqrtf((mx1 - mx2)*(mx1 - mx2) + (my1 - my2)*(my1 - my2));

    if (dist > std::max(length1, length2) * 0.5f)
        return false;

    return true;
}

vector<pair<Vec3i, vector<Vec4i>>> LineDetector::houghLineIntersect(Vec4i boundingBox, Mat src, Mat *grad_mag, Mat* grad_dir){

    int maxX = 360; 
    int maxY = 720;

    

    vector<pair<Vec3i, vector<Vec4i>>> final_intersectsAndLines; 
    Mat mat_finalLines = src.clone();
    //** GET LINES FROM HOUGH SPACE **//
    // GaussianBlur(src, mat_thr, Size(5,5), 0, 0);
    for(int canny = 50; canny < 80; canny+=10){
    for(int huff = 70; huff < 110; huff+=10){
            
            Mat mat_canny; 
            Mat mat_thr; 
            Mat mat_originalLines = src.clone();
            Mat mat_mergedLines = src.clone();
            Mat mat_removedLines = src.clone();
            Mat mat_elongatedLines = src.clone();
            vector<Vec4i> lines;
            Canny(src, mat_canny, canny, 200, 3, true);
            //imshow("Canny", mat_canny);
            HoughLinesP(mat_canny, lines, 2, CV_PI/180, huff, 40, 20);
            for( size_t i = 0; i < lines.size(); i++ )
            {
            Vec4i l = lines[i];
            line( mat_originalLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,0), 2, CV_8UC1);
            }
            imshow("Original Lines", mat_originalLines);
        
            //** MERGE SIMILAR LINES **//
            for( size_t i = 0; i < lines.size(); i++ )
            {
                for( size_t k = 0; k < lines.size(); k++ )
                {
                    if(k!=i){
                        if(isEqual(lines[i], lines[k]))
                            lines.erase(lines.begin() + k);
                    }
                }
            }
            for( size_t i = 0; i < lines.size(); i++ )
            {
            Vec4i l = lines[i];
            line( mat_mergedLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,0), 2, CV_8UC1);
            }
            imshow("Hough Lines Merged", mat_mergedLines);

            //** REMOVE LINES OUTSIDE BOUNDING BOX**//
            vector<Vec4i> filteredLines;
            for(int i = 0; i < lines.size(); i++){
                Vec4i t_line = lines[i];
                if(isLineInBox(boundingBox, t_line)){
                    filteredLines.push_back(t_line);
                }
            }
            for( size_t i = 0; i < filteredLines.size(); i++ )
            {
            Vec4i l = filteredLines[i];
            line(mat_removedLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,0), 2, CV_8UC1);
            }
            line(mat_removedLines, Point(boundingBox[0], boundingBox[1]), Point(boundingBox[2], boundingBox[1]), Scalar(0,255,0), 2, CV_8UC1);
            line(mat_removedLines, Point(boundingBox[0], boundingBox[1]), Point(boundingBox[0], boundingBox[3]), Scalar(0,255,0), 2, CV_8UC1);
            line(mat_removedLines, Point(boundingBox[2], boundingBox[3]), Point(boundingBox[2], boundingBox[1]), Scalar(0,255,0), 2, CV_8UC1);
            line(mat_removedLines, Point(boundingBox[2], boundingBox[3]), Point(boundingBox[0], boundingBox[3]), Scalar(0,255,0), 2, CV_8UC1);
            
            imshow("Removed Lines", mat_removedLines);

            //** SCALE LINES UP **//
            int scaleFactor = 10;
            for( size_t i = 0; i < filteredLines.size(); i++ )
            {
            Vec4i l = filteredLines[i];
            Point p1, p2;
            double lenAB = sqrt( (l[0] - l[2])*(l[0] - l[2]) + (l[1] - l[3])*(l[1] - l[3]) );
            p1.x = l[0] + (l[0] - l[2]) / lenAB * scaleFactor;
            p1.y = l[1] + (l[1] - l[3]) / lenAB * scaleFactor;
            p2.x = l[2] - (l[0] - l[2]) / lenAB * scaleFactor; 
            p2.y = l[3] - (l[1] - l[3]) / lenAB * scaleFactor; 
            filteredLines[i] = Vec4i(p1.x, p1.y, p2.x, p2.y);
            line( mat_elongatedLines, Point(filteredLines[i][0], filteredLines[i][1]), Point(filteredLines[i][2], filteredLines[i][3]), Scalar(255,255,0), 2, CV_8UC1);
            }
            imshow("Hough Lines Elongated", mat_elongatedLines);

            //** FIND COMMON INTERSECTIONS **//
            vector<pair<Vec3i, vector<Vec4i>>> intersectionsAndLines = findLineIntersections(boundingBox, src, filteredLines);

            int minIntersectionThreshold = 2; 
            for(int j = 0; j < intersectionsAndLines.size(); j++){
                Vec3i xyn = intersectionsAndLines[j].first;
                vector<Vec4i> intersectingLines = intersectionsAndLines[j].second;
                
                if(xyn[2] > minIntersectionThreshold){
                    final_intersectsAndLines.push_back(intersectionsAndLines[j]);
                    mat_finalLines.at<char>(xyn[1] + 1, xyn[0]) = 255; 
                    
                    // mat_finalLines.at<char>(xyn[1], xyn[0]) = 255;   
                    // mat_finalLines.at<char>(xyn[1] - 1, xyn[0]) = 255; 
                    // mat_finalLines.at<char>(xyn[1], xyn[0] + 1) = 255; 
                    // mat_finalLines.at<char>(xyn[1], xyn[0] - 1) = 255; 

                    for(int i = 0; i < intersectingLines.size(); i++){
                        Vec4i t_line = intersectingLines[i];
                        line( mat_finalLines, Point(t_line[0], t_line[1]), Point(t_line[2], t_line[3]), Scalar(0,255,0), 1, CV_8UC1);
                    }
                    line(mat_finalLines, Point(xyn[0], xyn[1] - 3), Point(xyn[0], xyn[1] + 3), Scalar(255,0,0), 1, CV_8UC1);
                    line(mat_finalLines, Point(xyn[0] - 3, xyn[1]), Point(xyn[0] + 3, xyn[1]), Scalar(255,0,0), 1, CV_8UC1);
                }
            }
       
    }
    }
    imshow("Final Hough Intersects", mat_finalLines);
    return final_intersectsAndLines;
}

vector<Vec4i> LineDetector::circlesToBoxes(vector<Circle> circles){
    vector<Vec4i> boundingBoxes;
    for(int i = 0; i < circles.size(); i++){
        Point cent = (circles)[i].p;
        float rad = (circles)[i].radius;
        Vec4i boundingBox(cent.x-rad, cent.y-rad, cent.x+rad, cent.y+rad);
        boundingBoxes.push_back(boundingBox);
    }
    return boundingBoxes;
}