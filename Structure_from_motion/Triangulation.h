
/*
 * Triangulation.h
 *
 *  Created on: 27/set/2016
 *      Author: trev
 */

#ifndef SRC_TRIANGULATION_H_
#define SRC_TRIANGULATION_H_

#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/flann/flann.hpp>
#include "Common.h"

#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <sstream>

using namespace std;
using namespace cv;

class Triangulation{

public:

	Mat_<double> linearLSTriangulation(Point3d p, Matx34d P, Point3d p1, Matx34d P1);

    double triangulatePoints(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2,
    		                 Mat K, Mat Kinv, Matx34d P, Matx34d P1, vector<Point3d>& pointcloud,
							 vector<CloudPoint>& pcloud, vector<double>& reproj_err);

private:



};



#endif /* SRC_TRIANGULATION_H_ */
