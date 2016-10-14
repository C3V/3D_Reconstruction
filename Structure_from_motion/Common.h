/*
 * Common.h
 *
 *  Created on: 10/ott/2016
 *      Author: trev
 */

#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_

using namespace cv;
using namespace std;

struct CloudPoint {  //stores 3D point and 2D image point from where it was obtained
	Point3d pt;
	vector<int> imgpt_for_img;  //imgpt_for_img[0] is relative to first image 2D point index
	double reprojection_error;
};


#endif /* SRC_COMMON_H_ */
