/*
 * CameraMatricesFinder.h
 *
 *  Created on: 17/set/2016
 *      Author: trev
 */

#ifndef SRC_CAMERAMATRICESFINDER_H_
#define SRC_CAMERAMATRICESFINDER_H_

#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

class CameraMatricesFinder {

public:

	CameraMatricesFinder();

	void convertToPoint2f(vector<DMatch> matches, vector<KeyPoint> keypoints_left, vector<KeyPoint> keypoints_right);

	Mat findMatrixE(vector<DMatch> matches, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, Mat K);

private:

	//inputs to the method
	vector<Point2f> imgpts_left, imgpts_right;

	vector<KeyPoint> keypoints_left, keypoints_right;

	vector<DMatch> matches;

	vector<uchar> vStatus;

	//fundamental matrix
	Mat F;

	//essential matrix
	//Mat_ it's like Mat but it does not have any extra data fields (see documentation)
	Mat_<double> E;
};



#endif /* SRC_CAMERAMATRICESFINDER_H_ */
