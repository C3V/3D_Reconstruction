/*
 * FindCameraMatrices.h
 *
 *  Created on: 26/set/2016
 *      Author: trev
 */

#ifndef SRC_FINDCAMERAMATRICES_H_
#define SRC_FINDCAMERAMATRICES_H_

#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

class FindCameraMatrices{
public:

	//bool findCameraMatrices();

	void KeyPointsToPoints(vector<KeyPoint> keypoints, vector<Point2f>* points);

	Mat getFundamentalMat(const vector<KeyPoint>& keypoints_left,
			const vector<KeyPoint>& keypoints_right, vector<KeyPoint>& keypoints_left_good,
			vector<KeyPoint>& keypoints_right_good, vector<DMatch>& matches);

	bool CheckCoherentRotation(Mat_<double>& R);

	void alignMatches(vector<KeyPoint> keypoints_left,
			vector<KeyPoint> keypoints_right, vector<DMatch> matches,
			vector<KeyPoint>* pts1, vector<KeyPoint>* pts2);

	void SVDofE(Mat_<double>& E, Mat& svd_u, Mat& svd_vt, Mat& svd_w);

	bool getRTFromE(Mat_<double>& E,Mat_<double>& R1, Mat_<double>& R2,
			Mat_<double>& t1,Mat_<double>& t2);

	bool findCameraMatrices(Mat K, Mat Kinv, vector<KeyPoint>& keypoints_left,
			vector<KeyPoint>& keypoints_right, vector<KeyPoint>& keypoints_left_good,
			vector<KeyPoint>& keypoints_right_good, Matx34d& P, Matx34d& P1, vector<DMatch>& matches,
			vector<Point3d>& outCloud);

private:

	Mat F;

};



#endif /* SRC_FINDCAMERAMATRICES_H_ */
