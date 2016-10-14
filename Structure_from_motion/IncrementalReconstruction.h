/*
 * IncrementalReconstruction.h
 *
 *  Created on: 08/ott/2016
 *      Author: trev
 */

#ifndef SRC_INCREMENTALRECONSTRUCTION_H_
#define SRC_INCREMENTALRECONSTRUCTION_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "OpticalFlowMatcher.h"
#include "FindCameraMatrices.h"
#include "Common.h"
#include "Triangulation.h"

using namespace std;
using namespace cv;


class IncrementalReconstruction{

public:

	void constructMatchesMatrix(vector<Mat> imgs, OpticalFlowMatcher matcher,
			                    map< pair<int,int>,vector<DMatch> >& matches_matrix,
								vector<vector< KeyPoint> >& keypoints_vector,
								int& max_index1, int& max_index2);

	void pruneMatchesBasedOnF(vector<Mat> imgs, vector<vector<KeyPoint> > keypoints_vector,
			map< pair<int,int>,vector<DMatch> > matches_matrix, bool dense, FindCameraMatrices f);

	bool getBaseLineTriangulation(vector<Mat> imgs, map< pair<int,int>,vector<DMatch> > matches_matrix,
			vector<vector<KeyPoint> > keypoints_vector, int max_index1, int max_index2,
			FindCameraMatrices f, Mat K, bool dense);

	void find3D2DCorrespondences(vector<Point3f>& ppcloud, vector<Point2f>& imgPoints,
			                     int working_view, map< pair<int,int>,vector<DMatch> > matches_matrix,
								 vector<vector< KeyPoint> > keypoints_vector);

	bool findPoseEstimation(int working_view, Mat_<double>& rvec, Mat_<double>& t, Mat_<double>& R,
                            vector<Point3f> ppcloud, vector<Point2f> imgPoints, Mat K,
							Mat distortion_coeff);

	bool triangulatePointsBetweenViews(vector<Mat> imgs, int working_view, int older_view,
                                vector<CloudPoint>& new_triangulated, vector<int>& add_to_cloud,
								vector<vector<KeyPoint> > keypoints_vector,
								map< pair<int,int>,vector<DMatch> > matches_matrix, Mat K,
								Triangulation t);

	void getKeyPointsFromMatches(vector<KeyPoint>& imgpts1,
			vector<KeyPoint>& imgpts2, map< pair<int,int>,vector<DMatch> > matches_matrix,
			int older_view, int working_view,vector<vector<KeyPoint> > keypoints_vector);

	void recover3D(vector<Mat> imgs, OpticalFlowMatcher matcher, bool dense, Mat K,
			       Mat distortion_coeff); //"main" function

private:

	vector<CloudPoint> p_cloud; //3D cloud
	map<int,cv::Matx34d> Pmats; //stores recovered camera matrices
	set<int> done_views; //views already used (or failed) for triangulation
	set<int> good_views; //views which produced good triangulation results (good F)

	vector<DMatch> flipMatches(vector<DMatch>& matches);

	void displayIMatches(Mat img1, vector<KeyPoint> k1, Mat img2, vector<KeyPoint> k2,
			             vector<DMatch> matches);

	void displayImage(Mat image, String name);
};



#endif /* SRC_INCREMENTALRECONSTRUCTION_H_ */
