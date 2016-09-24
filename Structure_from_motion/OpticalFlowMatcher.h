/*
 * OpticalFlowMatcher.h
 *
 *  Created on: 15/set/2016
 *      Author: trev
 */ 

#ifndef SRC_OPTICALFLOWMATCHER_H_
#define SRC_OPTICALFLOWMATCHER_H_

#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

class OpticalFlowMatcher {

public:

	//will contain the found matches
	//vector<DMatch> matches;

	//constructor
	OpticalFlowMatcher();

	~OpticalFlowMatcher();

	vector<KeyPoint> getKeypoints_1();

	vector<KeyPoint> getKeypoints_2();

	void testMethod();

	void displayImage(Mat image, String name);

	void displayKeyPoints(Mat image, vector<KeyPoint> keypoints, String name);

	void displayMatches(Mat image_1, Mat image_2, vector<DMatch> matches);

	//necessary for reuse of the matcher in a cycle
	void clearFields();

	//utility method for conversion
	void KeyPointsToPoints(vector<KeyPoint> keypoints, vector<Point2f>* points);

	//method of interest
	vector<DMatch> matchFeatures(Mat firstImage, Mat secondImage, vector<DMatch>* matches);

private:

    FastFeatureDetector detector;

    BFMatcher matcher;

	vector<KeyPoint> keypoints_1, keypoints_2;

	//vectors passed to the Optical Flow method
	vector<Point2f> left_points, right_points;

	vector<Point2f> right_features;

	//first and second images converted to grayscale
	Mat prevGray, gray;

	//output status vector (of unsigned chars); each element of the vector
	//is set to 1 if the flow for the corresponding features has been
	//found, otherwise, it is set to 0
	vector<uchar> vStatus;

	//output vector of errors; each element of the vector is set to an error
    //for the corresponding feature
    vector<float> vError;

    //this vector contains the filtered points calculated by OF, i.e. the points
    //with  vError < 12.0
    vector<Point2f> right_points_to_find;

    //this vector contains the original indexes in the right_points vector
    //stored due to the filtering of points with high error
    vector<int> right_points_to_find_back_index;

    //another way of representing vector<Point2f> right_points_to_find
    //and vector<Point2f> right_features
    //necessary to the match method?
    Mat right_points_to_find_flat;
    Mat right_features_flat;

    //vector of (vector of) matches, output of the brute-force matcher
    vector< vector<DMatch > > nearest_neighbors;

    //will contain the single matches of left feature points after motion (estimated by OF)
    //in right image and right feature points
    vector<DMatch> _m1;

    set<int>found_in_right_points; // for duplicate prevention

};



#endif /* SRC_OPTICALFLOWMATCHER_H_ */
