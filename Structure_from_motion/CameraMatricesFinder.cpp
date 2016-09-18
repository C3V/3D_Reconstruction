/*
 * CameraMatricesFinder.cpp
 *
 *  Created on: 17/set/2016
 *      Author: trev
 */

#include "CameraMatricesFinder.h"

#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/flann/flann.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <sstream>

using namespace std;
using namespace cv;



CameraMatricesFinder::CameraMatricesFinder(){

	//constructor

}

void CameraMatricesFinder::convertToPoint2f(vector<DMatch> matches, vector<KeyPoint> keypoints_left, vector<KeyPoint> keypoints_right){

	//need to convert KeyPoint to Point2f to use findFundamentalMat()
	for(unsigned int i=0; i<matches.size(); i++){
		imgpts_left.push_back(keypoints_left[ matches[i].queryIdx ].pt );
		imgpts_right.push_back(keypoints_right[ matches[i].trainIdx ].pt );
	}
	cout<<matches[1].queryIdx <<" first queryIdx "<<matches[1].trainIdx<<" first trainIdx"<<endl;
	cout<<imgpts_left.size()<<" =imgpts_left size; must be 34 and =imgpts_right.size()"<<endl;
	cout<<imgpts_right.size()<<" =imgpts_right size"<<endl;
	cout<<keypoints_left[matches[1].queryIdx].pt<<" first left image KeyPoint to be converted to Point2f"<<endl;
	cout<<imgpts_left[1]<<" first converted left KeyPoint"<< endl;
	cout<<imgpts_right[1]<<" first converted right KeyPoint"<< endl;

}

Mat CameraMatricesFinder::findMatrixE(vector<DMatch> matches, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2){

	convertToPoint2f(matches, keypoints_1, keypoints_2);

	//obtain fundamental matrix with RANSAC algorithm (correspondences >= 8)
	F = findFundamentalMat(imgpts_left, imgpts_right, FM_RANSAC, 0.1, 0.99, vStatus);

	//TODO need the internal parameters matrix K to compute E-->calibrate camera before
	//obtain essential matrix
	//E = K.t() * F * K;

	//[weird] program doesn't print "program ends" if I ignore the warning about returning some matrix
	return E;

	//TODO ricordati di fare release e clear di vettori e matrici come prima
}

