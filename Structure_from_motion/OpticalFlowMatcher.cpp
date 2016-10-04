/*
 * OpticalFlowMatcher.cpp
 *
 *  Created on: 15/set/2016
 *      Author: trev
 */

#include "OpticalFlowMatcher.h"

#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/flann/flann.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <sstream>

#include <set>

using namespace std;
using namespace cv;


OpticalFlowMatcher::OpticalFlowMatcher(){
	//constructor
    matcher = BFMatcher(CV_L2);
}


OpticalFlowMatcher::~OpticalFlowMatcher(){
	//destructor
}


void OpticalFlowMatcher::testMethod(){
	cout << "method works" << endl;
	cout << endl;
}


vector<KeyPoint> OpticalFlowMatcher::getKeypoints_1(){
	return keypoints_1;
}


vector<KeyPoint> OpticalFlowMatcher::getKeypoints_2(){
	return keypoints_2;
}


void OpticalFlowMatcher::displayImage(Mat image, String name){
	namedWindow(name, WINDOW_AUTOSIZE);
	//to resize, but remember to change to WINDOW_NORMAL WINDOW_AUTOSIZE
	resizeWindow(name, 500,400);
	imshow(name, image);
	waitKey(0);
}


void OpticalFlowMatcher::displayMatches(Mat image_1, Mat image_2, vector<DMatch> matches){
	Mat img_matches;
	drawMatches(image_1, keypoints_1, image_2, keypoints_2, matches, img_matches, Scalar(0,255));
	displayImage(img_matches, "Matches");
}


void OpticalFlowMatcher::clearFields(){
	keypoints_1.clear();
	keypoints_2.clear();
	left_points.clear();
	right_points.clear();
	right_features.clear();
	prevGray.release();
	gray.release();
	vStatus.clear();
	vError.clear();
	right_points_to_find.clear();
	right_points_to_find_back_index.clear();
	right_points_to_find_flat.release();
	right_features_flat.release();
	nearest_neighbors.clear();
	found_in_right_points.clear();
	_m1.clear();
}


void OpticalFlowMatcher::displayKeyPoints(Mat image, vector<KeyPoint> keypoints, String name){
	Mat img_keypoints;
	namedWindow(name, WINDOW_AUTOSIZE);
	//resizeWindow(name, 500, 400);
	drawKeypoints( image, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow(name, img_keypoints );
    waitKey(0);

}


void OpticalFlowMatcher::KeyPointsToPoints(vector<KeyPoint> keypoints, vector<Point2f>* points){

	//pay attention to pointer operations:
	//when you pass a pointer to a vector, doing (*pointer) you are
	//referencing the vector, and with (*pointer).field you access his fields
	for(unsigned int i=0; i < (keypoints).size(); i++){
		(*points).push_back( (keypoints)[i].pt );
	}

}


vector<DMatch> OpticalFlowMatcher::matchFeatures(Mat firstImage, Mat secondImage, vector<DMatch>* matches){



	//creating detector and detecting feature points
    detector.detect(firstImage, keypoints_1);
    detector.detect(secondImage, keypoints_2);
    /*
    cout << keypoints_1.size() << " KeyPoint points detected for image 1" << endl;
    cout << keypoints_2.size() << " KeyPoint points detected for image 2" << endl;
    cout << endl;
    */

    //draw KeyPoints
    /*
    displayKeyPoints(firstImage, keypoints_1, "KeyPoints_1");
    displayKeyPoints(secondImage, keypoints_2, "KeyPoints_2");
    */

    //convert vector of KeyPoint in vector of Point2f for the first image
    KeyPointsToPoints( keypoints_1, &left_points);

    //vector<Point2f> right_points must have same size as left_points
    right_points.resize(left_points.size());

    //print new vector<Point2f> sizes
    /*
    cout << left_points.size() << " KeyPoint points converted to Point2f for image1" << endl;
    cout << right_points.size() << " size of the Point2f vector for image2" << endl;
    cout << endl;
    */

    //print one random detected KeyPoint for both images
    /*
    cout << keypoints_1[1].pt << " first KeyPoint coordinates for image 1" << endl;
    cout << keypoints_2[1].pt << " first KeyPoint coordinates for image 2" << endl;
    cout << endl;
    */

    //print the same KeyPoints now converted to Point2f
    //to make sure coordinates are the same
    /*
    cout << left_points[1] << " first Point2f coordinates for image 1" << endl;
    /*
    cout << right_points[1] << " first Point2f coordinates for image 2" << endl;

    cout << endl;
    */


    // making sure images are grayscale for the OF algorithm
    if (firstImage.channels() == 3) {
        cvtColor(firstImage,prevGray,CV_RGB2GRAY);
        cvtColor(secondImage,gray,CV_RGB2GRAY);
    } else {
        prevGray = firstImage;
        gray = secondImage;
    }

    /*
    //display new grayscale images
    displayImage(prevGray, "First Gray Image");
    displayImage(gray, "Second Gray Image");
    */

    //make sure vector<Point2f> right_points is empty before computing OF
    /*
    cout << right_points[1] << " check if vector<Point2f> right_points is empty" << endl;
    */

    //calculate optical flow
    //vector<Point2f> right_points now has the new calculated coordinates
    //of left_points
    calcOpticalFlowPyrLK(prevGray, gray, left_points, right_points, vStatus, vError);

    //check if some random left_point has moved throw the image
    /*
    cout << left_points[1] << " coordinates of first left point"<<endl;
    cout << right_points[1] << " estimated coordinates of the corresponding new right point"<<endl;
    cout << endl;
    */

    //don't need the points with high error of estimation, filter out
    for(unsigned int i =0; i< vStatus.size(); i++){
    	if(vStatus[i] && vError[i] < 12.0){
    		//then we can use the point: store his index and his value
    		right_points_to_find_back_index.push_back(i);
    		right_points_to_find.push_back(right_points[i]);
    	}
    	else {
    		vStatus[i] = 0; //bad flow
    	}
    }
    /*
    cout << right_points_to_find.size() << " estimated points with acceptable accuracy after OF" << endl;
    cout << endl;
    */

    //convert vector<Point2f> right_points_to_find to a Nx2 1-channel matrix
    //without Mat::reshape(cn,rows) I would have a Nx1 2-channel matrix
    //probably the matching method I will use does not support multi-channel matrices
    right_points_to_find_flat = Mat(right_points_to_find).reshape(1,right_points_to_find.size());

    //convert vector<KeyPoint> keypoints_2 to vector<Point2f> right_features
    //right_features represents the detected Point2f keypoints in secondImage
    //not the OF new positions of left_points!
    KeyPointsToPoints(keypoints_2, &right_features);
    /*
    cout << right_features.size() << " Point2f converted keypoints of secondImage" << endl;
    */

    //convert to a Nx2 1-channel matrix like before
    right_features_flat = Mat(right_features).reshape(1, right_features.size());

    //README right_points_to_find_flat represents the OF points with low error
    //       right_features_flat represents the detected (as an early step) features
    //       in second image. The goal is to find a match between them

    //brute-force match
    //BFMatcher matcher(CV_L2);
    matcher.radiusMatch(right_points_to_find_flat, right_features_flat, nearest_neighbors, 2.0f);

    //in nearest_neighbors there are one or more matches for every right_point_to_find
    //but there can be only one right feature point for every left feature point moved (motion
    //estimated by OF), so throw away neighbors that are too closer together as they might
    //be confusing

    //N.B. _m substituted with _m1 to allow clear of the vector after the computation
    int index = 0;
    for(unsigned int i=0; i < nearest_neighbors.size(); i++){
    	//DMatch _m;
    	if(nearest_neighbors[i].size() == 1){
    		//_m = nearest_neighbors[i][0];  //only one neighbor
    		_m1.push_back(nearest_neighbors[i][0]);
    		index++;
    	}
    	else if(nearest_neighbors[i].size() > 1){
    		//more than one neighbor, take the closer one
    		double ratio = nearest_neighbors[i][0].distance / nearest_neighbors[i][1].distance;
    		if(ratio < 0.7){
    			//take the first one
    			//_m= nearest_neighbors[i][0];
    			_m1.push_back(nearest_neighbors[i][0]);
    			index++;
    		}
    		else{
    			//they are too close, hard to say which one is the best
    			continue;
    		}
    	}
    	else{
    		//no neighbors
    		continue;
    	}
    	//prevent duplicates; make sure each feature has only one match
    	//[when if() is true than the research terminated without finding the feature point]
    	//if(found_in_right_points.find(_m.trainIdx) == found_in_right_points.end()){
        if(found_in_right_points.find(_m1[index-1].trainIdx) == found_in_right_points.end()){
    		//the feature point of the right image was not yet used (or "assigned");
    		//must match it with the original index of the OF left point, because the one
    		//in use is referred to the array right_points_to_find, which contains only
    		//the "good" OF-calculated left points
    		//_m.queryIdx = right_points_to_find_back_index[_m.queryIdx];
    		_m1[index-1].queryIdx =  right_points_to_find_back_index[_m1[index-1].queryIdx];
    		//add this match to the output vector
    		//matches->push_back(_m);
    		matches->push_back(_m1[index-1]);
    		//mark this point as "used/assigned"
    		//found_in_right_points.insert(_m.trainIdx);
    		found_in_right_points.insert(_m1[index-1].trainIdx);
    	}
    }


    cout << matches->size() << " matches found" << endl;

    /*
    //print matches by index
    for(unsigned int i=0; i< (*matches).size(); i++){
    	cout<<(*matches)[i].queryIdx <<" queryIdx "<<(*matches)[i].trainIdx<<" trainIdx"<<endl;
    }
    */

    return *matches;

} //matchFeatures






