/*
 * main.cpp
 *
 *  Created on: 15/set/2016
 *      Author: trev
 */


#include <cv.h>
#include <highgui.h>
#include "OpticalFlowMatcher.h"
#include "CameraMatricesFinder.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv){

	cout << "program starts" << endl;

	//read images from folder
	Mat firstImage, secondImage;
	firstImage = imread("images/1.JPG");
	secondImage = imread("images/2.JPG");

	//compute matches between the two frames
	OpticalFlowMatcher matcher;
    vector<DMatch> matches;
    cout<<endl; cout<<"computing matches:"<<endl;
    matches = matcher.matchFeatures(firstImage, secondImage, &matches);

    //display found matches
    matcher.displayMatches(firstImage, secondImage, matches);
    //display just one match
    /*
    vector<DMatch> single_match;
    single_match.push_back(matches[18]);
    matcher.displayMatches(firstImage, secondImage, single_match);
    */

    //obtain fundamental matrix
    CameraMatricesFinder finder;
    //better Mat or Mat_ ?
    cout<<endl; cout<<"obtaining fundamental matrix:"<<endl;
    Mat E = finder.findMatrixE(matches, matcher.getKeypoints_1(), matcher.getKeypoints_2() );

    //clear vectors and Mats of the matcher at the end of the iteration
    //(avoid creating unnecessary objects)
    matcher.clearFields();
    cout<<endl;
    cout<<"program ends"<<endl;

}
