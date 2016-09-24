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

	if(argc < 2){
		cout<<"error! no configuration file in input"<<endl;
		return 1;
	}
	//read from configuration.xml
	string filename = argv[2];
	FileStorage fs;
	Mat K = Mat(3, 3, CV_32FC1); //camera matrix
	cout<<endl;cout<<"reading camera matrix of intrinsic parameters from configuration.xml: "<<endl;
	fs.open(filename, FileStorage::READ);
	fs["K"]>>K;
	cout<<"K= "<<K<<endl<<endl;
	fs.release();

	//read images from folder
	Mat firstImage, secondImage;
	firstImage = imread("images/2.3.jpg");
	secondImage = imread("images/2.4.jpg");

	//compute matches between the two frames
	OpticalFlowMatcher matcher;
    vector<DMatch> matches;
    cout<<endl; cout<<"computing matches:"<<endl;
    matches = matcher.matchFeatures(firstImage, secondImage, &matches);

    //display found matches
    matcher.displayMatches(firstImage, secondImage, matches);
    //display just one match
    vector<DMatch> single_match;
    single_match.push_back(matches[4]);
    matcher.displayMatches(firstImage, secondImage, single_match);


    //obtain fundamental matrix from correspondences and compute
    //essential matrix from fundamental
    CameraMatricesFinder finder;
    //better Mat or Mat_ ?
    Mat_<double> E = finder.findMatrixE(matches, matcher.getKeypoints_1(), matcher.getKeypoints_2(), K);
    cout<<"E= "<<E<<endl<<endl;

    //clear vectors and Mats of the matcher at the end of the iteration
    //(avoid creating unnecessary objects)
    matcher.clearFields();
    cout<<endl;
    cout<<"program ends"<<endl;

    //TODO ?mettere tutto in una sola funzione che calcola le coordinate 3D?

}
