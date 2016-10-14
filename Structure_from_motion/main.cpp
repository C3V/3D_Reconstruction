
/*
 * main.cpp
 *
 *  Created on: 30/set/2016
 *      Author: trev
 */



#include <cv.h>
#include <highgui.h>
#include "OpticalFlowMatcher.h"
#include "DenseOFFeatureMatcher.h"
#include"IncrementalReconstruction.h"
#include "Common.h"
#include "FindCameraMatrices.h"
#include "Triangulation.h"
#include <opencv2/viz.hpp> //visualization module based on VTK (requires VTK installed)

#include "Visualization.h"


using namespace cv;
using namespace std;

int main(int argc, char** argv){

	cout << "program starts" << endl;

	if(argc < 2){
		cout<<"error! no configuration file in input"<<endl;
		return 1;
	}

	//read from configuration.xml
	string filename = argv[1];
	FileStorage fs;
	Mat K = Mat(3, 3, CV_32FC1); //camera matrix
	Mat distortion_coeff;
	cout<<endl;cout<<"reading camera matrix of intrinsic parameters from configuration.xml: "<<endl;
	fs.open(filename, FileStorage::READ);
	fs["K"]>>K;
	cout<<"K= "<<K<<endl;cout<<endl;
	cout<<"reading distortion coefficients:"<<endl;
	fs["distortion_coeff"]>>distortion_coeff;
	cout<<"distortion_coeff= "<<distortion_coeff<<endl;
	fs.release();

	//read images from folder--good frames to use:
	//(3.11,3.12)-->notebook -  (3.15,3.16)-->ball - (5.1,5.2,5.3) ball sequence - (4.1,4.2,4.3) another ball sequence
	//with 7.0/1/2 the incremental works(maybe!) and adds just 1 POINT (!) to baseline cloud
	//(9.1,9.2,9.3 ) (10.1/4) (11.1/4)-->best one with 4 views
	vector<Mat> imgs; //storage vector, pass it to IncrementalReconstruction
	Mat firstImage, secondImage, thirdImage, fourthImage, fifthImage;
	firstImage = imread("images/11.1.jpg"); imgs.push_back(firstImage);
	secondImage = imread("images/11.2.jpg"); imgs.push_back(secondImage);
	thirdImage = imread("images/11.3.jpg"); imgs.push_back(thirdImage);
	fourthImage = imread("images/11.4.jpg"); imgs.push_back(fourthImage);
	//fifthImage = imread("images/12.5.jpg"); imgs.push_back(fifthImage);

	cout<<"\nobtained "<<imgs.size()<<" images to perform reconstruction"<<endl;

	{ //incremental reconstruction
	IncrementalReconstruction i_rec;
	bool dense = false;
	OpticalFlowMatcher matcher;

	i_rec.recover3D(imgs, matcher, dense, K, distortion_coeff);
	}

/*	//compute matches between the two frames
	bool dense = false;
	vector<DMatch> matches;
	vector<KeyPoint> keypoints_left, keypoints_right;
	map< pair<int,int>,vector<DMatch> > matches_matrix; //TODO passare al costrutt. di IncrementalR

	if(dense){
		DenseOFFeatureMatcher matcher;
		vector<KeyPoint> keypoints1, keypoints2;
		cout<<endl;cout<<"computing dense matching"<<endl;
		matches = matcher.matchFeatures(firstImage, secondImage, keypoints_left, keypoints_right);
	}

	else { //sparse
		OpticalFlowMatcher matcher;
	    cout<<endl; cout<<"computing sparse matching:"<<endl;
	    //matches = matcher.matchFeatures(firstImage, secondImage, &matches);
	    matches = matcher.matchFeatures(secondImage, thirdImage, &matches);

	    //display found matches
	    matcher.displayMatches(firstImage, secondImage, matches);
	    //matcher.displayMatches(secondImage, thirdImage, matches);
	    //display just one match
	    vector<DMatch> single_match;
	    single_match.push_back(matches[4]);
        matcher.displayMatches(firstImage, secondImage, single_match);

        keypoints_left = matcher.getKeypoints_1();
        keypoints_right = matcher.getKeypoints_2();
	}
*/
	/*
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
    FindCameraMatrices finder;
    vector<KeyPoint> keypoints_left, keypoints_right;
    keypoints_left = matcher.getKeypoints_1();
    keypoints_right = matcher.getKeypoints_2(); */
/*    FindCameraMatrices finder;
    vector<KeyPoint> keypoints_left_good, keypoints_right_good;
    Matx34d P(1, 0, 0, 0, //[I|0]
              0, 1, 0, 0,
              0, 0, 1, 0
    	     );
    Matx34d P1;
    //N.B. this function changes matches because of the points "pruned" by F
    //Mat F = finder1.getFundamentalMat(matcher.getKeypoints_1(), matcher.getKeypoints_2(),
    		//keypoints_left_good, keypoints_right_good, matches);
    Mat Kinv(K.inv()); //inverse
    //cout<<"Kinv= "<<Kinv<<endl;
    vector<Point3d> outCloud; //stores found 3D points
    //also performs triangulation
    vector<double> reproj_err;
    vector<CloudPoint> p_cloud;
    finder.findCameraMatrices(K, Kinv, keypoints_left, keypoints_right, keypoints_left_good,
    		keypoints_right_good, P, P1, matches, outCloud, p_cloud, dense, reproj_err);
    cout<<endl;cout<<"reconstructed "<<outCloud.size()<<" points out of "<<keypoints_left_good.size()<<endl;

    //cout<<endl;printf("help: triangulate points using matrices MPP=[I|0] and "
    //		"MPP1=[R|t] or using MPP=K[I|0] and MPP1=K[R|t]?");cout<<endl;

	//show 3D results
	Visualization1 display;
	vector<Mat_<double> > R_, t_;
	display.visualize3DPoints(outCloud, P, P1,R_, t_, K, reproj_err);

    //TODO distruggere tutti gli oggetti, sennò vengono reistanziati tutte le volte, oppure
    //crearli fuori dal loop
    //clear vectors and Mats of the matcher at the end of the iteration
    //(avoid creating unnecessary objects)

    //TODO implement matcher.clearFields(); inside the if(dense) else...
*/    cout<<endl;
    cout<<"program ends"<<endl;

    //TODO ?mettere tutto in una sola funzione che calcola le coordinate 3D?



}











