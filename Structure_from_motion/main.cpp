
/*
 * main.cpp
 *
 *  Created on: 30/set/2016
 *      Author: trev
 */



#include <cv.h>
#include <highgui.h>
#include "OpticalFlowMatcher.h"
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
	cout<<endl;cout<<"reading camera matrix of intrinsic parameters from configuration.xml: "<<endl;
	fs.open(filename, FileStorage::READ);
	fs["K"]>>K;
	cout<<"K= "<<K<<endl<<endl;
	fs.release();

	//read images from folder--good frames to use->(3.11, 3.12), (2.3, 2.4)
	Mat firstImage, secondImage;
	firstImage = imread("images/3.11.jpg"); //2.3, 2.4
	secondImage = imread("images/3.12.jpg");

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
    FindCameraMatrices finder;
    vector<KeyPoint> keypoints_left, keypoints_right;
    keypoints_left = matcher.getKeypoints_1();
    keypoints_right = matcher.getKeypoints_2();
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
    cout<<"Kinv= "<<Kinv<<endl;
    vector<Point3d> outCloud; //stores found 3D points
    //also performs triangulation
    finder.findCameraMatrices(K, Kinv, keypoints_left, keypoints_right, keypoints_left_good,
    		keypoints_right_good, P, P1, matches, outCloud);
    cout<<endl;cout<<"reconstructed "<<outCloud.size()<<" points out of "<<keypoints_left_good.size()<<endl;

    cout<<endl;printf("help: triangulate points using matrices MPP=[I|0] and "
    		"MPP1=[R|t] or using MPP=K[I|0] and MPP1=K[R|t]?");cout<<endl;

    string filename1 = argv[2];
    FileStorage fs1;
    cout<<endl;cout<<"writing point cloud in output_configuration.xml"<<endl;
	fs1.open(filename1, FileStorage::WRITE);
	fs1<<"pointCloud"<<outCloud;
	fs1.release();
	cout<<"writing done"<<endl;

	/*
	{//try to use OpenCV's triangulation
	    //camera matrices
	    Mat_<double> M = K * Mat(P);
	    Mat_<double> M1 = K * Mat(P1);

	    vector<Point2f> ptsl, ptsr;
	    for(unsigned int i=0; i<keypoints_left_good.size(); i++){
	    	ptsl.push_back( keypoints_left_good[i].pt );
	    	ptsr.push_back( keypoints_right_good[i].pt );
	    }
	    cout<<"ptsl.size()= "<<ptsl.size()<<" ptsr.size()= "<<ptsr.size()<<endl;

	    Mat_<double> X_cv;
	    triangulatePoints( M, M1, ptsl, ptsr, X_cv); //OpenCV triangulation
	    cout<<"primo punto triangolato= "<<X_cv(0,0)<<endl;
	}*/

	//show 3D results
	Visualization display;
	display.visualize3DPoints(outCloud, P, P1, K);

    //TODO distruggere tutti gli oggetti, sennò vengono reistanziati tutte le volte, oppure
    //crearli fuori dal loop
    //clear vectors and Mats of the matcher at the end of the iteration
    //(avoid creating unnecessary objects)
    matcher.clearFields();
    cout<<endl;
    cout<<"program ends"<<endl;

    //TODO ?mettere tutto in una sola funzione che calcola le coordinate 3D?



}



