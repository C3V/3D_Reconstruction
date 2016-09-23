/*
 * Calibration.cpp
 *
 *  Created on: 23/set/2016
 *      Author: trev
 */


#include "Calibration.h"

#include <sstream>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <opencv2/calib3d.hpp>
#include "Calibration.h"

using namespace cv;
using namespace std;

using namespace std;
using namespace cv;

Calibration::Calibration() {

	imageIndex=0;
	//calibration snapshots
	imageList.push_back("images/1.jpg");
	imageList.push_back("images/2.jpg");
	imageList.push_back("images/3.jpg");
	imageList.push_back("images/4.jpg");
	imageList.push_back("images/5.jpg");
	imageList.push_back("images/6.jpg");
	imageList.push_back("images/7.jpg");
	imageList.push_back("images/8.jpg");
	imageList.push_back("images/9.jpg");
	imageList.push_back("images/10.jpg");
	imageList.push_back("images/11.jpg");
	imageList.push_back("images/12.jpg");
	imageList.push_back("images/13.jpg");
	imageList.push_back("images/14.jpg");
	imageList.push_back("images/15.jpg");
	imageList.push_back("images/16.jpg");
	imageList.push_back("images/17.jpg");

}

Calibration::~Calibration() {

	// TODO Auto-generated destructor stub

}

vector<string> Calibration::getImageList(){

	return imageList;

}

Mat Calibration::nextImage(){

	Mat image;
	if(imageIndex < (int)imageList.size()){
		image = imread(imageList[imageIndex], 1);
		imageIndex++;
	}
	/*
	namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
	imshow( "Display Image", image );
    waitKey(0);
    */
	return image;

}

void Calibration::resetImageIndex(){
	imageIndex = 0;
}

double Calibration::calibrate(Mat* cameraMatrix, Mat* distCoeffs, vector<Mat>* rvecs,
		                      vector<Mat>* tvecs ,Size boardSize, vector<Point2f> currentCorners,
							  vector<Point3f> obj, int n_Snaps){
    Size imageSize;
    vector<vector<Point3f> > objectPoints; //3D coordinates of the corners for every snap (world)
    vector<vector<Point2f> > imagePoints; //2D coordinates of the corners for every snap (image)
    int successes=0;

	for(unsigned int i=0; i < getImageList().size(); i++){
			Mat image = nextImage();
			imageSize = image.size();
			bool found = findChessboardCorners( image, boardSize, currentCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
			if(found){ //refine the found corners' positions
			    Mat viewGray;
			    cvtColor(image, viewGray, COLOR_BGR2GRAY);
				cornerSubPix( viewGray, currentCorners, Size(11,11), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
		        imagePoints.push_back(currentCorners);
		        objectPoints.push_back(obj);
		        successes++;
		        cout<<"Snap stored"<<endl;cout<<endl;
		        drawChessboardCorners( image, boardSize, Mat(currentCorners), found );
		        namedWindow( "Corners", CV_WINDOW_AUTOSIZE );
		        imshow( "Corners", image );
		        waitKey(0);
			}

		}

		cout<<"stored "<<successes<<" snapshots out of "<<n_Snaps<<endl;cout<<endl;
		if(successes == n_Snaps){
			cout<<"calibrating..."<<endl;cout<<endl;
	        double rms = calibrateCamera(objectPoints, imagePoints, imageSize, *cameraMatrix, *distCoeffs, *rvecs, *tvecs);
	        resetImageIndex();
	        /*//display undistorted calibration images
	        for(unsigned int i=0; i < (calibration.getImageList() ).size(); i++){
	        	Mat image1 = calibration.nextImage();
	        	namedWindow("distorted", CV_WINDOW_AUTOSIZE);
	            imshow("distorted", image1);
	        	waitKey(0);
	        	Mat u_image;
	            undistort(image1, u_image, cameraMatrix, distCoeffs);
	            namedWindow("undistorted", CV_WINDOW_AUTOSIZE);
	            imshow("undistorted", u_image);
	            waitKey(0);
	        }*/

	        return rms;
		} else
			cout<<"terminated; need "<<(n_Snaps-successes)<<" more snapshots to complete calibration"<<endl;
		    return 1;

}


