/*
 * main.cpp
 *
 *  Created on: 23/set/2016
 *      Author: trev
 */


#include <cv.h>
#include <highgui.h>
#include <opencv2/calib3d.hpp>
#include "Calibration.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv){

	if(argc < 2){
		cout<<"error! no configuration file in input"<<endl;
		return 1;
	}

	int n_Snaps=0; //number of chessboard images
	int n_internalCornersH; //internal horizontal corners
	int n_internalCornersV; //internal vertical corners

	cout<<endl;
	printf("Enter number of HORIZONTAL internal corners: ");
	scanf("%d", &n_internalCornersH);

	cout<<endl;
	printf("Enter number of VERTICAL internal corners: ");
	scanf("%d", &n_internalCornersV);

	cout<<endl;
	printf("Enter number of snapshots to be used for calibration: ");
    scanf("%d", &n_Snaps);

    int n_Squares = n_internalCornersH * n_internalCornersV;
    cout<<endl;
    cout<<"Number of squares: "<<n_Squares<<endl;
    Size boardSize = Size(n_internalCornersH, n_internalCornersV);
    cout<<endl;
    cout<<"Board size: "<<boardSize<<endl;cout<<endl;

    Mat image;

    vector<Point3f> obj; //coordinates of corners for a chessboard (no measure unit [mill. ecc])
    for(int j=0;j<n_Squares;j++){
    	obj.push_back(Point3f(j / n_internalCornersH, j % n_internalCornersH, 0.0f));
    	cout<<obj[j]<<endl;
    }
    cout<<endl;

    Calibration calibrator;
    double re_projectionError; //distance (in pixels) of points projected on the 2D image, using the
                               //found cameraMatrix, from the actual image points

    vector<Point2f> currentCorners; //stores the current corners detected
    Mat cameraMatrix = Mat(3, 3, CV_32FC1); //camera matrix
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F); //distortion coefficients
    vector<Mat> rvecs; //rotation vector
    vector<Mat> tvecs; //translation vector

    //calibrate camera, i.e. found camera matrix, distorion coefficients, rotation and
    //translation vectors and return the re-projection error
    re_projectionError = calibrator.calibrate(&cameraMatrix, &distCoeffs, &rvecs, &tvecs,
    		                                  boardSize, currentCorners, obj, n_Snaps);

    cout<<"re-projection error: "<<re_projectionError<<endl;cout<<endl;
    cout<<"camera matrix has size: "<<cameraMatrix.size()<<endl;
    cout<<"cameraMatrix= "<<cameraMatrix<<endl;cout<<endl;
    cout<<"distortion coefficients vector has size: "<<distCoeffs.size()<<endl;
    cout<<"distCoeffs= "<<distCoeffs<<endl;cout<<endl;

    //writing calibration matrix to configuration.xml
    string filename = argv[1]; //read config. file
    FileStorage fs(filename, FileStorage::WRITE); //write mode
    //write
    cout<<endl;cout<<"writing K to configuration.xml"<<endl;
    fs<<"K"<<cameraMatrix;
    fs.release();
    cout<<"writing done"<<endl;

    /*//reading
    Mat K;
    cout<<"reading"<<endl;
    fs.open(filename, FileStorage::READ);
    fs["K"] >>K;
    cout<<"K= "<<K<<endl<<endl;
    fs.release();*/

    //undistort some image
    Mat distorted = imread("images/storta.jpg");
    namedWindow("distorted", CV_WINDOW_AUTOSIZE);
    imshow("distorted", distorted);
    waitKey(0);
    Mat undistorted;
    undistort(distorted, undistorted, cameraMatrix, distCoeffs);
    namedWindow("undistorted", CV_WINDOW_AUTOSIZE);
    imshow("undistorted", undistorted);
    waitKey(0);
    return 0;
    /*
    //show "black points" on undistorted image
    Mat view, rview, map1, map2;
    Size imageSize = calibrator.imageSize;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1,
                            imageSize, 0), imageSize, CV_16SC2, map1, map2);
    view = imread("images/storta.jpg");
    remap(view, rview, map1, map2, INTER_LINEAR);
    namedWindow("undistorted", CV_WINDOW_AUTOSIZE);
    imshow("undistorted", rview);
    waitKey(0);
    */
}

