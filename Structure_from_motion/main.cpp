
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
	double t = (double)getTickCount();

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

	vector<Mat> new_images;
	{
		VideoCapture cap(1); // 0 is the laptop webcam
		    if(!cap.isOpened())  // check if we succeeded
		        return -1;

		    vector<Mat> images;
		    char input;
		    int n,m;

		    cout<<"number of frames to be detected = n/m + 1"<<endl;
		    cout<<"\n( it will be taken a video stream of n frames as an input\n";
		    cout<<" then the program will take 1 frame every m from the stream )"<<endl;
		    printf("\nenter n: ");
		    scanf("%d", &n);
		    printf("\nenter m: ");
		    scanf("%d", &m);

		    cout<<"\npress key to exit video mode"<<endl;
		    cout<<"(when satisfied with camera positioning)\n";

		    namedWindow("frames", 1);
		    for(;;){
		    	Mat frame;
		    	cap >> frame; // get a new frame from camera
		    	imshow("frames", frame);
		    	if(waitKey(30) >= 0)
		    		break;
		    }
		    destroyWindow("frames");

		    cout<<"press s key to start capture loop"<<endl;
		    cout<<"(loop will terminate when n frames are taken)\n";
		    cin>>input;
		    if(input == 's'){
		    	    cout<<"loop starts"<<endl;
		            for(int i=0;i<n;i++)
		            {
		    		    Mat frame;
		    		    cap >> frame; // get a new frame from camera
		    	        images.push_back(frame);
		            }
		    }

		    cout<<"loop terminated"<<endl;
		    cout<<"images.size()= "<<images.size()<<endl;

		    //vector<Mat> new_images;
		    for(unsigned int j=0; j < images.size(); j++){
		    	if( (j%m) == 0 ){
		    		new_images.push_back(images[j]);
		    	}
		    }
		    cout<<"new_images.size()= "<<new_images.size()<<endl;
		    for(unsigned int j=0; j < new_images.size(); j++){
		    	imshow("frame", new_images[j]);
		    	waitKey(0);
		    }
	}

	{ //incremental reconstruction

	    IncrementalReconstruction i_rec;
	    bool dense = false;
	    OpticalFlowMatcher matcher;

	    //i_rec.recover3D(imgs, matcher, dense, K, distortion_coeff);
	    i_rec.recover3D(new_images, matcher, dense, K, distortion_coeff);

	}

    cout<<endl;
    cout<<"program ends"<<endl;

    t = ((double)getTickCount() - t)/getTickFrequency();

    cout<<"execution time: "<<t<<" seconds"<<endl;

}















