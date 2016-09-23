/*
 * Calibration.h
 *
 *  Created on: 23/set/2016
 *      Author: trev
 */

#ifndef SRC_CALIBRATION_H_
#define SRC_CALIBRATION_H_

#include <opencv2/highgui/highgui.hpp>
#include <sstream>

using namespace std;
using namespace cv;

class Calibration {

public:

	Calibration();

	virtual ~Calibration();

	vector<string> getImageList();

	Mat nextImage();

	void resetImageIndex();

	double calibrate(Mat* cameraMatrix, Mat* distCoeffs, vector<Mat>* rvecs, vector<Mat>* tvecs,
			         Size boardSize, vector<Point2f> currentCorners, vector<Point3f> obj, int n_Snaps);


private:
	//contains calibration images' names
	vector<string> imageList;

	int imageIndex;

};



#endif /* SRC_CALIBRATION_H_ */
