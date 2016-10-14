/*
 * Visualization1.h
 *
 *  Created on: 05/ott/2016
 *      Author: trev
 */

#ifndef SRC_VISUALIZATION_H_
#define SRC_VISUALIZATION_H_


#include <cv.h>
#include <highgui.h>
#include "OpticalFlowMatcher.h"
#include "FindCameraMatrices.h"
#include "Triangulation.h"
#include <opencv2/viz.hpp>

class Visualization1{
public:

	void visualize3DPoints(vector<Point3d> points, Matx34d P, Matx34d P1,vector<Mat_<double> > R_,
			               vector<Mat_<double> > t_, map< int , Mat_<double> > camera_Rs,
						   map< int , Mat_<double> > camera_ts, Mat K, vector<double> repr_err);

private:

};


#endif /* SRC_VISUALIZATION_H_ */
