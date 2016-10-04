/*
 * Visualization.h
 *
 *  Created on: 04/ott/2016
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

class Visualization{
public:

	void visualize3DPoints(vector<Point3d> points, Matx34d P, Matx34d P1, Mat K);

private:

};



#endif /* SRC_VISUALIZATION_H_ */
