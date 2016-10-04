/*
 * Visualization.cpp
 *
 *  Created on: 04/ott/2016
 *      Author: trev
 */


#include "Visualization.h"

void Visualization::visualize3DPoints(vector<Point3d> points, Matx34d P, Matx34d P1, Mat K){

	viz::Viz3d window("Coordinate Frame");
    window.setWindowSize(Size(650,525));
    window.setWindowPosition(Point(150,150));
	window.setBackgroundColor(); // black by default

	cout<<"----------------------------------------------------------------------------------------------------------"<<endl;
	cout<<endl;cout<<"populating point cloud with recovered 3D points..."<<endl;
	cout<<"(remember that in order to use the viz module, due to the multiple installations"<<endl;
	cout<<"of opencv I did, you need to add a new environment variable to the run configuration"<<endl;
	cout<<"see http://stackoverflow.com/questions/27907343/error-while-loading-shared-libraries-libopencv-core-so-3-0"<<endl;
	cout<<"for details)."<<endl;
	cout<<"----------------------------------------------------------------------------------------------------------"<<endl;

	vector<Point3d> point_cloud;
	vector<Point3d> points_ = points; //copy (to make sure the "if" does not modify points using fabs ecc.)
	for(unsigned int i=0; i<points.size(); i++){

		//check for erroneous coordinates
		if(points[i].x != points[i].x || isnan(points[i].x) ||
		   points[i].y != points[i].y || isnan(points[i].y) ||
		   points[i].z != points[i].z || isnan(points[i].z) ||
		   points[i].z <0 || fabsf(points[i].x) > 10.0 || fabsf(points[i].y) > 10.0 ||
		   fabsf(points[i].z) > 10.0){
			continue;
		}

		point_cloud.push_back(points_[i]);
		//cout<<points_[i]<<endl;

	}

	cout<<endl;
	cout<<"after check for erroneous coordinates, point_cloud has "<<point_cloud.size()<<" out of "<<points_.size()<<" points"<<endl;
/*	cout<<endl;cout<<"point_cloud: "<<endl;
	for(unsigned int j=0; j< point_cloud.size(); j++){
		cout<<point_cloud[j]<<endl;
	} */

	/*
	cout<<endl;cout<<"recovering cameras..."<<endl;

	Matx33d R(P(0,0), P(0,1), P(0,2),
			  P(1,0), P(1,1), P(1,2),
			  P(2,0), P(2,1), P(2,2));
	Matx31d t(P(0,3),
			  P(1,3),
			  P(2,3));
	cout<<"R= "<<R<<endl;cout<<" t= "<<t<<endl;

	Matx33d R1(P1(0,0), P1(0,1), P1(0,2),
			   P1(1,0), P1(1,1), P1(1,2),
			   P1(2,0), P1(2,1), P1(2,2));
	Matx31d t1(P1(0,3),
			   P1(1,3),
			   P1(2,3));
	cout<<"R1= "<<R1<<endl;cout<<"t1= "<<t1<<endl;

	vector<Affine3d> path;
	path.push_back( Affine3d(R,t) );
	path.push_back( Affine3d(R1,t1) );
	cout<<"done"<<endl;
*/
	if ( point_cloud.size() > 0 )
	{
	    cout<<endl;cout << "Rendering points   ... "<<endl;
		viz::WCloud cloud_widget(point_cloud, viz::Color::green());
	    window.showWidget("point_cloud", cloud_widget);
	    cout << "done." << endl;
	}
	else
	{
		cout<<endl;cout << "Cannot render points: Empty point_cloud" << endl;
    }
/*
	if ( path.size() > 0 )
    {
	    cout << "Rendering Cameras  ... ";
	    window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
	    window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
	    window.setViewerPose(path[0]);
	    cout << "done" << endl;
	}
	else
	{
	    cout << "Cannot render the cameras: Empty path" << endl;
    }
*/
	cout<<"press q to exit window"<<endl;
	window.spin(); //the window renders and starts the event loop

}

