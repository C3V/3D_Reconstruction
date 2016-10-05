/*
 * Visualization1.cpp
 *
 *  Created on: 05/ott/2016
 *      Author: trev
 */

#include "Visualization.h"

void Visualization1::visualize3DPoints(vector<Point3d> points, Matx34d P, Matx34d P1, Mat K){

	// Create a window
	viz::Viz3d myWindow("Coordinate Frame");
	myWindow.setWindowSize(Size(800,600));
    myWindow.setWindowPosition(Point(150,150));
	myWindow.setBackgroundColor(); // black by default

	// Add coordinate axes
	//myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

	cout<<endl;cout<<"recovering 3D points..."<<endl;

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

	} cout<<"done."<<endl;

	cout<<endl;cout << "recovering cameras... "<<endl;

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
    path.push_back( Affine3d( Mat(R), Mat(t) ) ); //need to cast to Mat (don't know why)
    path.push_back( Affine3d( Mat(R1), Mat(t1) ) );
    cout<<"done."<<endl;

	if ( point_cloud.size() > 0 )
	{
	    cout<<endl;cout<<endl;cout << "rendering points   ... "<<endl;
		viz::WCloud cloud_widget(point_cloud, viz::Color::green());
	    myWindow.showWidget("point_cloud", cloud_widget);
	    cout << "done." << endl;
	}
	else
	{
		cout<<endl;cout << "cannot render points: empty point_cloud" << endl;
	}

	if ( path.size() > 0 )
    {
        cout<<endl;cout << "rendering Cameras  ... ";
	    myWindow.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
	    myWindow.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, Matx33d(K), 0.1, viz::Color::yellow()));

	    /*
	     * hack to visualize the scene from a "simulated" point of view;
	     * set a virtual new camera position, relative to the first camera
	     * with matrix P[I|0], with no rotation and translation only in z
	     * (such pose will be just behind the  other two cameras).
	     * Then push in path and call setViewerPose() with new virtual path.
	     */
	    Matx33d Rg(1,0,0, //no rotation
	    		   0,1,0,
				   0,0,1);
	    Matx31d tg(0,  //translate only in z
	    		   0,
				   -7);
	    path.push_back( Affine3d( Mat(Rg), Mat(tg) ) );

	    //comment following setViewerPose() functions to remove specific points of view
	    cout<<"press q to exit window"<<endl;
	    myWindow.spin(); //no point of view-->explore the 3D structure of the scene
	    myWindow.setViewerPose(path[2]); //simulated global point of view
	    myWindow.spin(); //no point of view
	    myWindow.setViewerPose(path[1]); //point of view of the right camera
	    myWindow.spin();

	    cout << "done." << endl;
	}
    else
    {
	    cout << "cannot render the cameras: empty path" << endl;
    }

}


