/*
 * Triangulation.cpp
 *
 *  Created on: 27/set/2016
 *      Author: trev
 */

#include "Triangulation.h"

#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/flann/flann.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <sstream>

using namespace std;
using namespace cv;


Mat_<double> Triangulation::linearLSTriangulation(Point3d p, //first image point in homogenous coordinates (u,v,1)
		                                          Matx34d P, //first camera matrix
											      Point3d p1, //second image point (u1,v1,1)
											      Matx34d P1 //second camera matrix
											      ){

	//matrix A
	//point p(u,v,1) in OpenCV is p(x,y,z) with z=1
	Matx43d A(p.x*P(2,0)-P(0,0),	p.x*P(2,1)-P(0,1),		p.x*P(2,2)-P(0,2),
			  p.y*P(2,0)-P(1,0),	p.y*P(2,1)-P(1,1),		p.y*P(2,2)-P(1,2),
			  p1.x*P1(2,0)-P1(0,0), p1.x*P1(2,1)-P1(0,1),	p1.x*P1(2,2)-P1(0,2),
			  p1.y*P1(2,0)-P1(1,0), p1.y*P1(2,1)-P1(1,1),	p1.y*P1(2,2)-P1(1,2)
	);

    //matrix B
	Matx41d B( -(p.x*P(2,3)-P(0,3)),
			   -(p.y*P(2,3)-P(1,3)),
			   -(p1.x*P1(2,3)-P1(0,3)),
	           -(p1.y*P1(2,3)-P1(1,3))
			 );

	//assume X(x,y,z,1) in LinearLs (see Hartley)
	Mat_<double> X;

	solve(A,B,X,DECOMP_SVD);

	/*
	//hack to add one to X (homogenous) to make KP*X work
	//added by me, but it shouldn't exist!
	Mat one = Mat::ones(1,1, CV_64F);
	//Mat one = Mat(1,1, CV_64F); //doesn't reconstruct with this one
	X.push_back(one);
	//Mat M(1,1, CV_64F, 82.3);
	//X.push_back(M);
	*/

	return X; //not homogenous-->(x,y,z) != (x,y,z,1)

}


double Triangulation::triangulatePoints(vector<KeyPoint> keypoints1, //first "pruned" keypoints
		                                vector<KeyPoint> keypoints2, //second "pruned" keypoints
										Mat K, //calibration matrix
										Mat Kinv, //K inverse
										Matx34d P, //first camera matrix
										Matx34d P1,
										vector<Point3d>& pointcloud //store the 3D points
										){ //second camera matrix
	/*
	 * we are using NORMALIZED camera matrices P=[I|0] and P1=[R|t] in which
	 * the effect of calibration matrix K was removed, so the correspondent
	 * image points are xn=Kinv*x and x1n=Kinv*x1.
	 * In order to compute the triangulation we must multiply the homogenous
	 * image points per Kinv to normalize them,
	 *                 or
	 * we can use the matrices KP=K[I|0] and KP1=K[R|t] and not change x, x1
	 */

	cout<<endl;cout << "Triangulating..."<<endl;
	vector<double> reproj_error;
	unsigned int size = keypoints1.size();
	Mat_<double> KP1 = K * Mat(P1);  //de-normalize, will be used in re-projection
	//cout<<"KP1= "<<KP1<<endl;

	cout<<"normalizing image points"<<endl;
	for(unsigned int i=0; i<size; i++){
		Point2f p = keypoints1[i].pt; //convert first im. points to Point2f
		Point3d p_hom_norm(p.x, p.y, 1.0); //represent in homogenous coordinates (u,v,1)
		Mat_<double> p_mat_norm = Kinv * Mat_<double>(p_hom_norm); //multiply per Kinv to normalize
		p_hom_norm.x = p_mat_norm(0); //reassign normalized homogenous coordinates
		p_hom_norm.y = p_mat_norm(1); // same
		p_hom_norm.z = p_mat_norm(2); //same

		//same with second im. points
		Point2f p1 = keypoints2[i].pt;
		Point3d p_hom_norm1(p1.x, p1.y, 1.0);
		Mat_<double> p_mat_norm1 = Kinv * Mat_<double>(p_hom_norm1);
		p_hom_norm1.x = p_mat_norm1(0);
		p_hom_norm1.y = p_mat_norm1(1);
		p_hom_norm1.z = p_mat_norm1(2);

		//correspondent 3D point to p and p1 image points
		/*
		 * linearLSTriangulation returns a X not in homogenous coordinates, since we made the
		 * assumption X was (x,y,z,1) (and so we had 3 X variables instead of 4);
		 * To compute re-projection error, X must be (x,y,z,1) because we multiply KP1*X
		 * (3x4)*(4*1). So we must append 1.0 to the calculated X from the function
		 */
		Mat_<double> X_homogenous(4,1); //X must be homogenous to compute KP1*X to get re-projection error
		Mat_<double> X_not_homogenous = linearLSTriangulation(p_hom_norm, P, p_hom_norm1, P1);
        X_homogenous(0) = X_not_homogenous(0);
        X_homogenous(1) = X_not_homogenous(1);
        X_homogenous(2) = X_not_homogenous(2);
        X_homogenous(3) = 1.0; //homogenous
		//cout<<"X= "<<X_homogenous<<endl; //show recovered point

		//cout<<endl;cout<<"calculating re-projection error: "<<endl;
		//cout<<"trying to multiply KP1 with size: "<<KP1.size()<<" and X with size: "<<X.size()<<endl;
		Mat_<double> X_proj = KP1 * X_homogenous;  //project 3D found point on image plane of the second image
		Point2f X_proj_(X_proj(0)/X_proj(2),X_proj(1)/X_proj(2)); //homogenize, X was assumed (x,y,z,1)
		double reprj_err = norm(X_proj_-p1); //we re-projected X using P1, so the correspondent point to use is p1
		reproj_error.push_back(reprj_err);
		//cout<<"rpj_err= "<<reprj_err<<endl; //show re-projection error

		pointcloud.push_back(Point3d(X_homogenous(0),X_homogenous(1),X_homogenous(2))); //store 3D point

	}

	//return mean reprojection error
	Scalar mean_error = mean(reproj_error);

	cout<<endl;cout<<"-------------------------------------------------------------------------------------------"<<endl;
	cout<<"Done ("<<pointcloud.size()<<" points, " <<" mean re-projection err = " << mean_error[0] << ")."<< endl;

	return mean_error[0];

}





