/*
 * FindCameraMatrices.cpp
 *
 *  Created on: 30/set/2016
 *      Author: trev
 */



#include "FindCameraMatrices.h"
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

/*
bool FindCameraMatrices::findCameraMatrices(){

}*/

//Rotation matrix's determinant  must be =+-1
bool FindCameraMatrices::CheckCoherentRotation(cv::Mat_<double>& R) {

	if(fabsf(determinant(R))-1.0 > 1e-07) {
		cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
		return false;
	}

	return true;
}

void FindCameraMatrices::alignMatches(vector<KeyPoint> keypoints_left,
		vector<KeyPoint> keypoints_right, vector<DMatch> matches,
		vector<KeyPoint>* imgpts1_tmp, vector<KeyPoint>* imgpts2_tmp){
	//put the matches found during OFMatching into two arrays;
	//the index in one array must correspond to the index in the other array
	//need to convert KeyPoint to Point2f to use findFundamentalMat()
	for(unsigned int i=0; i<matches.size(); i++){
		(*imgpts1_tmp).push_back(keypoints_left[ matches[i].queryIdx ] );
		(*imgpts2_tmp).push_back(keypoints_right[ matches[i].trainIdx ] );
	}

	//pts1, pts2 contain aligned correspondent points

}

void FindCameraMatrices::KeyPointsToPoints(vector<KeyPoint> keypoints, vector<Point2f>* points){

	//pay attention to pointer operations:
	//when you pass a pointer to a vector, doing (*pointer) you are
	//referencing the vector, and with (*pointer).field you access his fields
	for(unsigned int i=0; i < (keypoints).size(); i++){
		(*points).push_back( (keypoints)[i].pt );
	}

}

/*
 * differences with the book:
 * use two temporary vector<KeyPoint> in which align keypoints, then convert them to Point2f
 * and compute F ()N.B. using a threshold different than in the book)
 * and then use the status vector to "eliminate" points that were not used to compute F;
 * keep the new keypoints and matches in two new "good" vectors
 */
Mat FindCameraMatrices::getFundamentalMat(const vector<KeyPoint>& keypoints_left,
		const vector<KeyPoint>& keypoints_right, vector<KeyPoint>& keypoints_left_good,
		vector<KeyPoint>& keypoints_right_good, vector<DMatch>& matches, bool dense){

	//bool dense = true;
	cout<<"keypoints_left.size()= "<<keypoints_left.size()<<endl;
	cout<<"keypoints_right.size()= "<<keypoints_right.size()<<endl;
	cout<<"matches.size()= "<<matches.size()<<endl;

	//we will not consider points that were not used to compute F
	vector<uchar> status(keypoints_left.size());

	keypoints_left_good.clear(); keypoints_right_good.clear();

	//will contain aligned correspondent points
	//i point in pts1 corresponds to i point in pts2
	vector<Point2f> pts1;
	vector<Point2f> pts2;

	vector<KeyPoint> imgpts1_tmp;
	vector<KeyPoint> imgpts2_tmp;

	//if dese reconstruction, take directly two KeyPoint vectors, otherwise align the match vector
	//coming from sparse matching
	if(dense){
		imgpts1_tmp = keypoints_left;
		imgpts2_tmp = keypoints_right;
	}
	else
		alignMatches(keypoints_left, keypoints_right, matches, &imgpts1_tmp, &imgpts2_tmp);

	KeyPointsToPoints(imgpts1_tmp, &pts1);
	KeyPointsToPoints(imgpts2_tmp, &pts2);

    /*
	cout<<"first keypoint in match: "<<keypoints_left[matches[1].queryIdx].pt<<endl;
	cout<<"correspondent keypoint: "<<keypoints_right[matches[1].trainIdx].pt<<endl;
	cout<<"first point: "<<imgpts1_tmp[1].pt<<endl;
	cout<<"correspondent point"<<imgpts2_tmp[1].pt<<endl;
    */
	cout<<endl;cout<<"pts1 size: "<<pts1.size()<<"(orig pts: "<<matches.size()<<")"<<endl;;
	cout<<"pts2 size: "<<pts2.size()<<endl;

	double minVal,maxVal;
	cv::minMaxIdx(pts1,&minVal,&maxVal);
	//F=findFundamentalMat(pts1,pts2,FM_RANSAC,0.1,0.99, status);
	F = findFundamentalMat(pts1, pts2, FM_RANSAC, 0.006 * maxVal, 0.99, status); //threshold from [Snavely07 4.1]
	cout<<endl;cout<<"F= "<<F<<endl;

	vector<DMatch> new_matches;
	cout<<endl;cout << "F keeping " << countNonZero(status) << " / " << status.size() <<" points"<< endl;
    for(unsigned int i=0; i < status.size(); i++){
    	if(status[i]){
    		keypoints_left_good.push_back(imgpts1_tmp[i]);
    		keypoints_right_good.push_back(imgpts2_tmp[i]);
    		new_matches.push_back(matches[i]);
    		//TODO support dense reconstruction allowing matches[i] support, now it doesn't work
    	}
    }

    cout << matches.size() << " matches before, " << new_matches.size() << " new matches after Fundamental Matrix\n";
    matches = new_matches; //keep only those points who survived the fundamental matrix
    cout<<endl;cout<<endl;
    cout<<"FIRST PRUNED MATCH: matches[0].trainIdx= "<<matches[0].trainIdx<<endl; //debug
	cout<<endl;
    return F;
}

void FindCameraMatrices::SVDofE(Mat_<double>& E, Mat& svd_u, Mat& svd_vt, Mat& svd_w){
	//compute Singular Value Decomposition
	SVD svd(E,SVD::MODIFY_A);
	svd_u = svd.u;
	svd_vt = svd.vt;
	svd_w = svd.w;
	cout<<"SVD(E): "<<endl;
	cout << "U:\n"<<svd_u<<"\nW:\n"<<svd_w<<"\nVt:\n"<<svd_vt<<endl;cout<<endl;
}

bool FindCameraMatrices::getRTFromE(Mat_<double>& E,Mat_<double>& R1, Mat_<double>& R2,
		Mat_<double>& t1,Mat_<double>& t2){

	//HZ decomposition
	Mat svd_u, svd_vt, svd_w;
	SVDofE(E, svd_u, svd_vt, svd_w);

	//check if first and second singular values are the same (as they should be)
	double singular_values_ratio = fabsf(svd_w.at<double>(0) / svd_w.at<double>(1));
	if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
	if (singular_values_ratio < 0.7) {
		cout << "singular values are too far apart\n";
		return false;
	}

	Matx33d W(0,-1,0,	//HZ 9.13
			  1,0,0,
			  0,0,1);
	Matx33d Wt(0,1,0,
			  -1,0,0,
			   0,0,1);
	//two ways of computing both R ad t
	R1 = svd_u * Mat(W) * svd_vt; //HZ 9.19
	R2 = svd_u * Mat(Wt) * svd_vt; //HZ 9.19
	t1 = svd_u.col(2); //u3
	t2 = -svd_u.col(2); //u3

	return true;
}

//TODO add pointCloud
bool TestTriangulation(const vector<Point3d>& pcloud, const Matx34d& P, vector<uchar>& status) {
	//vector<Point3d> pcloud_pt3d = CloudPointsToPoints(pcloud);
	//vector<Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());
	vector<Point3d> pcloud_projected(pcloud.size());

	Matx44d P4x4 = Matx44d::eye();
	for(int i=0;i<12;i++) P4x4.val[i] = P.val[i];

	/*
	 * cv::operations on arrays
	 * take a 3D point of pcloud (x,y,z), transforms it using (x',y',z',w) = P4x4* (x,y,z,1)
	 * and then obtain 3D point pcloud_projected (x'/w, y'/w, z'/w)
	 */
	perspectiveTransform(pcloud, pcloud_projected, P4x4);

	status.resize(pcloud.size(),0);
	for (unsigned int i=0; i<pcloud.size(); i++) {
		status[i] = (pcloud_projected[i].z > 0) ? 1 : 0;
	}
	int count = countNonZero(status);

	double percentage = ((double)count / (double)pcloud.size());
	cout << count << "/" << pcloud.size() << " = " << percentage*100.0 << "% are in front of camera" << endl;
	if(percentage < 0.75)
		return false; //less than 75% of the points are in front of the camera

	//TODO includere questo insieme ai PointCloud
	//check for coplanarity of points
	if(false) //not
	{
		cv::Mat_<double> cldm(pcloud.size(),3);
		for(unsigned int i=0;i<pcloud.size();i++) {
			cldm.row(i)(0) = pcloud[i].x;
			cldm.row(i)(1) = pcloud[i].y;
			cldm.row(i)(2) = pcloud[i].z;
		}
		cv::Mat_<double> mean;
		cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);

		int num_inliers = 0;
		cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
		cv::Vec3d x0 = pca.mean;
		double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

		for (unsigned int i=0; i<pcloud.size(); i++) {
			Vec3d w = Vec3d(pcloud[i]) - x0;
			double D = fabs(nrm.dot(w));
			if(D < p_to_plane_thresh) num_inliers++;
		}

		cout << num_inliers << "/" << pcloud.size() << " are coplanar" << endl;
		if((double)num_inliers / (double)(pcloud.size()) > 0.85)
		    return false;
	}


	return true;
}

/*
 * difference with the book: more error checking, like matches.size(), E's determinant
 * R's determinant and flip; compute not only R and T but R1, R2 t1, t2 because there are
 * two ways to compute each rotation/translation component; test triangulation at the end of the
 * function and if it's not goodd then use R2 instead of R1
 */
bool FindCameraMatrices::findCameraMatrices(Mat K, Mat Kinv, vector<KeyPoint>& keypoints_left,
		vector<KeyPoint>& keypoints_right, vector<KeyPoint>& keypoints_left_good,
		vector<KeyPoint>& keypoints_right_good, Matx34d& P, Matx34d& P1, vector<DMatch>& matches,
		vector<Point3d>& outCloud, vector<CloudPoint>& p_cloud, bool dense,
		vector<double>& repr_err){

	cout<<endl;cout<<"computing camera matrices: "<<endl;

	Mat F = getFundamentalMat(keypoints_left, keypoints_right, keypoints_left_good,
			keypoints_right_good, matches, dense);

	if(matches.size() < 100) { // || ((double)imgpts1_good.size() / (double)imgpts1.size()) < 0.25
				cerr << "not enough inliers after F matrix" << endl;
				return false;
	}

	//Essential matrix: compute then extract cameras [R|t]
	Mat_<double> E = K.t() * F * K; //according to HZ (9.12)
	cout<<"E= "<<E<<endl;cout<<endl;

	//according to http://en.wikipedia.org/wiki/Essential_matrix#Properties_of_the_essential_matrix
	if(fabsf(determinant(E)) > 1e-07) {
		cout << "det(E) != 0 : " << determinant(E) << "\n";
		P1 = 0;
		return false;
	}

	Mat_<double> R1(3,3);
	Mat_<double> R2(3,3);
	Mat_<double> t1(1,3);
	Mat_<double> t2(1,3);

	//decompose E to obtain P1
	if (!getRTFromE(E,R1,R2,t1,t2))
		return false;

	if(determinant(R1)+1.0 < 1e-09) {
	    //according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
	    cout << "det(R) == -1 ["<<determinant(R1)<<"]: flip E's sign" << endl;
	    E = -E;
		getRTFromE(E,R1,R2,t1,t2);
	}

	if (!CheckCoherentRotation(R1)) {
		cout << "resulting rotation is not coherent\n";
		P1 = 0;
		return false;
	}

	cout<<"computing P1:"<<endl;
	// [R1|t1]
	P1 = Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t1(0),
				 R1(1,0),	R1(1,1),	R1(1,2),	t1(1),
	             R1(2,0),   R1(2,1),    R1(2,2),    t1(2));

	cout<<"P1= "<<P1<<endl;

	{
		/*
		 * there are four possible matrices for P1 (while P=[I|0]):
		 * P1=[R1|t1] or [R1|t2] or [R2|t1] or [R2|t2]
		 * just one of them is correct, that is the one which produces triangulated
		 * points in front of the camera (X(x,y,z,1) with z>1).
		 * for every combination of P1, check if triangulated points are in front and, if not,
		 * change P1 and retry.
		 */
	}

	cout<<"testing P1: "<<endl;
	vector<Point3d> pointcloud, pointcloud1;
	vector<CloudPoint> pcloud, pcloud1;
	Triangulation triangulator;  //TODO distruggere triangulator
	double reproj_error1 = triangulator.triangulatePoints(keypoints_left_good, keypoints_right_good, K, Kinv, P, P1, pointcloud, pcloud, repr_err);
	double reproj_error2 = triangulator.triangulatePoints(keypoints_right_good, keypoints_left_good, K, Kinv, P1, P, pointcloud1, pcloud1, repr_err);

	vector<uchar> tmp_status;
	int index=1;
    //only one P1 of the 4 possible is correct, that is the one which produces points in front of the camera (z>0)
	if (!TestTriangulation(pointcloud,P1,tmp_status) || !TestTriangulation(pointcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {

		cout<<"[R1|t1] was incorrect"<<endl;
		//[R1|t2]
		P1 = Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t2(0),
					 R1(1,0),	R1(1,1),	R1(1,2),	t2(1),
					 R1(2,0),	R1(2,1),	R1(2,2),	t2(2));

	    cout<<endl;cout << "Testing P1 "<< endl << Mat(P1) << endl;
	    index=2;
	    pointcloud.clear(); pointcloud1.clear();
	    pcloud.clear(); pcloud1.clear();
	    //retry triangulation with new P1
	    double reproj_error1 = triangulator.triangulatePoints(keypoints_left_good, keypoints_right_good, K, Kinv, P, P1, pointcloud, pcloud, repr_err);
	    double reproj_error2 = triangulator.triangulatePoints(keypoints_right_good, keypoints_left_good, K, Kinv, P1, P, pointcloud1, pcloud1, repr_err);
	    if (!TestTriangulation(pointcloud,P1,tmp_status) || !TestTriangulation(pointcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {

	    	cout<<"[R1|t2] was incorrect"<<endl;
	    	//new rotation matrix R2 may be incorrect
	    	if (!CheckCoherentRotation(R2)) {
	    		cout << "resulting rotation is not coherent\n";
	    		P1 = 0;
	    		return false;
	    	}

	    	//[R2|t1]
	    	P1 = Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t1(0),
	    				 R2(1,0),	R2(1,1),	R2(1,2),	t1(1),
	    				 R2(2,0),	R2(2,1),	R2(2,2),	t1(2));

	    	cout << "\nTesting P1 "<< endl << Mat(P1) << endl;
	    	index=3;
	    	pointcloud.clear(); pointcloud1.clear();
	    	pcloud.clear(); pcloud1.clear();
	        //retry triangulation with new P1
	        double reproj_error1 = triangulator.triangulatePoints(keypoints_left_good, keypoints_right_good, K, Kinv, P, P1, pointcloud, pcloud, repr_err);
	    	double reproj_error2 = triangulator.triangulatePoints(keypoints_right_good, keypoints_left_good, K, Kinv, P1, P, pointcloud1, pcloud1, repr_err);
	    	if (!TestTriangulation(pointcloud,P1,tmp_status) || !TestTriangulation(pointcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {

	    		cout<<"[R2|t1] was incorrect"<<endl;
	    		//[R2|t2]
	    		P1 = Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t2(0),
	    					 R2(1,0),	R2(1,1),	R2(1,2),	t2(1),
	    			         R2(2,0),	R2(2,1),	R2(2,2),	t2(2));

	    		cout << "\nTesting P1 "<< endl << Mat(P1) << endl;
	    		index=4;
	    		pointcloud.clear(); pointcloud1.clear();
	    		pcloud.clear(); pcloud1.clear();
	    	    double reproj_error1 = triangulator.triangulatePoints(keypoints_left_good, keypoints_right_good, K, Kinv, P, P1, pointcloud, pcloud, repr_err);
	    		double reproj_error2 = triangulator.triangulatePoints(keypoints_right_good, keypoints_left_good, K, Kinv, P1, P, pointcloud1, pcloud1, repr_err);
	    		if (!TestTriangulation(pointcloud,P1,tmp_status) || !TestTriangulation(pointcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
	    			cout<<"[R2|t2] was incorrect"<<endl;
	    			cout << "All four matrices incorrect" << endl;
	    			return false;
	    		}
	    	} //[R2|t2]
	    } //[R2|t1]
	} //[R1|t2]

	cout<<endl;cout<<"correct matrix was "<<index<<" type";
	cout<<" (1 corresponds to [R1|t1], 4 to [R2|t2] )"<<endl;
	cout<<endl;cout<<"matrix P1= "<<P1<<endl;

	//cout<<"re-projection error= "<<reproj_error1<<endl;

	//store found 3D points
	for (unsigned int i=0; i<pointcloud.size(); i++) {
		outCloud.push_back(pointcloud[i]);
	}

	//same, but store in CloudPoint (for multiview reconstruction)
	for(unsigned int j=0; j < pcloud.size(); j++){
		p_cloud.push_back(pcloud[j]);
	}
	cout<<endl;cout<<"CloudPoint now has "<<p_cloud.size()<<" 3D points"<<endl;
	     //SOLVED BUG-->needed to clear() CloudPoints vector after failed matrix try
	     //cout<<"WHATCHOUT! it is exactly the number of pruned mathes * 3 ! why?"<<endl;

	return true;
}




















