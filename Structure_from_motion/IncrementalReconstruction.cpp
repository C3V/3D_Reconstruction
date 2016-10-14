/*
 * IncrementalReconstruction.cpp
 *
 *  Created on: 08/ott/2016
 *      Author: trev
 */

#include "IncrementalReconstruction.h"
#include "FindCameraMatrices.h"
#include "Visualization.h"



vector<DMatch> IncrementalReconstruction::flipMatches(vector<DMatch>& matches){

	vector<cv::DMatch> flip;
    for(unsigned int i=0;i<matches.size();i++) {
	    flip.push_back(matches[i]);
		swap(flip.back().queryIdx,flip.back().trainIdx);
	}
	return flip;

}


void alignMatches(vector<KeyPoint> keypoints_left,
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

void KeyPointsToPoints(vector<KeyPoint> keypoints, vector<Point2f>* points){

	//pay attention to pointer operations:
	//when you pass a pointer to a vector, doing (*pointer) you are
	//referencing the vector, and with (*pointer).field you access his fields
	for(unsigned int i=0; i < (keypoints).size(); i++){
		(*points).push_back( (keypoints)[i].pt );
	}

}


bool TestTriangulation1(const vector<Point3d>& pcloud, const Matx34d& P, vector<uchar>& status) {
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


//Rotation matrix's determinant  must be =+-1
bool CheckCoherentRotation(cv::Mat_<double>& R) {

	if(fabsf(determinant(R))-1.0 > 1e-07) {
		cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
		return false;
	}

	return true;
}

void IncrementalReconstruction::displayImage(Mat image, String name){
	namedWindow(name, WINDOW_AUTOSIZE);
	//to resize, but remember to change to WINDOW_NORMAL WINDOW_AUTOSIZE
	resizeWindow(name, 500,400);
	imshow(name, image);
	waitKey(0);
}

void IncrementalReconstruction::displayIMatches(Mat img1, vector<KeyPoint> k1, Mat img2, vector<KeyPoint> k2,
        vector<DMatch> matches){

	Mat img_matches;
	drawMatches(img1, k1, img2, k2, matches, img_matches, Scalar(0,255));
	displayImage(img_matches, "Matches");
}

void IncrementalReconstruction::constructMatchesMatrix(vector<Mat> imgs, OpticalFlowMatcher matcher,
		                                       map< pair<int,int>,vector<DMatch> >& matches_matrix,
											   vector<vector< KeyPoint> >& keypoints_vector,
											   int& max_index1, int& max_index2){

	int index1 = imgs.size()-1;
	int index2 = imgs.size();
	unsigned int highest_match=0;

	cout<<endl;cout<<"matching images keypoints ..."<<endl;
	cout<<"(constructing matches_matrix and keypoints_vector)"<<endl;

	/*
	 * matching on frames (0,1), (0,2),...,(0,n), and then
	 * matching on frames (1,2), (1,3),...,(1,n)
	 * ...
	 * matching on frames (n-1, n)
	 */
	//vector<KeyPoint> keypoints1, keypoints2;
	for(int i=0; i < index1; i++){

		for(int j=i+1; j < index2; j++){

			vector<DMatch> matches_tmp; //matching vector between img[index1] and img[index2]
			//(index1, index2)
			matches_tmp = matcher.matchFeatures(imgs[i], imgs[j], &matches_tmp);
			//keypoints1 = matcher.getKeypoints_1(); //to display
			//keypoints2 = matcher.getKeypoints_2(); //to display

			matches_matrix[make_pair(i, j)] = matches_tmp; //add to matrix

			if(j == index2-1){ //n
				keypoints_vector[j] = matcher.getKeypoints_2();
			}
			keypoints_vector[i] = matcher.getKeypoints_1();

			//(index2, index1)
			vector<DMatch> matches_tmp_flip = flipMatches(matches_tmp);
			matches_matrix[make_pair(j, i)] = matches_tmp_flip;

			//store the highest matching couple
            if(matches_tmp.size() > highest_match){
            	highest_match = matches_tmp.size();
            	max_index1 = i; max_index2 = j;
            }

            cout<<matches_tmp.size()<<" matches between img "<<i<<" and img "<<j<<endl;
            matcher.clearFields();

		} //j

		//keypoints_vector[i] = matcher.getKeypoints_1();

	} //i

	cout<<"done."<<endl;
	cout<<"matches_matrix has now "<<matches_matrix.size()<<" DMatch vectors"<<endl;
	cout<<"(every (i,j) and (j,i) location of matrix represents the matching vector between img i and img j)"<<endl;
	cout<<"keypoints_vector.size()= "<<keypoints_vector.size()<<endl;
	cout<<"keypoints_vector[0].size()= "<<keypoints_vector[0].size()<<endl; //debug
	cout<<"highest matching was "<<highest_match<<" from img "<<max_index1<<" and img "<<max_index2<<endl;

	//display matches between first and second image (debug)
	/*
	cout<<endl;cout<<"displaying incremental matches ..."<<endl;
	vector<DMatch> i_matches = matches_matrix[make_pair(0,1)];
	displayIMatches(imgs[0], keypoints1, imgs[1], keypoints2, i_matches);
	cout<<"done."<<endl;

	cout<<endl;cout<<"displaying incremental matches ..."<<endl;
	vector<DMatch> i_matches1 = matches_matrix[make_pair(1,0)];
	displayIMatches(imgs[1], keypoints2, imgs[0], keypoints1, i_matches1);
	cout<<"done."<<endl;
	*/


}

//put KeyPoints in imgpts1 and imgpts2--aligning function
//!!!no need to use it, since getFundamentalMat() aligns the keypoints vectors by herself!!!
void IncrementalReconstruction::getKeyPointsFromMatches(vector<KeyPoint>& imgpts1,
		vector<KeyPoint>& imgpts2, map< pair<int,int>,vector<DMatch> > matches_matrix,
		int older_view, int working_view, vector<vector<KeyPoint> > keypoints_vector){

	vector<DMatch> match_tmp; //current matches between old and working
    match_tmp = matches_matrix[make_pair(older_view, working_view)];
    cout<<endl;cout<<"match_tmp.size()= "<<match_tmp.size()<<endl; //debug

    for(unsigned int i = 0; i < match_tmp.size(); i++){
    	imgpts1.push_back( keypoints_vector[older_view] [ match_tmp[i].queryIdx ] );
    	imgpts2.push_back( keypoints_vector[working_view] [ match_tmp[i].trainIdx ]);
    }
    //now imgpts1 is the vector<KeyPoint> of older_view image
    //imgpts2 is the vector<KeyPoint> of working_view image

}

//matches_matrix is not modified! lookout!
//if I put & matches_matrix, getBaseLineTriang() doesn't work!
void IncrementalReconstruction::pruneMatchesBasedOnF(vector<Mat> imgs,
		                                             vector<vector<KeyPoint> > keypoints_vector,
													 map< pair<int,int>,vector<DMatch> > matches_matrix,
													 bool dense, FindCameraMatrices f){

	cout<<endl;cout<<"pruning matches by computing F for every match vector ..."<<endl;
	cout<<"matches_matrix[make_pair(0,1)].size()= "<<matches_matrix[make_pair(0,1)].size()<<" matches before pruning"<<endl;
	vector<vector<KeyPoint> > imgpts_good(imgs.size());
	//FindCameraMatrices f;

	//iterate over the images
    for(unsigned int i=0; i < imgs.size()-1; i++){ //0...n-1

    	for(unsigned int j=i+1; j < imgs.size(); j++){ //1...n
    		int older_view = i;
    		int working_view = j;

    		cout<<"pruning matches between img "<<i<<" and img "<<j<<" ..."<<endl;

    		vector<KeyPoint> kpts_left, kpts_right; //store keypoints from DMatch

    		/*//no need to use it, aligning is already implemented in getFundamentalMat()
    		getKeyPointsFromMatches(kpts_left, kpts_right, matches_matrix, older_view, working_view,
    				keypoints_vector);
    		*/
    		kpts_left = keypoints_vector[older_view];
    		kpts_right = keypoints_vector[working_view];

    		/*
    		 * getFundamentalMat() updates DMatch vector between older view and working view, keeping
    		 * only the matches retained by F computation. imgpts_good changes at every execution
    		 * but it's cleared in the successive one (see line " matches=new_matches; " line in
    		 * FindCameraMatrices.cpp for details), while the DMatch vector
    		 * matches_matrix[make_pair(0,1)] (for example) is modified forever.
    		 */
    		f.getFundamentalMat(kpts_left, kpts_right, imgpts_good[older_view],
    				imgpts_good[working_view], matches_matrix[make_pair(older_view,working_view)],
				dense);

    		//modify also the specular DMatch vector between working_view and older_view
            //using the found new DMatch vector
    		matches_matrix[make_pair(working_view,older_view)] = flipMatches(matches_matrix[make_pair(older_view,working_view)]);

    	} //j

    } //i

    cout<<endl;cout<<"now matches_matrix map contains only the pruned matches"<<endl;
    cout<<"matches_matrix[0,1].size()= "<<matches_matrix[make_pair(0,1)].size()<<" after pruning"<<endl; //example
    cout<<"matches_matrix[1,0].size()= "<<matches_matrix[make_pair(1,0)].size()<<endl;

}


bool IncrementalReconstruction::getBaseLineTriangulation(vector<Mat> imgs,
		                             map< pair<int,int>,vector<DMatch> > matches_matrix,
									 vector<vector<KeyPoint> > keypoints_vector,
									 int max_index1, int max_index2, FindCameraMatrices f,
									 Mat K, bool dense){

	 cout<<endl;cout<<"***********************************************************"<<endl;
	            cout<<"*****************Baseline Triangulation********************"<<endl;
	            cout<<"***********************************************************"<<endl;

/*	cout<<endl;cout<<"help-->remember to use a baseline which 'make sense': \n"
			"if the cloud you obtain as a baseline is composed by 3D points that \n"
			"have erroneous coordinates (see Visualization.cpp), although their reprojection\n"
			"errors are acceptable, everything will never make sense.\n"
			"In images 7.1, 7.2 and 7.3 the baseline is obtained by 7.2,7.3 but if you look at\n"
			"the reconstruction from two views, there is only ONE 3D point with correct\n "
			"coordinates!"<<endl;
*/
	//Pmats.clear();

	Matx34d P(1, 0, 0, 0, //[I|0]
	          0, 1, 0, 0,
	          0, 0, 1, 0
	    	  );
	Matx34d P1;

	cout<<endl;
	cout<<"[using the two images with highest matching between them] "<<endl;
    //remember that if triangulation does not succeed you have to pick another couple!
	    //Mat firstImage = imgs[max_index1];
	    //Mat secondImage = imgs[max_index2];

	vector<KeyPoint> keypoints_left, keypoints_right, keypoints_left_good, keypoints_right_good;
	keypoints_left = keypoints_vector[max_index1];
	keypoints_right = keypoints_vector[max_index2];

	bool goodF = false; //good F matrix
	Mat Kinv(K.inv()); //inverse

	vector<Point3d> outCloud;
	//vector<CloudPoint> p_cloud;
	vector<double> reproj_err;

	//recovering of P1 + triangulation of pruned points
	goodF = f.findCameraMatrices(K, Kinv, keypoints_left, keypoints_right, keypoints_left_good,
	           keypoints_right_good, P, P1, matches_matrix[make_pair(max_index1, max_index2)],
	             outCloud, p_cloud, dense, reproj_err);

	if(goodF){ //P1 recovering and triangulation succeeded

	//cout<<endl;cout<<"filtering points with high re-projection error (>8.0)"<<endl;
	int older_view = max_index1; int working_view = max_index2;
	vector<DMatch> matches = matches_matrix[make_pair(older_view,working_view)]; //pruned by F

	//populate the "imgpt_for_img" field of CloudPoints with the two views older_view and working
	cout<<endl;cout<<"populating 'imgpt_for_img' field of the CloudPoint baseline ..."<<endl;
	for(unsigned int j=0; j < p_cloud.size(); j++){

	   	p_cloud[j].imgpt_for_img = vector<int>(imgs.size(), -1); //initialize with "illegal" values
	    p_cloud[j].imgpt_for_img[older_view] =  matches[j].queryIdx;
	    p_cloud[j].imgpt_for_img[working_view] =  matches[j].trainIdx;
        //(ogni pto 3D ha l'indice, relativo al vettore di keypoint di ogni immagine, dei punti che
	    // lo rappresentano su ognuno dei piani immagine delle viste)

	}

	//debug
	cout<<endl;cout<<"FIRST CLOUDPOINT: p_cloud[0].imgpt_for_img[working_view]= "<<p_cloud[0].imgpt_for_img[working_view]<<endl;
	cout<<"(must be equal to matches[0].trainIdx)"<<endl;

    cout<<endl;cout<<"filtering points with high re-projection error (>8.0)"<<endl;
	vector<CloudPoint> new_p_cloud;
	vector<DMatch> new_matches;

	for(unsigned int i=0; i < p_cloud.size(); i++){
		if(p_cloud[i].reprojection_error > 8.0){
			continue; //discard point;
		}
		else{
			new_p_cloud.push_back(p_cloud[i]);
			new_matches.push_back(matches[i]);
		}
	}
	cout<<"discarded "<<p_cloud.size() - new_p_cloud.size()<<" points"<<endl;

	if(new_p_cloud.size()<=0){
		cout<<"filtered out all the recovered points!"<<endl;
	}

	//update the originals
	p_cloud = new_p_cloud;
	matches = new_matches;
	matches_matrix[make_pair(older_view,working_view)] = new_matches; //not really necessary?
	matches_matrix[make_pair(working_view,older_view)] = flipMatches(new_matches); //update the specular

    //cout<<"p_cloud[0].imgpt_for_img[0]= "<<p_cloud[0].imgpt_for_img[0]<<endl; //debug
    cout<<endl;cout<<"now there is a 3D baseline for recovering more cameras incrementally,"<<endl;
    cout<<"and every 3D point 'knows' the index of his corresponding projection"<<endl;
    cout<<"on the older_view and working_view image planes"<<endl;
    cout<<"************************************************************************"<<endl;

	 //if(goodF)

	//if(goodF){
		cout<<endl;cout<<"(reconstructed "<<outCloud.size()<<" points out of "<<keypoints_left_good.size()<<")"<<endl;

		//store recovered matrices
		cout<<"P and P1 matrices correctly recovered"<<endl;
		Pmats[older_view] = P;
		Pmats[working_view] = P1;

		//store done and good views
		done_views.insert(older_view); done_views.insert(working_view);
		good_views.insert(older_view); good_views.insert(working_view);
		cout<<"done_views.size()= "<<done_views.size()<<", good_views.size() "<<good_views.size()<<endl;

/*		//extracting recovered 3D points from CloudPoint struct
		vector<Point3d> outCloud1;
		for(unsigned int i=0; i < p_cloud.size(); i++){
			outCloud1.push_back( p_cloud[i].pt );
		}

		Visualization1 display;
		cout<<endl;cout<<"visualizing CloudPoint results ..."<<endl;
		//display.visualize3DPoints(outCloud, P, P1, K, reproj_err);
		display.visualize3DPoints(outCloud1, P, P1, K, reproj_err); */
	//}
	//else{
	//	cout<<"baseline recovering with highest matching images failed"<<endl;
	//}

		return true;
	} //if(goodF)
	else{
		cout<<"baseline recovering with highest matching images failed"<<endl;
		return false;
	}

}


void IncrementalReconstruction::find3D2DCorrespondences(vector<Point3f>& ppcloud,
		                                                vector<Point2f>& imgPoints,
														int working_view,
														map< pair<int,int>,vector<DMatch> > matches_matrix,
														vector<vector< KeyPoint> > keypoints_vector){

	cout<<"finding 3D-2D correspondences ..."<<endl;
	vector<int> pcloud_status(p_cloud.size(),0); //to avoid duplicates

	//test the 3D-2D matching on the good_views
	for (set<int>::iterator done_view = good_views.begin();
		                    done_view != good_views.end(); ++done_view){

		int old_view = *done_view;
		//match vector between already used view and current new one
		vector<DMatch> matches_from_old_to_working = matches_matrix[make_pair(old_view, working_view)];

		for(unsigned int match_index=0; match_index < matches_from_old_to_working.size(); match_index++){

			int old_view_index = matches_from_old_to_working[match_index].queryIdx;

			//check if it was used to obtain a 3D point in the p_cloud
		    for(unsigned int j=0; j < p_cloud.size(); j++){

		    	if(old_view_index == p_cloud[j].imgpt_for_img[old_view] &&
		    			pcloud_status[j] == 0){ //prevent duplicates

		    		ppcloud.push_back(p_cloud[j].pt);
		    		imgPoints.push_back(keypoints_vector[working_view][matches_from_old_to_working[match_index].trainIdx].pt);
                    pcloud_status[j] = 1; //this point it found ONCE AND FOR ALL
                    break;

		    	}

		    }

		}

	}

	cout<<"found "<<ppcloud.size()<<" correspondences for img "<<working_view<<endl;

}


bool IncrementalReconstruction::findPoseEstimation(int working_view, Mat_<double>& rvec,
		                                           Mat_<double>& t, Mat_<double>& R,
		                                           vector<Point3f> ppcloud,
	                                               vector<Point2f> imgPoints, Mat K,
												   Mat distortion_coeff){

	if(ppcloud.size() <= 7 || imgPoints.size() <= 7 || ppcloud.size() != imgPoints.size()) {
		//something went wrong aligning 3D to 2D points..
		cerr << "couldn't find [enough] corresponding cloud points... (only " << ppcloud.size() << ")" <<endl;
		return false;
	}

	vector<int> inliers;
	double minVal,maxVal; minMaxIdx(imgPoints,&minVal,&maxVal);

	//use initial extrinsic guess for tvec and rvec
	solvePnPRansac(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, 1000, 0.006 * maxVal, 0.25 * (double)(imgPoints.size()), inliers, CV_EPNP);

	//TODO check parameters in function!
	//solvePnPRansac(ppcloud, imgPoints, K, distortion_coeff, rvec, t, false, 1000, CV_ITERATIVE); //or CV_EPNP

	vector<Point2f> projected3D;
	projectPoints(ppcloud, rvec, t, K, distortion_coeff, projected3D);

	if(inliers.size()==0) { //get inliers

		for(unsigned int i=0;i<projected3D.size();i++) {
			if(norm(projected3D[i]-imgPoints[i]) < 10.0)
				inliers.push_back(i);
		}

	}

	if(inliers.size() < (double)(imgPoints.size())/5.0) {
		cerr << "not enough inliers to consider a good pose ("<<inliers.size()<<"/"<<imgPoints.size()<<")"<< endl;
		return false;
	}

	if(norm(t) > 200.0) {
		// this is bad...
		cerr << "estimated camera movement is too big, skip this camera\r\n";
		return false;
	}

	Rodrigues(rvec, R);

	if(!CheckCoherentRotation(R)) {
		cerr << "rotation is incoherent. we should try a different base view..." << endl;
		return false;
	}

	std::cout << "found t = " << t << "\nR = \n"<<R<<std::endl;
	return true;

}


bool IncrementalReconstruction::triangulatePointsBetweenViews(vector<Mat> imgs,
		                       int working_view, int older_view,
		                       vector<CloudPoint>& new_triangulated, vector<int>& add_to_cloud,
							   vector<vector<KeyPoint> > keypoints_vector,
							   map< pair<int,int>,vector<DMatch> > matches_matrix, Mat K,
							   Triangulation t){

	cout<<endl;cout<<"triangulating "<<working_view<<" view and "<<older_view<<" previous good view"<<endl;

	Matx34d P = Pmats[older_view];
	Matx34d P1 = Pmats[working_view];

	Mat Kinv(K.inv());

	vector<KeyPoint> pt_set1,pt_set2;
	vector<DMatch> matches = matches_matrix[make_pair(older_view,working_view)];

	//like in FindCameraMatrices.cpp , see getFundamentalMat()
	cout<<endl;cout<<"aligning the two keypoints vectors using matches"<<endl;
	alignMatches(keypoints_vector[older_view],keypoints_vector[working_view],
			     matches, &pt_set1, &pt_set2);

	//now pt_set1 and pt_set2 have the "aligned" correspondent keypoints
	//I could now proceed to triangulate, but first must prune matches computing F
	//(like in getFundamentalMat() to obtain left and right keypoints_good, which will be passed to
	//triangulation)

	vector<Point2f> pts1;
	vector<Point2f> pts2;

	KeyPointsToPoints(pt_set1, &pts1);
	KeyPointsToPoints(pt_set2, &pts2);
	vector<uchar> status(keypoints_vector[older_view].size());

	double minVal,maxVal;
	minMaxIdx(pts1,&minVal,&maxVal);
    //F=findFundamentalMat(pts1,pts2,FM_RANSAC,0.1,0.99, status);
	Mat F = findFundamentalMat(pts1, pts2, FM_RANSAC, 0.006 * maxVal, 0.99, status); //threshold from [Snavely07 4.1]
	cout<<endl;cout<<"F= "<<F<<endl;

	vector<KeyPoint> keypoints_left_good, keypoints_right_good; //store pruned keypoints

	vector<DMatch> new_matches_;
	cout<<endl;cout << "F keeping " << countNonZero(status) << " / " << status.size() <<" points"<< endl;
    for(unsigned int i=0; i < status.size(); i++){
	    if(status[i]){
	    	keypoints_left_good.push_back(pt_set1[i]);
	    	keypoints_right_good.push_back(pt_set2[i]);
	    	new_matches_.push_back(matches[i]);
	    }
	}

    cout << pt_set1.size() << " matched keypoints before, " << keypoints_left_good.size() << " keypoints after Fundamental Matrix"<<endl;


    if(new_matches_.size() < 100) { // || ((double)imgpts1_good.size() / (double)imgpts1.size()) < 0.25
    				cerr << "not enough inliers after F matrix using views "<<working_view<<","<<older_view<<endl;
    				return false;
    	}

    cout<<"pruning with F done, now triangulate :"<<endl;

    vector<Point3d> pointcloud; //useless now, it was when not incremental
    vector<double> repr_err;

    double reproj_error = t.triangulatePoints(keypoints_left_good, keypoints_right_good, K, Kinv,
    		                                P, P1, pointcloud, new_triangulated, repr_err);

    cout << "triangulation reproj error " << reproj_error << endl;

    vector<uchar> trig_status;
    if(!TestTriangulation1(pointcloud, P, trig_status) || !TestTriangulation1(pointcloud, P1, trig_status)) {
    	cerr << "Triangulation did not succeed (failed TestTriangulation() )" << endl;
    	return false;
    }

    //filter out outlier points with high reprojection
    vector<double> reprj_errors;
    for(unsigned int i=0;i<new_triangulated.size();i++) { reprj_errors.push_back(new_triangulated[i].reprojection_error); }
    sort(reprj_errors.begin(),reprj_errors.end());
    //get the 80% precentile
    double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4; //threshold from Snavely07 4.2
    cout<<"reprojection error cutoff (Snavely)= "<<reprj_err_cutoff<<endl;

    vector<CloudPoint> new_triangulated_filtered;
    vector<DMatch> new_matches;
    for(unsigned int i=0;i<new_triangulated.size();i++) {
    	if(trig_status[i] == 0)
    		continue; //point was not in front of camera
    	if(new_triangulated[i].reprojection_error > 16.0) {
    		continue; //reject point
    	}
    	if(new_triangulated[i].reprojection_error < 10.0 ||
    		new_triangulated[i].reprojection_error < reprj_err_cutoff)
    	{
    		new_triangulated_filtered.push_back(new_triangulated[i]);
    		new_matches.push_back(matches[i]);
    	}
    	else
    	{
    		continue;
    	}
    }

    cout<<"filtered out " <<(new_triangulated.size() - new_triangulated_filtered.size())<<" high-error points"<<endl;
    //all points filtered?
    if(new_triangulated_filtered.size() <= 0)
    	return false;

    new_triangulated = new_triangulated_filtered;
    matches = new_matches;
    matches_matrix[make_pair(older_view,working_view)] = new_matches; //just to make sure, remove if unneccesary
    matches_matrix[make_pair(working_view,older_view)] = flipMatches(new_matches);

    add_to_cloud.clear();
   	add_to_cloud.resize(new_triangulated.size(),1);

   	int found_other_views_count = 0;
    int num_views = imgs.size();

    cout<<endl;cout<<"scanning the 3D cloud to see if a recovered new 3D point was already found ..."<<endl;

    /*
     *                                   ! N.B. !
     * you can't just confront two 3D points since they are recovered using a noisy
     * triangulation, so the same point will probably be a little bit different in two
     * different reconstructions. You must confront index vectors instead!
     */
    for(unsigned int j=0; j < new_triangulated.size(); j++){

    	new_triangulated[j].imgpt_for_img = vector<int>(imgs.size(), -1); //initialize
    	new_triangulated[j].imgpt_for_img[older_view] = matches[j].queryIdx;
    	new_triangulated[j].imgpt_for_img[working_view] = matches[j].trainIdx;

    	bool found_in_other_view = false;

    	for(int view_=0; view_ < num_views; view_++){

    		if(view_ != older_view){ //TODO anche view_ ! working view?

    			vector<DMatch> submatches = matches_matrix[make_pair(view_, working_view)];

    			for(unsigned int ii=0; ii< submatches.size(); ii++){

    				if(submatches[ii].trainIdx == matches[j].trainIdx && !found_in_other_view){

    					//the i-rd 3D point is also a 2D keypoint in view_
    					//now check if it was already triangulated:
    					for(unsigned int p=0; p < p_cloud.size(); p++){

    						if(p_cloud[p].imgpt_for_img[view_] == submatches[ii].queryIdx){

    							//i-rd 3D point was already triangulated!

    							//set imgpt_for_img field (useful?)
    							p_cloud[p].imgpt_for_img[older_view] = matches[j].queryIdx;
    							p_cloud[p].imgpt_for_img[working_view] = matches[j].trainIdx;

    							found_in_other_view = true;
    							add_to_cloud[j] = 0; //no need to re-add the same point
    						}

    					} //for( p )

    				} //if()

    			} //for( ii )

    		} //if()

    	} // for( view_ )

    	if( found_in_other_view){ //point already triangulated

    		found_other_views_count++;

    	}
    	else{

    		add_to_cloud[j] = 1; //point must be added to baseline 3D cloud

    	}

    } // for( j )

    cout<<found_other_views_count<<"/"<<new_triangulated.size()<<" points were already triangulated"<<endl;
    cout<<"adding "<<countNonZero(add_to_cloud)<<" new points to 3D cloud"<<endl;

    return true;
}



//**************************************************************************************************
//***************************[    recover3D() takes all the work    ]*******************************
//**************************************************************************************************


//"main" function
void IncrementalReconstruction::recover3D(vector<Mat> imgs, OpticalFlowMatcher matcher, bool dense,
		                                  Mat K, Mat distortion_coeff){

    cout<<endl;cout<<"***************************************************************************************"<<endl;
	           cout<<"****************************recovering 3D points ...***********************************"<<endl;
               cout<<"***************************************************************************************"<<endl;

    vector<vector< KeyPoint> > keypoints_vector(imgs.size() ); //stores keypoints for every view
    map< pair<int,int>,vector<DMatch> > matches_matrix; //stored DMatch vecs for every (i,j)

    FindCameraMatrices f;

    int max_index1, max_index2; //imgs indexes of the highest matching couple

    cout<<endl;cout<<"[constructMatchesMatrix() executes ...]"<<endl;
    constructMatchesMatrix(imgs, matcher, matches_matrix, keypoints_vector, max_index1, max_index2);

    //cout<<endl;cout<<"[not using pruneMatchesBasedOnF(), pruning only using findFundamentalMat()]"<<endl;
    //cout<<"(it seems to filter out too many points computing F two times)"<<endl;
    //pruneMatchesBasedOnF(imgs, keypoints_vector, matches_matrix, dense, f); //discard points not used in F computation

    cout<<endl;cout<<"[getBaseLineTriangulation() executes ...]"<<endl;
    bool ok = false;
    ok = getBaseLineTriangulation(imgs, matches_matrix, keypoints_vector, max_index1, max_index2, f, K,
    		                 dense); //first triangulation to get initial 3D cloud

    if(!ok){ //failed with that couple of views, try the first two (TODO work it out)

        ok = getBaseLineTriangulation(imgs, matches_matrix, keypoints_vector, 0, 1, f, K,
    	    		                 dense);

    }

    //now I have the p_cloud OutCloud vector and the corresponding indexes of 2d first and
    //second images (the two used for the baseline triangulation), and the DMatch vector for
    //every couple of the three frames. Now:
    //  ->take the highest 3D-2D matching new image, recover his P1 and triangulate with
    //    all the other already used views
    //  ->repeat until there are no more new views

    cout<<endl;cout<<"first two camera matrices recovered:"<<endl;
    cout<<"Pmats[max_index1]= "<<Pmats[max_index1]<<endl;
    cout<<"Pmats[max_index2]= "<<Pmats[max_index2]<<endl;

    Mat_<double> t;
    Mat_<double> rvec;
    Mat_<double> R;
    Mat_<double> t0, R0;

    map< int , Mat_<double> > camera_Rs;
    map< int , Mat_<double> > camera_ts;

    Matx34d P = Pmats[max_index1];
    Matx34d P1 = Pmats[max_index2];

    //for initial guess in camera motion estimation
    t0 = Mat_<double>(1,3) << P(0,3), P(1,3), P(2,3);
    R0 = Mat_<double>(3,3) << P(0,0), P(0,1), P(0,2),
        			          P(1,0), P(1,1), P(1,2),
                              P(2,0), P(2,1), P1(2,2);

    t = Mat_<double>(1,3) << P1(0,3), P1(1,3), P1(2,3);
    R = Mat_<double>(3,3) << P1(0,0), P1(0,1), P1(0,2),
    			             P1(1,0), P1(1,1), P1(1,2),
                             P1(2,0), P1(2,1), P1(2,2);

    camera_Rs[max_index1] = R0; camera_Rs[max_index2] = R;
    camera_ts[max_index1] = t0; camera_ts[max_index2] = t;

    vector<Mat_<double> > R_, t_;

    cout<<endl;cout<<"[ loop for adding more points to the baseline cloud starts :]"<<endl;

    while(done_views.size() != imgs.size()){ //cycle until there are no more images to try

    	unsigned int max_3D2DCorr = 0;
	    int max_corr_view = -1;
	    vector<Point3f> max_3D_corr;
	    vector<Point2f> max_2D_corr;

    	//find image with highest 3D-2D correspondence
	    cout<<endl;cout<<"finding view with max number of 3D-2D correspondences"<<endl;
    	for(unsigned int i=0; i < imgs.size(); i++){

    		if(done_views.find(i) != done_views.end()){

    			continue; //already done with this view, skip

    		}

            //else...
    		vector<Point3f> ppcloud; //3D points
    	    vector<Point2f> imgPoints; //correspondent 2D points on new image
    	    find3D2DCorrespondences(ppcloud, imgPoints, i, matches_matrix, keypoints_vector);

            if(ppcloud.size() > max_3D2DCorr){

            	max_3D2DCorr = ppcloud.size();
            	max_corr_view = i;
            	max_3D_corr = ppcloud;
             	max_2D_corr = imgPoints;

            }

    	}

    	cout<<"view "<<max_corr_view<<" has max number of 3D-2D correspondences: "<<max_3D2DCorr<<endl;

    	done_views.insert(max_corr_view);

    	//can also use the first t, rvec and R as initial guess for this function
    	bool pose_estimated = findPoseEstimation(max_corr_view, rvec, t, R, max_3D_corr,
    			                                 max_2D_corr, K, distortion_coeff);

    	if(!pose_estimated){
    		cout<<"failed to compute estimated pose for view"<<max_corr_view<<endl;
    		cout<<"view discarded"<<endl;
    		continue;
    	}

    	R_.push_back(R); t_.push_back(t);

    	camera_Rs[max_corr_view] = R;
        camera_ts[max_corr_view] = t;

    	//store estimated pose
    	Pmats[max_corr_view] = Matx34d(R(0,0),R(0,1),R(0,2), t(0),
    								   R(1,0),R(1,1),R(1,2), t(1),
    	                               R(2,0),R(2,1),R(2,2), t(2));

    	// start triangulating with previous GOOD views
    	for (set<int>::iterator done_view = good_views.begin(); done_view != good_views.end(); ++done_view){

    		int view = *done_view;
    		cout<<endl;cout<<"triangulating view "<<max_corr_view<<" with previous good view "<<view<<endl;

    		if( view == max_corr_view ){

    			continue; //skip the actual view

    		}

    		cout<<endl;cout<<"[ triangulatePointsBetweenViews executes ...]"<<endl;
    		Triangulation tr;
    		vector<CloudPoint> new_triangulated;
    		vector<int> add_to_cloud;
    		bool good_triangulation = triangulatePointsBetweenViews(imgs, max_corr_view, view,
    				new_triangulated, add_to_cloud, keypoints_vector, matches_matrix, K, tr);

    		if( ! good_triangulation){
    			cout<<"triangulation failed, no points to add"<<endl;
    			continue;
    		}

    		cout << "before triangulation: " << p_cloud.size()<<" 3D points"<<endl;

    		for (unsigned int j=0; j<add_to_cloud.size(); j++) {

    			if(add_to_cloud[j] == 1)

    				p_cloud.push_back(new_triangulated[j]);

    		}

    		cout<<"after triangulation: "<<p_cloud.size()<<" 3D points"<<endl;

    	} //for

    	good_views.insert(max_corr_view);
    	cout<<endl;cout<<max_corr_view<<" view added to good_views"<<endl;

    } //while

    //extracting recovered 3D points from CloudPoint struct
    vector<Point3d> outCloud1;
    vector<double> reproj_err;
   	for(unsigned int i=0; i < p_cloud.size(); i++){
    	outCloud1.push_back( p_cloud[i].pt );
    	reproj_err.push_back( p_cloud[i].reprojection_error );
    }

    Visualization1 display;
    Matx34d P_first, P_second;
    P_first = Pmats[max_index1];
    P_second = Pmats[max_index2];
    cout<<endl;cout<<"*****************************************************************************"<<endl;
               cout<<"*****************************************************************************"<<endl;
    cout<<endl;cout<<"visualizing CloudPoint results ..."<<endl;
    //display.visualize3DPoints(outCloud, P, P1, K, reproj_err);
    display.visualize3DPoints(outCloud1, P_first, P_second, R_, t_, camera_Rs, camera_ts,
    		                  K, Pmats, reproj_err);

    cout<<endl;
    cout<<"****************************************************************************************"<<endl;
    cout<<"****************************** 3D STRUCTURE RECOVERED **********************************"<<endl;
    cout<<"****************************************************************************************"<<endl;

}

















