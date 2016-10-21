#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <cv.h>
#include <highgui.h>
#include <opencv2/viz.hpp>

using namespace std;
using namespace cv;

int
  main (int argc, char** argv)
{
	if(argc < 4){
			cout<<"error! no configuration file in input"<<endl;
			return 1;
	}

	cout<<"help-->This program reads an input cloud of points coming from the SFM ""scanner""\n";
	cout<<"       and writes it in a .pcd file so that it can be used as an input for Meshlab.\n";
	cout<<"       Remember that in order to open the .pcd file in Meshlab you need to remove the\n";
	cout<<"       .pcd header and save the file in .xyz format, then 'import mesh' in Meshlab.\n";

	//read from configuration.xml
	cout<<"\nreading cloud from .xml file\n"<<endl;
	//argv[i] opens the i-th .xml file in input (see Run Configurations)
	string filename = argv[3];
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	//fs["K"]>>K;

	vector<Point3d> pointCloud;
	fs["points"]>>pointCloud;
	cout<<"cloud.size()= "<<pointCloud.size()<<endl;
	fs.release();
	cout<<"\nreading terminated\n"<<endl;

	//std::cout<<"\nopencv and pcl work together!\n"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = pointCloud.size();
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  cout<<"\npopulating cloud with read points"<<endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {/*
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
   */
	cloud.points[i].x = pointCloud[i].x;
	cloud.points[i].y = pointCloud[i].y;
	cloud.points[i].z = pointCloud[i].z;

  }
  cout<<"\ncloud populated"<<endl;

  pcl::io::savePCDFileASCII ("\ntest_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

/*  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
*/
  return (0);
}
