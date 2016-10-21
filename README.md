#3D Reconstruction
Extract 3D informations performing Structure From Motion reconstruction using OpenCV and PCL libs.
"Feature Matching" will be computed using Optical Flow.

First obtain the configuration.xml file with CameraCalibration, then use it as one of the inputs for
Structure_from_motion. It will display the recovered sparse scene basing on sparse reconstruction.
SFM will write the recovered 3D cloud of points to a cloud.xml file that can be converted to a .pcd file
using PointCloud_To_PCD_Writer.
Then the .pcd file can be used to open the cloud with Meshlab and obtain a rough surface from points
reconstruction.

Code inspired by the book "Mastering OpenCV with Practical Computer Vision Projects", in particular the work of Roy Shilkrot on chapter 4.



