#Readme
Main program: use the camera as an input to get images of the scene for the reconstruction.
Set the number of frames to be used in the video and then set the sample frequency for 
images, because you cannot use a video stream with Optical Flow, it wouldn't work.

The program will then compute the 3D sparse reconstruction obtaining a vector of 3D points.

Then it will write the result as a .xml output file called "cloud.xml".

Program needs a "configuration.xml" file as an input with the calibration matrix and
distortion coefficients.
It also needs a "cloud.xml" input/output file where the results will be written.
