#Readme
Main program: use the camera as an input to get images of the scene for the reconstruction.
Set the number of frames to be used in the video and then set the sample frequency for 
images, because you cannot use a video stream with Optical Flow, it wouldn't work.

Then the program wil compute the 3D sparse reconstruction obtaining a vector of 3D points.

Then it will write the result as a .xml output file called "cloud.xml".
