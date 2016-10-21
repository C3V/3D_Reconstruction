#README

This is an utility to convert a PointCloud (collection of unordered 3D points) to a .pcd file.
The reason is Meshlab tool doesn't read simple vectors of Point3d, so in order to open the 
recovered cloud in Meshlab you convert it to a .pcd file and the save it as a .xyz file, after
removing the .pcd header.
Then "Import Mesh" in Meshlab.

In order to run the program you need PCL Library, so build it from source (assuming Linux)
after installing the dependancies (see PCL documentation).
Then put the code in a "src" directory and create a different directory called "build" (both
src and build must be subdirectories of the root directory of the project).
Change directory to build and run ccmake with the src directory as source, set variables
like Opencv_Found to On and OpenCV_Dir to the build directory of OpenCV (the one in which
you built Opencv while installng the first time), and maybe something else, not sure.
Then generate and run make.
If you want to import it to Eclipse, instead of giving simple ccmake command just type
$cmake -G "Eclipse CDT4 - Unix Makefiles" <path_to_src_directory_of_the_project>
then set the variables as before and run make.
Then open Eclipse and import existing project, let the indexer do his work, compile and run.
The project must have one or more .xml files as input (set them in Run Configurations).
Don't mind the apparent IDE errors, the project will compile and run anyway (don't know why).


