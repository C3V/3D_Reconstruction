This is an utility to convert a PointCloud (collection of unordered 3D points) to a .pcd file.
The reason is Meshlab tool doesn't read simple vectors of Point3d, so in order to open the 
recovered cloud in Meshlab you convert it to a .pcd file and the save it as a .xyz file, after
removing the .pcd header.
Then "Import Mesh" in Meshlab.
