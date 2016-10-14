

here is the configuration xml file in which the calibration process
wrote the camera matrix K and the distortion coefficients that will be used to compute E (essential matrix)
from F (fundamental matrix); they will also be used to recover the incremental motion of cameras in multi-view
reconstruction via the solvePnpRansac() OpenCV function.
