#Readme
Simple utility to obtain camera matrix and distortion coefficients that will be used in Structure From Motion.
Need an input of calibration images (I used 17 images).
Use a black/white chessboard.
Images must not be biased to a particular region of the screen, so move the chess also to the "corners".

Results of the calibration (Matrix K and distortion coeff.) will be written to a input/output
"configuration.xml" file.
