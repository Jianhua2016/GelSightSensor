# GelSightCamera
1. install OpenCV 3.2
2. open a terminal and download the code through $git clone https://github.com/Jianhua2016/GelSightCamera.git
3. go to the /GelSightCamera folder and create a folder through $mkdir build && cd ./build
4. $cmake ..
5. $make
6. when you calibrate a new sensor, you need to change the value "CALIB_Pixel2mm_ratio" and "CALIB_BALL_ACTUALRADIUS_mm" in the "CGelSightHeightMapCalibration.h". recompile the code, and run the "testCalibrateGelSightHeightMap" by $./testCalibrateGelSightHeightMap to calibrate the GelSight sensor. By default, the calibration file is saved in the folder GelSightCamera/CameraData/ and the folder name is defined like this "2107_11_7_16_59_56_data" and it is the latest one.
7. change the value "GelSightCalibData_FileName" in the file "testGelSightCamera.cpp" to be file name you get in last step, recompile the code, run the "testGelSightCamera" by $./testGelSightCamera to get the markers motion and 3D height map.

Note: Now the code is compiled in Ubuntu 16.04. If you decide to run in Windows, please let me know and I will fix the problem. 
