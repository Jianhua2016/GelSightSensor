# GelSightCamera
1. install OpenCV 3.2
2. install VS2015 (if you run the code in Windows 10)

If you run the code in Ubuntu 16.04
1. open a terminal and download the code through $git clone https://github.com/Jianhua2016/GelSightCamera.git
2. go to the /GelSightCamera folder and create a folder through $mkdir build && cd ./build
3. $cmake ..
4. $make
5. when you calibrate a new sensor, you need to change the value "CALIB_Pixel2mm_ratio" and "CALIB_BALL_ACTUALRADIUS_mm" in the "CGelSightHeightMapCalibration.h". recompile the code, and run the "testCalibrateGelSightHeightMap" by $./testCalibrateGelSightHeightMap to calibrate the GelSight sensor. By default, the calibration file is saved in the folder GelSightCamera/CameraData/ and the folder name is defined like this "2107_11_7_16_59_56_data" and it is the latest one.
6. change the value "GelSightCalibData_FileName" in the file "testGelSightCamera.cpp" to be file name you get in last step, recompile the code, run the "testGelSightCamera" by $./testGelSightCamera to get the markers motion and 3D height map.

If you run the code in Windows 10, the codes are compiled with VS2015.
1. download the code "GelSightCameraWindows"
2. when you calibrate a new sensor, you need to change the value "CALIB_Pixel2mm_ratio" and "CALIB_BALL_ACTUALRADIUS_mm" in the "CGelSightHeightMapCalibration.h". recompile the code, and run the "GelSightCalibration" to calibrate the GelSight sensor. By default, the calibration file is saved in the folder GelSightCamera/CameraData/ and the folder name is defined like this "2107_11_7_16_59_56_data" and it is the latest one.
3. change the value "GelSightCalibData_FileName" in the file "testGelSightCamera.cpp" to be file name you get in last step, recompile the code, run the "GelSightCamera" by to get the markers motion and 3D height map.
 
