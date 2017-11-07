# GelSightCamera
1. install OpenCV 3.2
2. open a teminal and download the code through $git clone https://github.com/Jianhua2016/GelSightCamera.git
3. go to the /GelSightCamera folder and $mkdir build && cd ./build
4. $cmake ..
5. $make

6. run the "testCalibrateGelSightHeightMap" by $./testCalibrateGelSightHeightMap to calibrate the GelSight sensor. By default, the calibration file is save in the folder GelSightCamera/CameraData/ and the filder name is like this "2107_11_7_16_59_56_data" and it is the latet one.
7. change the value "GelSightCalibData_FileName" in the file "testGelSightCamera.cpp" to be file name you get in last step, recompile the code, run the "testGelSightCamera" by $./testGelSightCamera to get the markers motion and 3D height map.
