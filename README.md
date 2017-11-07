# GelSightCamera
1. install OpenCV 3.2
2. open a teminal and download the code through $git clone https://github.com/Jianhua2016/GelSightCamera.git
3. go to the /GelSightCamera folder and $mkdir build && cd ./build
4. $cmake ..
5. $make

6. run the "testCalibrateGelSightHeightMap" by $./testCalibrateGelSightHeightMap to calibrate the GelSight sensor
7. run the "testGelSightCamera" by $./testGelSightCamera to get the markers motion and 3D height map.
