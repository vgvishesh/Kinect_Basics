Contains basic sample codes to get started with MS Kinect Sensor using windows Kinect SDK

1. depth.cpp : Displays the raw incoming depth from kinect sensor
2. color.cpp : Displays the Color image stream 
(above 2 file taken from http://www.cs.princeton.edu/~edwardz/tutorials/kinect/kinect2.html)

3. PointCloud.cpp : to visualize and save the the point cloud as .ply file
4. Transform.cpp : to convert a depth image into point cloud
5. PCDfromIMAGES.cpp : takes in a rgb PPM imahe and a raw depth .txt image and converts them into  .ply point cloud.

6. KinectSDK.props : MS Visual Studios 2012 property sheet to having setting for working with Kinect Windows SDK. Modify the property sheet paths according to location of respective directories on your computer.