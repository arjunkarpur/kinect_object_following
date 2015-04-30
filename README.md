# ROS Object Following
Package name: kinect_demo

Two nodes included in package:

    - detect_cap - Detects cap using color (neon green) and follow cap. 
    Does no filtering of point cloud and is often confused by noise in 
    the masked point cloud (identifies some random areas as neon green 
    and follows the point when cap isn't present). 


    - detect_multiple_euclid - Uses Euclidean Clustering (through PCL) 
    to identify multiple different caps/objects that are neon green. 
    Able to tell how many objects are found using clustering so that only 
    notable objects are followed. If 0 objects are found, the robot does 
    nothing. If multiple objects are found, the robot follows the closest 
    object until it is 1 meter away
