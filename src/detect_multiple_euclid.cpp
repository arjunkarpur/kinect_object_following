#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cstdio>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include "angles/angles.h"
#include <algorithm>

// camera_link for fixed frame in global option in rviz

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

//Point Cloud to store out neon cap
PointCloudT::Ptr neon_cloud (new PointCloudT);

// Message required to publish the cloud - Convert from pcl to msg
sensor_msgs::PointCloud2 cloud_ros;

enum RobotType {segbot, turtlebot};

void cloud_sub(const sensor_msgs::PointCloud2ConstPtr& msg)
{
		//convert the msg to PCL format
		pcl::fromROSMsg (*msg, *cloud);

		//state that a new cloud is available
		new_cloud_available_flag = true;
		 
		/**PointCloudT::iterator myIterator;
		for(myIterator = cloud->begin();  
			myIterator != cloud->end();
			myIterator++)
		{
			std::cout<<*myIterator<<" ";
		}**/
}

PointCloudT::Ptr computeNeonVoxels(PointCloudT::Ptr in) {
    int total_neon = 0;
    
    //Point Cloud to store out neon cap
	PointCloudT::Ptr temp_neon_cloud (new PointCloudT);

    for (int i = 0; i < in->points.size(); i++) {
        unsigned int r, g, b;
        r = in->points[i].r;
        g = in->points[i].g;
        b = in->points[i].b;
        // Look for mostly neon value points
        if (g > 150 && (r + b) < 250) {
			temp_neon_cloud->push_back(in->points[i]);
	    }
    }

    return temp_neon_cloud;
}


int main (int argc, char** argv)
{
    RobotType robot = turtlebot;

	// Initialize ROS
	ros::init (argc, argv, "kinect_fun");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub;
    if (robot == turtlebot) {
	    sub = nh.subscribe ("/camera/depth_registered/points", 1000, cloud_sub);
    } else if (robot == segbot) {
        sub  = nh.subscribe ("/nav_kinect/depth_registered/points", 1000, cloud_sub);
    }
	
	//debugging publisher --> can create your own topic and then subscribe to it through rviz
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cap/cloud", 10);

    // publish to base
    ros::Publisher velocity_pub;

    if (robot == turtlebot) {
        velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
    } else if (robot == segbot) {
        velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    }
	
	//refresh rate
	double ros_rate = 3.0;
	ros::Rate r(ros_rate);
	
    tf::TransformListener transformListener;

	while (ros::ok())
	{
		ros::spinOnce();
		
		if (new_cloud_available_flag){
			new_cloud_available_flag = false;
			
			// Voxel Grid reduces the computation time. Its a good idea to do it if you will be doing
			//sequential processing or frame-by-frame
			// Create the filtering object: downsample the dataset using a leaf size of 1cm
			pcl::VoxelGrid<PointT> vg;
			pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
			vg.setInputCloud (cloud);
			vg.setLeafSize (0.005f, 0.005f, 0.005f);
			vg.filter (*cloud_filtered);
			

			int max_num_neon = 0;
			
			//Send the filtered point cloud to be processed in order to get the neon blob
			neon_cloud = computeNeonVoxels(cloud_filtered);
	        
            // Create KDTree
            pcl::search::KdTree<PointT>::Ptr kdTree (new pcl::search::KdTree<PointT>);
            kdTree->setInputCloud(neon_cloud);

            // Create vector of clusters of points. Each value in vector is a vector of points in cluster
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointT> ec;
            ec.setClusterTolerance(0.02);   // tolerance of 2 cm. TODO: tweak
            ec.setMinClusterSize(100);
            ec.setMaxClusterSize(1500);
            ec.setSearchMethod(kdTree);
            ec.setInputCloud(neon_cloud);
            ec.extract(cluster_indices);

            ROS_INFO("%i clusters found", cluster_indices.size());
            
            // Stop loop if no clusters found
            if (cluster_indices.size() == 0) {
                continue;
            }

            // Create PointClouds of clusters found
            std::vector<PointCloudT::Ptr> cluster_clouds;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
                    it != cluster_indices.end(); ++it) {
                PointCloudT::Ptr curr_cloud (new PointCloudT);

                for (std::vector<int>::const_iterator pIt = it->indices.begin();
                        pIt != it->indices.end(); ++pIt) {
                    curr_cloud->points.push_back(neon_cloud->points[*pIt]);
                }
                curr_cloud->width = curr_cloud->points.size();
                curr_cloud->height = 1;
                curr_cloud->is_dense = true;
                cluster_clouds.push_back(curr_cloud);
            }

            //TODO: fix! For now, finds closest to kinect, not to base
            PointCloudT::Ptr closest_cloud = cluster_clouds[0];
            double best_distance = 99999999;
            for (int i = 0; i < cluster_clouds.size(); i++) {
                PointCloudT::Ptr curr_cloud = cluster_clouds[i];
                Eigen::Vector4f cloud_centroid;
                pcl::compute3DCentroid(*curr_cloud, cloud_centroid);
                double distance = sqrt(pow(cloud_centroid[0], 2) + pow(cloud_centroid[1], 2));
                if (distance < best_distance && distance > 1) {
                    best_distance = distance;
                    closest_cloud = cluster_clouds[i];
                }
            }

            neon_cloud = closest_cloud;
    

			//Publish the cloud with the neon cap
			pcl::toROSMsg(*neon_cloud,cloud_ros);
			
			//Set the frame ID to the first cloud we took in coz we want to replace that one
			cloud_ros.header.frame_id = cloud->header.frame_id;

			
			// Find the centroid of the neon cap
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*neon_cloud, centroid);

			ROS_INFO("The centroid of the neon cap is: (%f, %f, %f)", 
                centroid(0), centroid(1), centroid(2));

            tf::StampedTransform transform;
            try {
                if (robot == turtlebot) {
                    transformListener.waitForTransform("base_footprint","camera_depth_optical_frame",
                        ros::Time(0), ros::Duration(3.0));
                    transformListener.lookupTransform("base_footprint", "camera_depth_optical_frame",
                        ros::Time(0), transform);
                } else if (robot == segbot) {
                    transformListener.waitForTransform("base_footprint","nav_kinect_depth_frame",
                        ros::Time(0), ros::Duration(3.0));
                    transformListener.lookupTransform("base_footprint", "nav_kinect_depth_frame",
                        ros::Time(0), transform);

                }
                	        } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }		
            tf::Vector3 centeroidV(centroid(0), centroid(1), centroid(2));
            
            tf::Vector3 transformedVector = transform * centeroidV;

            double angle = atan2(transformedVector[1], transformedVector[0]);
            angle = angles::normalize_angle(angle);
            double distanceToTarget = sqrt(pow(transformedVector[0], 2) + pow(transformedVector[1], 2));
            geometry_msgs::Twist vel_msg;

            if (distanceToTarget < 5 && distanceToTarget > 1) {
                ROS_INFO("Distane to target: %f m", distanceToTarget);
                ROS_INFO("Angle: %f", angle);
                vel_msg.linear.x = 0.1;
                vel_msg.angular.z = angle;
            } else if (distanceToTarget < 1) {
                ROS_INFO("Within 1 meter of target! (%f m)", distanceToTarget);
                vel_msg.linear.x = 0;
                vel_msg.angular.z = 0;
            } else {
                vel_msg.linear.x = 0;
                vel_msg.angular.z = 0;
            }
            velocity_pub.publish(vel_msg);
		}
	}
	
	return 0;
}
