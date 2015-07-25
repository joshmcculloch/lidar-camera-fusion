#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"


void cameraCallback(const sensor_msgs::Image::ConstPtr& img)
{
  ROS_INFO("Got Image!");
}

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& scan)
{
  ROS_INFO("Got Pointcloud!");
}


int main(int argc, char **argv)
{
 ros::init(argc, argv, "colour_cloud");
 
 ros::NodeHandle n;

 ros::Subscriber sub1 = n.subscribe("/usb_cam/image_raw", 100, cameraCallback);
 ros::Subscriber sub2 = n.subscribe("/velodyne_points", 100, lidarCallback);
 ros::spin();

 return 0;
}

