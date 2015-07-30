#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>

#include <cv_bridge/cv_bridge.h>


#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>


using namespace cv;

ros::Publisher cloud_pub;

cv::Mat rect_view, map1, map2;
bool map_initialised = false;

Eigen::Matrix4d sensorTransform;
Eigen::Matrix3d tmat;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());


void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& scan)
{
  ROS_INFO("Got point cloud!");
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*scan, *laserCloudIn);

  /*
  for(unsigned int i=0; i<laserCloudIn->size(); i++) {
      pcl::PointXYZRGB p;
      pcl::PointXYZ oldp = laserCloudIn->at(i);
      p.x = oldp.x;
      p.y = oldp.y;
      p.z = oldp.z;
      global_pointcloud->push_back(p);
  }
  ROS_INFO_STREAM(global_pointcloud->size());*/

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour_laser_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      for(unsigned int i=0; i<laserCloudIn->size(); i++) {

          float x3d = laserCloudIn->at(i).x;
          float y3d = laserCloudIn->at(i).y;
          float z3d = laserCloudIn->at(i).z;

          Eigen::Vector4d point_world_space;
          Eigen::Vector3d point_camera_space;
          Eigen::Vector3d point_image_space;

          point_world_space << x3d, -z3d, y3d, 1;

          point_camera_space << x3d, -z3d, y3d;

          point_world_space = sensorTransform * point_world_space;
          //ROS_INFO_STREAM(sensorTransform * point_world_space);
          point_camera_space = point_world_space.head<3>();
          point_image_space = tmat * point_camera_space;

          pcl::PointXYZRGB p;
          p.x = x3d;
          p.y = y3d;
          p.z = z3d;
          p.r = 255;
          p.g = 255;
          p.b = 255;
          if (point_image_space[2] > 0) {
              int image_u = (int)(point_image_space[0]/point_image_space[2]);
              int image_v = (int)(point_image_space[1]/point_image_space[2]);

              if (0 <= image_u && image_u < rect_view.size().width &&
                  0 <= image_v && image_v < rect_view.size().height) {
                Vec3b colour= rect_view.at<Vec3b>(Point(image_u, image_v));
                p.r = colour[0];
                p.g = colour[1];
                p.b = colour[2];
              }

          }
          colour_laser_cloud->push_back(p);
      }

      sensor_msgs::PointCloud2 scan_color = sensor_msgs::PointCloud2();
      pcl::toROSMsg(*colour_laser_cloud, scan_color);
      scan_color.header.frame_id = "camera_init";
      cloud_pub.publish(scan_color);
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  //timeLaserOdometry = laserOdometry->header.stamp.toSec();

  double transformSum[6];
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;

  //tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  // Get 4x4 Roation Matrix
  Eigen::Quaterniond eigenQuat = Eigen::Quaternion<double>(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
  Eigen::Matrix3d eigenRot = eigenQuat.toRotationMatrix();
  Eigen::Matrix4d eigenRot4 = Eigen::Matrix4d::Identity();
  eigenRot4(0,0) = eigenRot(0,0);
  eigenRot4(1,0) = eigenRot(2,0);
  eigenRot4(2,0) = eigenRot(2,0);

  eigenRot4(0,1) = eigenRot(0,1);
  eigenRot4(1,1) = eigenRot(2,1);
  eigenRot4(2,1) = eigenRot(2,1);

  eigenRot4(0,2) = eigenRot(0,2);
  eigenRot4(1,2) = eigenRot(2,2);
  eigenRot4(2,2) = eigenRot(2,2);


  // Get translation matrix
  Eigen::Affine3d trans(Eigen::Translation3d(
      laserOdometry->pose.pose.position.x,
      laserOdometry->pose.pose.position.y,
      laserOdometry->pose.pose.position.z));


  sensorTransform = trans.matrix();
  sensorTransform *= eigenRot4;

  ROS_INFO_STREAM(sensorTransform);

}

void cameraCallback(const sensor_msgs::Image::ConstPtr& img)
{
  ROS_INFO("Got Image!");

  Size imageSize = Size(img->width,img->height);
  cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(img, "8UC3");

  if (!map_initialised) {
    ROS_INFO("Initalising camera mapping.");

    float distMat[9] = {754.53892599834842f, 0.0f, 319.5f,
        0.0f, 754.53892599834842f, 239.5f,
        0.0f, 0.0f, 1.0f};

    float distCoef[5] = {5.2038044809064208e-03, 1.5288890999953295e-01, 0., 0., -1.7854072082302619e+00};

    Mat cameraMatrix, distCoeffs;
    cameraMatrix = Mat(3,3, CV_32F,distMat);
    distCoeffs = Mat(5,1, CV_32F,distCoef);

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                imageSize, CV_16SC2, map1, map2);

    map_initialised = true;
  }

  remap(cv_cam->image, rect_view, map1, map2, INTER_LINEAR);

}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "scan_archiver");

 tmat = Eigen::Matrix3d();
 tmat <<   754.53892599834842f, 0.0f,                319.5f,
            0.0f,                754.53892599834842f, 239.5f,
            0.0f,                0.0f,                1.0f;

 ros::NodeHandle n;

 ros::Subscriber velodyne = n.subscribe("/velodyne_cloud_registered", 2, lidarCallback);

 ros::Subscriber sub1 = n.subscribe("/usb_cam/image_raw", 2, cameraCallback);

 ros::Subscriber subLaserOdometry = n.subscribe<nav_msgs::Odometry>
                                      ("/integrated_to_init", 5, laserOdometryHandler);

 cloud_pub = n.advertise<sensor_msgs::PointCloud2>("global_colour", 10);

 ros::spin();

 return 0;
}
