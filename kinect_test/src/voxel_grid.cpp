#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"

#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>
#include <kinect_test/KinectTestConfig.h>

#include <tf/transform_broadcaster.h>

#define PI 3.1415f

double static MAX_UPDATE_RATE = 10.0;
float static KINECT_PITCH_ANGLE = 0.41;

float voxel_size = 0.025f;
float plane_tresh = 0.02f;


int outlier_neighbors = 10;
float outlier_radius = 0.1f;

ros::Subscriber sub;
ros::Publisher pub, pub_voxel, pub_scan;
ros::Time last_update;

sensor_msgs::PointCloud2::Ptr last_cloud;

int count = 0;



// dynamic reconfigure shit goes here
void callback(kinect_test::KinectTestConfig &config, uint32_t level) {
  ROS_DEBUG("dyn reconf call (voxel_size, plane_tresh, outlier_radius, outlier_neighbors): %f %f %f %d", 
	    config.voxel_size,
	    config.plane_tresh,
	    config.outlier_radius,
	    config.outlier_neighbors);
  voxel_size = (float) config.voxel_size;
  plane_tresh = (float) config.plane_tresh;
  outlier_radius = (float) config.outlier_radius;
  outlier_neighbors = config.outlier_neighbors;
}


float radToDeg(float a)
{
  return a / PI * 180.0f;
}

void cb_pointcloud2(const sensor_msgs::PointCloud2::Ptr cloud_in)
{
  last_cloud = cloud_in;
}


sensor_msgs::LaserScan::Ptr pointcloud_to_laser(sensor_msgs::PointCloud2::Ptr cloud) {
  sensor_msgs::LaserScan::Ptr scan (new sensor_msgs::LaserScan());
  scan->header = cloud->header;
  scan->header.frame_id = "/kinect_laser";

  // TODO: parameterize
  scan->scan_time = 1.0/30.0;
  scan->time_increment = 0.0f;

  scan->range_min = 0.45f;
  scan->range_max = 3.0f;
  
  scan->angle_min = -M_PI/2.0;
  scan->angle_max = M_PI/2.0;
  scan->angle_increment = M_PI/180.0/2.0;

  uint32_t ranges_size = std::ceil((scan->angle_max - scan->angle_min) /
				   scan->angle_increment);

  scan->ranges.assign(ranges_size, scan->range_max + 1.0);

  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);

  if ( 0 == pcl_cloud.points.size() )
    return scan;


  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
  {
    if (isnan(it->x) || isnan(it->y) || isnan(it->z))
      continue;

    double angle = -atan2(it->y, it->z);
    
    if (angle < scan->angle_min || angle > scan->angle_max)
    {
      continue;
    }
    
    int index = (angle - scan->angle_min) / scan->angle_increment;

    double range_sq = it->z*it->z+it->y*it->y;

    if (scan->ranges[index] * scan->ranges[index] > range_sq)
      scan->ranges[index] = sqrt(range_sq);
  }

  return scan;
}


void filter(sensor_msgs::PointCloud2::Ptr cloud_in)
{
  sensor_msgs::PointCloud2::Ptr cloud_voxel (new sensor_msgs::PointCloud2);
  // sensor_msgs::PointCloud2::Ptr cloud_pass_z (new sensor_msgs::PointCloud2());
  // sensor_msgs::PointCloud2::Ptr cloud_pass_x (new sensor_msgs::PointCloud2());
  //sensor_msgs::PointCloud2::Ptr cloud_outlier (new sensor_msgs::PointCloud2());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_floor (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_floor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  // pcl::PassThrough<sensor_msgs::PointCloud2> pass_through_x;
  // pass_through_x.setInputCloud(cloud_voxel);
  // pass_through_x.setFilterFieldName("x"); // kinext is rotated 90 deg
  // pass_through_x.setFilterLimits(-1.1, 0.5);
  // pass_through_x.filter(*cloud_pass_x);

  // pub_voxel.publish(cloud_voxel);


  pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_filter;
  voxel_filter.setInputCloud (cloud_in);
  voxel_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
  voxel_filter.filter (*cloud_voxel);

  pub_voxel.publish(cloud_voxel);

  pcl::fromROSMsg(*cloud_voxel, *pcl_cloud);

  if ( 0 == pcl_cloud->points.size() )
  {
    ROS_ERROR("Cloud is empty!");
    return;
  }

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (plane_tresh);
  seg.setInputCloud (pcl_cloud);
  seg.segment (*inliers, *coefficients);

  float x = -coefficients->values[0];
  float y = -coefficients->values[1];
  float z = -coefficients->values[2];



  //  ROS_INFO("coefficients: %f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);


  float roll = atan2(z,-x) - PI/2.0f;
  float pitch = atan2(z,y) - PI/2.0f + KINECT_PITCH_ANGLE;

  ROS_INFO("roll: %f", radToDeg(roll));
  ROS_INFO("pitch: %f", radToDeg(pitch));
   

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("Could not estimate a planar model for the given dataset!");
    return;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (pcl_cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_without_floor);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
  outlier_filter.setInputCloud(cloud_without_floor);
  outlier_filter.setRadiusSearch(outlier_radius);
  outlier_filter.setMinNeighborsInRadius(outlier_neighbors);
  outlier_filter.filter(*cloud_without_floor_filtered);

  //  pub_voxel.publish(cloud_without_floor_filtered);

  // char str [200];
  // sprintf(str, "cloud_%d.pcd", ++count);
  // ROS_INFO("writing cloud to file %s ...", str);
  // pcl::io::savePCDFileASCII (str, pcl_cloud);
  // ROS_INFO("cloud written to file %s ...", str);

  // pcl::PassThrough<sensor_msgs::PointCloud2> pass_through_z;
  // pass_through_z.setInputCloud(cloud_pass_x);
  // pass_through_z.setFilterFieldName("z");
  // pass_through_z.setFilterLimits(0.0, 3.0);
  // pass_through_z.filter(*cloud_pass_z);


  // pcl::RadiusOutlierRemoval<sensor_msgs::PointCloud2> outlier_filter;
  // outlier_filter.setInputCloud(cloud_pass_x);
  // outlier_filter.setRadiusSearch(0.1f);
  // outlier_filter.setMinNeighborsInRadius(5);
  // outlier_filter.filter(*cloud_outlier);

  // ROS_INFO("pointcloud after voxel, pass_through_z, pass_through_x, outlier: %d %d %d %d", 
  // 	   cloud_voxel->width * cloud_voxel->height,
  // 	   cloud_pass_z->width * cloud_pass_z->height,
  // 	   cloud_pass_x->width * cloud_pass_x->height,
  // 	   cloud_outlier->width * cloud_outlier->height
  // 	   );

  // pub.publish(cloud_outlier);

  sensor_msgs::PointCloud2::Ptr cloud_out (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_without_floor_filtered, *cloud_out);
  pub.publish(cloud_out);

  // pub_scan.publish(pointcloud_to_laser(cloud_outlier));


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, cloud_in->header.stamp, "/bob/odom", "kinect_frame"));
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "kinect_test");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<kinect_test::KinectTestConfig> server;
  dynamic_reconfigure::Server<kinect_test::KinectTestConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  sub = n.subscribe("/cloud_in", 1, cb_pointcloud2);
  pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
  pub_voxel = n.advertise<sensor_msgs::PointCloud2>("/cloud_voxel", 1);
  pub_scan = n.advertise<sensor_msgs::LaserScan>("/scan", 1);

  ros::Rate loop_rate(MAX_UPDATE_RATE);

  while (ros::ok())
  {
    if (last_cloud)
      filter(last_cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return (0);
}
