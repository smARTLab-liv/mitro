#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>

float static VOXEL_SCALE = 0.025f;
double static MAX_UPDATE_RATE = 10.0;


ros::Subscriber sub;
ros::Publisher pub, pub_voxel, pub_scan;
ros::Time last_update;

sensor_msgs::PointCloud2::Ptr last_cloud;

int count = 0;

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

  ROS_INFO("pcl points %ld", pcl_cloud.points.size());
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
  sensor_msgs::PointCloud2::Ptr cloud_voxel (new sensor_msgs::PointCloud2());
  sensor_msgs::PointCloud2::Ptr cloud_pass_z (new sensor_msgs::PointCloud2());
  sensor_msgs::PointCloud2::Ptr cloud_pass_x (new sensor_msgs::PointCloud2());
  sensor_msgs::PointCloud2::Ptr cloud_outlier (new sensor_msgs::PointCloud2());

  pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_filter;
  voxel_filter.setInputCloud (cloud_in);
  voxel_filter.setLeafSize (VOXEL_SCALE, VOXEL_SCALE, VOXEL_SCALE);
  voxel_filter.filter (*cloud_voxel);
  
  pcl::PassThrough<sensor_msgs::PointCloud2> pass_through_x;
  pass_through_x.setInputCloud(cloud_voxel);
  pass_through_x.setFilterFieldName("x"); // kinext is rotated 90 deg
  pass_through_x.setFilterLimits(-1.1, 0.5);
  pass_through_x.filter(*cloud_pass_x);

  pub_voxel.publish(cloud_voxel);


  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*cloud_voxel, pcl_cloud);

  if ( 0 == pcl_cloud.points.size() )
    return;

   

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

  // pub_scan.publish(pointcloud_to_laser(cloud_outlier));

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "kinect_test");
  ros::NodeHandle n;

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
