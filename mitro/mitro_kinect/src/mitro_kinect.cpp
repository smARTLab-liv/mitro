#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

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
#include <mitro_kinect/MitroKinectConfig.h>

#include <tf/transform_broadcaster.h>

#define PI 3.141592653589793238462643383


// TODO: these should become ros params
double static MAX_UPDATE_RATE = 10.0;
float static KINECT_PITCH_ANGLE = 0.41;
std::string static TOPIC_CLOUD_IN = "cloud";
std::string static TOPIC_CLOUD_OBSTACLES = "cloud_obstacles";
std::string static TOPIC_CLOUD_VOXEL = "cloud_voxel";
std::string static TF_FRAME = "/odom_kinect";
std::string static TF_TARGET_FRAME = "/base_link";

// dynamic reconfigure parameters
float voxel_size = 0.025f;
float plane_tresh = 0.02f;
int outlier_neighbors = 10;
float outlier_radius = 0.1f;

ros::Subscriber sub;
ros::Publisher pub_obstacles, pub_voxel;

ros::Time last_update;
sensor_msgs::PointCloud2::Ptr last_cloud;

tf::TransformBroadcaster *tf_broadcaster;

// dynamic reconfigure
void cb_dynreconf(mitro_kinect::MitroKinectConfig &config, uint32_t level) {
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


double wrap_angle(double a)
{
  if (a < 0)
  {
    while (a < -PI) a+= 2.0 * PI; 
    return a;
  } else {
    while (a > PI) a-= 2.0 * PI;
    return a;
  }
}

void cb_pointcloud2(const sensor_msgs::PointCloud2::Ptr cloud_in)
{
  last_cloud = cloud_in;
}

void filter(sensor_msgs::PointCloud2::Ptr cloud_in)
{
  sensor_msgs::PointCloud2::Ptr cloud_voxel (new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_floor (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_floor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
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

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("Could not estimate a planar model for the given dataset!");
    return;
  }

  // computing transform based on normal vector
  float x = -coefficients->values[0];
  float y = -coefficients->values[1];
  float z = fabs(coefficients->values[2]);

  float roll = atan2(z,-x) - PI/2.0f;
  float pitch = atan2(z,y) - PI/2.0f + KINECT_PITCH_ANGLE;
  roll = wrap_angle(roll);
  pitch = wrap_angle(pitch);

  ROS_DEBUG("roll: %f, pitch: %f", pitch, roll);

  // publishing transform
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion quaternion;
  quaternion.setEuler(pitch, roll, 0.0d);
  transform.setRotation( quaternion );
  tf_broadcaster->sendTransform(tf::StampedTransform(transform, cloud_in->header.stamp, TF_FRAME, TF_TARGET_FRAME));

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (pcl_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_without_floor);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
  outlier_filter.setInputCloud(cloud_without_floor);
  outlier_filter.setRadiusSearch(outlier_radius);
  outlier_filter.setMinNeighborsInRadius(outlier_neighbors);
  outlier_filter.filter(*cloud_without_floor_filtered);

  sensor_msgs::PointCloud2::Ptr cloud_obstacles (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_without_floor_filtered, *cloud_obstacles);
  pub_obstacles.publish(cloud_obstacles);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "mitro_kinect");
  ros::NodeHandle n;

  // tf broadcaster
  tf_broadcaster = new tf::TransformBroadcaster();

  // dynamic reconfigure server
  dynamic_reconfigure::Server<mitro_kinect::MitroKinectConfig> server;
  dynamic_reconfigure::Server<mitro_kinect::MitroKinectConfig>::CallbackType f;
  f = boost::bind(&cb_dynreconf, _1, _2);
  server.setCallback(f);

  // subscribers and publishers
  sub = n.subscribe(TOPIC_CLOUD_IN, 1, cb_pointcloud2);
  pub_obstacles = n.advertise<sensor_msgs::PointCloud2>(TOPIC_CLOUD_OBSTACLES, 1);
  pub_voxel = n.advertise<sensor_msgs::PointCloud2>(TOPIC_CLOUD_VOXEL, 1);

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
