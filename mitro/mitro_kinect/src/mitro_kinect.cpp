#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <mitro_kinect/MitroKinectConfig.h>

#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>

#define PI 3.141592653589793238462643383

// plane constraints
static const float MAX_ROLL_ANGLE = 15/180.0f * PI;
static const float MAX_PITCH_ANGLE = 15/180.0f * PI;
static const float MIN_DIST = 1.5f;
static const float MAX_DIST = 1.6f;

// dynamic reconfigure
float voxel_size = 0.025f;
float plane_tresh = 0.02f;
int outlier_neighbors = 10;
float outlier_radius = 0.1f;

// other params
double max_update_rate = 10.0;
float kinect_pitch_angle = 0.39; // opposite of the angle defined in urdf.. should make this a param
std::string topic_cloud_in = "/kinect/depth/points";
std::string topic_cloud_obstacles = "/cloud_obstacles";
std::string topic_cloud_voxel = "/cloud_voxel";
std::string tf_frame = "odom_kinect";
std::string tf_target_frame = "base_link";

// (exponential) moving average
static const float ALPHA = 0.1f;
float avg_pitch = 0.0;
float avg_roll = 0.0;

// ros
ros::Subscriber sub_depth;
ros::Publisher pub_obstacles, pub_voxel;
sensor_msgs::PointCloud2::Ptr last_cloud;
tf::TransformBroadcaster *tf_broadcaster;


// dynamic reconfigure
void cb_dynreconf(mitro_kinect::MitroKinectConfig &config, uint32_t level) {
  ROS_INFO("Reconfifure request (voxel_size, plane_tresh, outlier_radius, outlier_neighbors): %f %f %f %d", 
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

void filter(const sensor_msgs::PointCloud2 &cloud_in)
{
  if ( 0 == cloud_in.width * cloud_in.height )
    {
      ROS_ERROR("Input cloud is empty!");
      return;
    }
  ROS_DEBUG("Input cloud size: %d", cloud_in.width * cloud_in.height );
      
  pcl::PCLPointCloud2::Ptr cloud_voxel (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_floor (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_floor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  
  //pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_filter;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  
  pcl::PCLPointCloud2::Ptr pcl_pc (new pcl::PCLPointCloud2 ());      
  pcl_conversions::toPCL(cloud_in, *pcl_pc);
        
  voxel_filter.setInputCloud (pcl_pc);
  voxel_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
  voxel_filter.filter (*cloud_voxel);

  if ( 0 == cloud_voxel->width * cloud_voxel->height )
    {
      ROS_ERROR("Voxel cloud is empty!");
      return;
    }

  ROS_DEBUG("Voxel cloud size: %d", cloud_voxel->width * cloud_voxel->height );

  pcl::fromPCLPointCloud2(*cloud_voxel, *pcl_cloud);
  pcl::fromPCLPointCloud2(*cloud_voxel, *temp_cloud);

  if ( 0 == pcl_cloud->points.size() )
    {
      ROS_ERROR("PCL cloud is empty!");
      return;
    }

  ROS_DEBUG("PCL cloud size: %zd", pcl_cloud->points.size() );


  // prepare identity transform
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) );
  bool found = false;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


  int it;
  for (it = 0; it < 2; it ++)
    {

      // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

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

      if (inliers->indices.size() == 0)
	{
	  ROS_ERROR("Could not estimate a planar model for the given dataset!");
	  pub_obstacles.publish(*cloud_voxel);
	  return;
	}
      ROS_DEBUG("Model inliers: %zd", inliers->indices.size() );

      // computing transform based on normal vector
      float x = coefficients->values[0];
      float y = coefficients->values[1];
      float z = coefficients->values[2];
      float d = coefficients->values[3];

      if ( d > 0 )
	{
	  x = -x;
	  y = -y;
	  z = -z;
	} else {
	d = - d;
      }

      float roll = atan2(z,-x) - PI/2.0f;
      float pitch = atan2(z,y) - PI/2.0f + kinect_pitch_angle;
      roll = wrap_angle(roll);
      pitch = wrap_angle(pitch);
      avg_pitch = (1 - ALPHA) * avg_pitch + ALPHA * pitch;
      avg_roll = (1 - ALPHA) * avg_roll + ALPHA * roll;
      
      if ( fabs(avg_roll) > MAX_ROLL_ANGLE || fabs(avg_pitch) > MAX_PITCH_ANGLE || fabs(d) > MAX_DIST || fabs(d) < MIN_DIST)
	{
	  ROS_DEBUG("SACMODEL_PLANE coefficients dont seem right (pitch: %f, roll: %f, height: %f).\n Publishing entire cloud as obstacles.", avg_pitch, avg_roll, d);
	  // prepare for second iteration
	  pcl::ExtractIndices<pcl::PointXYZ> extract;
	  extract.setInputCloud (pcl_cloud);
	  extract.setIndices (inliers);
	  extract.setNegative (true);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  extract.filter(*filtered_cloud);

	  extract.setNegative (false);
	  extract.filter(*temp_cloud);

	  pcl_cloud = filtered_cloud;
	  continue;
	} else {
	// found floor, set transform
	ROS_DEBUG("found floor in %d iteration! (pitch: %f, roll: %f, height: %f)", (it+1), avg_pitch, avg_roll, d);
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion quaternion;
	quaternion.setEuler(avg_pitch, avg_roll, 0.0d);
	transform.setRotation( quaternion );
	found = true;
	break;
      }
    }

  // publishing transform
  tf_broadcaster->sendTransform(tf::StampedTransform(transform, cloud_in.header.stamp, tf_frame, tf_target_frame));

  if (!found)
    {
      // publish everything as obstacles
      pub_obstacles.publish(cloud_voxel);
      return; 
    }

  if (it > 0)
    *pcl_cloud += *temp_cloud;
  //pcl::concatenateFields(pcl_cloud, temp_cloud, pcl_cloud);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (pcl_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_without_floor);

  if ( 0 == cloud_without_floor->points.size() )
    {
      ROS_ERROR("Cloud without cloud is empty!");
      return;
    }

  ROS_DEBUG("Cloud without floor: %zd", cloud_without_floor->points.size() );
    
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
  outlier_filter.setInputCloud(cloud_without_floor);
  outlier_filter.setRadiusSearch(outlier_radius);
  outlier_filter.setMinNeighborsInRadius(outlier_neighbors);
  outlier_filter.filter(*cloud_without_floor_filtered);

  if ( 0 == cloud_without_floor_filtered->points.size() )
    {
      ROS_ERROR("Filtered cloud is empty!");
      return;
    }

  ROS_DEBUG("Filtered cloud w/o floor: %zd", cloud_without_floor_filtered->points.size() );

  sensor_msgs::PointCloud2::Ptr cloud_obstacles (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_without_floor_filtered, *cloud_obstacles);

  // publish 
  //      cloud_obstacles->msg.header = cloud_in->msg.header;
  pub_obstacles.publish(cloud_obstacles);
  pub_voxel.publish(cloud_voxel);

}

void cb_pointcloud2(const sensor_msgs::PointCloud2 &cloud_in)
{
  filter(cloud_in);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "mitro_kinect");
  
  dynamic_reconfigure::Server<mitro_kinect::MitroKinectConfig> server;
  dynamic_reconfigure::Server<mitro_kinect::MitroKinectConfig>::CallbackType f;
  
  f = boost::bind(&cb_dynreconf, _1, _2);
  server.setCallback(f);

  ros::NodeHandle nh;
  
  tf_broadcaster = new tf::TransformBroadcaster();
  
  sub_depth = nh.subscribe(topic_cloud_in, 1, &cb_pointcloud2);
  pub_obstacles = nh.advertise<sensor_msgs::PointCloud2>(topic_cloud_obstacles, 1);
  pub_voxel = nh.advertise<sensor_msgs::PointCloud2>(topic_cloud_voxel, 1);
  
  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
