#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

int main (int argc, char** argv) 
{

  pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
    cloud_floor (new pcl::PointCloud<pcl::PointXYZ>);


  std::string fn = "cloud_84.pcd";
  if (argc > 1)
    fn = argv[1];

  ROS_INFO("reading cloud from file %s ...", fn.c_str());
  pcl::io::loadPCDFile(fn, rgb_cloud);

  copyPointCloud(rgb_cloud, *cloud);

  // std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  // for (size_t i = 0; i < cloud.points.size (); ++i)
  //   std::cerr << "    " << cloud.points[i].x << " " 
  // 	      << cloud.points[i].y << " " 
  // 	      << cloud.points[i].z << std::endl;


  ROS_INFO("Starting ...");

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  // seg.setInputCloud (cloud.makeShared ());
  // seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);


  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  	    << coefficients->values[1] << " "
  	    << coefficients->values[2] << " " 
  	    << coefficients->values[3] << std::endl;
  
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    return -1;
  }

  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_without_floor);

  ROS_INFO("done!");


  coefficients->values[3] += 0.1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);


  std::cerr << "PointCloud representing the planar component: " << cloud_floor->width * cloud_floor->height << " data points." << std::endl;


  std::string out_fn = "output.pcd";
  pcl::io::savePCDFileASCII(out_fn.c_str(), *cloud_floor);



      // // Create the filtering object
      // extract.setNegative (true);
      // extract.filter (*cloud_filtered);




  

  // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  // for (size_t i = 0; i < inliers->indices.size (); ++i)
  //   std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
  // 	      << cloud.points[inliers->indices[i]].y << " "
  // 	      << cloud.points[inliers->indices[i]].z << std::endl;

  return (0);
}