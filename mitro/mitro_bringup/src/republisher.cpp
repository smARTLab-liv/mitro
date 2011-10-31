#include <ros/ros.h>
#include <mitro_diagnostics/SysInfo.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher sys_pub, voxel_full_pub, voxel_obst_pub;

void sysinfo_cb(const mitro_diagnostics::SysInfo::ConstPtr& msg) {
    sys_pub.publish(*msg);
}

void voxel_full_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    voxel_full_pub.publish(*msg);
}

void voxel_obst_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    voxel_obst_pub.publish(*msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "republisher");
    ros::NodeHandle nh;
    
    std::string source_topic, target_topic, costmap_topic;
    
    ros::Subscriber sys_sub = nh.subscribe<mitro_diagnostics::SysInfo>("sysinfo", 10, sysinfo_cb);
    ros::Subscriber voxel_full_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud_voxel", 10, voxel_full_cb);
    ros::Subscriber voxel_obst_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud_obstacles", 10, voxel_obst_cb);
    sys_pub = nh.advertise<mitro_diagnostics::SysInfo>("republisher/sysinfo", 10);
    voxel_full_pub = nh.advertise<sensor_msgs::PointCloud2>("republisher/cloud_voxel", 10);
    voxel_obst_pub = nh.advertise<sensor_msgs::PointCloud2>("republisher/cloud_obstacles", 10);
      
    ros::spin();
    return 0;
}
