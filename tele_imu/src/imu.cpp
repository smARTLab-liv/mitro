#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <time.h>
#include <iomanip>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


const float GYRO_X_MEAN = 381.6f;
const float GYRO_X_SCALE = 0.322f;

const float GYRO_Z_MEAN = 381.6f;
const float GYRO_Z_SCALE = 0.322f;

const float ACC_X_MEAN = 512.0f;
const float ACC_X_SCALE = 0.004028f;

const float ACC_Y_MEAN = 512.0f;
const float ACC_Y_SCALE = 0.004028f;

const float ACC_Z_MEAN = 760.242f;
const float ACC_Z_SCALE = 0.004028f;

const float DEG_TO_RAD = 1/180.0f;

const float G = 9.8112f;

void sync(boost::asio::serial_port &port) {
  std::vector<unsigned char> data(1);
  while (1) {
    boost::asio::read(port, boost::asio::buffer(data));
    if (data[0] == char(10)) break;
  }
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imu");
  ros::NodeHandle n;

  std::string name = "/dev/ttyUSB0";
  boost::asio::io_service io;
  boost::asio::serial_port port( io, name );
  port.set_option( boost::asio::serial_port_base::baud_rate( 1000000 ) );
  port.set_option( boost::asio::serial_port_base::flow_control( boost::asio::serial_port_base::flow_control::none ) );
  port.set_option( boost::asio::serial_port_base::parity( boost::asio::serial_port_base::parity::none) );
  port.set_option( boost::asio::serial_port_base::stop_bits( boost::asio::serial_port_base::stop_bits::one ) );
  port.set_option( boost::asio::serial_port_base::character_size( 8 ) );  
  // std::cout << "open? " << port.is_open() << std::endl;

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);

  // int count = 0;
  ros::Time last = ros::Time::now();

  std::vector<unsigned char> data(12);

  sensor_msgs::Imu msg;
  msg.header.frame_id = "imu";

  //msg.angular_velocity_covariance = { 1.f, .0f, .0f, .0f, 1.f, .0f, .0f, .0f, 1.f};
  //msg.linear_acceleration_covariance = { 1.f, .0f, .0f, .0f, 1.f, .0f, .0f, .0f, 1.f};

  sync(port);
  while (ros::ok()) {
    boost::asio::read(port, boost::asio::buffer(data));
    if (data[11] != char(10)) {
      std::cout << "sync error" << std::endl;
      sync(port);
      continue;
    }
  
    //    count++;
    
    // // hex output
    // for (std::vector<unsigned char>::iterator it = data.begin(); it!=data.end(); ++it) {
    //    unsigned char c = *it;
    //    std::cout << std::setw( 2 ) << std::setfill( '0' ) << std::hex << std::uppercase << (int) c;       std::cout << " ";
    // }
    // std::cout << std::endl;
 
    msg.header.stamp = ros::Time::now();

    int m = data[10];
    int gx = ((data[0] << 8) | data[1]) / m;
    int gz = ((data[2] << 8) | data[3]) / m;
    int ax = ((data[4] << 8) | data[5]) / m;
    int ay = ((data[6] << 8) | data[7]) / m;
    int az = ((data[8] << 8) | data[9]) / m;

    // convert gyro
    float rad_gx = (float) (gx - GYRO_X_MEAN) * GYRO_X_SCALE * DEG_TO_RAD; 
    float rad_gz = (float) (gz - GYRO_Z_MEAN) * GYRO_Z_SCALE * DEG_TO_RAD; 

    float ms2_ax = (float) (ax - ACC_X_MEAN) * ACC_X_SCALE * 1/G; 
    float ms2_ay = (float) (ay - ACC_Y_MEAN) * ACC_Y_SCALE * 1/G; 
    float ms2_az = (float) (az - ACC_Z_MEAN) * ACC_Z_SCALE * 1/G; 

    msg.angular_velocity.x = rad_gx;
    msg.angular_velocity.z = rad_gz;

    msg.linear_acceleration.x = ms2_ax;
    msg.linear_acceleration.y = ms2_ay;
    msg.linear_acceleration.z = ms2_az;

    imu_pub.publish(msg);

    // if (count % 100 == 0) {

    //   printf("%5d %5d %5d %5d %5d %2d\n",gx,gz,ax,ay,az,m);
    //   printf("%8.6f\n", (ros::Time::now() - last).toSec() / (double)count);
    // }

    //last = now;
    
  }

  return 0;
}
