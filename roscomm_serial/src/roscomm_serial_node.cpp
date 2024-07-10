#include <ros/ros.h>
#include "roscomm_msgs/roscomm_serial.h"
#include "roscomm_serial/roscomm_serial.hpp"
#include "roscomm_serial/roscomm_serial_protocol.hpp"

int main (int argc, char** argv){
  ros::init(argc, argv, "roscomm_serial_node");
  ros::NodeHandle nh("~");

  std::string port;
  int baudrate, timeout, max_bufsize, stx, etx;

  nh.param<std::string>("port", port, "/dev/ttyUSB0");
  nh.param<int>("baudrate", baudrate, 9600);
  nh.param<int>("timeout", timeout, 1000);
  nh.param<int>("bufsize", max_bufsize, 1024);
  nh.param<int>("stx", stx, 0xFF);
  nh.param<int>("etx", etx, 0xFE);

  ROS_INFO("port : %s, baudrate: %d, timeout: %d, bufsize: %d, stx: %d, etx:%d", port.c_str(), baudrate, timeout, max_bufsize, stx, etx);
  roscomm::RosCommSerialProtocol<roscomm_msgs::roscomm_serial> SerialProtocol(nh, port, baudrate, timeout, max_bufsize, stx, etx);

  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
    ros::spinOnce();
    
    SerialProtocol.HandleSerialRead();    
     
    loop_rate.sleep();
  }
}
