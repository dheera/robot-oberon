#include "avoidance/AvoidanceActivity.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

namespace avoidance {

AvoidanceActivity::AvoidanceActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
	nh(_nh),
	nh_priv( _nh_priv )
{
  ROS_INFO("initializing");
  nh_priv.param("ns_motion", ns_motion, (std::string)"/motion");
  nh_priv.param("ns_lidar", ns_lidar, (std::string)"/lidar");
  nh_priv.param("inverted", inverted, (bool)false);
  nh_priv.param("angle_offset", angle_offset, (double)0.0);
}

bool AvoidanceActivity::start() {
  ROS_INFO("starting");

  if(!pub_max) pub_max = nh.advertise<geometry_msgs::Twist>(ns_motion + "max", 1);
  if(!pub_max) pub_max = nh.advertise<geometry_msgs::Twist>(ns_motion + "min", 1);

  if(!sub_scan) {
    sub_scan = nh.subscribe(ns_lidar + "/scan", 1, &AvoidanceActivity::onScan, this);
  }

  return true;
}

bool AvoidanceActivity::spinOnce() {
  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

  return true;	
}

bool AvoidanceActivity::stop() {
  ROS_INFO("stopping");

  if(pub_max) pub_max.shutdown();
  if(pub_min) pub_min.shutdown();
  if(sub_scan) sub_scan.shutdown();

  return true;
}

void AvoidanceActivity::onScan(const sensor_msgs::LaserScanPtr& msg) {
 
}

}
