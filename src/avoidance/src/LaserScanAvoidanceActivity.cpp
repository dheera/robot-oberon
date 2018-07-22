#include "avoidance/LaserScanAvoidanceActivity.hpp"

#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

namespace avoidance {

LaserScanAvoidanceActivity::LaserScanAvoidanceActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
	nh(_nh),
	nh_priv(_nh_priv)
{
  ROS_INFO("initializing");
  nh_priv.param("ns_motion", ns_motion, (std::string)"/motion");
  nh_priv.param("ns_lidar", ns_lidar, (std::string)"/lidar");
  nh_priv.param("inverted", inverted, (bool)true);
  nh_priv.param("angle_offset", angle_offset, (double)0.0);
  nh_priv.param("avoidance_width", avoidance_width, (double)0.3);
}

bool LaserScanAvoidanceActivity::start() {
  ROS_INFO("starting");

  if(!pub_multiplier) pub_multiplier = nh.advertise<std_msgs::Float32>(ns_motion + "/multiplier", 1);

  if(!sub_scan) {
    sub_scan = nh.subscribe(ns_lidar + "/scan", 1, &LaserScanAvoidanceActivity::onScan, this);
  }

  if(!sub_winning) {
    sub_winning = nh.subscribe(ns_motion + "/winning", 1, &LaserScanAvoidanceActivity::onWinning, this);
  }

  return true;
}

bool LaserScanAvoidanceActivity::spinOnce() {
  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

  return true;	
}

bool LaserScanAvoidanceActivity::stop() {
  ROS_INFO("stopping");

  if(pub_multiplier) pub_multiplier.shutdown();
  if(sub_scan) sub_scan.shutdown();
  if(sub_winning) sub_winning.shutdown();

  return true;
}

void LaserScanAvoidanceActivity::onWinning(const geometry_msgs::TwistPtr& msg) {
  if(abs(msg->linear.x) < 0.001 && abs(msg->linear.y) < 0.001) return;
  motion_angle = atan2(msg->linear.y, msg->linear.x);
}

void LaserScanAvoidanceActivity::onScan(const sensor_msgs::LaserScanPtr& msg) {
  double theta;
  double r;
  int i;

  double multiplier_min = 1.0;

  for(i=0;i<msg->ranges.size();i++) {
    r = msg->ranges[i];
    if(r > 0.7) continue;

    if(inverted) {
      theta = angle_offset + msg->angle_min - msg->angle_increment * i;
    } else {
      theta = angle_offset + msg->angle_min + msg->angle_increment * i;
    }

    if(cos(theta - motion_angle) > 0.5 && std::abs(r*sin(theta - motion_angle)) < avoidance_width/2) {
      if(r < 0.3) {
         ROS_WARN_STREAM_THROTTLE(2, "obstacle in path");
      }
      multiplier_min = std::min(
              multiplier_min, 
              pow((std::max(r, 0.25) - 0.25) / 0.7, 0.5)
      );
    }
  }

  ROS_DEBUG_STREAM("multiplier = " << multiplier_min);

  std_msgs::Float32 msg_multiplier;
  msg_multiplier.data = multiplier_min;
  pub_multiplier.publish(msg_multiplier);

}

}
