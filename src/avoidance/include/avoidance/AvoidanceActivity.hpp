#ifndef _AvoidanceActivity_hpp
#define _AvoidanceActivity_hpp

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

namespace avoidance {

class AvoidanceActivity {
  public:
	AvoidanceActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);

	bool start();
	bool stop();
	bool spinOnce();
    void onScan(const sensor_msgs::LaserScanPtr& msg);

  private:

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;

    std::string ns_motion;
    std::string ns_lidar;
    bool inverted;
    double angle_offset;

	ros::Publisher pub_max;
	ros::Publisher pub_min;
	ros::Subscriber sub_scan;
};

}

#endif /* _AvoidanceActivity_hpp */

