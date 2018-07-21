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
    void onWinning(const geometry_msgs::TwistPtr& msg);

  private:

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;

    std::string ns_motion;
    std::string ns_lidar;
    bool inverted;
    double angle_offset;
    double avoidance_width;

    double motion_angle;

	ros::Publisher pub_multiplier;
	ros::Subscriber sub_scan;
	ros::Subscriber sub_winning;
};

}

#endif /* _AvoidanceActivity_hpp */

