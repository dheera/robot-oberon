#ifndef _pololu_qik_hpp
#define _pololu_qik_hpp

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace pololu_qik
{

class pololu_qik
{
public:
	pololu_qik( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );

	bool open( );
	void close( );
	bool start( );
	void stop( );
	bool spin_once( );
private:
	bool is_open( ) const;
	void command_callback(const std_msgs::Float32MultiArrayPtr &msg);
	bool set( int device_id, int channel, double speed );

        uint64_t last_command_time;
	int num_devices;

	std::string port;
	int fd;

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Subscriber sub_command;
};

}

#endif /* _pololu_qik_hpp */

