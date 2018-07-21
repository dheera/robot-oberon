#ifndef _imu_bno055_hpp
#define _imu_bno055_hpp

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include "serial/serial.h"

namespace imu_bno055 {

class imu_bno055 {
  public:
	imu_bno055( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );

	bool open( );
	void close( );
	bool start( );
	void stop( );
	bool spin_once( );
  private:
	bool is_open( ) const;

	std::string frame_id;
	std::string port;
	int baud;
	int seq;

	serial::Serial ser;
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Publisher pub_data;
	ros::Publisher pub_raw;
	ros::Publisher pub_mag;
	ros::Publisher pub_temp;
};

}

#endif /* _imu_bno055_hpp */

