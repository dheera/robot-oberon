#include "imu_bno055/imu_bno055.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

namespace imu_bno055 {

imu_bno055::imu_bno055( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
	port( "" ),
	fd( -1 ),
	nh( _nh ),
	nh_priv( _nh_priv )
{
	ROS_INFO("initializing");
	nh_priv.param( "port", port, (std::string)"/dev/imu0" );
}

bool imu_bno055::open( )
{
	struct termios fd_options;
	unsigned char baud_autodetect = 0xAA;

	if(is_open()) {
		ROS_INFO("port already open; closing to reopen");
		close();
	}

	fd = ::open(port.c_str( ), O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd < 0) {
		ROS_FATAL("failed to open port: %s", strerror(errno));
		return false;
	}

	if(0 > fcntl(fd, F_SETFL, 0)) {
		ROS_FATAL("failed to set port descriptor: %s", strerror(errno));
		return false;
	}

	if(0 > tcgetattr(fd, &fd_options)) {
		ROS_FATAL("failed to fetch port attributes: %s", strerror(errno));
		return false;
	}

	if(0 > cfsetispeed(&fd_options, B9600))	{
		ROS_FATAL("failed to set input baud: %s", strerror(errno));
		return false;
	}

	if(0 > cfsetospeed(&fd_options, B9600)) {
		ROS_FATAL("failed to set output baud: %s", strerror(errno));
		return false;
	}

	if(0 > tcsetattr(fd, TCSANOW, &fd_options)) {
		ROS_FATAL("failed to set port attributes: %s", strerror(errno));
		return false;
	}

	if( 0 > write( fd, &baud_autodetect, 1 ) ) {
		ROS_FATAL("failed to initialize device: %s", strerror(errno));
		return false;
	}

	return true;
}

void imu_bno055::close() {
	ROS_INFO("closing port");
	::close(fd);
}

bool imu_bno055::start() {
	if(!is_open() && !open()) return false;

	ROS_INFO("starting");

	if(!pub_data) pub_data = nh.advertise<sensor_msgs::Imu>("data", 1);
	if(!pub_raw) pub_raw = nh.advertise<sensor_msgs::Imu>("raw", 1);
	if(!pub_mag) pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag", 1);
	if(!pub_temp) pub_temp = nh.advertise<sensor_msgs::Temperature>("temp", 1);

	return true;
}

bool imu_bno055::spin_once( ) {
  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

  char read_buffer[64];
  char c = 0;
  while(c != 0xF0) {
    read(fd, &c, 1);
  }

  read(fd, &read_buffer, 45);


  return true;	
}

void imu_bno055::stop() {
	ROS_INFO("stopping");

	if(pub_data) pub_data.shutdown();
	if(pub_raw) pub_raw.shutdown();
	if(pub_mag) pub_mag.shutdown();
	if(pub_temp) pub_temp.shutdown();

	close();
}

bool imu_bno055::is_open() const {
	return (fd >= 0);
}

void spin_once() {

}

}
