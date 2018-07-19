#include "imu_bno055/imu_bno055.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

// 54 bytes
typedef struct {
  int8_t chip_id_bno055;
  int8_t chip_id_acc;
  int8_t chip_id_mag;
  int8_t chip_id_gyro;
  int16_t sw_revision_id;
  int8_t bootloader_version;
  int8_t page_id;
  int16_t raw_linear_acceleration_x;
  int16_t raw_linear_acceleration_y;
  int16_t raw_linear_acceleration_z;
  int16_t raw_magnetic_field_x;
  int16_t raw_magnetic_field_y;
  int16_t raw_magnetic_field_z;
  int16_t raw_angular_velocity_x;
  int16_t raw_angular_velocity_y;
  int16_t raw_angular_velocity_z;
  int16_t fused_heading;
  int16_t fused_roll;
  int16_t fused_pitch;
  int16_t fused_orientation_w;
  int16_t fused_orientation_x;
  int16_t fused_orientation_y;
  int16_t fused_orientation_z;
  int16_t fused_linear_acceleration_x;
  int16_t fused_linear_acceleration_y;
  int16_t fused_linear_acceleration_z;
  int16_t temperature;
  int16_t calibration_status;
} imu_record;

namespace imu_bno055 {

imu_bno055::imu_bno055( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
	port(""),
	nh(_nh),
	seq(0),
	nh_priv( _nh_priv )
{
	ROS_INFO("initializing");
	nh_priv.param( "port", port, (std::string)"/dev/imu0" );
	nh_priv.param( "baud", baud, (int)115200 );
	nh_priv.param( "frame_id", frame_id, (std::string)"imu" );
}

bool imu_bno055::open() {
	try {
	ser.setPort(port);
	ser.setBaudrate(baud);
	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	ser.setTimeout(to);
	ser.open();
	} catch(serial::IOException& e) {
          ROS_ERROR_STREAM("cannot open serial port");
	  exit(EXIT_FAILURE);
	}

	if(!ser.isOpen()) {
          ROS_ERROR_STREAM("cannot open serial port");
	  exit(EXIT_FAILURE);
	}

	return true;
}

void imu_bno055::close() {
	ROS_INFO("closing port");
	ser.close();
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

  if(!ser.available()) {
    return false;
  }

  int header_bytes = 0;
  imu_record record;

  unsigned char c = 0;
  while(header_bytes < 4) {
    ser.read(&c, 1);
    if(c == 0xF0) header_bytes++;
    else header_bytes = 0;
  }

  size_t bytes_read;

  bytes_read = ser.read((uint8_t*)&record, 54);

  ROS_DEBUG_STREAM("read " << bytes_read << " bytes");

  if(bytes_read < 54) {
    return false;
  }

  seq++;

  sensor_msgs::Imu msg_raw;
  msg_raw.header.stamp = time;
  msg_raw.header.frame_id = frame_id;
  msg_raw.header.seq = seq;
  msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
  msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
  msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
  msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
  msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
  msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

  sensor_msgs::MagneticField msg_mag;
  msg_mag.header.stamp = time;
  msg_mag.header.frame_id = frame_id;
  msg_mag.header.seq = seq;
  msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
  msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
  msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

  sensor_msgs::Imu msg_data;
  msg_data.header.stamp = time;
  msg_data.header.frame_id = frame_id;
  msg_data.header.seq = seq;
  msg_data.orientation.w = (double)record.fused_orientation_w;
  msg_data.orientation.x = (double)record.fused_orientation_x;
  msg_data.orientation.y = (double)record.fused_orientation_y;
  msg_data.orientation.z = (double)record.fused_orientation_z;
  msg_data.linear_acceleration.x = (double)record.fused_linear_acceleration_x / 100.0;
  msg_data.linear_acceleration.y = (double)record.fused_linear_acceleration_y / 100.0;
  msg_data.linear_acceleration.z = (double)record.fused_linear_acceleration_z / 100.0;
  msg_data.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
  msg_data.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
  msg_data.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

  sensor_msgs::Temperature msg_temp;
  msg_temp.header.stamp = time;
  msg_temp.header.frame_id = frame_id;
  msg_temp.header.seq = seq;
  msg_temp.temperature = (double)record.temperature;

  pub_data.publish(msg_data);
  pub_raw.publish(msg_raw);
  pub_mag.publish(msg_mag);
  pub_temp.publish(msg_temp);

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
	return ser.isOpen();
}

void spin_once() {

}

}
