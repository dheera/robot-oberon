#include <imu_bno055/imu_bno055.hpp>

int main(int argc, char *argv[]) {
	ros::NodeHandle *nh = NULL;
	ros::NodeHandle *nh_priv = NULL;
	imu_bno055::imu_bno055 *imu = NULL;

	ros::init(argc, argv, "imu_bno055_node");

	nh = new ros::NodeHandle( );
	if(!nh) {
		ROS_FATAL( "Failed to initialize NodeHanlde" );
		ros::shutdown( );
		return -1;
	}

	nh_priv = new ros::NodeHandle("~");
	if( !nh_priv ) {
		ROS_FATAL("Failed to initialize private NodeHanlde");
		delete nh;
		ros::shutdown( );
		return -2;
	}

	imu = new imu_bno055::imu_bno055( *nh, *nh_priv );
	if( !imu ) {
		ROS_FATAL( "Failed to initialize driver" );
		delete nh_priv;
		delete nh;
		ros::shutdown( );
		return -3;
	}
	if( !imu->start( ) )
		ROS_ERROR( "Failed to start the driver" );

	ros::Rate rate(20);
	while(1) {
          rate.sleep();
          ros::spinOnce();
          imu->spin_once();
	}

	delete imu;
	delete nh_priv;
	delete nh;

	return 0;
}
