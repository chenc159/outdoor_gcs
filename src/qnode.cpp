/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/outdoor_gcs/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace outdoor_gcs {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"outdoor_gcs");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	uav_state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, &QNode::state_callback, this);
	uav_imu_sub = n.subscribe<Imu>("/mavros/imu/data", 1, &QNode::imu_callback, this);
	uav_gps_sub = n.subscribe<Gpsraw>("/mavros/gpsstatus/gps1/raw", 1, &QNode::gps_callback, this);
	uav_bat_sub = n.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1, &QNode::bat_callback, this);
	uav_from_sub = n.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 1, &QNode::from_callback, this);
	uav_arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	uav_setmode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	uav_sethome_client = n.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");
	uav_takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	uav_land_client = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		ros::spinOnce();

		uav_received.rosReceived = false;
		uav_received.gpsReceived = false;
		if (uav_received.preros){
			uav_received.rosReceived = true;
		}
		if (uav_received.pregps){
			uav_received.gpsReceived = true;
		}
		uav_received.preros = false;
		uav_received.pregps = false;

		/* signal a ros loop update  */
		Q_EMIT rosLoopUpdate();
		loop_rate.sleep();
		// ++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::state_callback(const mavros_msgs::State::ConstPtr &msg){
	uav_state = *msg;
}

void QNode::imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
	uav_imu = *msg;
	uav_received.preros = true;
}

void QNode::gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg){
	uav_gps = *msg;
	uav_received.pregps = true;
}

void QNode::bat_callback(const sensor_msgs::BatteryState::ConstPtr &msg){
	uav_bat = *msg;
}

void QNode::from_callback(const mavros_msgs::Mavlink::ConstPtr &msg){
	uav_from = *msg;
}

void QNode::Set_Arm(bool arm_disarm){
	uav_arm.request.value = arm_disarm;
	uav_arming_client.call(uav_arm);
}

void QNode::Set_Mode(std::string command_mode){
	uav_setmode.request.custom_mode = command_mode;
	uav_setmode_client.call(uav_setmode);
}

void QNode::Set_Home(){
	uav_sethome.request.current_gps = true;
	uav_sethome_client.call(uav_sethome);
	// std::cout << uav_sethome.response.success << std::endl;
}

mavros_msgs::State QNode::GetState(){
	return uav_state;
}

Gpsraw QNode::GetGPS(){
	return uav_gps;
}

 sensor_msgs::BatteryState QNode::GetBat(){
	return uav_bat;
}

mavros_msgs::Mavlink QNode::GetFrom(){
	return uav_from;
}

outdoor_gcs::signalRec QNode::Update_uav_signal(){
	return uav_received;
}

}  // namespace outdoor_gcs
