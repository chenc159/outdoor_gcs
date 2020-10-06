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
// #include "../include/outdoor_gcs/math_utils.h"

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
	uav_gpsG_sub = n.subscribe<Gpsglobal>("/mavros/global_position/global", 1, &QNode::gpsG_callback, this);
	uav_gpsL_sub = n.subscribe<Gpslocal>("/mavros/global_position/local", 1, &QNode::gpsL_callback, this);
	uav_gpsH_sub = n.subscribe<GpsHomePos>("/mavros/home_position/home", 1, &QNode::gpsH_callback, this);
	uav_bat_sub = n.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1, &QNode::bat_callback, this);
	uav_from_sub = n.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 1, &QNode::from_callback, this);

	// uav_setpoint_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	uav_setpoint_pub = n.advertise<PosTarg>("/mavros/setpoint_raw/local", 1);

	uav_arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	uav_setmode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	uav_sethome_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
	// uav_takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	// uav_land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		pub_command();
		ros::spinOnce();

		uav_received.stateReceived = false;
		uav_received.imuReceived = false;
		uav_received.gpsReceived = false;
		uav_received.gpsGReceived = false;
		uav_received.gpsLReceived = false;
		uav_received.gpsHReceived = false;
		if (uav_received.prestate){
			uav_received.stateReceived = true;
		}
		if (uav_received.preimu){
			uav_received.imuReceived = true;
		}
		if (uav_received.pregps){
			uav_received.gpsReceived = true;
		}
		if (uav_received.pregpsG){
			uav_received.gpsGReceived = true;
		}
		if (uav_received.pregpsL){
			uav_received.gpsLReceived = true;
		}
		if (uav_received.pregpsH){
			uav_received.gpsHReceived = true;
		}
		uav_received.prestate = false;
		uav_received.preimu = false;
		uav_received.pregps = false;
		uav_received.pregpsG = false;
		uav_received.pregpsL = false;
		uav_received.pregpsH = false;

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
	uav_received.prestate = true;
}
void QNode::imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
	uav_imu = *msg;
	uav_received.preimu = true;
}
void QNode::gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg){
	uav_gps = *msg;
	uav_received.pregps = true;
}
void QNode::gpsG_callback(const Gpsglobal::ConstPtr &msg){
	uav_gpsG = *msg;
	uav_received.pregpsG = true;
}
void QNode::gpsL_callback(const Gpslocal::ConstPtr &msg){
	uav_gpsL = *msg;
	uav_received.pregpsL = true;
}
void QNode::gpsH_callback(const GpsHomePos::ConstPtr &msg){
	uav_gpsH = *msg;
	uav_received.pregpsH = true;
}
void QNode::bat_callback(const sensor_msgs::BatteryState::ConstPtr &msg){
	uav_bat = *msg;
}
void QNode::from_callback(const mavros_msgs::Mavlink::ConstPtr &msg){
	uav_from = *msg;
}

void QNode::pub_command(){
	uav_setpoint_pub.publish(uav_setpoint);
}

void QNode::Set_Arm(bool arm_disarm){
	uav_arm.request.value = arm_disarm;
	uav_arming_client.call(uav_arm);
}

void QNode::Set_Mode(std::string command_mode){
	uav_setmode.request.custom_mode = command_mode;
	uav_setmode_client.call(uav_setmode);
	// std::cout << uav_setmode.response.mode_sent << std::endl;
}

void QNode::Set_Home(){
	uav_sethome.request.current_gps = true;
	// uav_sethome.request.yaw = 0.0;
	// uav_sethome.request.latitude = uav_gps.lat*1e-7;
	// uav_sethome.request.longitude = uav_gps.lon*1e-7;
	// uav_sethome.request.altitude = uav_gps.alt/1000.0;
	uav_sethome_client.call(uav_sethome);
	// std::cout << uav_sethome.response.success << std::endl;
}

void QNode::move_uav(float target[3], float target_yaw){
	uav_setpoint.header.stamp = ros::Time::now();
	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    uav_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    uav_setpoint.coordinate_frame = 1;
	uav_setpoint.position.x = target[0];
	uav_setpoint.position.y = target[1];
	uav_setpoint.position.z = target[2];
	uav_setpoint.yaw = target_yaw;
	// uav_setpoint.pose.position.x = target[0];
	// uav_setpoint.pose.position.y = target[1];
	// uav_setpoint.pose.position.z = target[2];
}

State QNode::GetState(){
	return uav_state;
}

Imu QNode::GetImu(){
	return uav_imu;
}

Gpsraw QNode::GetGPS(){
	return uav_gps;
}

Gpsglobal QNode::GetGPSG(){
	return uav_gpsG;
}

Gpslocal QNode::GetGPSL(){
	return uav_gpsL;
}

GpsHomePos QNode::GetGPSH(){
	return uav_gpsH;
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
