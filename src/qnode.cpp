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
	
	// uav_state_sub 	= n.subscribe<mavros_msgs::State>("/mavros/state", 1, &QNode::state_callback, this);
	uav_imu_sub 	= n.subscribe<Imu>("/mavros/imu/data", 1, &QNode::imu_callback, this);
	uav_gps_sub 	= n.subscribe<Gpsraw>("/mavros/gpsstatus/gps1/raw", 1, &QNode::gps_callback, this);
	uav_gpsG_sub 	= n.subscribe<Gpsglobal>("/mavros/global_position/global", 1, &QNode::gpsG_callback, this);
	uav_gpsL_sub 	= n.subscribe<Gpslocal>("/mavros/global_position/local", 1, &QNode::gpsL_callback, this);
	uav_gpsH_sub 	= n.subscribe<GpsHomePos>("/mavros/home_position/home", 1, &QNode::gpsH_callback, this);
	uav_bat_sub 	= n.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1, &QNode::bat_callback, this);
	uav_from_sub 	= n.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 1, &QNode::from_callback, this);

	uav_setpoint_pub = n.advertise<PosTarg>("/mavros/setpoint_raw/local", 1);
	uav_gps_home_pub = n.advertise<GpsHomePos>("/mavros/global_position/home", 1);

	uav_arming_client 	= n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	uav_setmode_client 	= n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	uav_sethome_client 	= n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");

	for (int i = 0; i < DroneNumber ; i++) {
		// std::cout << "/uav" + std::to_string(i+1) + "/mavros/imu/data" << std::endl;
		// std::string name = "/uav" + std::to_string(i+1) + "/mavros/imu/data";
		uavs_state_sub[i]	= n.subscribe<mavros_msgs::State>("/uav" + std::to_string(i+1) + "/mavros/state", 1, std::bind(&QNode::uavs_state_callback, this, std::placeholders::_1, i));
		uavs_imu_sub[i] 	= n.subscribe<Imu>("/uav" + std::to_string(i+1) + "/mavros/imu/data", 1, std::bind(&QNode::uavs_imu_callback, this, std::placeholders::_1, i));
		uavs_gps_sub[i] 	= n.subscribe<Gpsraw>("/uav" + std::to_string(i+1) + "/mavros/gpsstatus/gps1/raw", 1, std::bind(&QNode::uavs_gps_callback, this, std::placeholders::_1, i));
		uavs_gpsG_sub[i] 	= n.subscribe<Gpsglobal>("/uav" + std::to_string(i+1) + "/mavros/global_position/global", 1, std::bind(&QNode::uavs_gpsG_callback, this, std::placeholders::_1, i));
		uavs_gpsL_sub[i] 	= n.subscribe<Gpslocal>("/uav" + std::to_string(i+1) + "/mavros/global_position/local", 1, std::bind(&QNode::uavs_gpsL_callback, this, std::placeholders::_1, i));
		uavs_from_sub[i] 	= n.subscribe<mavros_msgs::Mavlink>("/uav" + std::to_string(i+1) + "/mavlink/from", 1, std::bind(&QNode::uavs_from_callback, this, std::placeholders::_1, i));
	
		uavs_setpoint_pub[i] 		= n.advertise<PosTarg>("/uav" + std::to_string(i+1) + "/mavros/setpoint_raw/local", 1);
		uavs_setpoint_alt_pub[i] 	= n.advertise<AltTarg>("/uav" + std::to_string(i+1) + "/mavros/setpoint_raw/attitude", 1);
		uavs_gps_home_pub[i] 		= n.advertise<GpsHomePos>("/uav" + std::to_string(i+1) + "/mavros/global_position/home", 1);

		uavs_arming_client[i] 	= n.serviceClient<mavros_msgs::CommandBool>("/uav" + std::to_string(i+1) +"/mavros/cmd/arming");
		uavs_setmode_client[i] 	= n.serviceClient<mavros_msgs::SetMode>("/uav" + std::to_string(i+1) +"/mavros/set_mode");
	
	}
	last_change = ros::Time::now();

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(10); // change update rate here

	while ( ros::ok() ) {

		pub_command();
		UAVS_Do_Plan(); // for multi-uav
		uavs_call_service(); // for multi-uav
		uavs_pub_command(); // for multi-uav
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
		// std::cout << uav_received.stateReceived << std::endl;


		//////////////// Multi-uav /////////////////

    	for (const auto &i : avail_uavind){
			UAVs_info[i].stateReceived = false;
			UAVs_info[i].imuReceived = false;
			UAVs_info[i].gpsReceived = false;
			UAVs_info[i].gpsLReceived = false;
			if (UAVs_info[i].prestateReceived){
				UAVs_info[i].stateReceived = true;
			}
			if (UAVs_info[i].preimuReceived){
				UAVs_info[i].imuReceived = true;
			}
			if (UAVs_info[i].pregpsReceived){
				UAVs_info[i].gpsReceived = true;
			}
			if (UAVs_info[i].pregpsLReceived){
				UAVs_info[i].gpsLReceived = true;
			}
			UAVs_info[i].prestateReceived = false;
			UAVs_info[i].preimuReceived = false;
			UAVs_info[i].pregpsReceived = false;
			UAVs_info[i].pregpsLReceived = false;
		}

		/* signal a ros loop update  */
		Q_EMIT rosLoopUpdate();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

///////////// Single uav ///////////////////
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
	uav_setpoint.header.stamp = ros::Time::now();
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
}
void QNode::Set_GPS_Home(){
	uav_gps_home.geo.latitude  = uav_gpsG.latitude;
	uav_gps_home.geo.longitude = uav_gpsG.longitude;
	uav_gps_home.geo.altitude  = uav_gpsG.altitude;
	uav_gps_home.orientation.x = uav_imu.orientation.x;
	uav_gps_home.orientation.y = uav_imu.orientation.y;
	uav_gps_home.orientation.z = uav_imu.orientation.z;
	uav_gps_home.orientation.w = uav_imu.orientation.w;
	uav_gps_home_pub.publish(uav_gps_home);
}

// void QNode::move_uav(float target[3], float target_yaw){
// 	uav_setpoint.header.stamp = ros::Time::now();
// 	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
//     //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
//     //Bit 10 should set to 0, means is not force sp
//     uav_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
//     uav_setpoint.coordinate_frame = 1;
// 	uav_setpoint.position.x = target[0];
// 	uav_setpoint.position.y = target[1];
// 	uav_setpoint.position.z = target[2];
// 	uav_setpoint.yaw = target_yaw;
// }
void QNode::move_uav(bool mask[3], float target[3]){
	uav_setpoint.header.stamp = ros::Time::now();
    uav_setpoint.coordinate_frame = 1;
	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
	if (mask[0]){
		uav_setpoint.type_mask = 0b100111111000;
		uav_setpoint.position.x = target[0];
		uav_setpoint.position.y = target[1];
		uav_setpoint.position.z = target[2];
	}
	else if (mask[1]){
		uav_setpoint.type_mask = 0b100111000111;
		uav_setpoint.velocity.x = target[0];
		uav_setpoint.velocity.y = target[1];
		uav_setpoint.velocity.z = target[2];
	}
	else if (mask[2]){
		uav_setpoint.type_mask = 0b100000111111;
		uav_setpoint.acceleration_or_force.x = target[0];
		uav_setpoint.acceleration_or_force.y = target[1];
		uav_setpoint.acceleration_or_force.z = target[2];
	}
	uav_setpoint.yaw = 0.0;
}

void QNode::move_uav_height(float height){
	// uav_setpoint.header.stamp = ros::Time::now();
	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    uav_setpoint.type_mask = 0b110111111011;
    uav_setpoint.coordinate_frame = 1;
	uav_setpoint.position.z = height;
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
outdoor_gcs::signalRec QNode::Get_uav_signal(){
	return uav_received;
}


////////////////////////////////////////////// Multi-uav ///////////////////////////////////////////////////

void QNode::uavs_call_service(){
	for (const auto &ind : avail_uavind){
		if (service_flag[ind] == 1){ // arm or disarm
			uavs_arming_client[ind].call(uavs_arm[ind]);
			service_flag[ind] = 0;
			// if(uavs_arming_client[ind].call(uavs_arm[ind]) && uavs_arm[ind].response.success){
				// break;
			// }
		}
		else if (service_flag[ind] == 2){ // set mode (AUTO.TAKEOFF, AUTO.LAUBD, OFFBOARD ... etc)
			uavs_setmode_client[ind].call(uavs_setmode[ind]);
			service_flag[ind] = 0;
			// break;
		}
	}
}

void QNode::uavs_pub_command(){
	for (const auto &ind : avail_uavind){
		if (publish_flag[ind] == 1){ // gps set origin
			uavs_gps_home_pub[ind].publish(uavs_gps_home[ind]);
			publish_flag[ind] = 0;
		}
		else if (publish_flag[ind] == 2){ // setpoint/local
			uavs_setpoint_pub[ind].publish(uavs_setpoint[ind]);
			publish_flag[ind] = 0;
		}
	}
}

void QNode::uavs_state_callback(const mavros_msgs::State::ConstPtr &msg, int ind){
	uavs_state[ind] = *msg;
	UAVs_info[ind].prestateReceived = true;
}
void QNode::uavs_imu_callback(const sensor_msgs::Imu::ConstPtr &msg, int ind){
	uavs_imu[ind] = *msg;
	UAVs_info[ind].preimuReceived = true;
}
void QNode::uavs_gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg, int ind){
	uavs_gps[ind] = *msg;
	UAVs_info[ind].pregpsReceived = true;
}
void QNode::uavs_gpsG_callback(const Gpsglobal::ConstPtr &msg, int ind){
	uavs_gpsG[ind] = *msg;
}
void QNode::uavs_gpsL_callback(const Gpslocal::ConstPtr &msg, int ind){
	uavs_gpsL[ind] = *msg;
	UAVs_info[ind].pregpsLReceived = true;
	UAVs_info[ind].pos_cur[0] = uavs_gpsL[ind].pose.pose.position.x;
	UAVs_info[ind].pos_cur[1] = uavs_gpsL[ind].pose.pose.position.y;
	UAVs_info[ind].pos_cur[2] = uavs_gpsL[ind].pose.pose.position.z;
	UAVs_info[ind].vel_cur[0] = uavs_gpsL[ind].twist.twist.linear.x;
	UAVs_info[ind].vel_cur[1] = uavs_gpsL[ind].twist.twist.linear.y;
	UAVs_info[ind].vel_cur[2] = -uavs_gpsL[ind].twist.twist.linear.z; //Somehow z-velocity is in opposite direction
}
void QNode::uavs_from_callback(const mavros_msgs::Mavlink::ConstPtr &msg, int ind){
	uavs_from[ind] = *msg;
	UAVs_info[ind].id = uavs_from[ind].sysid;
}

void QNode::Set_Arm_uavs(bool arm_disarm, int ind){
	uavs_arm[ind].request.value = arm_disarm;
	service_flag[ind] = 1;
}

void QNode::Set_Mode_uavs(std::string command_mode, int ind){
	uavs_setmode[ind].request.custom_mode = command_mode;
	service_flag[ind] = 2;
}
void QNode::Set_GPS_Home_uavs(int host_ind, int origin_ind){
	uavs_gps_home[host_ind].geo.latitude  = uavs_gpsG[origin_ind].latitude;
	uavs_gps_home[host_ind].geo.longitude = uavs_gpsG[origin_ind].longitude;
	publish_flag[host_ind] = 1;
}

void QNode::move_uavs(int ind, float pos_input[3]){
	uavs_setpoint[ind].header.stamp = ros::Time::now();
    uavs_setpoint[ind].type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    uavs_setpoint[ind].coordinate_frame = 1;
	uavs_setpoint[ind].position.x = pos_input[0];
	uavs_setpoint[ind].position.y = pos_input[1];
	uavs_setpoint[ind].position.z = pos_input[2];
	uavs_setpoint[ind].yaw = 0.0;
	publish_flag[ind] = 2;
	// Set_Mode_uavs("OFFBOARD", ind);
}

// void QNode::Move_uavs_alt(int ind){
// 	uavs_setpoint_alt[ind].orientation.x = 
// 	uavs_setpoint_alt[ind].orientation.y = 
// 	uavs_setpoint_alt[ind].orientation.z = 
// 	uavs_setpoint_alt[ind].orientation.w =
// 	uavs_setpoint_alt[ind].thrust = 
// }

void QNode::UAVS_Do_Plan(){
	for (const auto &host_ind : avail_uavind){
		if (!Move[host_ind]){ continue; }
		else{
			if (Plan_Dim == 0){
				float pos_input[3];
				pos_input[0] = UAVs_info[host_ind].pos_des[0] - UAVs_info[host_ind].pos_ini[0];
				pos_input[1] = UAVs_info[host_ind].pos_des[1] - UAVs_info[host_ind].pos_ini[1];
				pos_input[2] = UAVs_info[host_ind].pos_des[2] - UAVs_info[host_ind].pos_ini[2];
				move_uavs(host_ind, pos_input);
			}
			else if (Plan_Dim == 2){
				float force[2];
				force[0] = -c1*(UAVs_info[host_ind].vel_cur[0])-c2*(UAVs_info[host_ind].pos_cur[0]-UAVs_info[host_ind].pos_des[0]);
				force[1] = -c1*(UAVs_info[host_ind].vel_cur[1])-c2*(UAVs_info[host_ind].pos_cur[1]-UAVs_info[host_ind].pos_des[1]);
				for (const auto &other_ind : avail_uavind){
					float dist_v[2] = {	UAVs_info[host_ind].pos_cur[0]-UAVs_info[other_ind].pos_cur[0],
										UAVs_info[host_ind].pos_cur[1]-UAVs_info[other_ind].pos_cur[1]};
					float dist = std::pow(std::pow(dist_v[0],2) + std::pow(dist_v[1], 2), 0.5);
					if (host_ind != other_ind && dist < r_alpha){
						float ForceComponent = -RepulsiveGradient*std::pow(dist - r_alpha, 2);
						force[0] += ForceComponent*(dist_v[0]/dist);
						force[1] += ForceComponent*(dist_v[1]/dist);
					}
				}
				float pos_input[3];
				pos_input[0] = (UAVs_info[host_ind].pos_des[0] + (UAVs_info[host_ind].vel_cur[0] + force[0]*dt)*dt) - UAVs_info[host_ind].pos_ini[0];
				pos_input[1] = (UAVs_info[host_ind].pos_des[1] + (UAVs_info[host_ind].vel_cur[1] + force[1]*dt)*dt) - UAVs_info[host_ind].pos_ini[1];
				pos_input[2] = UAVs_info[host_ind].pos_des[2] - UAVs_info[host_ind].pos_ini[2];
				move_uavs(host_ind, pos_input);
			}
			else if (Plan_Dim == 3){
				float force[3];
				force[0] = -c1*(UAVs_info[host_ind].vel_cur[0])-c2*(UAVs_info[host_ind].pos_cur[0]-UAVs_info[host_ind].pos_des[0]);
				force[1] = -c1*(UAVs_info[host_ind].vel_cur[1])-c2*(UAVs_info[host_ind].pos_cur[1]-UAVs_info[host_ind].pos_des[1]);
				force[2] = -c1*(UAVs_info[host_ind].vel_cur[2])-c2*(UAVs_info[host_ind].pos_cur[2]-UAVs_info[host_ind].pos_des[2]);
				for (const auto &other_ind : avail_uavind){
					float dist_v[3] = {	UAVs_info[host_ind].pos_cur[0]-UAVs_info[other_ind].pos_cur[0],
										UAVs_info[host_ind].pos_cur[1]-UAVs_info[other_ind].pos_cur[1],
										UAVs_info[host_ind].pos_cur[2]-UAVs_info[other_ind].pos_cur[2]};
					float dist = std::pow(std::pow(dist_v[0],2) + std::pow(dist_v[1], 2) + std::pow(dist_v[2], 2), 0.5);
					if (host_ind != other_ind && dist < r_alpha){
						float ForceComponent = -RepulsiveGradient*std::pow(dist - r_alpha, 2);
						force[0] += ForceComponent*(dist_v[0]/dist);
						force[1] += ForceComponent*(dist_v[1]/dist);
						force[2] += ForceComponent*(dist_v[2]/dist);
					}
				}
				float pos_input[3];
				pos_input[0] = (UAVs_info[host_ind].pos_des[0] + (UAVs_info[host_ind].vel_cur[0] + force[0]*dt)*dt) - UAVs_info[host_ind].pos_ini[0];
				pos_input[1] = (UAVs_info[host_ind].pos_des[1] + (UAVs_info[host_ind].vel_cur[1] + force[1]*dt)*dt) - UAVs_info[host_ind].pos_ini[1];
				pos_input[2] = (UAVs_info[host_ind].pos_des[2] + (UAVs_info[host_ind].vel_cur[2] + force[2]*dt)*dt) - UAVs_info[host_ind].pos_ini[2];
				move_uavs(host_ind, pos_input);
			}
			else if (Plan_Dim == 10){
				float pos_input[3];
				// pos_input[0], UAVs_info[host_ind].pos_des[0] = square_x[square_i], square_x[square_i];
				// pos_input[1], UAVs_info[host_ind].pos_des[1] = square_y[square_i], square_y[square_i];
				// pos_input[2], UAVs_info[host_ind].pos_des[2] = 2.5, 2.5;
				pos_input[0] = square_x[square_i];
				pos_input[1] = square_y[square_i];
				pos_input[2] = 2.5;
				move_uavs(host_ind, pos_input);
				if (ros::Time::now() - last_change >= ros::Duration(5.0)){
					square_i += 1;
					if (square_i == 8){
						square_i = 0;
					}
					UAVs_info[host_ind].pos_des[0] = square_x[square_i];
					UAVs_info[host_ind].pos_des[1] = square_y[square_i];
					UAVs_info[host_ind].pos_des[2] = 2.5;
					last_change = ros::Time::now();
				}
			}
		}
	}	
	// for cf_id in self.cf_ids:
	// 	other_cfs = self.cf_ids[:]
	// 	other_cfs.remove(cf_id)
	// 	self._dist_to_goal[str(cf_id)] = np.linalg.norm(self._pos[str(cf_id)] - self.finals[str(cf_id)])
	// 	force = -self.c1*self._vel[str(cf_id)] - self.c2*(self._pos[str(cf_id)] - self.finals[str(cf_id)])
	// 	for other_cf in other_cfs:
	// 		dist_v = self._pos[str(other_cf)] - self._pos[str(cf_id)]
	// 		dist = np.linalg.norm(dist_v)
	// 		d = 2*self.radius + self.d_star
	// 		CommunicationRadious = np.cbrt(3*np.square(self.MaxVelo)/(2*self.RepulsiveGradient)) + d
	// 		if dist < CommunicationRadious:
	// 			ForceComponent = -self.RepulsiveGradient * np.square(dist - CommunicationRadious)
	// 			force += ForceComponent * (dist_v)/dist
	// 		velocity_d = self._vel[str(cf_id)] + force * self.dt
	// 		position_d = self._pos[str(cf_id)] + velocity_d*self.dt
	// 		position_d[2] = 1.0
	// 		self.update_pos(str(cf_id), position_d)
}

void QNode::Update_UAV_info(outdoor_gcs::uav_info UAV_input, int ind){
	UAVs_info[ind] = UAV_input;
}
void QNode::Update_Avail_UAVind(std::list<int> avail_uavind_input){
	avail_uavind = avail_uavind_input;
}
void QNode::Update_Move(int i){
	Move[i] = true;
}
void QNode::Update_Planning_Dim(int i){
	Plan_Dim = i; // 0 for no planning, 2 for 2D, 3 for 3D
}

State QNode::GetState_uavs(int ind){
	return uavs_state[ind];
}
Imu QNode::GetImu_uavs(int ind){
	// std::cout << "Pass data" << std::endl;
	return uavs_imu[ind];
}
Gpsraw QNode::GetGPS_uavs(int ind){
	return uavs_gps[ind];
}
Gpslocal QNode::GetGPSL_uavs(int ind){
	return uavs_gpsL[ind];
}
mavros_msgs::Mavlink QNode::GetFrom_uavs(int ind){
	return uavs_from[ind];
}
outdoor_gcs::uav_info QNode::Get_UAV_info(int ind){
	return UAVs_info[ind];
}

QStringList QNode::lsAllTopics(){
	ros::master::getTopics(topic_infos);
	QStringList topic_names;
	for (const auto &it : topic_infos)
	{
		topic_names += QString::fromStdString(it.name);
	}
	return topic_names;
}

outdoor_gcs::Angles QNode::quaternion_to_euler(float quat[4]){
    outdoor_gcs::Angles ans;
    ans.roll = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans.pitch = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans.yaw = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}


}  // namespace outdoor_gcs
