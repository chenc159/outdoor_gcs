/**
 * @file /include/outdoor_gcs/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef outdoor_gcs_QNODE_HPP_
#define outdoor_gcs_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <ros/message_event.h>
#include <ros/master.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#endif

#include <string>
// #include <Eigen/Eigen>
#include <QThread>
#include <QStringListModel>


#include <outdoor_gcs/GPSRAW.h>
#include <outdoor_gcs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Mavlink.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

using State 	= mavros_msgs::State;
using Imu 		= sensor_msgs::Imu;
using Gpsraw 	= outdoor_gcs::GPSRAW;
using Gpsglobal = sensor_msgs::NavSatFix;
using Gpslocal 	= nav_msgs::Odometry;
using GpsHomePos = outdoor_gcs::HomePosition;
using PosTarg 	= mavros_msgs::PositionTarget;

namespace outdoor_gcs {

/*****************************************************************************
** Class
*****************************************************************************/

	struct signalRec
	{
		bool prestate = false;
		bool preimu = false;
		bool pregps = false;
		bool pregpsG = false;
		bool pregpsL = false;
		bool pregpsH = false;
		bool stateReceived = false;
		bool imuReceived = false;
		bool gpsReceived = false;
		bool gpsGReceived = false;
		bool gpsLReceived = false;
		bool gpsHReceived = false;
	};

	struct Angles
	{
		float roll;
		float pitch;
		float yaw;
	};

	struct uav_info
	{
		int id = 0;
		float pos_cur[3];
		float pos_des[3];
		bool rosReceived = false;
		bool stateReceived = false;
		bool imuReceived = false;
		bool gpsReceived = false;
		bool gpsLReceived = false;
		bool prerosReceived = false;
		bool prestateReceived = false;
		bool preimuReceived = false;
		bool pregpsReceived = false;
		bool pregpsLReceived = false;
	};

	struct checkbox_status
	{
		bool print_imu = false;
		bool print_state = false;
		bool print_gps = false;
		bool print_local = false;
		bool print_des = false;
		bool clear_each_print = false;
	};
	
	


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	
	ros::master::V_TopicInfo topic_infos;

	////////////////////// Single uav ////////////////////////////
	void pub_command();
	
	void Set_Arm(bool arm_disarm);
	void Set_Mode(std::string command_mode);
	void Set_Home();
	void Set_GPS_Home();
	void move_uav(float target[3], float target_yaw);
	void move_uav_height(float height);
	void Do_Plan();

	State GetState();
	Imu GetImu();
	Gpsraw GetGPS();
	Gpsglobal GetGPSG();
	Gpslocal GetGPSL();
	GpsHomePos GetGPSH();
	sensor_msgs::BatteryState GetBat();
	mavros_msgs::Mavlink GetFrom();
	outdoor_gcs::signalRec Get_uav_signal();

	////////////////////// Multi-uav ////////////////////////////
	void Set_Arm_uavs(bool arm_disarm, int ind);
	void Set_Mode_uavs(std::string command_mode, int ind);
	void Set_GPS_Home_uavs(int host_ind, int origin_ind);

	void Update_UAV_info(outdoor_gcs::uav_info UAV_input, int ind);
	void Update_Avail_UAVind(std::list<int> avail_uavind_input);

	State GetState_uavs(int ind);
	Imu GetImu_uavs(int ind);
	Gpsraw GetGPS_uavs(int ind);
	Gpslocal GetGPSL_uavs(int ind);
	mavros_msgs::Mavlink GetFrom_uavs(int ind);
	outdoor_gcs::uav_info Get_UAV_info(int ind);

	QStringList lsAllTopics();
	outdoor_gcs::Angles quaternion_to_euler(float quat[4]);

Q_SIGNALS:
	void rosLoopUpdate();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

	////////////////////// Single uav ////////////////////////////
	mavros_msgs::State uav_state;
	Imu uav_imu;
	Gpsraw uav_gps;
	Gpsglobal uav_gpsG;
	Gpslocal uav_gpsL;
	GpsHomePos uav_gpsH;
	sensor_msgs::BatteryState uav_bat;
	mavros_msgs::Mavlink uav_from;
	mavros_msgs::CommandBool uav_arm;
	mavros_msgs::SetMode uav_setmode;
	mavros_msgs::CommandHome uav_sethome;

	PosTarg uav_setpoint;
	GpsHomePos uav_gps_home; //origin of gps local
	signalRec uav_received;

	ros::Subscriber uav_state_sub;
	ros::Subscriber uav_imu_sub;
	ros::Subscriber uav_gps_sub;
	ros::Subscriber uav_gpsG_sub;
	ros::Subscriber uav_gpsL_sub;
	ros::Subscriber uav_gpsH_sub;
	ros::Subscriber uav_bat_sub;
	ros::Subscriber uav_from_sub;

	ros::Publisher uav_setpoint_pub;
	ros::Publisher uav_gps_home_pub;

	ros::ServiceClient uav_arming_client;
	ros::ServiceClient uav_setmode_client;
	ros::ServiceClient uav_sethome_client;

	void state_callback(const mavros_msgs::State::ConstPtr &msg);
	void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
	void gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg);
	void gpsG_callback(const Gpsglobal::ConstPtr &msg);
	void gpsL_callback(const Gpslocal::ConstPtr &msg);
	void gpsH_callback(const GpsHomePos::ConstPtr &msg);
	void bat_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
	void from_callback(const mavros_msgs::Mavlink::ConstPtr &msg);

	////////////////////// Multi-uav ////////////////////////////
	int DroneNumber = 10;
	outdoor_gcs::uav_info UAVs_info[10];
	std::list<int> avail_uavind;

	std::vector<ros::Subscriber> uavs_state_sub;
	std::vector<ros::Subscriber> uavs_imu_sub;
	std::vector<ros::Subscriber> uavs_gps_sub;
	std::vector<ros::Subscriber> uavs_gpsL_sub;
	std::vector<ros::Subscriber> uavs_from_sub;

	std::vector<ros::Publisher> uavs_setpoint_pub;
	std::vector<ros::Publisher> uavs_gps_home_pub;

	std::vector<ros::ServiceClient> uavs_arming_client;
	std::vector<ros::ServiceClient> uavs_setmode_client;

	mavros_msgs::State uavs_state[10];
	Imu uavs_imu[10];
	Gpsraw uavs_gps[10];
	Gpslocal uavs_gpsL[10];
	mavros_msgs::Mavlink uavs_from[10];
	mavros_msgs::CommandBool uavs_arm[10];
	mavros_msgs::SetMode uavs_setmode[10];
	PosTarg uavs_setpoint[10];
	GpsHomePos uavs_gps_home[10]; //origin of gps local
	
	void uavs_state_callback(const mavros_msgs::State::ConstPtr &msg, int ind);
	void uavs_imu_callback(const sensor_msgs::Imu::ConstPtr &msg, int ind);
	void uavs_gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg, int ind);
	void uavs_gpsL_callback(const Gpslocal::ConstPtr &msg, int ind);
	void uavs_from_callback(const mavros_msgs::Mavlink::ConstPtr &msg, int ind);

};

}  // namespace outdoor_gcs

#endif /* outdoor_gcs_QNODE_HPP_ */
