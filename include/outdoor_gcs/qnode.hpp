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
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <outdoor_gcs/GPSRAW.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Mavlink.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

using State = mavros_msgs::State;
using Imu = sensor_msgs::Imu;
using Gpsraw = outdoor_gcs::GPSRAW;

namespace outdoor_gcs {

/*****************************************************************************
** Class
*****************************************************************************/

	struct signalRec
	{
		bool prestate = false;
		bool preimu = false;
		bool pregps = false;
		bool stateReceived = false;
		bool imuReceived = false;
		bool gpsReceived = false;
	};


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	
	void Set_Arm(bool arm_disarm);
	void Set_Mode(std::string command_mode);
	void Set_Home();

	mavros_msgs::State GetState();
	Gpsraw GetGPS();
	sensor_msgs::BatteryState GetBat();
	mavros_msgs::Mavlink GetFrom();
	outdoor_gcs::signalRec Update_uav_signal();



Q_SIGNALS:
	void rosLoopUpdate();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	
	int DroneNumber;

	mavros_msgs::State uav_state;
	Imu uav_imu;
	Gpsraw uav_gps;
	sensor_msgs::BatteryState uav_bat;
	mavros_msgs::Mavlink uav_from;
	mavros_msgs::CommandBool uav_arm;
	mavros_msgs::SetMode uav_setmode;
	mavros_msgs::CommandHome uav_sethome;


	signalRec uav_received;

	ros::Subscriber uav_state_sub;
	ros::Subscriber uav_imu_sub;
	ros::Subscriber uav_gps_sub;
	ros::Subscriber uav_bat_sub;
	ros::Subscriber uav_from_sub;

	ros::ServiceClient uav_arming_client;
	ros::ServiceClient uav_setmode_client;
	ros::ServiceClient uav_sethome_client;
	ros::ServiceClient uav_takeoff_client;
	ros::ServiceClient uav_land_client;

	void state_callback(const mavros_msgs::State::ConstPtr &msg);
	void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
	void gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg);
	void bat_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
	void from_callback(const mavros_msgs::Mavlink::ConstPtr &msg);
	
};

}  // namespace outdoor_gcs

#endif /* outdoor_gcs_QNODE_HPP_ */
