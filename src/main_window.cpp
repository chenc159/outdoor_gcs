/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/outdoor_gcs/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace outdoor_gcs {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	// ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    /*------------init ros node -----------*/
    bool init_ros_ok = qnode.init();
    if (!init_ros_ok)
    {
        showNoMasterMessage();
    }
    else
    {
        //ui.button_connect->setEnabled(false);
    }

	/*********************
	** Logging
	**********************/
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateuav()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

////////////////////////// Buttons /////////////////////////

void MainWindow::on_ARM_clicked(bool check){
	if (uav_ARMED){
		qnode.Set_Arm(false);
	}
	else{
		qnode.Set_Arm(true);
	}
}

void MainWindow::on_SET_MODE_clicked(bool check){
	// qnode.Set_Mode("OFFBOARD");
	// qnode.Set_Mode("AUTO.TAKEOFF");
	// qnode.Set_Mode("AUTO.LOITER");   
    int num = ui.MODE_INPUT->text().toInt();
    switch(num){
    case 1:
    	qnode.Set_Mode("AUTO.TAKEOFF");
        break;
    case 2:
    	qnode.Set_Mode("AUTO.LAND");
        break;
    case 3:
    	qnode.Set_Mode("AUTO.RTL");
        break;
    case 4:
    	qnode.Set_Mode("AUTO.LOITER");
        break;
    case 5:
    	qnode.Set_Mode("POSCTL");
        break;
    case 6:
    	qnode.Set_Mode("MANUAL");
        break;
    case 7:
    	qnode.Set_Mode("RATTITUDE");
        break;
    case 8:
    	qnode.Set_Mode("AUTO.MISSION");
        break;
    case 9:
    	qnode.Set_Mode("AUTO.READY");
        break;
    }
}

void MainWindow::on_SET_HOME_clicked(bool check){
	qnode.Set_Home();
}

void MainWindow::on_Button_Set_clicked(bool check){
    /* read values from line edit */
    float target_state[3];

    target_state[0] =  ui.x_input->text().toFloat();
    target_state[1] =  ui.y_input->text().toFloat();
    target_state[2] =  ui.z_input->text().toFloat();
    /*----------------determine whether the input is in safe range ------------------*/
    bool input_is_valid = true;

    if(target_state[0] < -10.0 || target_state[0] > 10.0) {
        input_is_valid = false;
    }

    if(target_state[1] < -10.0 || target_state[1] > 10.0) {
        input_is_valid = false;
    }

    if(target_state[2] < 0|| target_state[2] > 20.0) {
        input_is_valid = false;
    }

    /*----------------send input ------------------*/

    if(input_is_valid){
        /*  update the ENU target label */
        ui.des_x->setText(QString::number(target_state[0], 'f', 2));
        ui.des_y->setText(QString::number(target_state[1], 'f', 2));
        ui.des_z->setText(QString::number(target_state[2], 'f', 2));

        qnode.move_uav(target_state);

    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}

// void MainWindow::on_cf0_Button_Get_clicked(bool check){
//     cf_gs::Mocap temp_mocap = qnode.GetMocap(0);
//     ui.cf0_x_input->setText(QString::number(temp_mocap.position[0], 'f', 2));
//     ui.cf0_y_input->setText(QString::number(temp_mocap.position[1], 'f', 2));
//     ui.cf0_z_input->setText(QString::number(temp_mocap.position[2], 'f', 2));
// }


////////////////////////// Update signals /////////////////////////
void MainWindow::updateuav(){

	mavros_msgs::State state_data = qnode.GetState();
    sensor_msgs::BatteryState bat_data = qnode.GetBat();
	mavros_msgs::Mavlink from_data = qnode.GetFrom();
	outdoor_gcs::GPSRAW gps_data = qnode.GetGPS();
	Gpsglobal gpsG_data = qnode.GetGPSG();
	Gpslocal gpsL_data = qnode.GetGPSL();
	GpsHomePos gpsH_data = qnode.GetGPSH();
    outdoor_gcs::signalRec signal = qnode.Update_uav_signal();
	if (signal.imuReceived){
        ui.IMU_CONNECT->setText("<font color='green'>IMU CONNECTED</font>");
        ui.CONNECT->setText("UAV CONNECTED: " + QString::number(from_data.sysid));
        ui.Volt->setText(QString::number(bat_data.voltage, 'f', 2));
        if (state_data.connected){
            ui.STATE_CONNECT->setText("<font color='green'>STATE CONNECTED</font>");
        }
        else{
            ui.STATE_CONNECT->setText("<font color='red'>STATE UNCONNECTED</font>");
        }
        if (state_data.mode.empty()){
            ui.MODE->setText("<font color='red'>---</font>");
        }
        else{
		    ui.MODE->setText(QString::fromStdString(state_data.mode));
        }
		if (state_data.armed){
			uav_ARMED = true;
			ui.ARM->setText("DISARM");
		}
		else{
			uav_ARMED = false;
			ui.ARM->setText("ARM");
		}
	}
	else{
        ui.CONNECT->setText("<font color='red'>UAV UNCONNECTED</font>");
        ui.STATE_CONNECT->setText("<font color='red'>STATE UNCONNECTED</font>");
        ui.IMU_CONNECT->setText("<font color='red'>IMU UNCONNECTED</font>");
        ui.Volt->setText("<font color='red'>---</font>");
        ui.MODE->setText("<font color='red'>---</font>");
	}

	if (signal.gpsReceived){
        ui.gps_num->setText(QString::number(gps_data.satellites_visible));
		ui.gps_lat->setText(QString::number(gps_data.lat*1e-7, 'f', 7));
		ui.gps_lon->setText(QString::number(gps_data.lon*1e-7, 'f', 7));
		ui.gps_alt->setText(QString::number(gps_data.alt*1e-3, 'f', 3));
	}
	else{
		ui.gps_num->setText("<font color='red'>---</font>");
        ui.gps_lat->setText("<font color='red'>---</font>");
        ui.gps_lon->setText("<font color='red'>---</font>");
        ui.gps_alt->setText("<font color='red'>---</font>");
	}

    if (signal.gpsGReceived){
		ui.gps_lat_2->setText(QString::number(gpsG_data.latitude, 'f', 6));
		ui.gps_lon_2->setText(QString::number(gpsG_data.longitude, 'f', 6));
		ui.gps_alt_2->setText(QString::number(gpsG_data.altitude, 'f', 6));
	}
	else{
        ui.gps_lat_2->setText("<font color='red'>---</font>");
        ui.gps_lon_2->setText("<font color='red'>---</font>");
        ui.gps_alt_2->setText("<font color='red'>---</font>");
	}

    if (signal.gpsLReceived){
		ui.localx->setText(QString::number(gpsL_data.pose.pose.position.x, 'f', 6));
		ui.localy->setText(QString::number(gpsL_data.pose.pose.position.y, 'f', 6));
		ui.localz->setText(QString::number(gpsL_data.pose.pose.position.z, 'f', 6));
	}
	else{
        ui.localx->setText("<font color='red'>---</font>");
        ui.localy->setText("<font color='red'>---</font>");
        ui.localz->setText("<font color='red'>---</font>");
	}

    if (signal.gpsHReceived){
		ui.localx_2->setText(QString::number(gpsH_data.position.x, 'f', 6));
		ui.localy_2->setText(QString::number(gpsH_data.position.y, 'f', 6));
		ui.localz_2->setText(QString::number(gpsH_data.position.z, 'f', 6));
	}
	else{
        ui.localx_2->setText("<font color='red'>---</font>");
        ui.localy_2->setText("<font color='red'>---</font>");
        ui.localz_2->setText("<font color='red'>---</font>");
	}

}



void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "outdoor_gcs");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    // QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    // QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    // //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    // ui.line_edit_master->setText(master_url);
    // ui.line_edit_host->setText(host_url);
    // //ui.line_edit_topic->setText(topic_name);
    // bool remember = settings.value("remember_settings", false).toBool();
    // ui.checkbox_remember_settings->setChecked(remember);
    // bool checked = settings.value("use_environment_variables", false).toBool();
    // ui.checkbox_use_environment->setChecked(checked);
    // if ( checked ) {
    // 	ui.line_edit_master->setEnabled(false);
    // 	ui.line_edit_host->setEnabled(false);
    // 	//ui.line_edit_topic->setEnabled(false);
    // }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "outdoor_gcs");
    // settings.setValue("master_url",ui.line_edit_master->text());
    // settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    // settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    // settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace outdoor_gcs

