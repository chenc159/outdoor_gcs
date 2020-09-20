/**
 * @file /include/outdoor_gcs/main_window.hpp
 *
 * @brief Qt based gui for outdoor_gcs.
 *
 * @date November 2010
 **/
#ifndef outdoor_gcs_MAIN_WINDOW_H
#define outdoor_gcs_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace outdoor_gcs {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_ARM_clicked(bool check);
	void on_OFFBOARD_clicked(bool check);
	void on_SET_HOME_clicked(bool check);

    /******************************************
    ** Manual connections
    *******************************************/
	void updateuav();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	bool uav_ARMED;
};

}  // namespace outdoor_gcs

#endif // outdoor_gcs_MAIN_WINDOW_H
