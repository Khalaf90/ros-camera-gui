/**
 * @file /include/qtros/main_window.hpp
 *
 * @brief Qt based gui for qtros.
 *
 * @date May 2019
 **/
#ifndef qtros_MAIN_WINDOW_H
#define qtros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "QProcess"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qtros {


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
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);


    void start_cam_1();
    void stop_cam_1();
    void snapshot_cam_1();

    void start_cam_2();
    void stop_cam_2();
    void snapshot_cam_2();

    /******************************************
    ** Manual connections
    *******************************************/

    void displayMat(cv::Mat image);
    void displayMat2(cv::Mat image2);


private slots:

    void on_startvideo_clicked();
    void on_suspendvideo_clicked();

    void on_startvideo_2_clicked();
    void on_suspendvideo_2_clicked();



private:
	Ui::MainWindowDesign ui;
	QNode qnode;



};

}  // namespace qtros

#endif // qtros_MAIN_WINDOW_H
