/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date May 2019
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qtros/main_window.hpp"
#include<QPixmap>
#include<QProcess>
#include<signal.h>
#include<highgui.h>
#include <QFileDialog>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qRegisterMetaType<cv::Mat>("cv::Mat");
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


    /*********************
    ** Logging
    **********************/

    QObject::connect(&qnode, SIGNAL(imageSignal(cv::Mat)),this,SLOT(displayMat(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(imageSignal2(cv::Mat)),this,SLOT(displayMat2(cv::Mat)));

    /********************Camera-1 buttons**********************/
    QObject::connect(ui.startvideo, SIGNAL(clicked()), this, SLOT(start_cam_1()));
    QObject::connect(ui.suspendvideo, SIGNAL(clicked()), this, SLOT(stop_cam_1()));
    QObject::connect(ui.snapshot, SIGNAL(clicked()), this, SLOT(snapshot_cam_1()));
    /**********************************************************************/

    /********************Camera-2 buttons**********************/
    QObject::connect(ui.startvideo_2, SIGNAL(clicked()), this, SLOT(start_cam_2()));
    QObject::connect(ui.suspendvideo_2, SIGNAL(clicked()), this, SLOT(stop_cam_2()));
    QObject::connect(ui.snapshot_2, SIGNAL(clicked()), this, SLOT(snapshot_cam_2()));
    /**********************************************************************/


    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/************ Camera-1 Buttons******************/
int on=0,off=0;

void MainWindow::start_cam_1()
{
    on=1;

}

void MainWindow::stop_cam_1()
{
    off=1;
}

/************ Camera-2 Buttons******************/
int on_2=0,off_2=0;

void MainWindow::start_cam_2()
{
    on_2=1;

}

void MainWindow::stop_cam_2()
{
    off_2=1;
}


/********************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString()) ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qtros");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://ubiquityrobot.local:11311/")).toString();
    QString host_url = settings.value("host_url", QString("137.226.176.99")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qtros");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}


void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}


QImage img;

void MainWindow::displayMat(cv::Mat image)
{

          img =QImage((image.data),
                       image.cols,image.rows,
                       QImage::Format_Grayscale8);


   if(on && !off)

   {
     ui.view_logging->setPixmap(QPixmap::fromImage(img));
    // ui.view_logging->resize(ui.view_logging->pixmap()->size());


     ui.view_logging->setScaledContents( true );
     ui.view_logging->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );

     /*h=ui->view_logging->height;
     w=ui->view_logging->width;
     ui->view_logging->setPixmap(img.scaled(w,h,Qt::KeepAspectRatio));*/
   }

   else
     {
        on=0; off=0;
     }
 }

void MainWindow::snapshot_cam_1()
{
    /* QString imagePath = QFileDialog::getSaveFileName(this,
                                             "Save Snapshot Images",
                                             QDir::homePath(),
                                             tr("Images (*.png *.jpg)")); // in case the path is not fixed
    */

    QDateTime time = QDateTime::currentDateTime();
    QString timestamp = time.toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString name="Cam1_";
    QString path="/media/ubuntu/demo/"+name+timestamp+".jpg";


    qDebug() << "timestamp:" << timestamp;

    img.save(path);

}



QImage img2;

void MainWindow::displayMat2(cv::Mat image2)
{

          img2 =QImage((image2.data),
                       image2.cols,image2.rows,
                       QImage::Format_Grayscale8);


   if(on_2 && !off_2)

   {
     ui.view_logging_2->setPixmap(QPixmap::fromImage(img2));
     //ui.view_logging_2->resize(ui.view_logging_2->pixmap()->size());


     ui.view_logging_2->setScaledContents( true );
     ui.view_logging_2->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );

     /*h=ui->view_logging_2->height;
     w=ui->view_logging_2->width;
     ui->view_logging_2->setPixmap(img2.scaled(w,h,Qt::KeepAspectRatio));*/


   }

   else
     {
        on_2=0; off_2=0;
     }
 }

void MainWindow::snapshot_cam_2()
{
    /* QString imagePath = QFileDialog::getSaveFileName(this,
                                             "Save Snapshot Images",
                                             QDir::homePath(),
                                             tr("Images (*.png *.jpg)")); // in case the path is not fixed
    */

    QDateTime time2 = QDateTime::currentDateTime();
    QString timestamp2 = time2.toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString name="Cam2_";
    QString path2="/media/ubuntu/abr_demo/"+name+timestamp2+".jpg";


    qDebug() << "timestamp2:" << timestamp2;

    img2.save(path2);

}

}  // namespace qtros



void qtros::MainWindow::on_startvideo_clicked()
{
     ui.startvideo->setStyleSheet("background-color:#9c9c9c ");
     ui.suspendvideo->setStyleSheet("  ");
}

void qtros::MainWindow::on_suspendvideo_clicked()
{
   ui.startvideo->setStyleSheet("  ");
   ui.suspendvideo->setStyleSheet(" background-color:gray;");
}

void qtros::MainWindow::on_startvideo_2_clicked()
{
    ui.startvideo_2->setStyleSheet("background-color:#9c9c9c;  ");
    ui.suspendvideo_2->setStyleSheet(" ");
}

void qtros::MainWindow::on_suspendvideo_2_clicked()
{
    ui.startvideo_2->setStyleSheet(" ");
    ui.suspendvideo_2->setStyleSheet("background-color:gray; ");
}
