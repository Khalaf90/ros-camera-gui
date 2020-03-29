/**
 * @file /include/qtros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date May 2019
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtros_QNODE_HPP_
#define qtros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <QPixmap>

#include <string>
#include <std_msgs/Float64.h>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void myCallback(const sensor_msgs::ImageConstPtr& msg);
    void myCallback2(const sensor_msgs::ImageConstPtr& msg2);

    cv::Mat img;
    cv::Mat img2;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

Q_SIGNALS:

    void rosShutdown();
    void imageSignal(cv::Mat);
    void imageSignal2(cv::Mat);


private:
    image_transport::Subscriber image_sub_1;//cam1
    image_transport::Subscriber image_sub_2;//cam2

	int init_argc;
	char** init_argv;
	ros::Subscriber chatter_subscriber;
    QStringListModel logging_model;
    QStringListModel logging;


};

}  // namespace qtros

#endif /* qtros_QNODE_HPP_ */
