/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date May 2019
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qtros/qnode.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace qtros {

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
	ros::init(init_argc,init_argv,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh;
    // Add your ros communications here.
    /****camera 1***/
    image_transport::ImageTransport it_1(nh);
    image_transport::Subscriber image_sub_1;
    image_sub_1 = it_1.subscribe("/ir/flir_tau2/image1", 1, &QNode::myCallback,this);

    /****camera 2***/
    image_transport::ImageTransport it_2(nh);
    image_transport::Subscriber image_sub_2;
    image_sub_2 = it_2.subscribe("/ir/flir_tau2/image2", 1, &QNode::myCallback2,this);

    start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh;
	// Add your ros communications here.
    /****camera 1***/
    image_transport::ImageTransport it_1(nh);
    image_transport::Subscriber image_sub_1;
    image_sub_1 = it_1.subscribe("/ir/flir_tau2/image1", 1, &QNode::myCallback,this);
    /****camera 2***/
    image_transport::ImageTransport it_2(nh);
    image_transport::Subscriber image_sub_2;
    image_sub_2 = it_1.subscribe("/ir/flir_tau2/image2", 1, &QNode::myCallback2,this);

    start();
	return true;
}

void QNode::run() {
ros::NodeHandle nh;
/****camera 1***/
image_transport::ImageTransport it_1(nh);
image_transport::Subscriber image_sub_1;
image_sub_1 = it_1.subscribe("/ir/flir_tau2/image1", 1, &QNode::myCallback,this);

/****camera 2***/
image_transport::ImageTransport it_2(nh);
image_transport::Subscriber image_sub_2;
image_sub_2 = it_2.subscribe("/ir/flir_tau2/image2", 1, &QNode::myCallback2,this);

ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::myCallback(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr cv_ptr;
         try
           {
             cv_ptr = cv_bridge::toCvCopy(msg);
             img = cv_ptr->image;
             cv::normalize(img,img,0.,255.,cv::NORM_MINMAX,CV_8U);  //Convert from 16 bit to 8 bit per pixel
             Q_EMIT imageSignal(img);

           }
           catch (cv_bridge::Exception& e)
           {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return;
           }
              /**************************************/
   }

void QNode::myCallback2(const sensor_msgs::ImageConstPtr& msg2)
{
   cv_bridge::CvImagePtr cv_ptr;
         try
           {
             cv_ptr = cv_bridge::toCvCopy(msg2);
             img2 = cv_ptr->image;
             cv::normalize(img2,img2,0.,255.,cv::NORM_MINMAX,CV_8U);  //Convert from 16 bit to 8 bit per pixel
             Q_EMIT imageSignal2(img2);

           }
           catch (cv_bridge::Exception& e)
           {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return;
           }
              /**************************************/
   }


}   // namespace qtros

