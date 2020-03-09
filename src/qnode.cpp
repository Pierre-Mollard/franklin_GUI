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
#include <tf/tf.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include "../include/franklin_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace franklin_gui {

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

void test(const std_msgs::Float32 msg){
	//int progressDataTest = (int) (msg.data*100);
	ROS_INFO("Test ");

}

bool QNode::init() {
	ros::init(init_argc,init_argv,"franklin_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	pub_dest = n.advertise<geometry_msgs::Pose2D>("destination", 1000);
	pub_stop = n.advertise<std_msgs::Bool>("destination/stop", 1000);
	sub_info_dest = n.subscribe("f_info_dest", 1, &QNode::info_dest_Callback, this);
	sub_odom = n.subscribe("odom", 1, &QNode::odom_Callback, this);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"franklin_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	pub_dest = n.advertise<geometry_msgs::Pose2D>("destination", 1000);
	pub_stop = n.advertise<std_msgs::Bool>("destination/stop", 1000);
	sub_info_dest = n.subscribe("f_info_dest", 1, &QNode::info_dest_Callback, this); //&franklin_gui::QNode::info_dest_Callback, this
	sub_odom = n.subscribe("odom", 1, &QNode::odom_Callback, this);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
		/*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		*/
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendTargetPos(double pX, double pY, double pT){
	ROS_INFO("INFO POS SENT !");
	geometry_msgs::Pose2D pose2D;
	pose2D.x = pX;
	pose2D.y = pY;
	pose2D.theta = pT;
	pub_dest.publish(pose2D);
}

void QNode::info_dest_Callback(const std_msgs::Float32 msg){
	ROS_INFO("PROGRESS UPDATE RECEIVE");
	this->progressData = (int) (msg.data*100);
	Q_EMIT progressDataS();
}

void QNode::odom_Callback(const nav_msgs::Odometry odom){
	ROS_INFO("ODOM RECEIVE");
	this->odom_X = odom.pose.pose.position.x;
	this->odom_Y = odom.pose.pose.position.y;
/*
	// quaternion to RPY conversion
	tf::Quaternion q(
			odom.pose.pose.orientation.x,
			odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z,
			odom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
*/
	// angular position
	//this->odom_T = yaw;
	this->odom_T = odom.pose.pose.orientation.z;

	ROS_INFO("X = %lf", this->odom_X);

	Q_EMIT odomS();
}

void QNode::sendStop(bool b){
	ROS_INFO("INFO STOP SENT !");
	std_msgs::Bool msg;
	msg.data = b;
	pub_stop.publish(msg);
}

}  // namespace franklin_gui
