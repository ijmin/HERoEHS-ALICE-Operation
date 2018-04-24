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
#include <stdio.h>
#include "../include/alice_gui/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace alice {

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
	ros::init(init_argc,init_argv,"offset_tuner_operation");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	start();
	// Add your ros communications here.
	/*****************************************************************************
	 ** dynamixel test
	 *****************************************************************************/
	// Publisher
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 100);
	joint_offset_state_pub = n.advertise<offset_tuner_msgs::JointOffsetState>("/heroehs/joint_offset_state", 10);
	command_state_pub = n.advertise<std_msgs::String>("/heroehs/command_state", 10);
	enable_module_pub = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 100);


	// Subscriber
	moving_state_sub = n.subscribe("/heroehs/moving_state", 10, &QNode::MovingStateMsgsCallBack, this);

	// Service Client
	joint_torque_on_off_cl = n.serviceClient<offset_tuner_msgs::JointTorqueOnOff>("/heroehs/joint_torque_on_off");
	joint_torque_on_off_array_cl = n.serviceClient<offset_tuner_msgs::JointTorqueOnOffArray>("/heroehs/joint_torque_on_off_array");
	present_joint_state_array_cl = n.serviceClient<offset_tuner_msgs::PresentJointStateArray>("/heroehs/present_joint_state_array");

	/*****************************************************************************
	 ** walking test
	 *****************************************************************************/
	foot_step_command_pub = n.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_generator/walking_command",10);
	module_on_off = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 10);

	return true;
}
void QNode::run() {
	while ( ros::ok() ) {
		ros::spinOnce();
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
		logging_model_msg << "[INFO]: " << msg;
		break;
	}
	case(Warn) : {
		ROS_WARN_STREAM(msg);
		logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
		break;
	}
	case(Error) : {
		ROS_ERROR_STREAM(msg);
		logging_model_msg << "[ERROR]: " << msg;
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

/*****************************************************************************
 ** Communication
 *****************************************************************************/
void QNode::MovingStateMsgsCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data == true)
	{
		log(Info, "Moving Start");
	}
	else
	{
		log(Info, "Moving Stop");
	}
}


}  // namespace offset_tuner_operation
