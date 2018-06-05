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
{
	currentForceX_l_gui = 0;
	currentForceY_l_gui = 0;
	currentForceZ_l_gui = 0;
	currentForceX_r_gui = 0;
	currentForceY_r_gui = 0;
	currentForceZ_r_gui = 0;

	currentTorqueX_l_gui = 0;
	currentTorqueY_l_gui = 0;
	currentTorqueZ_l_gui = 0;
	currentTorqueX_r_gui = 0;
	currentTorqueY_r_gui = 0;
	currentTorqueZ_r_gui = 0;

	ft_init_done_check = false;
}

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

	set_balance_param_client =  n.serviceClient<alice_walking_module_msgs::SetBalanceParam>("/heroehs/online_walking/balance_param");
	joint_feedback_gain_client = n.serviceClient<alice_walking_module_msgs::SetJointFeedBackGain>("/heroehs/online_walking/joint_feedback_gain");


	/*****************************************************************************
	 ** Module on off
	 *****************************************************************************/

	// sensor initialize button & completed signal detect
	alice_ft_init_pub = n.advertise<std_msgs::Bool>("/alice/ft_init",10);
	alice_ft_init_done_sub = n.subscribe("/alice/ft_init_done", 10, &QNode::aliceFtInitDoneMsgCallback, this);

	// desired pose
	desired_pose_waist_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose_waist",10);
	desired_pose_head_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose_head",10);
	desired_pose_arm_pub = n.advertise<std_msgs::Float64MultiArray>("/desired_pose_arm",10);

	/*****************************************************************************
	 ** control on off
	 *****************************************************************************/
	//balance on off
	alice_balance_parameter_pub = n.advertise<alice_msgs::BalanceParam>("/alice/balance_parameter",10);

	//ball tracking
	ball_tracking_pub =  n.advertise<std_msgs::Float64MultiArray>("/ball_param",10);


	/*****************************************************************************
	 ** graph
	 *****************************************************************************/

	zmp_fz_sub = n.subscribe("/zmp_fz", 10, &QNode::zmpFzMsgCallback, this);

	alice_force_torque_data_sub = n.subscribe("/alice/force_torque_data", 10, &QNode::forceTorqueDataMsgCallback, this);/////////////
	joint_goal_state_sub        = n.subscribe("/robotis/goal_joint_states", 10, &QNode::goalJointStateMsgCallback, this);
	joint_present_state_sub     = n.subscribe("/robotis/present_joint_states", 10, &QNode::presentJointStateMsgCallback, this);

	//base module //
	init_pose_pub = n.advertise<std_msgs::String>("/init_pose",10);

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
void QNode::forceTorqueDataMsgCallback(const alice_msgs::ForceTorque::ConstPtr& msg)
{
	currentForceX_l_gui = (double) msg->force_x_raw_l;
	currentForceY_l_gui = (double) msg->force_y_raw_l;
	currentForceZ_l_gui = (double) msg->force_z_raw_l;

	currentForceX_r_gui = (double) msg->force_x_raw_r;
	currentForceY_r_gui = (double) msg->force_y_raw_r;
	currentForceZ_r_gui = (double) msg->force_z_raw_r;

	currentTorqueX_l_gui = (double) msg->torque_x_raw_l;
	currentTorqueY_l_gui = (double) msg->torque_y_raw_l;
	currentTorqueZ_l_gui = (double) msg->torque_z_raw_l;

	currentTorqueX_r_gui = (double) msg->torque_x_raw_r;
	currentTorqueY_r_gui = (double) msg->torque_y_raw_r;
	currentTorqueZ_r_gui = (double) msg->torque_z_raw_r;
}
void QNode::goalJointStateMsgCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int num=0; num < msg->name.size(); num++)
	{
		joint_index_to_name[num] = msg->name[num];
		joint_name_to_goal[joint_index_to_name[num]] = msg->position[num];
		//printf("%s ::  %f \n", joint_index_to_name[num].c_str(), joint_name_to_goal[joint_index_to_name[num]]);
	}
}
void QNode::presentJointStateMsgCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int num=0; num < msg->name.size(); num++)
	{
		joint_index_to_name[num] = msg->name[num];
		joint_name_to_present[joint_index_to_name[num]] = msg->position[num];
		//printf("%s ::  %f \n", joint_index_to_name[num].c_str(), joint_name_to_present[joint_index_to_name[num]]);
	}


}
void QNode::aliceFtInitDoneMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ft_init_done_check = msg->data;
}
void QNode::zmpFzMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	current_zmp_fz_x   = (double) msg->data[0];
	current_zmp_fz_y   = (double) msg->data[1];
	reference_zmp_fz_x = (double) msg->data[2];
	reference_zmp_fz_y = (double) msg->data[3];
}


}  // namespace offset_tuner_operation
