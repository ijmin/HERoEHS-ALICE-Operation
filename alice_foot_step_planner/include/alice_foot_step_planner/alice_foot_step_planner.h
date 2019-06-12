/*
 * alice_foot_step_planner.h
 *
 *  Created on: Apr 25, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_PLANNER_INCLUDE_ALICE_FOOT_STEP_PLANNER_H_
#define HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_PLANNER_INCLUDE_ALICE_FOOT_STEP_PLANNER_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>


#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"
#include "alice_foot_step_generator/FootStepCommand.h"
#include "alice_operation_msgs/WalkingPathPlanner.h"
#include "alice_walking_module_msgs/SetBalanceParam.h"
#include "alice_walking_module_msgs/SetJointFeedBackGain.h"
#include "alice_msgs/FoundObjectArray.h"
#include "alice_foot_step_generator/Step2DArray.h"

#include "alice_msgs/MoveCommand.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <math.h>
#include <cmath>
#include <stdio.h>


namespace alice
{
class FootStepPlanner
{
public:
	FootStepPlanner();
	~FootStepPlanner();

	std::string balance_param_file;
	std::string joint_feedback_file;

	// ros communication part
	ros::Subscriber walking_module_status_sub;
	ros::Subscriber walking_path_planner_test_sub;

	//
	ros::Subscriber walking_path_sub_;
	ros::Subscriber move_command_sub_;
	ros::Subscriber environment_detector_sub;

	ros::Subscriber test_joystic_sub;

	ros::Publisher  foot_step_command_pub;
	ros::Publisher  on_process_pub;

	ros::Publisher  walking_command_pub;
	ros::Publisher  foot_step_2d_pub;

	//
	ros::ServiceClient set_balance_param_client;
	ros::ServiceClient joint_feedback_gain_client;


	ros::ServiceServer set_balance_param_nuke_server;
	ros::ServiceServer joint_feedback_gain_nuke_server;





	//msg
	alice_foot_step_generator::FootStepCommand foot_set_command_msg;
	std_msgs::Bool on_process_msg;

	alice_walking_module_msgs::SetBalanceParam set_balance_param_msg;
	alice_walking_module_msgs::SetJointFeedBackGain joint_feedback_gain_msg;


	void walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);
	void moveCommandStatusMsgCallback(const alice_msgs::MoveCommand::ConstPtr& msg);
	void environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg);
	void testJoysticMsgCallback(const std_msgs::String::ConstPtr& msg);

	bool setBalanceParamServiceCallback(alice_walking_module_msgs::SetBalanceParam::Request  &req,
			alice_walking_module_msgs::SetBalanceParam::Response &res);

	bool setJointFeedBackGainServiceCallback(alice_walking_module_msgs::SetJointFeedBackGain::Request  &req,
			alice_walking_module_msgs::SetJointFeedBackGain::Response &res);


	void initialize();
	void data_initialize();
	void parse_init_data_(const std::string &path);
	void parse_online_balance_param(std::string path);
	void parse_online_joint_feedback_param(std::string path);
	void change_walking_kick_mode(std::string mode, std::string kick_mode);
	void readIDAlice();
	std::string alice_id_num_;
private:

	int walking_mode;
	double darwin_step_length_x, darwin_step_length_y;

	double step_length_max;
	double step_length_min;
	double pre_position_x, pre_position_y;

	double current_x,current_y;


	void loadWalkingParam(const std::string &path);
	void AlignRobotYaw(double yaw_rad, std::string command, int mode);
	void CalculateStepData(double x, double y, std::string command, int mode);
	void DecideStepNumLength(double distance, std::string command, int mode);
	//void ExpandedLeftStepArray();
	//void ExpandedRightStepArray();
};

}




#endif /* HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_PLANNER_INCLUDE_ALICE_FOOT_STEP_PLANNER_H_ */
