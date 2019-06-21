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
#include <diagnostic_msgs/KeyValue.h>

#include <math.h>
#include <cmath>
#include <stdio.h>

using namespace std;

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


	ros::Subscriber command_generator_sub;

	ros::Publisher  foot_step_command_pub;
	ros::Publisher  on_process_pub;

	ros::Publisher  walking_command_pub;

	//
	ros::ServiceClient set_balance_param_client;
	ros::ServiceClient joint_feedback_gain_client;


	ros::ServiceServer set_balance_param_nuke_server;
	ros::ServiceServer joint_feedback_gain_nuke_server;

	//command generator
  ros::Subscriber curr_status_sub_;
  ros::Subscriber alice_id_sub_;





	//msg
	alice_foot_step_generator::FootStepCommand foot_set_command_msg;
	std_msgs::Bool on_process_msg;

	alice_walking_module_msgs::SetBalanceParam set_balance_param_msg;
	alice_walking_module_msgs::SetJointFeedBackGain joint_feedback_gain_msg;


	void walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);
	void moveCommandStatusMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& move_command);
	void environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg);
	void commandGeneratorMsgCallback(const alice_foot_step_generator::FootStepCommandConstPtr& msg);

	bool setBalanceParamServiceCallback(alice_walking_module_msgs::SetBalanceParam::Request  &req,
			alice_walking_module_msgs::SetBalanceParam::Response &res);

	bool setJointFeedBackGainServiceCallback(alice_walking_module_msgs::SetJointFeedBackGain::Request  &req,
			alice_walking_module_msgs::SetJointFeedBackGain::Response &res);

  void alice_id_Callback(const std_msgs::String::ConstPtr& alice_id);
  void current_status_Callback(const std_msgs::String::ConstPtr& log_moving_status);

  ///


	void initialize();
	void data_initialize();
	void read_kick_param();
	void parse_init_data_(const std::string &path);
	void parse_online_balance_param(std::string path);
	void parse_online_joint_feedback_param(std::string path);
	void change_walking_kick_mode(std::string mode, std::string kick_mode);

	//void readIDAlice();
	std::string alice_id_num_;
  double kick_y_cob_;
  int alice_id_int;

private:

	int walking_mode;

	double step_length_max;
	double step_length_min;
	double pre_position_x, pre_position_y;
	double current_x,current_y;

	void AlignRobotYaw(double yaw_rad, std::string command, int mode);
	void CalculateStepData(double x, double y, std::string command, int mode);
	void DecideStepNumLength(double distance, std::string command, int mode);

};
class Command_generator
{
public:
  Command_generator();
  void parse_step_param_data(std::string path);
  std_msgs::String motion_command;
  std_msgs::String speed_command;
  alice_foot_step_generator::FootStepCommand FootParam;
  double default_step_num;
  double default_step_length;
  double default_side_step_length;
  double default_step_angle_rad;
  double default_step_time;
  double expanded_step_num;
  double expanded_step_length;
  double expanded_side_step_length;
  double expanded_step_angle_rad;
  double expanded_step_time;
  double centered_step_num;
  double centered_step_length;
  double centered_side_step_length;
  double centered_step_angle_rad;
  double centered_step_time;

  ros::Publisher vel_pub_;
  string step_type;
  void Set_FootParam(int alice_id);
  void Write_Log(void);
  int command_switch;
  string speed_switch;
  string init_log_path;
  string current_status;
  string accept_or_ignore;
  clock_t start_time;
  int init_hour, init_min, init_sec;
  ofstream out;
  //Text_Input//
  float Command_Period;
  /////////////

private:
  void Input_Text(void);
  void Make_Log(void);
};
}




#endif /* HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_PLANNER_INCLUDE_ALICE_FOOT_STEP_PLANNER_H_ */
