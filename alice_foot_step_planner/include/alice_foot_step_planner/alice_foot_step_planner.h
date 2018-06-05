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

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"
#include "alice_foot_step_generator/FootStepCommand.h"
#include "alice_operation_msgs/WalkingPathPlanner.h"
#include <std_msgs/String.h>

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

	// ros communication part
	ros::Subscriber walking_module_status_sub;
	ros::Subscriber walking_path_planner_test_sub;

	//
	ros::Subscriber walking_path_sub_;

	ros::Publisher  foot_step_command_pub;

	alice_foot_step_generator::FootStepCommand foot_set_command_msg;


	void walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);
	void walkingPathPlannerStatusMsgCallback(const alice_operation_msgs::WalkingPathPlanner::ConstPtr& msg);
	void initialize();
	void data_initialize();
	void parse_init_data_(const std::string &path);

	void walkingPathMsgCAllBack(const std_msgs::String::ConstPtr& msg);

private:

	double step_length_max;
	double step_length_min;
	double pre_position_x, pre_position_y;


	void AlignRobotYaw(double yaw_degree, std::string command);
	void CalculateStepData(double x, double y, std::string command);
	void DecideStepNumLength(double distance);
	//Eigen::Matrix4d TransfomationGoalPointOnRobot(double robot_x, double robot_y, double robot_yaw_degree, double g_point_x, double g_point_y);
};

}




#endif /* HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_PLANNER_INCLUDE_ALICE_FOOT_STEP_PLANNER_H_ */
