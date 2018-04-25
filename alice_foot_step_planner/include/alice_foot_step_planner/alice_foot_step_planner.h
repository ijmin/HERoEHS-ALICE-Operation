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
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"
#include "alice_foot_step_generator/FootStepCommand.h"
#include <cmath>


namespace alice
{
class FootStepPlanner
{
public:
	FootStepPlanner();
	~FootStepPlanner();

	// ros communication part
	ros::Subscriber walking_module_status_sub;

	ros::Publisher  foot_step_command_pub;

	alice_foot_step_generator::FootStepCommand foot_set_command_msg;


	void walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);
	void initialize();

private:

	void AlignRobotYaw(double yaw_degree);
	void CalculateStepData(double x, double pre_x, double y, double pre_y);
	bool CheckArrival(std::string status, int via_point_num, int all_point_num);
};

}




#endif /* HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_PLANNER_INCLUDE_ALICE_FOOT_STEP_PLANNER_H_ */
