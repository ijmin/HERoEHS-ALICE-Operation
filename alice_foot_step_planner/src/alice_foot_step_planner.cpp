/*
 * alice_foot_step_planner.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: robotemperor
 */

#include "alice_foot_step_planner/alice_foot_step_planner.h"

using namespace alice;
// Foot step planner algorithm
FootStepPlanner::FootStepPlanner()
{

}
FootStepPlanner::~FootStepPlanner()
{

}
void FootStepPlanner::walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)  //string
{
	if(msg->type == msg->STATUS_ERROR)
		ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
	else if(msg->type == msg->STATUS_INFO)
		ROS_INFO_STREAM("[Robot] : " << msg->status_msg);
	else if(msg->type == msg->STATUS_WARN)
		ROS_WARN_STREAM("[Robot] : " << msg->status_msg);
	else if(msg->type == msg->STATUS_UNKNOWN)
		ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
	else
		ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
}
void FootStepPlanner::initialize()
{
	//data initialize
	foot_set_command_msg.step_num = 2;
	foot_set_command_msg.step_length = 0.05;
	foot_set_command_msg.step_time = 2;
	foot_set_command_msg.step_angle_rad = 0;
	foot_set_command_msg.side_step_length = 0.05;

	ros::NodeHandle nh;

	walking_module_status_sub = nh.subscribe("/robotis/status", 10, &FootStepPlanner::walkingModuleStatusMsgCallback, this);
	foot_step_command_pub     = nh.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_planner/walking_command", 1);
}

void FootStepPlanner::AlignRobotYaw(double yaw_degree)
{
	double yaw_rad = yaw_degree*DEGREE2RADIAN;

	if(yaw_rad > 0) // turn left
	{
		foot_set_command_msg.step_angle_rad = yaw_rad;
		foot_set_command_msg.command = "turn left";
	}
	else if (yaw_rad < 0)// turn right
	{
		foot_set_command_msg.step_angle_rad = yaw_rad;
		foot_set_command_msg.command = "turn right";
	}
	else
	{
		return;
	}

	foot_step_command_pub.publish(foot_set_command_msg);
}
void FootStepPlanner::CalculateStepData(double x, double pre_x, double y, double pre_y)
{

}
bool FootStepPlanner::CheckArrival(std::string status, int via_point_num, int all_point_num)
{

	return true;

}


