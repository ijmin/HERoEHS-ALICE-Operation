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

	// load yaml
	step_length_max = 0;
	step_length_min = 0;

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

Eigen::Matrix4d FootStepPlanner::TransfomationGoalPointOnRobot(double robot_x, double robot_y, double robot_yaw_degree, double g_point_x, double g_point_y)
{
	Eigen::Matrix4d g_T_robot;
	Eigen::Matrix4d g_T_point;
	Eigen::Matrix4d robot_T_g;
	Eigen::Matrix4d robot_T_point;

	g_T_robot.fill(0);
	g_T_point.fill(0);
	robot_T_g.fill(0);
	robot_T_point.fill(0);

	g_T_point <<
			1, 0, 0, g_point_x,
			0, 1, 0, g_point_y,
			0, 0, 1, 0,
			0, 0, 0, 1;

	g_T_robot = robotis_framework::getTransformationXYZRPY(robot_x,robot_y,0,0,0,robot_yaw_degree*DEGREE2RADIAN);
	robot_T_g = robotis_framework::getInverseTransformation(g_T_robot);
	robot_T_point = robot_T_g * g_T_point;
	return robot_T_point;
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
void FootStepPlanner::CalculateStepData(double x, double pre_x, double y, double pre_y, double r_point_x, double r_point_y)
{
	int step_num_temp_ = 0;
	double step_length_temp_ = 0;
	double desired_distance_ = 0;

	desired_distance_ = sqrt(pow((x-pre_x),2) + pow((y-pre_y),2));

	if((pre_x - x) == 0 || (pre_y - y) == 0) // 게걸음 나중에.
	{

	}
	else
	{
		for(int num = 1; num < (desired_distance_/step_length_max)+1 ; num++)// 최대 걸음수 계산
		{
			if((step_length_min < desired_distance_/num) && ((step_length_max > desired_distance_/num))) // 로봇이 갈수 있는 (뻗을수있는 length 범위 안에 들어온다면,)
			{
				step_length_temp_ = desired_distance_/num;
				step_num_temp_ = num;
				foot_set_command_msg.step_num = step_num_temp_;
				foot_set_command_msg.step_length = step_length_temp_;
				printf("step_length_temp :: %f  step_num_temp :: %d  \n", step_length_temp_, num);
				break;
			}
		}
		if(r_point_y > 0)
			foot_set_command_msg.command = "forward";
		else if (r_point_y < 0)
			foot_set_command_msg.command = "backward";
		else
			foot_set_command_msg.command = "stop";

		foot_set_command_msg.step_time = 2; //변경 할 수 있음
		foot_step_command_pub.publish(foot_set_command_msg);
	}

}
bool FootStepPlanner::CheckArrival(std::string status, int via_point_num, int all_point_num)
{

	return true;

}


