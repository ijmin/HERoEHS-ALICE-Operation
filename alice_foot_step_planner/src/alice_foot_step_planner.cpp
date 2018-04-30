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
	pre_position_x = 0;
	pre_position_y = 0;
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
	ros::NodeHandle nh;

	walking_module_status_sub = nh.subscribe("/robotis/status", 10, &FootStepPlanner::walkingModuleStatusMsgCallback, this);
	walking_path_planner_test_sub = nh.subscribe("/heroehs/alice_walking_path_planner_test", 10, &FootStepPlanner::walkingPathPlannerStatusMsgCallback, this);
	foot_step_command_pub     = nh.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_generator/walking_command", 1);
}
void FootStepPlanner::data_initialize()
{
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	init_pose_path = ros::package::getPath("alice_foot_step_planner") + "/data/initial_condition.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	//data initialize
	parse_init_data_(init_pose_path);
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

void FootStepPlanner::DecideStepNumLength(double distance)
{
	int step_num_temp_ = 0;
	double step_length_temp_ = 0;
	//printf("step_distance :: %f  step_num_min :: %f  \n", desired_distance_, (desired_distance_/step_length_min)+1);
	for(int num = 1; num < ((distance/step_length_min)+1)/2 ; num++)//
	{
		if(step_length_max > (distance/num)) // 로봇이 갈수 있는 (뻗을수있는 length 범위 안에 들어온다면,)
		{
			step_length_temp_ = distance/num;
			step_num_temp_ = num;
			foot_set_command_msg.step_num = step_num_temp_;
			foot_set_command_msg.step_length = step_length_temp_;
			foot_set_command_msg.side_step_length = step_length_temp_;
			//printf("step_length_temp :: %f  step_num_temp :: %d  \n", step_length_temp_, num);
			break;
		}
	}
}
void FootStepPlanner::AlignRobotYaw(double yaw_degree, std::string command)
{
	data_initialize();// have to modify
	double yaw_rad = yaw_degree*DEGREE2RADIAN;
	foot_set_command_msg.step_angle_rad = yaw_rad;
	foot_set_command_msg.command = command;
	foot_step_command_pub.publish(foot_set_command_msg);
}
void FootStepPlanner::CalculateStepData(double x, double pre_x, double y, double pre_y, std::string command)
{
	data_initialize();// have to modify
	double desired_distance_ = 0;
	desired_distance_ = sqrt(pow((x-pre_x),2) + pow((y-pre_y),2));

	if((pre_x - x) == 0 || (pre_y - y) == 0) // 게걸음 나중에.
	{
		if((pre_x - x) == 0 && (pre_y - y) == 0)
			return;

		DecideStepNumLength(desired_distance_);
	}
	else
	{
		DecideStepNumLength(desired_distance_);
	}

	foot_set_command_msg.command = command;
	foot_step_command_pub.publish(foot_set_command_msg);

}
bool FootStepPlanner::CheckArrival(std::string status, int via_point_num, int all_point_num)
{

	return true;

}
void FootStepPlanner::walkingPathPlannerStatusMsgCallback(const alice_operation_msgs::WalkingPathPlanner::ConstPtr& msg)
{
	if(!msg->command.compare("turn left") || !msg->command.compare("turn right"))
		AlignRobotYaw(msg->yaw_degree, msg->command);
	else
	{
		CalculateStepData(msg->position_x, pre_position_x, msg->position_y, pre_position_y, msg->command);

		pre_position_x = msg->position_x;
		pre_position_y = msg->position_y;
	}
}
void FootStepPlanner::parse_init_data_(const std::string &path)
{
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}

	step_length_max = doc["step_length_max"].as<double>();
	step_length_min = doc["step_length_min"].as<double>();
	foot_set_command_msg.step_num = doc["step_num"].as<double>();
	foot_set_command_msg.step_length = doc["step_length"].as<double>();
	foot_set_command_msg.step_time = doc["step_time"].as<double>();
	foot_set_command_msg.step_angle_rad = doc["step_angle_rad"].as<double>();
	foot_set_command_msg.side_step_length = doc["side_step_length"].as<double>();

}


