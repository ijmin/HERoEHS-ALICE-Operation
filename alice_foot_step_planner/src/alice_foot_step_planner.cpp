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
	on_process_msg.data = 0;
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

	if(!msg->status_msg.compare("Walking_Finished"))
	{
		on_process_msg.data = 0;
	}
}
void FootStepPlanner::initialize()
{
	ros::NodeHandle nh;

	//pub
	foot_step_command_pub     = nh.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_generator/walking_command", 1);
	on_process_pub            = nh.advertise<std_msgs::Bool>("/heroehs/alice/on_process", 1);

	//sub
	move_command_sub_         = nh.subscribe("/heroehs/alice/move_command", 10, &FootStepPlanner::moveCommandStatusMsgCallback, this);
	walking_module_status_sub = nh.subscribe("/heroehs/status", 10, &FootStepPlanner::walkingModuleStatusMsgCallback, this);
	walking_path_planner_test_sub = nh.subscribe("/heroehs/alice_walking_path_planner_test", 10, &FootStepPlanner::walkingPathPlannerStatusMsgCallback, this);
}
void FootStepPlanner::data_initialize()
{
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	init_pose_path = ros::package::getPath("alice_foot_step_planner") + "/data/initial_condition.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	//data initialize
	parse_init_data_(init_pose_path);
}

/*Eigen::Matrix4d FootStepPlanner::TransfomationGoalPointOnRobot(double robot_x, double robot_y, double robot_yaw_degree, double g_point_x, double g_point_y)
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
}*/
void FootStepPlanner::DecideStepNumLength(double distance)
{
	int step_num_temp_ = 0;
	double step_length_temp_ = 0;
	//printf("step_distance :: %f  step_num_min :: %f  \n", desired_distance_, (desired_distance_/step_length_min)+1);


	/*for(int num = 1; num < (distance/step_length_min)/2 ; num++)//
	{
		if(step_length_max > (distance/num)) // 로봇이 갈수 있는 (뻗을수있는 length 범위 안에 들어온다면,) 최대로 뻗을수 있는 보폭으로
		{
			step_length_temp_ = distance/num;
			step_num_temp_ = num;
			foot_set_command_msg.step_num = step_num_temp_;
			foot_set_command_msg.step_length = step_length_temp_;
			foot_set_command_msg.side_step_length = step_length_temp_;
			//printf("step_length_temp :: %f  step_num_temp :: %d  \n", step_length_temp_, num);
			break;
		}
	}*/

	//step_length_temp_ = (distance/10)/2;
	step_num_temp_ = (int) (distance/0.1)/2;
	foot_set_command_msg.step_num = step_num_temp_;
	foot_set_command_msg.step_length = 0.1;
	foot_set_command_msg.side_step_length = step_length_temp_;
}
void FootStepPlanner::AlignRobotYaw(double yaw_degree, std::string command)
{
	data_initialize();// have to modify
	double yaw_rad = yaw_degree*DEGREE2RADIAN;
	foot_set_command_msg.step_angle_rad = yaw_rad;
	foot_set_command_msg.command = command;
	foot_step_command_pub.publish(foot_set_command_msg);
}
void FootStepPlanner::CalculateStepData(double x, double y, std::string command)
{
	data_initialize();// have to modify
	double desired_distance_ = 0;
	desired_distance_ = sqrt(pow(x,2) + pow(y,2));
	DecideStepNumLength(desired_distance_);

	foot_set_command_msg.command = command;
	foot_step_command_pub.publish(foot_set_command_msg);
}
void FootStepPlanner::walkingPathPlannerStatusMsgCallback(const alice_operation_msgs::WalkingPathPlanner::ConstPtr& msg)
{
	if(!msg->command.compare("turn left") || !msg->command.compare("turn right"))
		AlignRobotYaw(msg->position.z, msg->command);
	else
	{
		CalculateStepData(msg->position.x, msg->position.y, msg->command);
	}
}
void FootStepPlanner::moveCommandStatusMsgCallback(const alice_msgs::MoveCommand::ConstPtr& msg)
{
	if(on_process_msg.data == 0)
	{
		on_process_msg.data = 1;
		if(msg->mode == 0)
		{
			if(msg->command == 2)
			{
				if(msg->transform.z > 0)
				{
					AlignRobotYaw(msg->transform.z, "turn left");
				}
				if(msg->transform.z < 0)
				{
					AlignRobotYaw(msg->transform.z, "turn right");
				}
			}
			else
			{
				if(msg->command == 0)
				{
					if(msg->transform.x > 0)
						CalculateStepData(msg->transform.x, 0, "forward");
					else if(msg->transform.x < 0)
						CalculateStepData(msg->transform.x, 0, "backward");
					else
					{
						CalculateStepData(0, 0, "stop");
					}

				}
				if(msg->command == 1)
				{
					if(msg->transform.y > 0)
						CalculateStepData(0, msg->transform.y, "left");
					else if(msg->transform.y < 0)
						CalculateStepData(0, msg->transform.y, "right");
					else
					{
						CalculateStepData(0, 0, "stop");
					}
				}
			}
		}
		else if (msg->mode == 1)
		{
			foot_set_command_msg.command = "right kick";
			foot_step_command_pub.publish(foot_set_command_msg);
		}
		else
		{
			CalculateStepData(0, 0, "stop");
		}

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
void FootStepPlanner::walkingPathMsgCAllBack(const std_msgs::String::ConstPtr& msg)
{

}


