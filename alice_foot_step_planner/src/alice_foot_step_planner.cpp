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
	walking_mode = 0; // 0 : darwin walking / 1 : preview walking
	darwin_step_length_x = 0;
	darwin_step_length_y = 0;

	std::string default_param_path = ros::package::getPath("alice_op3_walking_module") + "/config/param.yaml";
	loadWalkingParam(default_param_path);
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

	//pub
	foot_step_command_pub     = nh.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_generator/walking_command", 1);
	on_process_pub            = nh.advertise<std_msgs::Bool>("/heroehs/alice/on_process", 1);

	walking_command_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 1);
	walking_param_pub   = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 1);

	//sub
	move_command_sub_         = nh.subscribe("/heroehs/alice/move_command", 10, &FootStepPlanner::moveCommandStatusMsgCallback, this);
	walking_module_status_sub = nh.subscribe("/heroehs/status", 10, &FootStepPlanner::walkingModuleStatusMsgCallback, this);
	//walking_path_planner_test_sub = nh.subscribe("/heroehs/alice_walking_path_planner_test", 10, &FootStepPlanner::walkingPathPlannerStatusMsgCallback, this);
}
void FootStepPlanner::data_initialize()
{
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	init_pose_path = ros::package::getPath("alice_foot_step_planner") + "/data/initial_condition.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	//data initialize
	parse_init_data_(init_pose_path);
}

void FootStepPlanner::DecideStepNumLength(double distance , int mode)
{
	if(mode == 1)// preview control
	{
		int step_num_temp_ = 0;
		double step_length_temp_ = 0;

		step_num_temp_ = (int) (distance/0.1)/2;
		foot_set_command_msg.step_num = step_num_temp_;
		foot_set_command_msg.step_length = 0.1;
		foot_set_command_msg.side_step_length = step_length_temp_;

	}
	else // darwin walking
	{

	}
}
void FootStepPlanner::AlignRobotYaw(double yaw_degree, std::string command, int mode)
{
	if(mode == 1) // preview control
	{
		data_initialize();// have to modify
		double yaw_rad = yaw_degree*DEGREE2RADIAN;
		foot_set_command_msg.step_angle_rad = yaw_rad;
		foot_set_command_msg.command = command;
		foot_step_command_pub.publish(foot_set_command_msg);
	}
	else // darwing walking
	{

	}
}
void FootStepPlanner::CalculateStepData(double x, double y, std::string command, int mode)
{
	if(mode == 1) // preview control
	{
		data_initialize();// have to modify
		double desired_distance_ = 0;
		desired_distance_ = sqrt(pow(x,2) + pow(y,2));
		DecideStepNumLength(desired_distance_, 0);

		foot_set_command_msg.command = command;
		foot_step_command_pub.publish(foot_set_command_msg);
	}
	else // darwing walking
	{

	}
}
void FootStepPlanner::moveCommandStatusMsgCallback(const alice_msgs::MoveCommand::ConstPtr& msg)
{

	if(msg->mode == 0)
	{
		if(msg->command == 2)
		{
			if(msg->transform.z > 0)
			{
				AlignRobotYaw(msg->transform.z, "turn left", 0);
			}
			if(msg->transform.z < 0)
			{
				AlignRobotYaw(msg->transform.z, "turn right", 0);
			}
		}
		else
		{
			if(msg->command == 0)
			{
				if(msg->transform.x > 0)
					CalculateStepData(msg->transform.x, 0, "forward", 0);
				else if(msg->transform.x < 0)
					CalculateStepData(msg->transform.x, 0, "backward", 0);
				else
				{
					CalculateStepData(0, 0, "stop", 0);
				}

			}
			if(msg->command == 1)
			{
				if(msg->transform.y > 0)
					CalculateStepData(0, msg->transform.y, "left", 0);
				else if(msg->transform.y < 0)
					CalculateStepData(0, msg->transform.y, "right", 0);
				else
				{
					CalculateStepData(0, 0, "stop", 0);
				}
			}
		}
	}
	else if (msg->mode == 1)
	{
		//foot_set_command_msg.command = "right kick";
		//foot_step_command_pub.publish(foot_set_command_msg);
	}
	else
	{
		CalculateStepData(0, 0, "stop", 0);
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
void FootStepPlanner::loadWalkingParam(const std::string &path)
{
	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str());
	} catch (const std::exception& e)
	{
		ROS_ERROR("Fail to load yaml file.");
		return;
	}

	// parse movement time
	walking_param_msgs.init_x_offset = doc["x_offset"].as<double>();
	walking_param_msgs.init_y_offset = doc["y_offset"].as<double>();
	walking_param_msgs.init_z_offset = doc["z_offset"].as<double>();
	walking_param_msgs.init_roll_offset = doc["roll_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_msgs.init_pitch_offset = doc["pitch_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_msgs.init_yaw_offset = doc["yaw_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_msgs.hip_pitch_offset = doc["hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
	// time
	walking_param_msgs.period_time = doc["period_time"].as<double>() * 0.001;    // ms -> s
	walking_param_msgs.dsp_ratio = doc["dsp_ratio"].as<double>();
	walking_param_msgs.step_fb_ratio = doc["step_forward_back_ratio"].as<double>();
	// walking
	walking_param_msgs.x_move_amplitude = 0;
	walking_param_msgs.y_move_amplitude = 0;
	walking_param_msgs.z_move_amplitude = doc["foot_height"].as<double>();
	walking_param_msgs.angle_move_amplitude = 0;
	walking_param_msgs.move_aim_on = 0;

	// balance
	// walking_param_.balance_enable
	walking_param_msgs.balance_hip_roll_gain = doc["balance_hip_roll_gain"].as<double>();
	walking_param_msgs.balance_knee_gain = doc["balance_knee_gain"].as<double>();
	walking_param_msgs.balance_ankle_roll_gain = doc["balance_ankle_roll_gain"].as<double>();
	walking_param_msgs.balance_ankle_pitch_gain = doc["balance_ankle_pitch_gain"].as<double>();
	walking_param_msgs.y_swap_amplitude = doc["swing_right_left"].as<double>();
	walking_param_msgs.z_swap_amplitude = doc["swing_top_down"].as<double>();
	walking_param_msgs.pelvis_offset = doc["pelvis_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_msgs.arm_swing_gain = doc["arm_swing_gain"].as<double>();

	// gain
	walking_param_msgs.p_gain = doc["p_gain"].as<int>();
	walking_param_msgs.i_gain = doc["i_gain"].as<int>();
	walking_param_msgs.d_gain = doc["d_gain"].as<int>();
}


