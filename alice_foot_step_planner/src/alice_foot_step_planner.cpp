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
	walking_mode = 1; // 0 : darwin walking / 1 : preview walking
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
void FootStepPlanner::environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg)
{
	for(int i = 0; i < msg->length; i++)
	{
		if(!msg->data[i].name.compare("ball"))
		{
			current_x = msg->data[i].roi.x_offset + msg->data[i].roi.width/2;
			current_y = msg->data[i].roi.y_offset + msg->data[i].roi.height/2;
		}
	}
}
void FootStepPlanner::initialize()
{
	ros::NodeHandle nh;

	/* Load ROS Parameter */
	balance_param_file = nh.param<std::string>("balance_param_path", "");
	joint_feedback_file  = nh.param<std::string>("joint_feedback_path", "");

	if(balance_param_file == "" || joint_feedback_file == "")
	{
		ROS_ERROR("NO file path in the ROS parameters.");
	}

	//pub
	foot_step_command_pub     = nh.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_generator/walking_command", 1);
	on_process_pub            = nh.advertise<std_msgs::Bool>("/heroehs/alice/on_process", 1);

	walking_command_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 1);
	walking_param_pub   = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 1);

	//sub
	move_command_sub_         = nh.subscribe("/heroehs/alice/move_command", 10, &FootStepPlanner::moveCommandStatusMsgCallback, this);
	walking_module_status_sub = nh.subscribe("/heroehs/status", 10, &FootStepPlanner::walkingModuleStatusMsgCallback, this);
	environment_detector_sub = nh.subscribe("/heroehs/environment_detector", 5, &FootStepPlanner::environmentDetectorMsgCallback, this);
	//walking_path_planner_test_sub = nh.subscribe("/heroehs/alice_walking_path_planner_test", 10, &FootStepPlanner::walkingPathPlannerStatusMsgCallback, this);

	//service
	set_balance_param_client =  nh.serviceClient<alice_walking_module_msgs::SetBalanceParam>("/heroehs/online_walking/set_balance_param");
	joint_feedback_gain_client = nh.serviceClient<alice_walking_module_msgs::SetJointFeedBackGain>("/heroehs/online_walking/joint_feedback_gain");

	set_balance_param_nuke_server   = nh.advertiseService("/heroehs/online_walking/set_balance_param_save", &FootStepPlanner::setBalanceParamServiceCallback, this);
	joint_feedback_gain_nuke_server = nh.advertiseService("/heroehs/online_walking/joint_feedback_gain_save", &FootStepPlanner::setJointFeedBackGainServiceCallback, this);

	parse_online_balance_param(balance_param_file);
	parse_online_joint_feedback_param(joint_feedback_file);
}
void FootStepPlanner::data_initialize()
{
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	init_pose_path = ros::package::getPath("alice_foot_step_planner") + "/data/initial_condition.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	//data initialize
	parse_init_data_(init_pose_path);
}

void FootStepPlanner::DecideStepNumLength(double distance , std::string command, int mode)
{
	if(mode == 1)// preview control
	{

		if(!command.compare("forward") || !command.compare("backward") )
		{
			if(distance >= 2.0)
			{

				foot_set_command_msg.step_num = ((int)(distance*6)) + 1;
				foot_set_command_msg.step_length = 0.1;
				foot_set_command_msg.step_time = 5;

			}
			else
			{
				foot_set_command_msg.step_num = ((int)(distance*5)) + 1;
				foot_set_command_msg.step_length = 0.1;
				foot_set_command_msg.step_time = 5;
			}
		}
		else  // left right
		{
			if(distance >= 0.05)
			{
				foot_set_command_msg.step_num =  (int) (distance*20);
				foot_set_command_msg.side_step_length = 0.05;
				foot_set_command_msg.step_time =5;
			}
			else
			{
				foot_set_command_msg.step_num = 1;
				foot_set_command_msg.side_step_length = distance;
				foot_set_command_msg.step_time =5;
			}
		}
	}
	else // darwin walking
	{

	}
}
void FootStepPlanner::AlignRobotYaw(double yaw_rad, std::string command, int mode)
{
	if(mode == 1) // preview control
	{
		if(yaw_rad >= 0.15)
		{
			foot_set_command_msg.step_num = (int) (yaw_rad/0.15);
			foot_set_command_msg.step_angle_rad = 0.15;
			foot_set_command_msg.step_time = 5;

		}
		else
		{
			foot_set_command_msg.step_num = 1;
			foot_set_command_msg.step_angle_rad = yaw_rad;
			foot_set_command_msg.step_time = 5;
		}
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
		if(command.compare("stop"))
			DecideStepNumLength(desired_distance_, command, mode);

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
		change_walking_kick_mode("walking", "");

		if(msg->command == 2)
		{
			if(msg->transform.z > 0)
			{
				AlignRobotYaw(msg->transform.z, "turn left", walking_mode);
			}
			if(msg->transform.z < 0)
			{
				AlignRobotYaw(fabs(msg->transform.z), "turn right", walking_mode);
			}
		}
		else
		{
			if(msg->command == 0)
			{
				if(msg->transform.x > 0)
					CalculateStepData(msg->transform.x, 0, "forward", walking_mode);
				else if(msg->transform.x < 0)
					CalculateStepData(fabs(msg->transform.x), 0, "backward", walking_mode);
				else
				{
					CalculateStepData(0, 0, "stop", walking_mode);
				}

			}
			if(msg->command == 1)
			{
				if(msg->transform.y > 0)
					CalculateStepData(0, msg->transform.y, "left", walking_mode);
				else if(msg->transform.y < 0)
					CalculateStepData(0, fabs(msg->transform.y), "right", walking_mode);
				else
				{
					CalculateStepData(0, 0, "stop", walking_mode);
				}
			}
		}
	}
	else if (msg->mode == 1) //kick
	{
		if(msg->command == 0)
		{
			change_walking_kick_mode("kick", "left kick");
			foot_set_command_msg.command = "left kick";
			foot_step_command_pub.publish(foot_set_command_msg);
		}
		else
		{
			change_walking_kick_mode("kick", "right kick");
			foot_set_command_msg.command = "right kick";
			foot_step_command_pub.publish(foot_set_command_msg);
		}
	}
	else
	{
		CalculateStepData(0, 0, "stop", walking_mode);
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
void FootStepPlanner::parse_online_balance_param(std::string path)
{
	YAML::Node doc; // YAML file class 선언!
	//std::string path_ = ros::package::getPath("alice_foot_step_planner") + "/data/balance_param.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}

	set_balance_param_msg.request.updating_duration = doc["updating_duration"].as<double>();
	set_balance_param_msg.request.balance_param.cob_x_offset_m = doc["cob_x_offset_m"].as<double>();
	set_balance_param_msg.request.balance_param.cob_y_offset_m = doc["cob_y_offset_m"].as<double>();

	//gain load //
	set_balance_param_msg.request.balance_param.foot_roll_gyro_p_gain = doc["foot_roll_gyro_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_gyro_d_gain = doc["foot_roll_gyro_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_p_gain = doc["foot_pitch_gyro_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_d_gain = doc["foot_pitch_gyro_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_roll_angle_p_gain = doc["foot_roll_angle_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_angle_d_gain = doc["foot_roll_angle_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_p_gain = doc["foot_pitch_angle_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_d_gain = doc["foot_pitch_angle_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_x_force_p_gain = doc["foot_x_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_x_force_d_gain = doc["foot_x_force_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_y_force_p_gain = doc["foot_y_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_y_force_d_gain = doc["foot_y_force_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_z_force_p_gain = doc["foot_z_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_z_force_d_gain = doc["foot_z_force_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_torque_p_gain = doc["foot_roll_torque_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_torque_d_gain = doc["foot_roll_torque_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_p_gain = doc["foot_pitch_torque_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_d_gain = doc["foot_pitch_torque_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.roll_gyro_cut_off_frequency = doc["roll_gyro_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.pitch_gyro_cut_off_frequency = doc["pitch_gyro_cut_off_frequency"].as<double>();

	set_balance_param_msg.request.balance_param.roll_angle_cut_off_frequency = doc["roll_angle_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.pitch_angle_cut_off_frequency = doc["pitch_angle_cut_off_frequency"].as<double>();

	set_balance_param_msg.request.balance_param.foot_x_force_cut_off_frequency = doc["foot_x_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_y_force_cut_off_frequency = doc["foot_y_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_z_force_cut_off_frequency = doc["foot_z_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_torque_cut_off_frequency = doc["foot_roll_torque_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_cut_off_frequency = doc["foot_pitch_torque_cut_off_frequency"].as<double>();

	set_balance_param_client.call(set_balance_param_msg);

}
void FootStepPlanner::parse_online_joint_feedback_param(std::string path)
{
	YAML::Node doc; // YAML file class 선언!
	//std::string path_ = ros::package::getPath("alice_foot_step_planner") + "/data/joint_feedback_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	joint_feedback_gain_msg.request.updating_duration = doc["updating_duration"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain = doc["r_leg_hip_y_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain = doc["r_leg_hip_y_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain = doc["r_leg_hip_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain = doc["r_leg_hip_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain = doc["r_leg_hip_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain = doc["r_leg_hip_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain = doc["r_leg_an_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain = doc["r_leg_an_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_p_gain = doc["r_leg_kn_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_d_gain = doc["r_leg_kn_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain = doc["r_leg_an_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain = doc["r_leg_an_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain = doc["l_leg_hip_y_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain = doc["l_leg_hip_y_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain = doc["l_leg_hip_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain = doc["l_leg_hip_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain = doc["l_leg_hip_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain = doc["l_leg_hip_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_p_gain = doc["l_leg_kn_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_d_gain = doc["l_leg_kn_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain = doc["l_leg_an_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain = doc["l_leg_an_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain = doc["l_leg_an_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain = doc["l_leg_an_r_d_gain"].as<double>();

	joint_feedback_gain_client.call(joint_feedback_gain_msg);

}
void FootStepPlanner::change_walking_kick_mode(std::string mode, std::string kick_mode)
{
	if(!mode.compare("walking"))
	{
		parse_online_balance_param(balance_param_file);
		parse_online_joint_feedback_param(joint_feedback_file);
	}
	else
	{
		if(!kick_mode.compare("right kick"))
			set_balance_param_msg.request.balance_param.cob_y_offset_m = 0.05;
		else
			set_balance_param_msg.request.balance_param.cob_y_offset_m = -0.05;

		joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_p_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_d_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_p_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_d_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain  = 0;
		joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain  = 0;

		set_balance_param_client.call(set_balance_param_msg);
		joint_feedback_gain_client.call(joint_feedback_gain_msg);
	}

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


bool FootStepPlanner::setJointFeedBackGainServiceCallback(alice_walking_module_msgs::SetJointFeedBackGain::Request &req,
		alice_walking_module_msgs::SetJointFeedBackGain::Response &res)
{

	YAML::Emitter out;
	//std::string path_ = ros::package::getPath("alice_foot_step_planner") + "/data/joint_feedback_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.

	out << YAML::BeginMap;
	out << YAML::Key << "r_leg_hip_y_p_gain" << YAML::Value << req.feedback_gain.r_leg_hip_y_p_gain;
	out << YAML::Key << "r_leg_hip_y_d_gain" << YAML::Value << req.feedback_gain.r_leg_hip_y_d_gain;
	out << YAML::Key << "r_leg_hip_r_p_gain" << YAML::Value << req.feedback_gain.r_leg_hip_r_p_gain;
	out << YAML::Key <<	"r_leg_hip_r_d_gain" << YAML::Value << req.feedback_gain.r_leg_hip_r_d_gain;
	out << YAML::Key <<	"r_leg_hip_p_p_gain" << YAML::Value << req.feedback_gain.r_leg_hip_p_p_gain;
	out << YAML::Key <<	"r_leg_hip_p_d_gain" << YAML::Value << req.feedback_gain.r_leg_hip_p_d_gain;
	out << YAML::Key <<	"r_leg_kn_p_p_gain" << YAML::Value << req.feedback_gain.r_leg_kn_p_p_gain ;
	out << YAML::Key <<	"r_leg_kn_p_d_gain" << YAML::Value << req.feedback_gain.r_leg_kn_p_d_gain ;
	out << YAML::Key <<	"r_leg_an_p_p_gain" << YAML::Value << req.feedback_gain.r_leg_an_p_p_gain ;
	out << YAML::Key <<	"r_leg_an_p_d_gain" << YAML::Value << req.feedback_gain.r_leg_an_p_d_gain ;
	out << YAML::Key <<	"r_leg_an_r_p_gain" << YAML::Value << req.feedback_gain.r_leg_an_r_p_gain ;
	out << YAML::Key <<	"r_leg_an_r_d_gain" << YAML::Value << req.feedback_gain.r_leg_an_r_d_gain ;
	out << YAML::Key <<	"l_leg_hip_y_p_gain" << YAML::Value << req.feedback_gain.l_leg_hip_y_p_gain;
	out << YAML::Key <<	"l_leg_hip_y_d_gain" << YAML::Value << req.feedback_gain.l_leg_hip_y_d_gain;
	out << YAML::Key <<	"l_leg_hip_r_p_gain" << YAML::Value << req.feedback_gain.l_leg_hip_r_p_gain;
	out << YAML::Key <<	"l_leg_hip_r_d_gain" << YAML::Value << req.feedback_gain.l_leg_hip_r_d_gain;
	out << YAML::Key <<	"l_leg_hip_p_p_gain" << YAML::Value << req.feedback_gain.l_leg_hip_p_p_gain;
	out << YAML::Key <<	"l_leg_hip_p_d_gain" << YAML::Value << req.feedback_gain.l_leg_hip_p_d_gain;
	out << YAML::Key <<	"l_leg_kn_p_p_gain" << YAML::Value << req.feedback_gain.l_leg_kn_p_p_gain ;
	out << YAML::Key <<	"l_leg_kn_p_d_gain" << YAML::Value << req.feedback_gain.l_leg_kn_p_d_gain ;
	out << YAML::Key <<	"l_leg_an_p_p_gain" << YAML::Value << req.feedback_gain.l_leg_an_p_p_gain ;
	out << YAML::Key <<	"l_leg_an_p_d_gain" << YAML::Value << req.feedback_gain.l_leg_an_p_d_gain ;
	out << YAML::Key <<	"l_leg_an_r_p_gain" << YAML::Value << req.feedback_gain.l_leg_an_r_p_gain ;
	out << YAML::Key <<	"l_leg_an_r_d_gain" << YAML::Value << req.feedback_gain.l_leg_an_r_d_gain ;
	out << YAML::Key <<	"updating_duration"<< YAML::Value << 1.0;

	out << YAML::EndMap;
	std::ofstream fout(joint_feedback_file.c_str());
	fout << out.c_str(); // dump it back into the file


	printf("Joint Feed Back SAVE!!\n");
	return true;
}

bool FootStepPlanner::setBalanceParamServiceCallback(alice_walking_module_msgs::SetBalanceParam::Request  &req,
		alice_walking_module_msgs::SetBalanceParam::Response &res)
{
	YAML::Emitter out;
	//std::string path_ = ros::package::getPath("alice_foot_step_planner") + "/data/balance_param.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.

	out << YAML::BeginMap;
	out << YAML::Key << "cob_x_offset_m" << YAML::Value << req.balance_param.cob_x_offset_m;
	out << YAML::Key << "cob_y_offset_m" << YAML::Value << req.balance_param.cob_y_offset_m;
	out << YAML::Key << "foot_roll_gyro_p_gain"  << YAML::Value << req.balance_param.foot_roll_gyro_p_gain;
	out << YAML::Key <<	"foot_roll_gyro_d_gain"  << YAML::Value << req.balance_param.foot_roll_gyro_d_gain;
	out << YAML::Key <<	"foot_pitch_gyro_p_gain" << YAML::Value << req.balance_param.foot_pitch_gyro_p_gain;
	out << YAML::Key <<	"foot_pitch_gyro_d_gain" << YAML::Value << req.balance_param.foot_pitch_gyro_d_gain;
	out << YAML::Key <<	"foot_roll_angle_p_gain" << YAML::Value << req.balance_param.foot_roll_angle_p_gain;
	out << YAML::Key <<	"foot_roll_angle_d_gain" << YAML::Value << req.balance_param.foot_roll_angle_d_gain;
	out << YAML::Key <<	"foot_pitch_angle_p_gain"<< YAML::Value << req.balance_param.foot_pitch_angle_p_gain;
	out << YAML::Key <<	"foot_pitch_angle_d_gain"<< YAML::Value << req.balance_param.foot_pitch_angle_d_gain;
	out << YAML::Key <<	"foot_x_force_p_gain" << YAML::Value << req.balance_param.foot_x_force_p_gain;
	out << YAML::Key <<	"foot_x_force_d_gain" << YAML::Value << req.balance_param.foot_x_force_d_gain;
	out << YAML::Key <<	"foot_y_force_p_gain" << YAML::Value << req.balance_param.foot_y_force_p_gain;
	out << YAML::Key <<	"foot_y_force_d_gain" << YAML::Value << req.balance_param.foot_y_force_d_gain;
	out << YAML::Key <<	"foot_z_force_p_gain" << YAML::Value << req.balance_param.foot_z_force_p_gain;
	out << YAML::Key <<	"foot_z_force_d_gain" << YAML::Value << req.balance_param.foot_z_force_d_gain;
	out << YAML::Key <<	"foot_roll_torque_p_gain" << YAML::Value << req.balance_param.foot_roll_torque_p_gain;
	out << YAML::Key <<	"foot_roll_torque_d_gain" << YAML::Value << req.balance_param.foot_roll_torque_d_gain;
	out << YAML::Key <<	"foot_pitch_torque_p_gain"<< YAML::Value << req.balance_param.foot_pitch_torque_p_gain;
	out << YAML::Key <<	"foot_pitch_torque_d_gain"<< YAML::Value << req.balance_param.foot_pitch_torque_d_gain;
	out << YAML::Key <<	"roll_gyro_cut_off_frequency"<< YAML::Value << req.balance_param.roll_gyro_cut_off_frequency;
	out << YAML::Key <<	"pitch_gyro_cut_off_frequency"<< YAML::Value << req.balance_param.pitch_gyro_cut_off_frequency;
	out << YAML::Key <<	"roll_angle_cut_off_frequency"<< YAML::Value << req.balance_param.roll_angle_cut_off_frequency;
	out << YAML::Key <<	"pitch_angle_cut_off_frequency"<< YAML::Value << req.balance_param.pitch_angle_cut_off_frequency;
	out << YAML::Key <<	"foot_x_force_cut_off_frequency"<< YAML::Value << req.balance_param.foot_x_force_cut_off_frequency;
	out << YAML::Key <<	"foot_y_force_cut_off_frequency"<< YAML::Value << req.balance_param.foot_y_force_cut_off_frequency;
	out << YAML::Key <<	"foot_z_force_cut_off_frequency"<< YAML::Value << req.balance_param.foot_z_force_cut_off_frequency;
	out << YAML::Key <<	"foot_roll_torque_cut_off_frequency"<< YAML::Value << req.balance_param.foot_roll_torque_cut_off_frequency;
	out << YAML::Key <<	"foot_pitch_torque_cut_off_frequency"<< YAML::Value << req.balance_param.foot_pitch_torque_cut_off_frequency;
	out << YAML::Key <<	"updating_duration"<< YAML::Value << 1.0;
	out << YAML::EndMap;
	std::ofstream fout(balance_param_file.c_str());
	fout << out.c_str(); // dump it back into the file

	printf("Balance Param SAVE!!\n");

	return true;
}


