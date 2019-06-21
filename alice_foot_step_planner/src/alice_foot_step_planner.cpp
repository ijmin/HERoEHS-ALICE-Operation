/*
 * alice_foot_step_planner.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: robotemperor
 */
#include "alice_foot_step_planner/alice_foot_step_planner.h"
using namespace alice;
// Foot step planner algorithm
Command_generator command_controller;

FootStepPlanner::FootStepPlanner()
{
  // load yaml
  step_length_max = 0;
  step_length_min = 0;
  pre_position_x = 0;
  pre_position_y = 0;
  on_process_msg.data = 0;
  walking_mode = 1; // 0 : darwin walking / 1 : preview walking
  alice_id_num_="";
  kick_y_cob_ = 0;
  current_x = 0;
  current_y = 0;
  alice_id_int = 1;
  //readIDAlice();
}
FootStepPlanner::~FootStepPlanner()
{

}
/*
void FootStepPlanner::readIDAlice()
{
  const char* env_p = std::getenv("ALICE_HOST");
  std::string alice_id;
  ROS_INFO("%s",env_p);
  if(env_p == NULL )
  {
    ROS_INFO("FROM Test yaml");
    alice_id="";
  }
  else
  {
    alice_id=env_p;
    if(alice_id == "" )
    {
      ROS_INFO("FROM 1 yaml");
      alice_id="_1";
    }

    else if(alice_id == "alice1nuke")
    {
      ROS_INFO("FROM 1 yaml");
      alice_id="_1";
    }
    else if(alice_id == "alice2nuke")
    {
      ROS_INFO("FROM 2 yaml");
      alice_id="_2";
    }
  }
  std::string kick_path = ros::package::getPath("alice_foot_step_generator")+"/data/kick_param"+alice_id+".yaml";
  YAML::Node kick_doc;
  try
  {
    kick_doc = YAML::LoadFile(kick_path.c_str());
  }catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load kick yaml file!");
    return;
  }
  int alice_id_int  = kick_doc["id"].as<int>();

  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  alice_id_num_ = alice_id_stream.str();
}
 */
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

  alice_id_int  = nh.param<int>("alice_userid",0);

  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  alice_id_num_ = alice_id_stream.str();

  if(balance_param_file == "" || joint_feedback_file == "")
  {
    ROS_ERROR("NO file path in the ROS parameters.");
  }

  //pub
  foot_step_command_pub     = nh.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_generator/walking_command", 1);
  on_process_pub            = nh.advertise<std_msgs::Bool>("/heroehs/alice/on_process", 1);

  walking_command_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 1);
  //foot_step_2d_pub    = nh.advertise<alice_foot_step_generator::Step2DArray>("/heroehs/alice_foot_step_generator/footsteps_2d", 1);

  //sub
  move_command_sub_         = nh.subscribe("/heroehs/move_command", 10, &FootStepPlanner::moveCommandStatusMsgCallback, this);
  walking_module_status_sub = nh.subscribe("/heroehs/status", 10, &FootStepPlanner::walkingModuleStatusMsgCallback, this);
  environment_detector_sub  = nh.subscribe("/heroehs/environment_detector", 5, &FootStepPlanner::environmentDetectorMsgCallback, this);
  //walking_path_planner_test_sub = nh.subscribe("/heroehs/alice_walking_path_planner_test", 10, &FootStepPlanner::walkingPathPlannerStatusMsgCallback, this);

  command_generator_sub  = nh.subscribe("/heroehs/command_generator", 5, &FootStepPlanner::commandGeneratorMsgCallback, this);


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
  init_pose_path = ros::package::getPath("alice_foot_step_planner") + "/data/initial_condition"+ alice_id_num_+".yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
  //data initialize
  parse_init_data_(init_pose_path);
}
void FootStepPlanner::read_kick_param()
{
  std::string kick_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
  kick_path = ros::package::getPath("alice_foot_step_generator") + "/data/kick_param_"+ alice_id_num_+".yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
  YAML::Node kick_doc;
  try
  {
    kick_doc = YAML::LoadFile(kick_path.c_str());
  }catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load kick yaml file!");
    return;
  }
  kick_y_cob_  = kick_doc["kick_y_cob"].as<double>();
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
}
void FootStepPlanner::moveCommandStatusMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& move_command)
{
  if(move_command->key == "left")
  {
    command_controller.FootParam.command = "left";
    command_controller.step_type = "default";
  }
  else if(move_command->key == "right")
  {
    command_controller.FootParam.command = "right";
    command_controller.step_type = "default";
  }
  else if(move_command->key == "forward")
  {
    command_controller.FootParam.command = "forward";
    command_controller.step_type = "default";
  }
  else if(move_command->key == "backward")
  {
    command_controller.FootParam.command = "backward";
    command_controller.step_type = "default";
  }
  else if(move_command->key == "turn left")
  {
    command_controller.FootParam.command = "turn_left";
    command_controller.step_type = "default";
  }
  else if(move_command->key == "turn right")
  {
    command_controller.FootParam.command = "turn_right";
    command_controller.step_type = "default";
  }
  else if(move_command->key == "expanded left")
  {
    command_controller.FootParam.command = "expanded_left";
    command_controller.step_type = "expanded";
  }
  else if(move_command->key == "expanded right")
  {
    command_controller.FootParam.command = "expanded_right";
    command_controller.step_type = "expanded";
  }
  else if(move_command->key == "centered left")
  {
    command_controller.FootParam.command = "centered_left";
    command_controller.step_type = "centered";
  }
  else if(move_command->key == "centered right")
  {
    command_controller.FootParam.command = "centered_right";
    command_controller.step_type = "centered";
  }
  else if(move_command->key == "stop")
  {
    command_controller.FootParam.command = "stop";
    command_controller.step_type = "default";
  }

  if(move_command->value == "1")
  {
    command_controller.speed_switch = "1";
  }
  else if(move_command->value == "3")
  {
    command_controller.speed_switch = "3";
  }
  else if(move_command->value == "2")
  {
    command_controller.speed_switch = "2";
  }
  command_controller.command_switch = 2;
  command_controller.Set_FootParam(alice_id_int);

  /*
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

   */

  foot_set_command_msg = command_controller.FootParam;

  foot_step_command_pub.publish(foot_set_command_msg);
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
      set_balance_param_msg.request.balance_param.cob_y_offset_m = kick_y_cob_;
    else
      set_balance_param_msg.request.balance_param.cob_y_offset_m = -kick_y_cob_;

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
void FootStepPlanner::commandGeneratorMsgCallback(const alice_foot_step_generator::FootStepCommandConstPtr& msg)
{
  foot_set_command_msg.step_num = msg->step_num;
  foot_set_command_msg.step_length = msg->step_length;
  foot_set_command_msg.side_step_length = msg->side_step_length;
  foot_set_command_msg.step_angle_rad = msg->step_angle_rad;
  foot_set_command_msg.step_time = msg->step_time;

  foot_set_command_msg.command = msg->command;

  foot_step_command_pub.publish(foot_set_command_msg);
}



void FootStepPlanner::alice_id_Callback(const std_msgs::String::ConstPtr& alice_id)
{
  std::string step_path_;
  if(alice_id->data == "1")
  {
    step_path_ = ros::package::getPath("alice_foot_step_planner") + "/config/step_parameter1.yaml";
  }
  else if(alice_id->data == "2")
  {
    step_path_ = ros::package::getPath("alice_foot_step_planner") + "/config/step_parameter2.yaml";
  }
  command_controller.parse_step_param_data(step_path_);
}
void FootStepPlanner::current_status_Callback(const std_msgs::String::ConstPtr& log_moving_status)
{
  command_controller.current_status = log_moving_status->data;
}
/////////////////////////
/////////////////////////
/////////////////////////
/////////////////////////
Command_generator::Command_generator()
{
  //Default_Setting//
  Input_Text();
  Make_Log();
  command_switch = 0;
  speed_switch = 2;
  FootParam.step_num = 0;
  FootParam.step_length = 0;
  FootParam.side_step_length = 0;
  FootParam.step_angle_rad = 0;
  FootParam.step_time = 0;
  start_time = clock();
  //////////////////

  ROS_INFO("command_generator_start");
}
void Command_generator::Set_FootParam(int alice_id)
{
  if(step_type == "default")
  {
    FootParam.step_num = default_step_num;
    FootParam.step_length = default_step_length;
    FootParam.side_step_length = default_side_step_length;
    FootParam.step_angle_rad = default_step_angle_rad;
    FootParam.step_time = default_step_time;
  }
  else if(step_type == "expanded")
  {
    FootParam.step_num = expanded_step_num;
    FootParam.step_length = expanded_step_length;
    FootParam.side_step_length = expanded_side_step_length;
    FootParam.step_angle_rad = expanded_step_angle_rad;
    FootParam.step_time = expanded_step_time;
  }
  else if(step_type == "centered")
  {
    FootParam.step_num = centered_step_num;
    FootParam.step_length = centered_step_length;
    FootParam.side_step_length = centered_side_step_length;
    FootParam.step_angle_rad = centered_step_angle_rad;
    FootParam.step_time = centered_step_time;
  }
  if(alice_id == 1)
  {
    if(speed_switch == "1")
    {
      FootParam.step_time = FootParam.step_time*1.5;
    }
    else if(speed_switch == "2")
    {
      FootParam.step_time = FootParam.step_time*1;
    }
    else if(speed_switch == "3")
    {
      FootParam.step_time = FootParam.step_time*0.5;
    }
  }
}
void Command_generator::parse_step_param_data(std::string path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

  }catch(const std::exception& e) // 에러 점검
  {
    ROS_ERROR("Fail to load yaml file!");
    return;
  }
  default_step_num = doc["default_step_num"].as<double>();
  default_step_length = doc["default_step_length"].as<double>();
  default_side_step_length = doc["default_side_step_length"].as<double>();
  default_step_angle_rad = doc["default_step_angle_radian"].as<double>();
  default_step_time = doc["default_step_time"].as<double>();

  expanded_step_num = doc["expanded_step_num"].as<double>();
  expanded_step_length = doc["expanded_step_length"].as<double>();
  expanded_side_step_length = doc["expanded_side_step_length"].as<double>();
  expanded_step_angle_rad = doc["expanded_step_angle_radian"].as<double>();
  expanded_step_time = doc["expanded_step_time"].as<double>();

  centered_step_num = doc["centered_step_num"].as<double>();
  centered_step_length = doc["centered_step_length"].as<double>();
  centered_side_step_length = doc["centered_side_step_length"].as<double>();
  centered_step_angle_rad = doc["centered_step_angle_radian"].as<double>();
  centered_step_time = doc["centered_step_time"].as<double>();
}

void Command_generator::Make_Log(void)
{
  time_t curr_time;
  struct tm *curr_tm;
  int year, month, day;
  curr_time = time(NULL);
  curr_tm = localtime(&curr_time);
  year = curr_tm->tm_year + 1900;
  month = curr_tm->tm_mon + 1;
  day = curr_tm->tm_mday;
  init_hour = curr_tm->tm_hour;
  init_min = curr_tm->tm_min;
  init_sec = curr_tm->tm_sec;
  char Logname[256];
  sprintf(Logname,"%d-%d-%d-%d-%d-%d",year,month,day,init_hour,init_min,init_sec);
  init_log_path = ros::package::getPath("command_generator") + "/log/" + Logname + ".txt";
  out.open(init_log_path.c_str());
  out<<"command|";
  out<<"status|";
  out<<"accept/ignore|";
  out<<"step_time|";
  out<<"step_num|";
  out<<"step_length|";
  out<<"side_step_length|";
  out<<"step_angle_rad|";
  out<<"logtime|"<<'\n';
}

void Command_generator::Write_Log(void)
{
  clock_t curr_t;
  curr_t = clock();
  float result_time;
  result_time = (float)(curr_t-start_time)/(CLOCKS_PER_SEC);
  time_t curr_time;
  struct tm *curr_tm;
  int year, month, day, hour, min, sec;
  curr_time = time(NULL);
  curr_tm = localtime(&curr_time);
  year = curr_tm->tm_year + 1900;
  month = curr_tm->tm_mon + 1;
  day = curr_tm->tm_mday;
  hour = curr_tm->tm_hour - init_hour;
  min = curr_tm->tm_min - init_min;
  sec = curr_tm->tm_sec - init_sec;
  if(sec < 0)
  {
    sec = sec+60;
    min = min - 1;
  }
  if(min < 0)
  {
    min = min+60;
    hour = hour - 1;
  }
  if(hour < 0)
  {
    hour = hour+24;
  }
  if(FootParam.command == current_status)accept_or_ignore = "accept";
  else accept_or_ignore = "ignore";
  char Logname[256];
  sprintf(Logname,"%d:%d:%d",hour,min,sec);
  out<<FootParam.command<<"|";
  out<<current_status<<"|";
  out<<accept_or_ignore<<"|";
  out<<FootParam.step_time<<"|";
  out<<FootParam.step_num<<"|";
  out<<FootParam.step_length<<"|";
  out<<FootParam.side_step_length<<"|";
  out<<FootParam.step_angle_rad<<"|";
  out<<Logname<<"|"<<'\n';
}
void Command_generator::Input_Text(void)
{
  int i = 0;
  size_t found;
  string init_pose_path;
  ifstream inFile;
  init_pose_path = ros::package::getPath("command_generator") + "/command_input.txt";
  inFile.open(init_pose_path.c_str());
  for(string line; std::getline(inFile,line);)
  {
    found=line.find("=");

    switch(i)
    {
    case 0: Command_Period = atof(line.substr(found+2).c_str()); break;
    }
    i +=1;
  }
  inFile.close();
}
/*int main(int argc, char** argv)
{
  Command_generator command_controller;
  float count;
  count = 0;

  while(ros::ok())
  {

    if(count > 1000*command_controller.Command_Period)
    {
      if(command_controller.command_switch > 0)
      {
        command_controller.vel_pub_.publish(command_controller.FootParam);
        command_controller.Write_Log();
      }
      count = 0;
    }
    else count += 1;
    usleep(1000);
    ros::spinOnce();
  }
  command_controller.out.close();
}*/



