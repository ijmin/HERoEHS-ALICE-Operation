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
  alice_id_num_="";
  kick_y_cob_ = 0;
  current_x = 0;
  current_y = 0;
  alice_id_int = 1;
  side_step_length_max = 0;
  side_step_length_min = 0;
  step_rad_max = 0;
  step_rad_min = 0;

  walking_check = false;
  motion_check  = false;
  previous_motion_check = 0;
  command_controller = new Command_generator;
  command_interval_check = 0;
  previous_command = "";
  forward_extended_length = 0.07;
  //readIDAlice();
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

  if(!msg->status_msg.compare("Walking_Started"))
  {
    walking_check = true;
  }
  if(!msg->status_msg.compare("Walking_Finished"))
  {
    walking_check = false;
    motion_check = false;
  }
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

  std::string step_path_;
  if(alice_id_int == 1)
  {
    step_path_ = ros::package::getPath("alice_foot_step_planner") + "/config/step_parameter1.yaml";
  }
  else if(alice_id_int == 2)
  {
    step_path_ = ros::package::getPath("alice_foot_step_planner") + "/config/step_parameter2.yaml";
  }
  command_controller->parse_step_param_data(step_path_);

  //pub
  foot_step_command_pub     = nh.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/alice_foot_step_generator/walking_command", 1);
  on_process_pub            = nh.advertise<std_msgs::Bool>("/heroehs/alice/on_process", 1);

  walking_command_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 1);
  //foot_step_2d_pub    = nh.advertise<alice_foot_step_generator::Step2DArray>("/heroehs/alice_foot_step_generator/footsteps_2d", 1);

  //sub
  //step_data_apply_sub = nh.subscribe("/heroehs/alice_foot_step_generator/walking_command", 10, &FootStepPlanner::stepDataApplyMsgCallback, this);
  move_command_sub_         = nh.subscribe("/heroehs/move_command", 10, &FootStepPlanner::moveCommandStatusMsgCallback, this);
  walking_module_status_sub = nh.subscribe("/heroehs/status", 10, &FootStepPlanner::walkingModuleStatusMsgCallback, this);
  environment_detector_sub  = nh.subscribe("/heroehs/environment_detector", 5, &FootStepPlanner::environmentDetectorMsgCallback, this);
  //walking_path_planner_test_sub = nh.subscribe("/heroehs/alice_walking_path_planner_test", 10, &FootStepPlanner::walkingPathPlannerStatusMsgCallback, this);
  alice_id_sub_ = nh.subscribe<std_msgs::String>("/heroehs/alice_id", 10, &FootStepPlanner::alice_id_Callback, this);



  //command_generator_sub  = nh.subscribe("/heroehs/command_generator", 5, &FootStepPlanner::commandGeneratorMsgCallback, this);


  //service
  set_balance_param_client =  nh.serviceClient<alice_walking_module_msgs::SetBalanceParam>("/heroehs/online_walking/set_balance_param");
  joint_feedback_gain_client = nh.serviceClient<alice_walking_module_msgs::SetJointFeedBackGain>("/heroehs/online_walking/joint_feedback_gain");

  set_balance_param_nuke_server   = nh.advertiseService("/heroehs/online_walking/set_balance_param_save", &FootStepPlanner::setBalanceParamServiceCallback, this);
  joint_feedback_gain_nuke_server = nh.advertiseService("/heroehs/online_walking/joint_feedback_gain_save", &FootStepPlanner::setJointFeedBackGainServiceCallback, this);

  parse_online_balance_param(balance_param_file);
  parse_online_joint_feedback_param(joint_feedback_file);
  data_initialize();
  read_kick_param();
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

void FootStepPlanner::DecideStepNumLength(double distance , std::string command, int robot_id)
{

  if(!command.compare("forward") || !command.compare("backward") )
  {
    if(distance >= step_length_max*3)
    {
      foot_set_command_msg.step_num = (int)((distance+step_length_max)/(step_length_max*2)+0.1);
      //foot_set_command_msg.step_num = 2;
      foot_set_command_msg.step_length = step_length_max;
     /* if(!command.compare("forward"))
        foot_set_command_msg.step_length = step_length_max + forward_extended_length;*/
    }
    else
    {
      foot_set_command_msg.step_num = 1;
      if(distance >= step_length_max)
        distance = step_length_max;
      foot_set_command_msg.step_length = distance;
    }
  }
  else  // left right
  {
    if(distance >= side_step_length_max)
    {
      foot_set_command_msg.step_num =  (int) (distance/side_step_length_max);
      foot_set_command_msg.side_step_length = side_step_length_max;
    }
    else
    {
      foot_set_command_msg.step_num = 1;
      foot_set_command_msg.side_step_length = distance;
    }
  }

  /*if(robot_id == 1)
  {
    foot_set_command_msg.step_time = 2;
  }
  else // number 2 robot
    foot_set_command_msg.step_time = 5;*/

}
void FootStepPlanner::AlignRobotYaw(double yaw_rad, std::string command, int robot_id)
{
  if(yaw_rad >= step_rad_max)
  {
    foot_set_command_msg.step_num = (int) (yaw_rad/step_rad_max);
  }
  else
  {
    foot_set_command_msg.step_num = 1;
    foot_set_command_msg.step_angle_rad = yaw_rad;
  }
  if(!command.compare("centered left") || !command.compare("centered right"))
  {
    if(yaw_rad > step_rad_max)
    {
      foot_set_command_msg.step_num = (int) (yaw_rad/step_rad_max);
      foot_set_command_msg.step_length = 0.3*(1-cos(step_rad_max));
      foot_set_command_msg.side_step_length = 0.3*sin(step_rad_max);
      foot_set_command_msg.step_angle_rad = step_rad_max;
    }
    else
    {
      foot_set_command_msg.step_num = 1;
      foot_set_command_msg.step_length = 0.3*(1-cos(yaw_rad));
      foot_set_command_msg.side_step_length = 0.3*sin(yaw_rad);
      foot_set_command_msg.step_angle_rad = yaw_rad;
    }
  }
  foot_set_command_msg.command = command;
  foot_step_command_pub.publish(foot_set_command_msg);
}
void FootStepPlanner::CalculateStepData(double x, double y, std::string command)
{
  //data_initialize();// have to modify
  double desired_distance_ = 0;
  desired_distance_ = sqrt(pow(x,2) + pow(y,2));
  if(command.compare("stop"))
    DecideStepNumLength(desired_distance_, command, alice_id_int);

  foot_set_command_msg.command = command;
  foot_step_command_pub.publish(foot_set_command_msg);
}
void FootStepPlanner::moveCommandStatusMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& move_command)
{
  if(previous_command == "stop" && walking_check == true)
    return;

  if(command_interval_check == 0)
    return;
  /////////////////////////////////////// motion check

  if(motion_check == true)
  {
    if(walking_check == true) // 명령이 씹힌것
    {
      command_interval_check = 0;
      return;
    }
  }

  if(move_command->key != "left_kick" && move_command->key != "right_kick" && move_command->key != "y_type_left_kick" && move_command->key != "y_type_right_kick")
  {
    if(previous_motion_check != motion_check && motion_check == false)
    {
      change_walking_kick_mode("walking", "");
      previous_motion_check = motion_check;
    }
  }

  if(move_command->key == "forward_precision" || move_command->key == "backward_precision" ||
      move_command->key == "left_precision" || move_command->key == "right_precision" ||
      move_command->key == "turn_left_precision" || move_command->key == "turn_right_precision"  ||
      move_command->key == "centered_left_precision" || move_command->key == "centered_right_precision")
  {
    //change_walking_kick_mode("walking", "");

    if(move_command->key == "forward_precision" || move_command->key == "backward_precision" )
    {
      command_controller->step_type = "default";
      command_controller->Set_FootParam(alice_id_int);
      foot_set_command_msg = command_controller->FootParam;

      double temp_value = atof(move_command->value.c_str());
      if(move_command->key == "forward_precision")
        CalculateStepData(fabs(temp_value), 0, "forward");
      else if(move_command->key == "backward_precision")
        CalculateStepData(fabs(temp_value), 0, "backward");
      else
      {
        CalculateStepData(0, 0, "stop");
      }
    }
    if(move_command->key == "left_precision" || move_command->key == "right_precision")
    {
      command_controller->step_type = "default";
      command_controller->Set_FootParam(alice_id_int);
      foot_set_command_msg = command_controller->FootParam;

      double temp_value = atof(move_command->value.c_str());
      if(move_command->key == "left_precision" )
        CalculateStepData(0, fabs(temp_value), "left");
      else if(move_command->key == "right_precision")
        CalculateStepData(0, fabs(temp_value), "right");
      else
      {
        CalculateStepData(0, 0, "stop");
      }
    }
    if(move_command->key == "turn_left_precision" || move_command->key == "turn_right_precision" )
    {
      command_controller->step_type = "turn";
      command_controller->Set_FootParam(alice_id_int);
      foot_set_command_msg = command_controller->FootParam;

      double temp_value = atof(move_command->value.c_str())*DEGREE2RADIAN;
      if(move_command->key == "turn_left_precision")
        AlignRobotYaw(fabs(temp_value), "turn left", walking_mode);
      else if(move_command->key == "turn_right_precision")
        AlignRobotYaw(fabs(temp_value), "turn right", walking_mode);
      else
      {
        CalculateStepData(0, 0, "stop");
      }
    }
    if(move_command->key == "centered_left_precision" || move_command->key == "centered_right_precision" )
    {
      command_controller->step_type = "centered";
      command_controller->Set_FootParam(alice_id_int);
      foot_set_command_msg = command_controller->FootParam;

      double temp_value = atof(move_command->value.c_str())*DEGREE2RADIAN;
      if(move_command->key == "centered_left_precision")
      {
        AlignRobotYaw(fabs(temp_value), "centered left", walking_mode);
      }
      else if(move_command->key == "centered_right_precision" )
      {
        AlignRobotYaw(fabs(temp_value), "centered right", walking_mode);
      }
      else
      {
        CalculateStepData(0, 0, "stop");
      }
    }

    command_interval_check = 0;
    //return;
  }
  else
  {
    //change_walking_kick_mode("walking", "");

    if(move_command->key == "left")
    {
      command_controller->FootParam.command = "left";
      command_controller->step_type = "default";
    }
    else if(move_command->key == "right")
    {
      command_controller->FootParam.command = "right";
      command_controller->step_type = "default";
    }
    else if(move_command->key == "forward")
    {
      command_controller->FootParam.command = "forward";
      command_controller->step_type = "default";
    }
    else if(move_command->key == "backward")
    {
      command_controller->FootParam.command = "backward";
      command_controller->step_type = "default";
    }
    else if(move_command->key == "turn_left" && alice_id_int == 2)
    {
      command_controller->FootParam.command = "turn left";
      command_controller->step_type = "default";
    }
    else if(move_command->key == "turn_right" && alice_id_int == 2)
    {
      command_controller->FootParam.command = "turn right";
      command_controller->step_type = "default";
    }
    else if(move_command->key == "expanded_left")
    {
      command_controller->FootParam.command = "expanded left";
      command_controller->step_type = "expanded";
    }
    else if(move_command->key == "expanded_right")
    {
      command_controller->FootParam.command = "expanded right";
      command_controller->step_type = "expanded";
    }
    else if(move_command->key == "centered_left")
    {
      command_controller->FootParam.command = "centered left";
      command_controller->step_type = "centered";
    }
    else if(move_command->key == "centered_right")
    {
      command_controller->FootParam.command = "centered right";
      command_controller->step_type = "centered";
    }
    else if(move_command->key == "turn_right" && alice_id_int == 1)
    {
      command_controller->FootParam.command = "turn right";
      command_controller->step_type = "turn";
    }
    else if(move_command->key == "turn_left" && alice_id_int == 1)
    {
      command_controller->FootParam.command = "turn left";
      command_controller->step_type = "turn";
    }
    else if(move_command->key == "stop")
    {
      command_controller->FootParam.command = "stop";
      command_controller->step_type = "default";
    }

    if(move_command->value == "1")
    {
      command_controller->speed_switch = "1";
    }
    else if(move_command->value == "3")
    {
      command_controller->speed_switch = "3";
    }
    else if(move_command->value == "2")
    {
      command_controller->speed_switch = "2";
    }

    command_controller->Set_FootParam(alice_id_int);
    foot_set_command_msg = command_controller->FootParam;

    previous_command = move_command->key;

    if ((move_command->key == "left_kick" || move_command->key == "right_kick" || move_command->key == "y_type_left_kick" || move_command->key == " y_type_right_kick")&& walking_check == true)
    {
      command_interval_check = 0;
      return;
    }
    if (move_command->key == "left_kick" && walking_check == false) //kick
    {
      //walking_check = true;
      change_walking_kick_mode("kick", "left kick");
      foot_set_command_msg.command = "left kick";
      foot_step_command_pub.publish(foot_set_command_msg);
      motion_check = true; // motion start
      previous_command = move_command->key;
      ///////////////////////////////////////
      previous_motion_check = motion_check;
      command_interval_check = 0;
      ///////////////////////////////////////
      return;
    }
    if(move_command->key == "right_kick" && walking_check == false)
    {
      //walking_check = true;
      change_walking_kick_mode("kick", "right kick");
      foot_set_command_msg.command = "right kick";
      foot_step_command_pub.publish(foot_set_command_msg);
      motion_check = true; // motion start
      previous_command = move_command->key;
      ///////////////////////////////////////
      previous_motion_check = motion_check;
      command_interval_check = 0;
      ///////////////////////////////////////
      return;
    }
    if (move_command->key == "y_type_left_kick" && walking_check == false) //kick
    {
      //walking_check = true;
      change_walking_kick_mode("kick", "left kick");
      foot_set_command_msg.command = "y type left kick";
      foot_step_command_pub.publish(foot_set_command_msg);
      motion_check = true; // motion start
      previous_command = move_command->key;
      ///////////////////////////////////////
      previous_motion_check = motion_check;
      command_interval_check = 0;
      ///////////////////////////////////////
      return;
    }
    if(move_command->key == "y_type_right_kick" && walking_check == false)
    {
      //walking_check = true;
      change_walking_kick_mode("kick", "right kick");
      foot_set_command_msg.command = "y type right kick";
      foot_step_command_pub.publish(foot_set_command_msg);
      motion_check = true; // motion start
      previous_command = move_command->key;
      ///////////////////////////////////////
      previous_motion_check = motion_check;
      command_interval_check = 0;
      ///////////////////////////////////////
      return;
    }

    foot_step_command_pub.publish(foot_set_command_msg);
    command_interval_check = 0;
    return;
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
  side_step_length_max = doc["side_step_length_max"].as<double>();
  side_step_length_min = doc["side_step_length_min"].as<double>();
  step_rad_max   = doc["step_rad_max"].as<double>();
  step_rad_min   = doc["step_rad_min"].as<double>();
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

  set_balance_param_msg.request.updating_duration                                 = doc["updating_duration"].as<double>();
  set_balance_param_msg.request.balance_param.cob_x_offset_m                      = doc["cob_x_offset_m"].as<double>();
  set_balance_param_msg.request.balance_param.cob_y_offset_m                      = doc["cob_y_offset_m"].as<double>();
  set_balance_param_msg.request.balance_param.foot_roll_gyro_p_gain               = doc["foot_roll_gyro_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_roll_gyro_d_gain               = doc["foot_roll_gyro_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_pitch_gyro_p_gain              = doc["foot_pitch_gyro_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_pitch_gyro_d_gain              = doc["foot_pitch_gyro_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_roll_angle_p_gain              = doc["foot_roll_angle_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_roll_angle_d_gain              = doc["foot_roll_angle_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_pitch_angle_p_gain             = doc["foot_pitch_angle_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_pitch_angle_d_gain             = doc["foot_pitch_angle_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_x_force_p_gain                 = doc["foot_x_force_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_x_force_d_gain                 = doc["foot_x_force_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_y_force_p_gain                 = doc["foot_y_force_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_y_force_d_gain                 = doc["foot_y_force_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_z_force_p_gain                 = doc["foot_z_force_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_z_force_d_gain                 = doc["foot_z_force_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_roll_torque_p_gain             = doc["foot_roll_torque_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_roll_torque_d_gain             = doc["foot_roll_torque_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_pitch_torque_p_gain            = doc["foot_pitch_torque_p_gain"].as<double>();
  set_balance_param_msg.request.balance_param.foot_pitch_torque_d_gain            = doc["foot_pitch_torque_d_gain"].as<double>();
  set_balance_param_msg.request.balance_param.roll_gyro_cut_off_frequency         = doc["roll_gyro_cut_off_frequency"].as<double>();
  set_balance_param_msg.request.balance_param.pitch_gyro_cut_off_frequency        = doc["pitch_gyro_cut_off_frequency"].as<double>();
  set_balance_param_msg.request.balance_param.roll_angle_cut_off_frequency        = doc["roll_angle_cut_off_frequency"].as<double>();
  set_balance_param_msg.request.balance_param.pitch_angle_cut_off_frequency       = doc["pitch_angle_cut_off_frequency"].as<double>();
  set_balance_param_msg.request.balance_param.foot_x_force_cut_off_frequency      = doc["foot_x_force_cut_off_frequency"].as<double>();
  set_balance_param_msg.request.balance_param.foot_y_force_cut_off_frequency      = doc["foot_y_force_cut_off_frequency"].as<double>();
  set_balance_param_msg.request.balance_param.foot_z_force_cut_off_frequency      = doc["foot_z_force_cut_off_frequency"].as<double>();
  set_balance_param_msg.request.balance_param.foot_roll_torque_cut_off_frequency  = doc["foot_roll_torque_cut_off_frequency"].as<double>();
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
  joint_feedback_gain_msg.request.updating_duration                 = doc["updating_duration"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain  = doc["r_leg_hip_y_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain  = doc["r_leg_hip_y_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain  = doc["r_leg_hip_r_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain  = doc["r_leg_hip_r_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain  = doc["r_leg_hip_p_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain  = doc["r_leg_hip_p_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain   = doc["r_leg_an_p_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain   = doc["r_leg_an_p_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_p_gain   = doc["r_leg_kn_p_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_d_gain   = doc["r_leg_kn_p_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain   = doc["r_leg_an_r_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain   = doc["r_leg_an_r_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain  = doc["l_leg_hip_y_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain  = doc["l_leg_hip_y_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain  = doc["l_leg_hip_r_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain  = doc["l_leg_hip_r_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain  = doc["l_leg_hip_p_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain  = doc["l_leg_hip_p_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_p_gain   = doc["l_leg_kn_p_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_d_gain   = doc["l_leg_kn_p_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain   = doc["l_leg_an_p_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain   = doc["l_leg_an_p_d_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain   = doc["l_leg_an_r_p_gain"].as<double>();
  joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain   = doc["l_leg_an_r_d_gain"].as<double>();


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
    ROS_INFO("KICK COB :: %f", kick_y_cob_);
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

  //parse_online_balance_param(balance_param_file);

  printf("Balance Param SAVE!!\n");

  return true;
}
/*void FootStepPlanner::stepDataApplyMsgCallback(const alice_foot_step_generator::FootStepCommand::ConstPtr& msg)
{
  foot_set_command_msg.step_num = msg->step_num;
  foot_set_command_msg.step_length = msg->step_length;
  foot_set_command_msg.side_step_length = msg->side_step_length;
  foot_set_command_msg.step_angle_rad = msg->step_angle_rad;
  foot_set_command_msg.step_time = msg->step_time;
  foot_set_command_msg.command = msg->command;

  if(foot_set_command_msg.command != "default stop" && foot_set_command_msg.command != "expanded stop" && foot_set_command_msg.command != "centered stop") return;
  YAML::Emitter out;
  std::string path_;
  if(alice_id_int == 1) path_ = ros::package::getPath("alice_foot_step_planner") + "/config/step_parameter1.yaml";
  if(alice_id_int == 2) path_ = ros::package::getPath("alice_foot_step_planner") + "/config/step_parameter2.yaml";

  if(foot_set_command_msg.command == "default stop")
  {
     out << YAML::BeginMap;
     out << YAML::Key << "default_step_num" << YAML::Value << foot_set_command_msg.step_num;
     out << YAML::Key << "default_step_length" << YAML::Value << foot_set_command_msg.step_length;
     out << YAML::Key << "default_side_step_length" << YAML::Value << foot_set_command_msg.side_step_length;
     out << YAML::Key << "default_step_angle_radian" << YAML::Value << foot_set_command_msg.step_angle_rad;
     out << YAML::Key << "default_step_time" << YAML::Value << foot_set_command_msg.step_time;

     out << YAML::Key << "expanded_step_num" << YAML::Value << command_controller->expanded_step_num;
     out << YAML::Key << "expanded_step_length" << YAML::Value << command_controller->expanded_step_length;
     out << YAML::Key << "expanded_side_step_length" << YAML::Value << command_controller->expanded_side_step_length;
     out << YAML::Key << "expanded_step_angle_radian" << YAML::Value << command_controller->expanded_step_angle_rad;
     out << YAML::Key << "expanded_step_time" << YAML::Value << command_controller->expanded_step_time;

     out << YAML::Key << "centered_step_num" << YAML::Value << command_controller->centered_step_num;
     out << YAML::Key << "centered_step_length" << YAML::Value << command_controller->centered_step_length;
     out << YAML::Key << "centered_side_step_length" << YAML::Value << command_controller->centered_side_step_length;
     out << YAML::Key << "centered_step_angle_radian" << YAML::Value << command_controller->centered_step_angle_rad;
     out << YAML::Key << "centered_step_time" << YAML::Value << command_controller->centered_step_time;

  }
  else if(foot_set_command_msg.command == "expanded stop")
  {
     out << YAML::BeginMap;
     out << YAML::Key << "default_step_num" << YAML::Value << command_controller->default_step_num;
     out << YAML::Key << "default_step_length" << YAML::Value << command_controller->default_step_length;
     out << YAML::Key << "default_side_step_length" << YAML::Value << command_controller->default_side_step_length;
     out << YAML::Key << "default_step_angle_radian" << YAML::Value << command_controller->default_step_angle_rad;
     out << YAML::Key << "default_step_time" << YAML::Value << command_controller->default_step_time;

     out << YAML::Key << "expanded_step_num" << YAML::Value << foot_set_command_msg.step_num;
     out << YAML::Key << "expanded_step_length" << YAML::Value << foot_set_command_msg.step_length;
     out << YAML::Key << "expanded_side_step_length" << YAML::Value << foot_set_command_msg.side_step_length;
     out << YAML::Key << "expanded_step_angle_radian" << YAML::Value << foot_set_command_msg.step_angle_rad;
     out << YAML::Key << "expanded_step_time" << YAML::Value << foot_set_command_msg.step_time;

     out << YAML::Key << "centered_step_num" << YAML::Value << command_controller->centered_step_num;
     out << YAML::Key << "centered_step_length" << YAML::Value << command_controller->centered_step_length;
     out << YAML::Key << "centered_side_step_length" << YAML::Value << command_controller->centered_side_step_length;
     out << YAML::Key << "centered_step_angle_radian" << YAML::Value << command_controller->centered_step_angle_rad;
     out << YAML::Key << "centered_step_time" << YAML::Value << command_controller->centered_step_time;
  }
  else if(foot_set_command_msg.command == "centered stop")
  {
     out << YAML::BeginMap;
     out << YAML::Key << "default_step_num" << YAML::Value << command_controller->default_step_num;
     out << YAML::Key << "default_step_length" << YAML::Value << command_controller->default_step_length;
     out << YAML::Key << "default_side_step_length" << YAML::Value << command_controller->default_side_step_length;
     out << YAML::Key << "default_step_angle_radian" << YAML::Value << command_controller->default_step_angle_rad;
     out << YAML::Key << "default_step_time" << YAML::Value << command_controller->default_step_time;

     out << YAML::Key << "expanded_step_num" << YAML::Value << command_controller->expanded_step_num;
     out << YAML::Key << "expanded_step_length" << YAML::Value << command_controller->expanded_step_length;
     out << YAML::Key << "expanded_side_step_length" << YAML::Value << command_controller->expanded_side_step_length;
     out << YAML::Key << "expanded_step_angle_radian" << YAML::Value << command_controller->expanded_step_angle_rad;
     out << YAML::Key << "expanded_step_time" << YAML::Value << command_controller->expanded_step_time;

     out << YAML::Key << "centered_step_num" << YAML::Value << foot_set_command_msg.step_num;
     out << YAML::Key << "centered_step_length" << YAML::Value << foot_set_command_msg.step_length;
     out << YAML::Key << "centered_side_step_length" << YAML::Value << foot_set_command_msg.side_step_length;
     out << YAML::Key << "centered_step_angle_radian" << YAML::Value << foot_set_command_msg.step_angle_rad;
     out << YAML::Key << "centered_step_time" << YAML::Value << foot_set_command_msg.step_time;
  }

  out << YAML::EndMap;
  std::ofstream fout(path_.c_str());
  fout << out.c_str(); // dump it back into the file

}*/

/*void FootStepPlanner::commandGeneratorMsgCallback(const alice_foot_step_generator::FootStepCommandConstPtr& msg)
{
  foot_set_command_msg.step_num = msg->step_num;
  foot_set_command_msg.step_length = msg->step_length;
  foot_set_command_msg.side_step_length = msg->side_step_length;
  foot_set_command_msg.step_angle_rad = msg->step_angle_rad;
  foot_set_command_msg.step_time = msg->step_time;
  foot_set_command_msg.command = msg->command;
  foot_step_command_pub.publish(foot_set_command_msg);
}*/

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
  command_controller->parse_step_param_data(step_path_);
}
/*void FootStepPlanner::current_status_Callback(const std_msgs::String::ConstPtr& log_moving_status)
{
  command_controller.current_status = log_moving_status->data;
}*/
/////////////////////////
/////////////////////////
/////////////////////////
/////////////////////////
Command_generator::Command_generator()
{
  //Default_Setting//
  //Input_Text();
  //Make_Log();
  command_switch = 0;
  speed_switch = 2;
  FootParam.step_num = 0;
  FootParam.step_length = 0;
  FootParam.side_step_length = 0;
  FootParam.step_angle_rad = 0;
  FootParam.step_time = 0;
  default_step_num = 0;
  default_step_length = 0;
  default_side_step_length = 0;
  default_step_angle_rad = 0;
  default_step_time = 0;
  expanded_step_num = 0;
  expanded_step_length = 0;
  expanded_side_step_length = 0;
  expanded_step_angle_rad = 0;
  expanded_step_time = 0;
  centered_step_num = 0;
  centered_step_length = 0;
  centered_side_step_length = 0;
  centered_step_angle_rad = 0;
  centered_step_time = 0;
  turn_step_num = 0;
  turn_step_length = 0;
  turn_side_step_length = 0;
  turn_step_angle_rad = 0;
  turn_step_time = 0;
  // start_time = clock();
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
  else if(step_type == "turn")
  {
    if(alice_id == 1)
    {
      FootParam.step_num = turn_step_num;
      FootParam.step_length = turn_step_length;
      FootParam.side_step_length = turn_side_step_length;
      FootParam.step_angle_rad = turn_step_angle_rad;
      FootParam.step_time =  turn_step_time;
    }
    else
      return;
  }
  if(alice_id == 1)
  {
    if(speed_switch == "1")
    {
      FootParam.step_time = FootParam.step_time*2;
    }
    else if(speed_switch == "2")
    {
      FootParam.step_time = FootParam.step_time*1.5;
    }
    else if(speed_switch == "3")
    {
      FootParam.step_time = FootParam.step_time*1;
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

  turn_step_num         = doc["turn_step_num"].as<double>();
  turn_step_length      = doc["turn_step_length"].as<double>();
  turn_side_step_length = doc["turn_side_step_length"].as<double>();
  turn_step_angle_rad   = doc["turn_step_angle_radian"].as<double>();
  turn_step_time        = doc["turn_step_time"].as<double>();
}



