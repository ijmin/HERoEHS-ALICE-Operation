/*
 * message_callback.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#include "alice_foot_step_generator/message_callback.h"



ros::ServiceClient   g_get_ref_step_data_client;
ros::ServiceClient   g_add_step_data_array_client;

ros::ServiceClient   g_is_running_client;

ros::ServiceClient  g_set_balance_param_client;

ros::Subscriber     g_walking_module_status_msg_sub;

ros::Subscriber     g_walking_command_sub;
ros::Subscriber     g_balance_command_sub;
ros::Subscriber     g_footsteps_2d_sub;

ros::Subscriber     g_dsp_sub;
ros::Subscriber     g_foot_z_swap_sub;
ros::Subscriber     g_body_z_swap_sub;
ros::Subscriber     g_y_zmp_convergence_sub;

ros::Publisher log_command_status_pub;

alice::FootStepGenerator g_foot_stp_generator;

alice_walking_module_msgs::AddStepDataArray     add_step_data_array_srv;

alice_foot_step_generator::FootStepCommand last_command;
double g_last_command_time = 0;
alice_walking_module_msgs::StepData ref_stp_data_com[1];

int alice_id=0;

bool g_is_running_check_needed = false;

void initialize(void)
{
  ros::NodeHandle nh;

  alice_id  = nh.param<int>("alice_userid",0);

  g_get_ref_step_data_client      = nh.serviceClient<alice_walking_module_msgs::GetReferenceStepData>("/heroehs/online_walking/get_reference_step_data");
  g_add_step_data_array_client    = nh.serviceClient<alice_walking_module_msgs::AddStepDataArray>("/heroehs/online_walking/add_step_data");
  g_set_balance_param_client      = nh.serviceClient<alice_walking_module_msgs::SetBalanceParam>("/heroehs/online_walking/set_balance_param");
  g_is_running_client             = nh.serviceClient<alice_walking_module_msgs::IsRunning>("/heroehs/online_walking/is_running");

  g_walking_module_status_msg_sub = nh.subscribe("/robotis/status", 10, walkingModuleStatusMSGCallback);

  g_walking_command_sub           = nh.subscribe("/heroehs/alice_foot_step_generator/walking_command", 0, walkingCommandCallback);
  g_footsteps_2d_sub              = nh.subscribe("/heroehs/alice_foot_step_generator/footsteps_2d",    0, step2DArrayCallback);
  g_dsp_sub                 = nh.subscribe("/heroehs/alice_foot_step_generator/dsp", 5, dspCallback);
  g_foot_z_swap_sub         = nh.subscribe("/heroehs/alice_foot_step_generator/foot_z_swap", 5, footZSwapCallback);
  g_body_z_swap_sub         = nh.subscribe("/heroehs/alice_foot_step_generator/body_z_swap", 5, bodyZSwapCallback);
  g_y_zmp_convergence_sub   = nh.subscribe("/heroehs/alice_foot_step_generator/y_zmp_convergence", 5, yZMPConvergenceCallback);
  log_command_status_pub    = nh.advertise<std_msgs::String>("/heroehs/log_moving_status", 10);

  g_last_command_time = ros::Time::now().toSec();
}

void publish_status(void)
{
  std_msgs::String pub_status_msg;

  if(g_foot_stp_generator.previous_step_type_ == STOP_WALKING)
    pub_status_msg.data="stop";
  else if(g_foot_stp_generator.previous_step_type_ == FORWARD_WALKING)
    pub_status_msg.data="forward";
  else if(g_foot_stp_generator.previous_step_type_ == BACKWARD_WALKING)
    pub_status_msg.data="backward";
  else if(g_foot_stp_generator.previous_step_type_ == RIGHTWARD_WALKING)
    pub_status_msg.data="right";
  else if(g_foot_stp_generator.previous_step_type_ == LEFTWARD_WALKING)
    pub_status_msg.data="left";
  else if(g_foot_stp_generator.previous_step_type_ == LEFT_ROTATING_WALKING)
    pub_status_msg.data="turn left";
  else if(g_foot_stp_generator.previous_step_type_ == RIGHT_ROTATING_WALKING)
    pub_status_msg.data="turn right";
  else if(g_foot_stp_generator.previous_step_type_ == REVOLUTE_LEFT_WALKING && g_foot_stp_generator.revolute_type_ == centered)
    pub_status_msg.data="centered left";
  else if(g_foot_stp_generator.previous_step_type_ == REVOLUTE_RIGHT_WALKING && g_foot_stp_generator.revolute_type_ == centered)
    pub_status_msg.data="centered right";
  else if(g_foot_stp_generator.previous_step_type_ == REVOLUTE_LEFT_WALKING && g_foot_stp_generator.revolute_type_ == expanded)
    pub_status_msg.data="expanded left";
  else if(g_foot_stp_generator.previous_step_type_ == REVOLUTE_RIGHT_WALKING && g_foot_stp_generator.revolute_type_ == expanded)
    pub_status_msg.data="expanded right";
  else
    pub_status_msg.data="invalid";

  log_command_status_pub.publish(pub_status_msg);

}
void dspCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if(msg->data <= 0 || msg->data >= 1)
  {
    ROS_ERROR("Invalid DSP Ratio");
    return;
  }

  ROS_INFO_STREAM("SET DSP RATIO : " << msg->data);
  g_foot_stp_generator.dsp_ratio_ = msg->data;
}

void footZSwapCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if(msg->data <= 0)
  {
    ROS_ERROR("Invalid Foot Z Swap");
    return;
  }

  ROS_INFO_STREAM("SET Foot Z Swap : " << msg->data);
  g_foot_stp_generator.foot_z_swap_m_ = msg->data;
}

void bodyZSwapCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if(msg->data < 0)
  {
    ROS_ERROR("Invalid Body Z Swap");
    return;
  }

  ROS_INFO_STREAM("SET Body Z Swap : " << msg->data);
  g_foot_stp_generator.body_z_swap_m_ = msg->data;
}

void yZMPConvergenceCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO_STREAM("SET Y ZMP Convergence : " << msg->data);
  g_foot_stp_generator.y_zmp_convergence_m_ = msg->data;
}

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
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

void walkingCommandCallback(const alice_foot_step_generator::FootStepCommand::ConstPtr &msg)
{


  double now_time = ros::Time::now().toSec();

  //ref_stp_data_com[1];
  //ROS_INFO("WALKING STATE MESG: %d",stp_data[0].time_data.walking_state);

  if((last_command.command == msg->command)
      && (last_command.step_num == msg->step_num)
      && (last_command.step_time == msg->step_time)
      && (last_command.step_length == msg->step_length)
      && (last_command.side_step_length == msg->side_step_length)
      && (last_command.step_angle_rad == msg->step_angle_rad))
  {
    //prevent double click & switching foot step time
    if( (last_command.step_num == 1) && isRunning() )
    {
       ROS_ERROR("STEP NUM 1 --->>> STOP");
       return;
    }

    if( (fabs(now_time - g_last_command_time) < 2*last_command.step_time) )
    {
      ROS_ERROR("Receive same command in short time & Foot Step Switching time");
      return;
    }


  }

  if( (last_command.step_num == 1) && isRunning() )
  {
     ROS_ERROR("STEP NUM 1 --->>> STOP");
     return;
  }

  if(last_command.command == "stop" && (msg->command == "stop"))
    return;

  g_last_command_time = now_time;

  last_command.command          = msg->command;
  last_command.step_num         = msg->step_num;
  last_command.step_time        = msg->step_time;
  last_command.step_length      = msg->step_length;
  last_command.side_step_length = msg->side_step_length;
  last_command.step_angle_rad   = msg->step_angle_rad;

  ROS_INFO("[Demo]  : Walking Command");
  ROS_INFO_STREAM("  command          : " << msg->command );
  ROS_INFO_STREAM("  step_num         : " << msg->step_num );
  ROS_INFO_STREAM("  step_time        : " << msg->step_time );
  ROS_INFO_STREAM("  step_length      : " << msg->step_length);
  ROS_INFO_STREAM("  side_step_length : " << msg->side_step_length );
  ROS_INFO_STREAM("  step_angle_rad   : " << msg->step_angle_rad );



  if((msg->step_num == 0)
      && (msg->command != "left kick")
      && (msg->command != "right kick")
      && (msg->command != "y type left kick")
      && (msg->command != "y type right kick")
      && (msg->command != "expanded stop")
      && (msg->command != "centered stop")
      && (msg->command != "y walking type")
      && (msg->command != "default walking type")
      && (msg->command != "stop")
      && (msg->command != "default stop"))
    return;




  //set walking parameter
  if(msg->step_length < 0)
  {
    if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
      //g_foot_stp_generator.ep_step_length_m_= 0;
      g_foot_stp_generator.ep_step_length_m_= msg->step_length;
    else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
      g_foot_stp_generator.ct_step_length_m_=  msg->step_length;
    else if(alice_id==1 && ( msg->command == "turn right" || msg->command == "turn left") )
    {
      g_foot_stp_generator.ep_step_length_m_= msg->step_length;
    }
    else
      g_foot_stp_generator.fb_step_length_m_ = 0;
    ROS_ERROR_STREAM("step_length is negative.");
    ROS_ERROR_STREAM("It will be set to zero.");
  }
  else
  {
    if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
      g_foot_stp_generator.ep_step_length_m_= msg->step_length;
    else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
      g_foot_stp_generator.ct_step_length_m_= msg->step_length;
    else if(alice_id==1 && ( msg->command == "turn right" || msg->command == "turn left") )
    {
      g_foot_stp_generator.ep_step_length_m_= msg->step_length;
    }
    else
      g_foot_stp_generator.fb_step_length_m_ = msg->step_length;
  }

  if(msg->side_step_length < 0)
  {
    if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
      g_foot_stp_generator.eps_step_length_m_= msg->side_step_length;
    else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
      g_foot_stp_generator.cts_step_length_m_= msg->side_step_length;
    else if(alice_id==1 && ( msg->command == "turn right" || msg->command == "turn left") )
    {
      g_foot_stp_generator.eps_step_length_m_= msg->side_step_length;
    }
    else
      g_foot_stp_generator.rl_step_length_m_ = 0;
    ROS_ERROR_STREAM("side_step_length is negative.");
    ROS_ERROR_STREAM("It will be set to zero.");
  }
  else
  {
    if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
      g_foot_stp_generator.eps_step_length_m_= msg->side_step_length;
    else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
      g_foot_stp_generator.cts_step_length_m_= msg->side_step_length;
    else if(alice_id==1 && ( msg->command == "turn right" || msg->command == "turn left") )
    {
      g_foot_stp_generator.eps_step_length_m_= msg->side_step_length;
    }
    else
      g_foot_stp_generator.rl_step_length_m_ = msg->side_step_length;

  }

  if(msg->step_angle_rad < 0)
  {
    if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
      g_foot_stp_generator.ep_step_angle_rad_= 0;
    else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
      g_foot_stp_generator.ct_step_angle_rad_= 0;
    else if(alice_id==1 && ( msg->command == "turn right" || msg->command == "turn left") )
    {
      g_foot_stp_generator.ep_step_angle_rad_= 0;
    }
    else
      g_foot_stp_generator.rotate_step_angle_rad_ = 0;
    ROS_ERROR_STREAM("step_angle_rad is negative.");
    ROS_ERROR_STREAM("It will be set to zero.");
  }
  else
  {
    if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
      g_foot_stp_generator.ep_step_angle_rad_= msg->step_angle_rad;
    else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
      g_foot_stp_generator.ct_step_angle_rad_= msg->step_angle_rad;
    else if(alice_id==1 && ( msg->command == "turn right" || msg->command == "turn left") )
    {
      g_foot_stp_generator.ep_step_angle_rad_= msg->step_angle_rad;
    }
    else
      g_foot_stp_generator.rotate_step_angle_rad_ = msg->step_angle_rad;

  }

  if(msg->step_time < MINIMUM_STEP_TIME_SEC)
  {
    if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
      g_foot_stp_generator.ep_step_time_sec_ = 0;
    else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
      g_foot_stp_generator.ct_step_time_sec_ = 0;
    else
      g_foot_stp_generator.step_time_sec_ = 0;
    ROS_ERROR_STREAM("step_time is less than minimum step time. ");
    ROS_ERROR_STREAM("It will be set to minimum step time(0.4 sec).");
  }
  else
  {
    //if(msg->command =="expanded stop"||msg->command =="expanded right"||msg->command =="expanded left")
    g_foot_stp_generator.ep_step_time_sec_ = msg->step_time;
    //else if(msg->command =="centered stop"||msg->command =="centered right"||msg->command =="centered left")
    g_foot_stp_generator.ct_step_time_sec_ = msg->step_time;
    //else
    g_foot_stp_generator.step_time_sec_ = msg->step_time;
  }

  g_foot_stp_generator.num_of_step_ = 2*(msg->step_num) + 2;


  alice_walking_module_msgs::GetReferenceStepData    get_ref_stp_data_srv;
  alice_walking_module_msgs::StepData                ref_step_data;
  alice_walking_module_msgs::AddStepDataArray        add_stp_data_srv;


  //get reference step data
  if(g_get_ref_step_data_client.call(get_ref_stp_data_srv) == false)
  {
    ROS_ERROR("Failed to get reference step data");
    return;
  }

  ref_step_data = get_ref_stp_data_srv.response.reference_step_data;
  ref_stp_data_com[0] = ref_step_data;

  if(isRunning() && (ref_stp_data_com[0].time_data.walking_state==alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING))
  {
    ROS_ERROR("RUNNING STOP STEP ++++++++++!");
    return;
  }



  //calc step data

  if(msg->command == "y walking type")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcYType( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }
  else if(msg->command == "default walking type")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcDefaultType( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }

  else if(msg->command == "forward")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, FORWARD_WALKING, 0);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "backward")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, BACKWARD_WALKING, 0);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "turn left")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    if(alice_id==2)
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, LEFT_ROTATING_WALKING, 0);
    else if(alice_id==1)
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, REVOLUTE_LEFT_WALKING, ip_from_launch);
    g_is_running_check_needed = false;

  }
  else if(msg->command == "turn right")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    if(alice_id==2)
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, RIGHT_ROTATING_WALKING, 0);
    else if(alice_id==1)
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, REVOLUTE_RIGHT_WALKING, ip_from_launch);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "right")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, RIGHTWARD_WALKING, 0);
    g_is_running_check_needed = false;

  }
  else if(msg->command == "left")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, LEFTWARD_WALKING, 0);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "right kick")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcRightKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }
  else if(msg->command == "left kick")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcLeftKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }
  else if(msg->command == "y type right kick")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcYRightKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }
  else if(msg->command == "y type left kick")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcYLeftKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }
  else if(msg->command == "expanded left" || msg->command == "centered left")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;
    if(msg->command == "centered left")
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, REVOLUTE_LEFT_WALKING, centered);
    else if(msg->command == "expanded left")
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, REVOLUTE_LEFT_WALKING, expanded);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "expanded right" || msg->command == "centered right")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;
    if(msg->command == "centered right")
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, REVOLUTE_RIGHT_WALKING, centered);
    else if(msg->command == "expanded right")
      g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, REVOLUTE_RIGHT_WALKING, expanded);
    g_is_running_check_needed = false;
  }

  else if(msg->command == "stop" || msg->command == "expanded stop" || msg->command == "centered stop" || msg->command == "default stop")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, STOP_WALKING, 0);
    g_is_running_check_needed = true;
  }
  else
  {
    ROS_ERROR("[Demo]  : Invalid Command");
    return;
  }

  //set add step data srv for auto start
  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  //add step data

  if(g_add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int add_stp_data_srv_result = add_stp_data_srv.response.result;
    if(add_stp_data_srv_result== alice_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
    {
      ROS_INFO("[Demo]  : Succeed to add step data array");
    }
    else
    {
      ROS_ERROR("[Demo]  : Failed to add step data array");

      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::TOO_MANY_STEP_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::TOO_MANY_STEP_DATA");
      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

      g_foot_stp_generator.initialize();

      return;
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
    g_foot_stp_generator.initialize();
    return;
  }

}

void step2DArrayCallback(const alice_foot_step_generator::Step2DArray::ConstPtr& msg)
{
  alice_walking_module_msgs::GetReferenceStepData get_ref_stp_data_srv;
  alice_walking_module_msgs::StepData             ref_step_data;
  alice_walking_module_msgs::AddStepDataArray     add_stp_data_srv;
  alice_walking_module_msgs::IsRunning            is_running_srv;

  if(isRunning() == true)
    return;


  //get reference step data
  if(g_get_ref_step_data_client.call(get_ref_stp_data_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to get reference step data");
    return;
  }

  ref_step_data = get_ref_stp_data_srv.response.reference_step_data;

  g_foot_stp_generator.getStepDataFromStepData2DArray(&add_stp_data_srv.request.step_data_array, ref_step_data, msg);
  g_is_running_check_needed = true;

  //set add step data srv fot auto start and remove existing step data
  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  //add step data
  if(g_add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int add_stp_data_srv_result = add_stp_data_srv.response.result;
    if(add_stp_data_srv_result== alice_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to add step data array");
    else {
      ROS_ERROR("[Demo]  : Failed to add step data array");

      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(add_stp_data_srv_result & alice_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

      return;
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
    return;
  }
}

bool isRunning(void)
{
  alice_walking_module_msgs::IsRunning is_running_srv;
  if(g_is_running_client.call(is_running_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to Walking Status");
    return true;
  }
  else
  {
    if(is_running_srv.response.is_running == true)
    {
      ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
      return true;
    }
  }
  return false;
}
