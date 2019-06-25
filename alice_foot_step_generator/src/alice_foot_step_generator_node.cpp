/*
 * alice_foot_step_generator_node.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#include <cmath>
#include <stdlib.h>

#include "alice_foot_step_generator/alice_foot_step_generator_node.h"


using namespace alice;

#define RAD2DEG  (M_PI/180.0)

double sign(double n)
{
  if(n < 0)
    return -1;
  else if(n > 0)
    return 1;
  else
    return 0;
}

FootStepGenerator::FootStepGenerator()
{
  readFootStep_Yaml();

  kick_dsp_ratio_  =kick_dsp_ratio;
  kick_height_m_  =kick_height_m;
  kick_back_time_  =kick_back_time;
  kick_back_m_  =kick_back_m;
  kick_time_  =kick_time;
  kick_far_m_  =kick_far_m;
  kick_pitch_rad_  =kick_pitch_rad;
  kick_time_sec_  =kick_time_sec;


  num_of_step_             = num_of_step;

  fb_step_length_m_        = default_step_set[0];
  rl_step_length_m_        = default_step_set[1];
  rotate_step_angle_rad_   = default_step_set[2]*RAD2DEG;
  step_time_sec_           = default_step_set[3];

  ep_step_length_m_        = ep_step_set[0];
  eps_step_length_m_       = ep_step_set[1];
  ep_step_angle_rad_       = ep_step_set[2]*RAD2DEG;
  ep_step_time_sec_        = ep_step_set[3];

  ct_step_length_m_        = ct_step_set[0];
  cts_step_length_m_       = ct_step_set[1];
  ct_step_angle_rad_       = ct_step_set[2]*RAD2DEG;
  ct_step_time_sec_        = ct_step_set[3];

  dsp_ratio_ = dsp_ratio_set;
  foot_z_swap_m_ = foot_z_swap_set;
  body_z_swap_m_ = body_z_swap_set;

  y_zmp_convergence_m_ = y_zmp_conv_set;

  start_end_time_sec_ = startend_time_set;
  default_y_feet_offset_m_ = leg_offset_ ;
  default_yaw_feet_offset_m_ = 0;

  type_offset_y_ = 0;
  type_offset_yaw_ = 0;

  revolute_type_=0;
  previous_step_type_ = STOP_WALKING;


}


FootStepGenerator::~FootStepGenerator()
{    }

void FootStepGenerator::readFootStep_Yaml()
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
    //if(alice_id == "" )
    //{
    /// ROS_INFO("FROM 1 yaml");
    //  alice_id="_1";
    // }

    if(alice_id == "alice1nuke")
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


  kick_dsp_ratio = kick_doc["kick_dsp_ratio"].as<double>();
  kick_height_m = kick_doc["kick_height_m"].as<double>();
  kick_back_time = kick_doc["kick_back_time"].as<double>();
  kick_back_m = kick_doc["kick_back_m"].as<double>();
  kick_time = kick_doc["kick_time"].as<double>();
  kick_far_m   = kick_doc["kick_far_m"].as<double>();
  kick_pitch_rad = kick_doc["kick_pitch"].as<double>()*RAD2DEG;
  kick_time_sec = kick_doc["kick_time_sec"].as<double>();

  y_kick_front_time= kick_doc["y_kick_front_time"].as<double>();
  y_kick_front_x_m= kick_doc["y_kick_front_x_m"].as<double>();
  y_kick_front_y_m= kick_doc["y_kick_front_y_m"].as<double>();
  y_kick_time= kick_doc["y_kick_time"].as<double>();

  num_of_step = kick_doc["num_of_step"].as<int>();
  default_step_set = kick_doc["default_set"].as<std::vector<double> >();
  ep_step_set = kick_doc["ep_set"].as<std::vector<double> >();
  ct_step_set = kick_doc["ct_set"].as<std::vector<double> >();

  dsp_ratio_set = kick_doc["dsp_ratio"].as<double>();
  foot_z_swap_set= kick_doc["foot_z_swap_m"].as<double>();
  body_z_swap_set= kick_doc["body_z_swap_m"].as<double>();
  y_zmp_conv_set= kick_doc["y_zmp_convergence_m"].as<double>();
  startend_time_set= kick_doc["start_end_time_sec"].as<double>();

  alice_id_int  = kick_doc["id"].as<double>();

  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  std::string alice_id_kin = alice_id_stream.str();
  ROS_INFO("ID FROM FOOTSTEP GEN :  %d",alice_id_int);

  std::string kinematics_path = ros::package::getPath("alice_kinematics_dynamics")+"/data/kin_dyn_"+alice_id_kin+".yaml";
  YAML::Node kinematics_doc;
  try
  {
    kinematics_doc = YAML::LoadFile(kinematics_path.c_str());
  }catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load kinematics yaml file!");
    return;
  }
  leg_offset_ = kinematics_doc["leg_side_offset_m"].as<double>();
  y_steptype_offset_y_= kinematics_doc["y_steptype_y_offeset"].as<double>();
  y_steptype_offset_yaw_= kinematics_doc["y_steptype_yaw_offeset"].as<double>()*RAD2DEG;
}

void FootStepGenerator::initialize()
{
  previous_step_type_ = STOP_WALKING;
  //step_data_array_.clear();
}

Eigen::MatrixXd FootStepGenerator::getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw)
{
  double sr = sin(roll), cr = cos(roll);
  double sp = sin(pitch), cp = cos(pitch);
  double sy = sin(yaw), cy = cos(yaw);

  Eigen::MatrixXd mat_roll(4,4);
  Eigen::MatrixXd mat_pitch(4,4);
  Eigen::MatrixXd mat_yaw(4,4);

  mat_roll <<
      1, 0, 0, 0,
      0, cr, -sr, 0,
      0, sr, cr, 0,
      0, 0, 0, 1;

  mat_pitch <<
      cp, 0, sp, 0,
      0, 1, 0, 0,
      -sp, 0, cp, 0,
      0, 0, 0, 1;

  mat_yaw <<
      cy, -sy, 0, 0,
      sy, cy, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Eigen::MatrixXd mat_xyzrpy = (mat_yaw*mat_pitch)*mat_roll;

  mat_xyzrpy.coeffRef(0, 3) = position_x;
  mat_xyzrpy.coeffRef(1, 3) = position_y;
  mat_xyzrpy.coeffRef(2, 3) = position_z;


  return mat_xyzrpy;
}

void FootStepGenerator::getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform, double *position_x, double *position_y, double *position_z, double *roll, double *pitch, double *yaw)
{
  *position_x = matTransform.coeff(0, 3);
  *position_y = matTransform.coeff(1, 3);
  *position_z = matTransform.coeff(2, 3);
  *roll       = atan2( matTransform.coeff(2,1), matTransform.coeff(2,2));
  *pitch      = atan2(-matTransform.coeff(2,0), sqrt(matTransform.coeff(2,1)*matTransform.coeff(2,1) + matTransform.coeff(2,2)*matTransform.coeff(2,2)) );
  *yaw        = atan2( matTransform.coeff(1,0), matTransform.coeff(0,0));
}

alice_walking_module_msgs::PoseXYZRPY FootStepGenerator::getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform)
{
  alice_walking_module_msgs::PoseXYZRPY pose;

  double pose_x     = 0;
  double pose_y     = 0;
  double pose_z     = 0;
  double pose_roll  = 0;
  double pose_pitch = 0;
  double pose_yaw   = 0;

  getPosefromTransformMatrix(matTransform, &pose_x, &pose_y, &pose_z, &pose_roll, &pose_pitch, &pose_yaw);

  pose.x     = pose_x;
  pose.y     = pose_y;
  pose.z     = pose_z;
  pose.roll  = pose_roll;
  pose.pitch = pose_pitch;
  pose.yaw   = pose_yaw;

  return pose;
}

Eigen::MatrixXd FootStepGenerator::getInverseTransformation(Eigen::MatrixXd transform)
{
  // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

  Eigen::Vector3d vec_boa;
  Eigen::Vector3d vec_x, vec_y, vec_z;
  Eigen::MatrixXd inv_t(4,4);

  vec_boa(0) = -transform(0,3);
  vec_boa(1) = -transform(1,3);
  vec_boa(2) = -transform(2,3);

  vec_x(0) = transform(0,0); vec_x(1) = transform(1,0); vec_x(2) = transform(2,0);
  vec_y(0) = transform(0,1); vec_y(1) = transform(1,1); vec_y(2) = transform(2,1);
  vec_z(0) = transform(0,2); vec_z(1) = transform(1,2); vec_z(2) = transform(2,2);

  inv_t <<
      vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
      vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
      vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
      0, 0, 0, 1;

  return inv_t;
}

void FootStepGenerator::getStepData(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data,int desired_step_type,double desired_step_type_num)
{
  step_data_array->clear();
  step_data_array_.clear();

  if(calcStep(ref_step_data, previous_step_type_, desired_step_type, desired_step_type_num))
  {
    revolute_type_ = desired_step_type_num;
    previous_step_type_ = desired_step_type;
    for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
    {
      step_data_array->push_back(step_data_array_[stp_idx]);
    }
  }
  else
  {
    return;
  }
}



void FootStepGenerator::getStepDataFromStepData2DArray(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data,
    const alice_foot_step_generator::Step2DArray::ConstPtr& request_step_2d)
{
  step_data_array->clear();

  alice_walking_module_msgs::StepData stp_data;

  stp_data = ref_step_data;
  stp_data.time_data.abs_step_time += start_end_time_sec_;
  stp_data.time_data.dsp_ratio = dsp_ratio_;
  stp_data.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  stp_data.time_data.start_time_delay_ratio_x     = 0.0;
  stp_data.time_data.start_time_delay_ratio_y     = 0.0;
  stp_data.time_data.start_time_delay_ratio_z     = 0.0;
  stp_data.time_data.start_time_delay_ratio_roll  = 0.0;
  stp_data.time_data.start_time_delay_ratio_pitch = 0.0;
  stp_data.time_data.start_time_delay_ratio_yaw   = 0.0;
  stp_data.time_data.finish_time_advance_ratio_x     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_y     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_z     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_roll  = 0.0;
  stp_data.time_data.finish_time_advance_ratio_pitch = 0.0;
  stp_data.time_data.finish_time_advance_ratio_yaw   = 0.0;

  stp_data.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  stp_data.position_data.foot_z_swap = 0;
  stp_data.position_data.body_z_swap = 0;

  step_data_array->push_back(stp_data);

  for(unsigned int stp_idx = 0; stp_idx < request_step_2d->footsteps_2d.size(); stp_idx++)
  {
    stp_data.time_data.abs_step_time += step_time_sec_;
    stp_data.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;

    if(request_step_2d->footsteps_2d[stp_idx].moving_foot == alice_foot_step_generator::Step2D::LEFT_FOOT_SWING)
    {
      stp_data.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data.position_data.body_z_swap = body_z_swap_m_;
      stp_data.position_data.foot_z_swap = foot_z_swap_m_;
      stp_data.position_data.left_foot_pose.x   = request_step_2d->footsteps_2d[stp_idx].step2d.x;
      stp_data.position_data.left_foot_pose.y   = request_step_2d->footsteps_2d[stp_idx].step2d.y;
      stp_data.position_data.left_foot_pose.yaw = request_step_2d->footsteps_2d[stp_idx].step2d.theta;

    }
    else if(request_step_2d->footsteps_2d[stp_idx].moving_foot == alice_foot_step_generator::Step2D::RIGHT_FOOT_SWING)
    {
      stp_data.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data.position_data.body_z_swap = body_z_swap_m_;
      stp_data.position_data.foot_z_swap = foot_z_swap_m_;
      stp_data.position_data.right_foot_pose.x   = request_step_2d->footsteps_2d[stp_idx].step2d.x;
      stp_data.position_data.right_foot_pose.y   = request_step_2d->footsteps_2d[stp_idx].step2d.y;
      stp_data.position_data.right_foot_pose.yaw = request_step_2d->footsteps_2d[stp_idx].step2d.theta;
    }
    else
    {
      ROS_ERROR("Invalid Step2D");
      step_data_array->clear();
      return;
    }

    if(fabs(stp_data.position_data.right_foot_pose.yaw - stp_data.position_data.left_foot_pose.yaw) > M_PI)
    {
      stp_data.position_data.body_pose.yaw = 0.5*(stp_data.position_data.right_foot_pose.yaw + stp_data.position_data.left_foot_pose.yaw)
                                                                                                                                                                                                                                                                                                    - sign(0.5*(stp_data.position_data.right_foot_pose.yaw - stp_data.position_data.left_foot_pose.yaw))*M_PI;
    }
    else
    {
      stp_data.position_data.body_pose.yaw = 0.5*(stp_data.position_data.right_foot_pose.yaw
          + stp_data.position_data.left_foot_pose.yaw);
    }

    step_data_array->push_back(stp_data);
  }

  stp_data.time_data.abs_step_time += start_end_time_sec_;
  stp_data.time_data.dsp_ratio = dsp_ratio_;
  stp_data.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  stp_data.time_data.start_time_delay_ratio_x     = 0.0;
  stp_data.time_data.start_time_delay_ratio_y     = 0.0;
  stp_data.time_data.start_time_delay_ratio_z     = 0.0;
  stp_data.time_data.start_time_delay_ratio_roll  = 0.0;
  stp_data.time_data.start_time_delay_ratio_pitch = 0.0;
  stp_data.time_data.start_time_delay_ratio_yaw   = 0.0;
  stp_data.time_data.finish_time_advance_ratio_x     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_y     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_z     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_roll  = 0.0;
  stp_data.time_data.finish_time_advance_ratio_pitch = 0.0;
  stp_data.time_data.finish_time_advance_ratio_yaw   = 0.0;

  stp_data.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  stp_data.position_data.foot_z_swap = 0;
  stp_data.position_data.body_z_swap = 0;

  step_data_array->push_back(stp_data);
}


bool FootStepGenerator::calcStep(const alice_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type,  double desired_step_type_num)
{

  int direction = 0;
  alice_walking_module_msgs::StepData stp_data[2];

  alice_walking_module_msgs::PoseXYZRPY poseGtoRF, poseGtoLF;
  alice_walking_module_msgs::PoseXYZRPY poseLtoRF, poseLtoLF;

  poseGtoRF = ref_step_data.position_data.right_foot_pose;
  poseGtoLF = ref_step_data.position_data.left_foot_pose;

  Eigen::MatrixXd mat_g_to_rf = getTransformationXYZRPY(poseGtoRF.x, poseGtoRF.y, poseGtoRF.z, 0, 0, poseGtoRF.yaw);
  Eigen::MatrixXd mat_g_to_lf = getTransformationXYZRPY(poseGtoLF.x, poseGtoLF.y, poseGtoLF.z, 0, 0, poseGtoLF.yaw);

  //the local coordinate is set as below.
  //the below local does not means real local coordinate.
  //it is just for calculating step data.
  //the local coordinate will be decide by the moving foot of ref step data

  //ROS_INFO("REFERENCE STEP DATA=======================================");
  //ROS_INFO("RIGHT FOOT :  %f  |  %f  |  %f",poseGtoRF.x, poseGtoRF.y,poseGtoRF.yaw);
  //ROS_INFO(" LEFT FOOT :  %f  |  %f  |  %f",poseGtoLF.x, poseGtoLF.y,poseGtoLF.yaw);



  Eigen::MatrixXd mat_lf_to_local = getTransformationXYZRPY(0, -0.5*default_y_feet_offset_m_, 0, 0, 0, -0.5*default_yaw_feet_offset_m_);
  Eigen::MatrixXd mat_rf_to_local = getTransformationXYZRPY(0,  0.5*default_y_feet_offset_m_, 0, 0, 0,  0.5*default_yaw_feet_offset_m_);
  Eigen::MatrixXd mat_global_to_local, mat_local_to_global;
  if(ref_step_data.position_data.moving_foot == alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
  {
    //ROS_INFO("MOVING_FOOT : RIGHT    %d",ref_step_data.position_data.moving_foot);
    mat_global_to_local = mat_g_to_rf*mat_rf_to_local;
    mat_local_to_global = getInverseTransformation(mat_global_to_local);
    mat_lf_to_local     = getInverseTransformation(mat_g_to_lf) * mat_global_to_local;
  }
  else
  {
    //ROS_INFO("MOVING_FOOT : LEFT    %d",ref_step_data.position_data.moving_foot);
    mat_global_to_local = mat_g_to_lf * mat_lf_to_local;;
    mat_local_to_global = getInverseTransformation(mat_global_to_local);
    mat_rf_to_local     = getInverseTransformation(mat_g_to_rf) * mat_global_to_local;
  }

  Eigen::MatrixXd mat_local_to_rf = mat_local_to_global * mat_g_to_rf;
  Eigen::MatrixXd mat_local_to_lf = mat_local_to_global * mat_g_to_lf;

  poseLtoRF = getPosefromTransformMatrix(mat_local_to_rf);
  poseLtoLF = getPosefromTransformMatrix(mat_local_to_lf);


  if((desired_step_type == FORWARD_WALKING) || (desired_step_type == LEFTWARD_WALKING) || (desired_step_type == LEFT_ROTATING_WALKING) || (desired_step_type == REVOLUTE_LEFT_WALKING))
    direction = 1;
  else if((desired_step_type == BACKWARD_WALKING ) || (desired_step_type == RIGHTWARD_WALKING) || (desired_step_type == RIGHT_ROTATING_WALKING)|| (desired_step_type == REVOLUTE_RIGHT_WALKING))
    direction = -1;
  else if(desired_step_type == STOP_WALKING)
    direction = 0;
  else
    return false;


  stp_data[0] = ref_step_data;
  stp_data[0].position_data.torso_yaw_angle_rad = 0.0*M_PI;

  stp_data[0].position_data.right_foot_pose = poseLtoRF;
  stp_data[0].position_data.left_foot_pose = poseLtoLF;

  stp_data[0].time_data.start_time_delay_ratio_x     = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_y     = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_z     = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_roll  = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_pitch = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_yaw   = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_x     = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_y     = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_z     = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_roll  = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_pitch = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_yaw   = 0.0;



  //ROS_INFO("REFERENCE STEP DATA=======================================");
  //ROS_INFO("RIGHT FOOT :  %f  |  %f  |  %f",poseLtoRF.x, poseLtoRF.y,poseLtoRF.yaw);
  //ROS_INFO(" LEFT FOOT :  %f  |  %f  |  %f",poseLtoLF.x, poseLtoLF.y,poseLtoLF.yaw);

  if(stp_data[0].time_data.walking_state != alice_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    //ROS_INFO("11111111111111111111");
    if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING )
      calcFBStep(stp_data[0], direction,0);
    else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == LEFTWARD_WALKING )
      calcRLStep(stp_data[0], direction);
    else if(desired_step_type == LEFT_ROTATING_WALKING || desired_step_type == RIGHT_ROTATING_WALKING )
      calcRoStep(stp_data[0], direction);
    else if(desired_step_type == REVOLUTE_LEFT_WALKING || desired_step_type == REVOLUTE_RIGHT_WALKING )
      calcRevRLStep(stp_data[0], direction, desired_step_type_num);
    else if(desired_step_type == STOP_WALKING)
      calcStopStep(stp_data[0], direction);
    else
      return false;
  }
  else
  {
    if(desired_step_type != previous_step_type)
    {

      stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
      if((fabs(poseLtoRF.yaw - poseLtoLF.yaw) > default_yaw_feet_offset_m_)
          || (fabs(poseLtoRF.y - poseLtoLF.y) > default_y_feet_offset_m_)
          || (fabs(poseLtoRF.x - poseLtoLF.x) > 0))
      {
        stp_data[0].time_data.abs_step_time += step_time_sec_;
        if(ref_step_data.position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
        {
          //ROS_INFO("2MOVING_FOOT : LEFT    %d",ref_step_data.position_data.moving_foot);
          stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
          stp_data[0].position_data.right_foot_pose.x   = stp_data[0].position_data.left_foot_pose.x;
          stp_data[0].position_data.right_foot_pose.y   = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m_;
          stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw -default_yaw_feet_offset_m_;
        }
        else
        {
          //("2MOVING_FOOT : RIGHT   %d",ref_step_data.position_data.moving_foot);
          stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
          stp_data[0].position_data.left_foot_pose.x   = stp_data[0].position_data.right_foot_pose.x;
          stp_data[0].position_data.left_foot_pose.y   = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m_;
          stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw + default_yaw_feet_offset_m_;
        }
        step_data_array_.push_back(stp_data[0]);
      }

      stp_data[1] = stp_data[0];
      //ROS_INFO("2222222222222");
      if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING || desired_step_type == STOP_WALKING)
      {

      }
      else if(desired_step_type == LEFTWARD_WALKING || desired_step_type == LEFT_ROTATING_WALKING || desired_step_type ==  REVOLUTE_LEFT_WALKING)
      {

        //ROS_INFO("CHAGE FOOT LEFT   %d",ref_step_data.position_data.moving_foot);
        if(stp_data[0].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
        {
          //ROS_INFO("CHAGE STEP ADDED");
          stp_data[1].time_data.abs_step_time += step_time_sec_;
          stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
          step_data_array_.push_back(stp_data[1]);

        }
      }
      else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == RIGHT_ROTATING_WALKING|| desired_step_type ==  REVOLUTE_RIGHT_WALKING)
      {
        //ROS_INFO("CHAGE FOOT RIGHT   %d",ref_step_data.position_data.moving_foot);
        if(stp_data[0].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
        {
          //ROS_INFO("CHAGE STEP ADDED");
          stp_data[1].time_data.abs_step_time += step_time_sec_;
          stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
          step_data_array_.push_back(stp_data[1]);

        }
      }
      else
      {
        return false;
      }

      //ROS_INFO("33333333333333");
      if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING )
      {
        calcFBStep(stp_data[1], direction,0);
      }
      else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == LEFTWARD_WALKING )
      {
        calcRLStep(stp_data[1], direction);
      }
      else if(desired_step_type == LEFT_ROTATING_WALKING || desired_step_type == RIGHT_ROTATING_WALKING )
      {
        calcRoStep(stp_data[1], direction);
      }
      else if(desired_step_type == REVOLUTE_LEFT_WALKING || desired_step_type == REVOLUTE_RIGHT_WALKING )
      {
        calcRevRLStep(stp_data[1], direction,desired_step_type_num);
      }

      else if(desired_step_type == STOP_WALKING)
      {
        calcStopStep(stp_data[1], direction);
      }
      else
      {
        return false;
      }
    }
    else
    {

      //ROS_INFO("4444444444444444");
      if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING )
      {
        calcFBStep(stp_data[0], direction,0);
      }
      else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == LEFTWARD_WALKING )
      {
        calcRLStep(stp_data[0], direction);
      }
      else if(desired_step_type == LEFT_ROTATING_WALKING || desired_step_type == RIGHT_ROTATING_WALKING )
      {
        calcRoStep(stp_data[0], direction);
      }
      else if(desired_step_type == REVOLUTE_LEFT_WALKING || desired_step_type == REVOLUTE_RIGHT_WALKING )
      {
        if(desired_step_type == REVOLUTE_LEFT_WALKING)
        {
          if(ref_step_data.position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
          {
            //ROS_INFO("CHAGE FOOT LEFT");

            stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;

            stp_data[0].time_data.abs_step_time += step_time_sec_;
            stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
            stp_data[0].position_data.right_foot_pose.x   = stp_data[0].position_data.left_foot_pose.x;
            stp_data[0].position_data.right_foot_pose.y   = stp_data[0].position_data.left_foot_pose.y- default_y_feet_offset_m_;
            stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw-default_yaw_feet_offset_m_;

            step_data_array_.push_back(stp_data[0]);
          }

        }
        if(desired_step_type == REVOLUTE_RIGHT_WALKING)
        {
          if(ref_step_data.position_data.moving_foot == alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
          {
            //ROS_INFO("CHAGE FOOT RIGHT");
            stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;

            stp_data[0].time_data.abs_step_time += step_time_sec_;
            stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
            stp_data[0].position_data.left_foot_pose.x   = stp_data[0].position_data.right_foot_pose.x;
            stp_data[0].position_data.left_foot_pose.y   = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m_;
            stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw + default_yaw_feet_offset_m_;
            step_data_array_.push_back(stp_data[0]);
          }
        }

        calcRevRLStep(stp_data[0], direction, desired_step_type_num);
      }

      else if(desired_step_type == STOP_WALKING)
      {
        calcStopStep(stp_data[0], direction);
      }
      else
      {
        return false;
      }
    }
  }


  //ROS_INFO("55555555555555555");
  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    Eigen::MatrixXd mat_r_foot = getTransformationXYZRPY(step_data_array_[stp_idx].position_data.right_foot_pose.x,
        step_data_array_[stp_idx].position_data.right_foot_pose.y,
        step_data_array_[stp_idx].position_data.right_foot_pose.z,
        step_data_array_[stp_idx].position_data.right_foot_pose.roll,
        step_data_array_[stp_idx].position_data.right_foot_pose.pitch,
        step_data_array_[stp_idx].position_data.right_foot_pose.yaw);

    Eigen::MatrixXd mat_l_foot = getTransformationXYZRPY(step_data_array_[stp_idx].position_data.left_foot_pose.x,
        step_data_array_[stp_idx].position_data.left_foot_pose.y,
        step_data_array_[stp_idx].position_data.left_foot_pose.z,
        step_data_array_[stp_idx].position_data.left_foot_pose.roll,
        step_data_array_[stp_idx].position_data.left_foot_pose.pitch,
        step_data_array_[stp_idx].position_data.left_foot_pose.yaw);

    step_data_array_[stp_idx].position_data.right_foot_pose = getPosefromTransformMatrix(mat_global_to_local * mat_r_foot);
    step_data_array_[stp_idx].position_data.left_foot_pose  = getPosefromTransformMatrix(mat_global_to_local * mat_l_foot);


    if(fabs(step_data_array_[stp_idx].position_data.right_foot_pose.yaw - step_data_array_[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
    {
      step_data_array_[stp_idx].position_data.body_pose.yaw = 0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw + step_data_array_[stp_idx].position_data.left_foot_pose.yaw)
                                                                                                                  - sign(0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw - step_data_array_[stp_idx].position_data.left_foot_pose.yaw))*M_PI;
    }
    else
    {
      step_data_array_[stp_idx].position_data.body_pose.yaw = 0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw
          + step_data_array_[stp_idx].position_data.left_foot_pose.yaw);
    }

    if(step_data_array_[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      step_data_array_[stp_idx].position_data.y_zmp_shift = y_zmp_convergence_m_;
    else if(step_data_array_[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
      step_data_array_[stp_idx].position_data.y_zmp_shift = -y_zmp_convergence_m_;
    else
      step_data_array_[stp_idx].position_data.y_zmp_shift = 0;
  }

  /*
  ROS_INFO("-------------------------------------");
  ROS_INFO("STEP DATA ARRAY");

  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    ROS_INFO("index :  %d",stp_idx);
    ROS_INFO("RIGHT FOOT :  %f  |  %f  |  %f",step_data_array_[stp_idx].position_data.right_foot_pose.x,step_data_array_[stp_idx].position_data.right_foot_pose.y,step_data_array_[stp_idx].position_data.right_foot_pose.yaw);
    ROS_INFO("LEFTT FOOT :  %f  |  %f  |  %f",step_data_array_[stp_idx].position_data.left_foot_pose.x,step_data_array_[stp_idx].position_data.left_foot_pose.y,step_data_array_[stp_idx].position_data.left_foot_pose.yaw);
  }

  ROS_INFO("-------------------------------------");
   */
  return true;
}

void FootStepGenerator::calcFBStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction, double desired_step_type_num)
{
  alice_walking_module_msgs::StepData stp_data[num_of_step_];
  stp_data[0] = ref_step_data;

  if(ref_step_data.time_data.walking_state == alice_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.abs_step_time += step_time_sec_;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;
    if(stp_data[0].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.x = stp_data[0].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
    }
    else
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.x = stp_data[0].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
    }

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = stp_data[num_of_step_-2].position_data.left_foot_pose.x;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = stp_data[num_of_step_-2].position_data.right_foot_pose.x;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }
  else
  {
    stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      stp_data[stp_idx].time_data.dsp_ratio = dsp_ratio_;
      stp_data[stp_idx].position_data.body_z_swap = body_z_swap_m_;
      stp_data[stp_idx].position_data.foot_z_swap = foot_z_swap_m_;

      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = stp_data[num_of_step_-2].position_data.left_foot_pose.x;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = stp_data[num_of_step_-2].position_data.right_foot_pose.x;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }

  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);
    //std::cout << "++++++++"<< stp_idx<< "+++++++++++" << std::endl;
    //std::cout << step_data_array_[stp_idx] << std::endl;

    //ROS_INFO("idx : %d    foot:  %d",stp_idx,stp_data[stp_idx].position_data.moving_foot);
    //ROS_INFO("right %f   %f   %f",stp_data[stp_idx].position_data.right_foot_pose.x,stp_data[stp_idx].position_data.right_foot_pose.y ,stp_data[stp_idx].position_data.right_foot_pose.yaw );
    //ROS_INFO("left %f   %f   %f",stp_data[stp_idx].position_data.left_foot_pose.x,stp_data[stp_idx].position_data.left_foot_pose.y ,stp_data[stp_idx].position_data.left_foot_pose.yaw );

  }
  //ROS_INFO("+++++++++++++++++++++++++");

}

void FootStepGenerator::calcRLStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction)
{
  alice_walking_module_msgs::StepData stp_data[num_of_step_];
  stp_data[0] = ref_step_data;

  if(ref_step_data.time_data.walking_state == alice_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.abs_step_time += step_time_sec_;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;
    if(stp_data[0].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.y = stp_data[0].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
    }
    else
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.y = stp_data[0].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
    }

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = stp_data[num_of_step_-2].position_data.left_foot_pose.y - default_y_feet_offset_m_;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = stp_data[num_of_step_-2].position_data.right_foot_pose.y + default_y_feet_offset_m_;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }
  else
  {
    stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;


    stp_data[1] = stp_data[0];
    stp_data[1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[1].time_data.abs_step_time += step_time_sec_;
    stp_data[1].time_data.dsp_ratio = dsp_ratio_;
    stp_data[1].position_data.body_z_swap = body_z_swap_m_;
    stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;

    if(direction < 0)
    {
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[1].position_data.right_foot_pose.y = stp_data[1].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
    }
    else
    {
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[1].position_data.left_foot_pose.y = stp_data[1].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
    }

    for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = stp_data[num_of_step_-2].position_data.left_foot_pose.y - default_y_feet_offset_m_;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = stp_data[num_of_step_-2].position_data.right_foot_pose.y + default_y_feet_offset_m_;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;

  }

  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);
  }
}

void FootStepGenerator::calcRoStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction)
{
  alice_walking_module_msgs::StepData stp_data[num_of_step_];


  stp_data[0] = ref_step_data;

  if(ref_step_data.time_data.walking_state == alice_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[0].time_data.abs_step_time += step_time_sec_;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;

    if(stp_data[0].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[0].position_data.right_foot_pose.yaw) > 2.0*M_PI)
        stp_data[0].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[0].position_data.right_foot_pose.yaw);

      stp_data[0].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[0].position_data.right_foot_pose.yaw + 0.5*default_yaw_feet_offset_m_);
      stp_data[0].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[0].position_data.right_foot_pose.yaw + 0.5*default_yaw_feet_offset_m_);
    }
    else
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[0].position_data.left_foot_pose.yaw) > 2.0*M_PI)
        stp_data[0].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[0].position_data.left_foot_pose.yaw);

      stp_data[0].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[0].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
      stp_data[0].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[0].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
    }


    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);

        stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw + 0.5*default_yaw_feet_offset_m_);
        stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw + 0.5*default_yaw_feet_offset_m_);
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);

        stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
        stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);

      }

      if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.right_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);
      if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.left_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw-default_yaw_feet_offset_m_;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
      stp_data[num_of_step_-2].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);

      //stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw;
      //stp_data[num_of_step_-2].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);
      //stp_data[num_of_step_-2].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);

    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw+default_yaw_feet_offset_m_;
      //stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
      stp_data[num_of_step_-2].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }
  else
  {
    stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;
    stp_data[0].position_data.foot_z_swap = 0;


    stp_data[1] = stp_data[0];
    stp_data[1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[1].time_data.abs_step_time += step_time_sec_;
    stp_data[1].time_data.dsp_ratio = dsp_ratio_;
    stp_data[1].position_data.body_z_swap = body_z_swap_m_;
    stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;

    if(direction < 0)
    {
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[1].position_data.right_foot_pose.yaw  = stp_data[1].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[1].position_data.right_foot_pose.yaw) > 2.0*M_PI)
        stp_data[1].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[1].position_data.right_foot_pose.yaw);

      stp_data[1].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m_*sin(stp_data[1].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
      stp_data[1].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m_*cos(stp_data[1].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
    }
    else
    {
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[1].position_data.left_foot_pose.yaw  = stp_data[1].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[1].position_data.left_foot_pose.yaw) > 2.0*M_PI)
        stp_data[1].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[1].position_data.left_foot_pose.yaw);


      stp_data[1].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m_*sin(stp_data[1].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
      stp_data[1].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m_*cos(stp_data[1].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
    }

    for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);

        stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
        stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);

        stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
        stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);

      }

      if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.right_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);
      if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.left_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      //stp_data[num_of_step_-2].position_data.right_foot_pose.yaw  = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw;
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw-default_yaw_feet_offset_m_;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw - 0.5*default_yaw_feet_offset_m_);

    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      //stp_data[num_of_step_-2].position_data.left_foot_pose.yaw  = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw;
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw+default_yaw_feet_offset_m_;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
      stp_data[num_of_step_-2].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw+ 0.5*default_yaw_feet_offset_m_);
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;

  }


  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);
  }
}

void FootStepGenerator::calcRevRLStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction, double desired_step_type_num)
{
  alice_walking_module_msgs::StepData stp_data[num_of_step_];
  stp_data[0] = ref_step_data;

  double dispose_x,dispose_y,dispose_yaw,dispose_time;
  double y = default_y_feet_offset_m_/(double)2;
  double sum_theta = 0;

  Eigen::Matrix4d Trans_FW;
  Eigen::MatrixXd foot_left;
  Eigen::MatrixXd foot_right;


  if(desired_step_type_num == centered)
  {
    dispose_x = ct_step_length_m_;
    dispose_y =  (double)direction*cts_step_length_m_;
    dispose_yaw = -(double)direction*ct_step_angle_rad_;
    dispose_time = ct_step_time_sec_;

  }
  else if(desired_step_type_num == expanded)
  {
    dispose_x = ep_step_length_m_;
    dispose_y =  (double)direction*eps_step_length_m_;
    dispose_yaw = (double)direction*ep_step_angle_rad_;
    dispose_time = ep_step_time_sec_;
  }

  else if(desired_step_type_num == ip_from_launch)
  {

    dispose_x = fb_step_length_m_;
    dispose_y =  (double)direction*rl_step_length_m_;
    dispose_yaw = (double)direction*rotate_step_angle_rad_;
    dispose_time = step_time_sec_;
  }



  foot_left.resize(4,1);
  foot_right.resize(4,1);

  foot_left<< 0    ,
      y,
      0    ,
      1.0  ;

  foot_right<< 0     ,
      -y ,
      0     ,
      1.0   ;

  Trans_FW = robotis_framework::getTransformationXYZRPY(dispose_x, dispose_y, 0, 0, 0, dispose_yaw);


  if(ref_step_data.time_data.walking_state == alice_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.abs_step_time += dispose_time;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;

    if(stp_data[0].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING )
      //if(direction<0)
    {
      foot_left = Trans_FW*foot_left;
      foot_right = Trans_FW*foot_right;
      sum_theta += dispose_yaw;
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.x = (double)foot_right(0,0); //stp_data[0].position_data.right_foot_pose.x
      stp_data[0].position_data.right_foot_pose.y = (double)foot_right(1,0);
      stp_data[0].position_data.right_foot_pose.yaw =(double)sum_theta - 0.5 * default_yaw_feet_offset_m_;


    }
    else
    {
      foot_left = Trans_FW*foot_left;
      foot_right = Trans_FW*foot_right;
      sum_theta += dispose_yaw;
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.x = (double)foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
      stp_data[0].position_data.left_foot_pose.y = (double)foot_left(1,0);
      stp_data[0].position_data.left_foot_pose.yaw = (double)sum_theta + 0.5 * default_yaw_feet_offset_m_;
    }

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      if(stp_idx%2==0)
      {
        foot_left = Trans_FW*foot_left;
        foot_right = Trans_FW*foot_right;
        sum_theta += dispose_yaw;
      }
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += dispose_time;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = (double)foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
        stp_data[stp_idx].position_data.right_foot_pose.y = (double)foot_right(1,0);
        stp_data[stp_idx].position_data.right_foot_pose.yaw = (double)sum_theta - 0.5 * default_yaw_feet_offset_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = (double)foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
        stp_data[stp_idx].position_data.left_foot_pose.y = (double)foot_left(1,0);
        stp_data[stp_idx].position_data.left_foot_pose.yaw = (double)sum_theta + 0.5 * default_yaw_feet_offset_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += dispose_time;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = (double)foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = (double)foot_right(1,0);
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = (double)sum_theta- 0.5 * default_yaw_feet_offset_m_;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = (double)foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = (double)foot_left(1,0);
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = (double)sum_theta + 0.5 * default_yaw_feet_offset_m_;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;


  }
  else
  {
    stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;

    stp_data[1] = stp_data[0];
    stp_data[1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[1].time_data.abs_step_time += dispose_time;
    stp_data[1].time_data.dsp_ratio = dsp_ratio_;
    stp_data[1].position_data.body_z_swap = body_z_swap_m_;
    stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;
    if(direction < 0)
    {
      foot_left = Trans_FW*foot_left;
      foot_right = Trans_FW*foot_right;
      sum_theta += dispose_yaw;
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[1].position_data.right_foot_pose.x = (double)foot_right(0,0); //stp_data[1].position_data.right_foot_pose.x
      stp_data[1].position_data.right_foot_pose.y = (double)foot_right(1,0);
      stp_data[1].position_data.right_foot_pose.yaw = (double)sum_theta- 0.5 * default_yaw_feet_offset_m_;
    }
    else
    {
      foot_left = Trans_FW*foot_left;
      foot_right = Trans_FW*foot_right;
      sum_theta += dispose_yaw;
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[1].position_data.left_foot_pose.x = (double)foot_left(0,0); //stp_data[1].position_data.left_foot_pose.x
      stp_data[1].position_data.left_foot_pose.y = (double)foot_left(1,0);
      stp_data[1].position_data.left_foot_pose.yaw =  (double)sum_theta+ 0.5 * default_yaw_feet_offset_m_;
    }

    for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++)
    {
      if(stp_idx%2==1)
      {
        foot_left = Trans_FW*foot_left;
        foot_right = Trans_FW*foot_right;
        sum_theta += dispose_yaw;
      }
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += dispose_time;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {

        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = (double)foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
        stp_data[stp_idx].position_data.right_foot_pose.y = (double)foot_right(1,0);
        stp_data[stp_idx].position_data.right_foot_pose.yaw = (double)sum_theta - 0.5 * default_yaw_feet_offset_m_;

      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = (double)foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
        stp_data[stp_idx].position_data.left_foot_pose.y = (double)foot_left(1,0);
        stp_data[stp_idx].position_data.left_foot_pose.yaw = (double)sum_theta + 0.5 * default_yaw_feet_offset_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += dispose_time;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = (double)foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = (double)foot_right(1,0);
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = (double)sum_theta- 0.5 * default_yaw_feet_offset_m_;

    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = (double)foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = (double)foot_left(1,0);
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = (double)sum_theta+ 0.5 * default_yaw_feet_offset_m_;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;

  }


  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);

    //ROS_INFO("idx : %d    foot:  %d",stp_idx,stp_data[stp_idx].position_data.moving_foot);
    //ROS_INFO("right %f   %f   %f",stp_data[stp_idx].position_data.right_foot_pose.x,stp_data[stp_idx].position_data.right_foot_pose.y ,stp_data[stp_idx].position_data.right_foot_pose.yaw );
    //ROS_INFO("left %f   %f   %f",stp_data[stp_idx].position_data.left_foot_pose.x,stp_data[stp_idx].position_data.left_foot_pose.y ,stp_data[stp_idx].position_data.left_foot_pose.yaw );

  }


  /*
  alice_walking_module_msgs::StepData stp_data[num_of_step_];
  stp_data[0] = ref_step_data;


  //ROS_INFO("Local REF DATA   %d++++++++++++",stp_data[0].position_data.moving_foot);
  //ROS_INFO("right %f   %f   %f",stp_data[0].position_data.right_foot_pose.x,stp_data[0].position_data.right_foot_pose.y ,stp_data[0].position_data.right_foot_pose.yaw );
  //ROS_INFO("left %f   %f   %f",stp_data[0].position_data.left_foot_pose.x,stp_data[0].position_data.left_foot_pose.y ,stp_data[0].position_data.left_foot_pose.yaw );

  Eigen::Matrix4d trans_expanded;
  Eigen::MatrixXd goal_foot_left;//, goal_foot_left_p,goal_foot_left_c;
  Eigen::MatrixXd goal_foot_right;//, goal_foot_right_p,goal_foot_right_c;
  double dispose_x,dispose_y,dispose_yaw,dispose_time;

  goal_foot_left.resize(4,1);
  goal_foot_right.resize(4,1);
  //goal_foot_left_p.resize(4,1);
  //goal_foot_right_p.resize(4,1);
  //goal_foot_left_c.resize(4,1);
  //goal_foot_right_c.resize(4,1);



  goal_foot_left<< 0    ,
      stp_data[0].position_data.left_foot_pose.y ,
      0    ,
      1.0  ;

  goal_foot_right<< 0     ,
      stp_data[0].position_data.right_foot_pose.y ,
      0     ,
      1.0   ;

  if(desired_step_type_num == centered)
  {
    dispose_x = ct_step_length_m_;
    dispose_y =  (double)direction*cts_step_length_m_;
    dispose_yaw = -(double)direction*ct_step_angle_rad_;
    dispose_time = ct_step_time_sec_;

  }
  else if(desired_step_type_num == expanded)
  {
    dispose_x = ep_step_length_m_;
    dispose_y =  (double)direction*eps_step_length_m_;
    dispose_yaw = (double)direction*ep_step_angle_rad_;
    dispose_time = ep_step_time_sec_;

  }

  trans_expanded = robotis_framework::getTransformationXYZRPY(dispose_x, dispose_y, 0, 0, 0, dispose_yaw);

  goal_foot_left = trans_expanded*goal_foot_left;
  goal_foot_right = trans_expanded*goal_foot_right;


  if(ref_step_data.time_data.walking_state == alice_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.abs_step_time += dispose_time;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;

    if(stp_data[0].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING )
      //if(direction<0)
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.x = (double)goal_foot_right(0,0); //stp_data[0].position_data.right_foot_pose.x
      stp_data[0].position_data.right_foot_pose.y = (double)goal_foot_right(1,0);
      stp_data[0].position_data.right_foot_pose.yaw =(double)dispose_yaw - 0.5 * default_yaw_feet_offset_m_;


    }
    else
    {
      stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.x = (double)goal_foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
      stp_data[0].position_data.left_foot_pose.y = (double)goal_foot_left(1,0);
      stp_data[0].position_data.left_foot_pose.yaw = (double)dispose_yaw+ 0.5 * default_yaw_feet_offset_m_;
    }

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += dispose_time;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = (double)goal_foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
        stp_data[stp_idx].position_data.right_foot_pose.y = (double)goal_foot_right(1,0);
        stp_data[stp_idx].position_data.right_foot_pose.yaw = (double)dispose_yaw- 0.5 * default_yaw_feet_offset_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = (double)goal_foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
        stp_data[stp_idx].position_data.left_foot_pose.y = (double)goal_foot_left(1,0);
        stp_data[stp_idx].position_data.left_foot_pose.yaw = (double)dispose_yaw+ 0.5 * default_yaw_feet_offset_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += dispose_time;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = (double)goal_foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = (double)goal_foot_right(1,0);
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = (double)dispose_yaw- 0.5 * default_yaw_feet_offset_m_;



    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = (double)goal_foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = (double)goal_foot_left(1,0);
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = (double)dispose_yaw+ 0.5 * default_yaw_feet_offset_m_;

    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;


  }
  else
  {
    stp_data[0].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;

    stp_data[1] = stp_data[0];
    stp_data[1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[1].time_data.abs_step_time += dispose_time;
    stp_data[1].time_data.dsp_ratio = dsp_ratio_;
    stp_data[1].position_data.body_z_swap = body_z_swap_m_;
    stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;
    if(direction < 0)
    {
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[1].position_data.right_foot_pose.x = (double)goal_foot_right(0,0); //stp_data[1].position_data.right_foot_pose.x
      stp_data[1].position_data.right_foot_pose.y = (double)goal_foot_right(1,0);
      stp_data[1].position_data.right_foot_pose.yaw = (double)dispose_yaw- 0.5 * default_yaw_feet_offset_m_;
    }
    else
    {
      stp_data[1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[1].position_data.left_foot_pose.x = (double)goal_foot_left(0,0); //stp_data[1].position_data.left_foot_pose.x
      stp_data[1].position_data.left_foot_pose.y = (double)goal_foot_left(1,0);
      stp_data[1].position_data.left_foot_pose.yaw =  (double)dispose_yaw+ 0.5 * default_yaw_feet_offset_m_;
    }

    for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += dispose_time;
      if(stp_data[stp_idx].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = (double)goal_foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
        stp_data[stp_idx].position_data.right_foot_pose.y = (double)goal_foot_right(1,0);
        stp_data[stp_idx].position_data.right_foot_pose.yaw = (double)dispose_yaw- 0.5 * default_yaw_feet_offset_m_;

      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = (double)goal_foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
        stp_data[stp_idx].position_data.left_foot_pose.y = (double)goal_foot_left(1,0);
        stp_data[stp_idx].position_data.left_foot_pose.yaw = (double)dispose_yaw+ 0.5 * default_yaw_feet_offset_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += dispose_time;
    if(stp_data[num_of_step_-2].position_data.moving_foot == alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = (double)goal_foot_right(0,0); //stp_data[stp_idx].position_data.right_foot_pose.x
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = (double)goal_foot_right(1,0);
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = (double)dispose_yaw- 0.5 * default_yaw_feet_offset_m_;

    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = (double)goal_foot_left(0,0); //stp_data[0].position_data.left_foot_pose.x
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = (double)goal_foot_left(1,0);
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = (double)dispose_yaw+ 0.5 * default_yaw_feet_offset_m_;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;

  }


  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);

    //ROS_INFO("idx : %d    foot:  %d",stp_idx,stp_data[stp_idx].position_data.moving_foot);
    //ROS_INFO("right %f   %f   %f",stp_data[stp_idx].position_data.right_foot_pose.x,stp_data[stp_idx].position_data.right_foot_pose.y ,stp_data[stp_idx].position_data.right_foot_pose.yaw );
    //ROS_INFO("left %f   %f   %f",stp_data[stp_idx].position_data.left_foot_pose.x,stp_data[stp_idx].position_data.left_foot_pose.y ,stp_data[stp_idx].position_data.left_foot_pose.yaw );

  }
  //ROS_INFO("+++++++++++++++++++++++++");
   */
}


void FootStepGenerator::calcStopStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction)
{
  alice_walking_module_msgs::StepData stp_data;
  stp_data = ref_step_data;
  stp_data.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  stp_data.time_data.abs_step_time += start_end_time_sec_;
  stp_data.position_data.body_z_swap = 0;
  stp_data.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;

  step_data_array_.push_back(stp_data);
}

void FootStepGenerator::calcYType(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data)
{
  alice_walking_module_msgs::StepData step_data_msg;

  step_data_msg = ref_step_data;

  double dispose_step_time = step_time_sec_;

  step_data_array->clear();
  step_data_array_.clear();

  if(type_offset_y_== y_steptype_offset_y_ && type_offset_yaw_== y_steptype_offset_yaw_)
  {
    return;
  }
  type_offset_y_ = y_steptype_offset_y_;
  type_offset_yaw_ = y_steptype_offset_yaw_;
  default_y_feet_offset_m_ += type_offset_y_;
  default_yaw_feet_offset_m_ +=  type_offset_yaw_;



  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1;//start_end_time_sec_;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.body_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += dispose_step_time;
  step_data_msg.time_data.dsp_ratio = dsp_ratio_;
  step_data_msg.position_data.body_z_swap = body_z_swap_m_;
  step_data_msg.position_data.foot_z_swap = foot_z_swap_m_;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.y -= 0.5*type_offset_y_;
  step_data_msg.position_data.right_foot_pose.yaw -= 0.5*type_offset_yaw_;
  step_data_array_.push_back(step_data_msg);


  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += dispose_step_time;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.y += 0.5*type_offset_y_;
  step_data_msg.position_data.left_foot_pose.yaw += 0.5*type_offset_yaw_;
  step_data_array_.push_back(step_data_msg);

  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;
  step_data_msg.time_data.dsp_ratio = 0.0;
  step_data_msg.position_data.body_z_swap = 0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);
  }
}
void FootStepGenerator::calcDefaultType(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data)
{
  alice_walking_module_msgs::StepData step_data_msg;

  step_data_msg = ref_step_data;

  double dispose_step_time = step_time_sec_ ;

  step_data_array->clear();
  step_data_array_.clear();

  if(type_offset_y_==0 && type_offset_yaw_==0)
  {
    return;
  }
  default_y_feet_offset_m_ -= type_offset_y_;
  default_yaw_feet_offset_m_ -=  type_offset_yaw_;

  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1;//start_end_time_sec_;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.body_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += dispose_step_time;
  step_data_msg.time_data.dsp_ratio = dsp_ratio_;
  step_data_msg.position_data.body_z_swap = body_z_swap_m_;
  step_data_msg.position_data.foot_z_swap = foot_z_swap_m_;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.y += 0.5*type_offset_y_;
  step_data_msg.position_data.right_foot_pose.yaw += 0.5*type_offset_yaw_;
  step_data_array_.push_back(step_data_msg);


  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += dispose_step_time;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.y -= 0.5*type_offset_y_;
  step_data_msg.position_data.left_foot_pose.yaw -= 0.5*type_offset_yaw_;
  step_data_array_.push_back(step_data_msg);

  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;
  step_data_msg.time_data.dsp_ratio = 0.0;
  step_data_msg.position_data.body_z_swap = 0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);
  }
  type_offset_y_ = 0;
  type_offset_yaw_ = 0;
}


void FootStepGenerator::calcRightKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data)
{

  alice_walking_module_msgs::StepData step_data_msg;

  step_data_msg = ref_step_data;   //this!!!!

  step_data_array->clear();
  step_data_array_.clear();

  //Start 1 Step Data
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1; //kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = kick_dsp_ratio_;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_msg.position_data.body_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 2 move zmp to left foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_array_.push_back(step_data_msg);


  //StepData 3 kick - 1st : raise foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0;

  //std::cout << "kick_height00 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.z += kick_height_m_;
  step_data_msg.position_data.right_foot_pose.pitch = 0.0;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);
  //std::cout << "kick_height11 :::::::: "<< kick_height_m_ << std::endl;


  //StepData 4 kick - 2nd : move right foot back
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += kick_back_time_;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = -kick_back_m_;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);


  //StepData 5 kick - 3rd : kick
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += kick_time_;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = kick_far_m_;
  step_data_msg.position_data.right_foot_pose.pitch = -kick_pitch_rad_;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 6 kick - 4th : move right foot to x0
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = 0;
  step_data_msg.position_data.right_foot_pose.pitch = 0;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 7 move right foot to original place
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
  step_data_msg.time_data.dsp_ratio = 0;

  //std::cout << "kick_height22 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = 0;
  step_data_msg.position_data.right_foot_pose.z -= kick_height_m_;
  step_data_msg.position_data.right_foot_pose.pitch = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "kick_height33 :::::::: "<< kick_height_m_ << std::endl;
  //StepData 8 End
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "FOOT STEP PARAM" << std::endl;
  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);


    //ROS_INFO("idx : %d    foot:  %d",stp_idx,step_data_array_[stp_idx].position_data.moving_foot);
    //ROS_INFO("right %f   %f   %f  ",step_data_array_[stp_idx].position_data.right_foot_pose.x,step_data_array_[stp_idx].position_data.right_foot_pose.y ,step_data_array_[stp_idx].position_data.right_foot_pose.yaw );
    //ROS_INFO("right %f   %f   %f",step_data_array_[stp_idx].position_data.left_foot_pose.x,step_data_array_[stp_idx].position_data.left_foot_pose.y ,step_data_array_[stp_idx].position_data.left_foot_pose.yaw );

    //std::cout << "++++++++"<< stp_idx<< "+++++++++++" << std::endl;
    //std::cout << step_data_array_[stp_idx] << std::endl;
  }
  //std::cout << "------------------------------------" << std::endl;
}

void FootStepGenerator::calcLeftKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data)
{
  alice_walking_module_msgs::StepData step_data_msg;


  step_data_msg = ref_step_data;

  step_data_array->clear();
  step_data_array_.clear();

  //Start 1 Step Data
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1;//0.1; //kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = kick_dsp_ratio_;//1.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_msg.position_data.body_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 2 move zmp to left foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_array_.push_back(step_data_msg);


  //StepData 3 kick - 1st : raise foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0;

  //std::cout << "kick_height00 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.z += kick_height_m_;
  step_data_msg.position_data.left_foot_pose.pitch = 0.0;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "kick_height11 :::::::: "<< kick_height_m_ << std::endl;


  //StepData 4 kick - 2nd : move right foot back
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += kick_back_time_;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = -kick_back_m_;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 5 kick - 3rd : kick
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += kick_time_;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = kick_far_m_;
  step_data_msg.position_data.left_foot_pose.pitch = -kick_pitch_rad_;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 6 kick - 4th : move right foot to x0
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = 0;
  step_data_msg.position_data.left_foot_pose.pitch = 0;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 7 move right foot to original place
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
  step_data_msg.time_data.dsp_ratio = 0;


  //std::cout << "kick_height22 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = 0;
  step_data_msg.position_data.left_foot_pose.z -= kick_height_m_;
  step_data_msg.position_data.left_foot_pose.pitch = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "kick_height33 :::::::: "<< kick_height_m_ << std::endl;


  //StepData 8 End
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "FOOT STEP PARAM" << std::endl;

  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);
    //ROS_INFO("idx : %d    foot:  %d",stp_idx,step_data_array_[stp_idx].position_data.moving_foot);
    //   ROS_INFO("right %f   %f   %f",step_data_array_[stp_idx].position_data.right_foot_pose.x,step_data_array_[stp_idx].position_data.right_foot_pose.y ,step_data_array_[stp_idx].position_data.right_foot_pose.yaw );
    //   ROS_INFO("right %f   %f   %f",step_data_array_[stp_idx].position_data.left_foot_pose.x,step_data_array_[stp_idx].position_data.left_foot_pose.y ,step_data_array_[stp_idx].position_data.left_foot_pose.yaw );

    //std::cout << "++++++++"<< stp_idx<< "+++++++++++" << std::endl;
    //std::cout << step_data_array_[stp_idx] << std::endl;

  }
  //std::cout << "------------------------------------" << std::endl;
}

void FootStepGenerator::calcYRightKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data)
{
  alice_walking_module_msgs::StepData step_data_msg;


  step_data_msg = ref_step_data;   //this!!!!

  step_data_array->clear();
  step_data_array_.clear();

  //Start 1 Step Data
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1; //kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = kick_dsp_ratio_;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 2 move zmp to left foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_array_.push_back(step_data_msg);


  //StepData 3 kick - 1st : raise foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0;

  //std::cout << "kick_height00 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.z += kick_height_m_;
  step_data_msg.position_data.right_foot_pose.pitch = 0.0;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);
  //std::cout << "kick_height11 :::::::: "<< kick_height_m_ << std::endl;

  //StepData 4 kick - 2nd : move right foot back
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += y_kick_front_time;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = y_kick_front_x_m;
  step_data_msg.position_data.right_foot_pose.y -= y_kick_front_y_m;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);


  //StepData 5 kick - 3rd : kick
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += y_kick_time;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.y += y_kick_front_y_m;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 6 kick - 4th : move right foot to x0
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = 0;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 7 move right foot to original place
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
  step_data_msg.time_data.dsp_ratio = 0;

  //std::cout << "kick_height22 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = 0;
  step_data_msg.position_data.right_foot_pose.z -= kick_height_m_;
  step_data_msg.position_data.right_foot_pose.pitch = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "kick_height33 :::::::: "<< kick_height_m_ << std::endl;
  //StepData 8 End
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.body_z_swap = 0;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "FOOT STEP PARAM" << std::endl;
  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);


    //ROS_INFO("idx : %d    foot:  %d",stp_idx,step_data_array_[stp_idx].position_data.moving_foot);
    //ROS_INFO("right %f   %f   %f  ",step_data_array_[stp_idx].position_data.right_foot_pose.x,step_data_array_[stp_idx].position_data.right_foot_pose.y ,step_data_array_[stp_idx].position_data.right_foot_pose.yaw );
    //ROS_INFO("right %f   %f   %f",step_data_array_[stp_idx].position_data.left_foot_pose.x,step_data_array_[stp_idx].position_data.left_foot_pose.y ,step_data_array_[stp_idx].position_data.left_foot_pose.yaw );

    //std::cout << "++++++++"<< stp_idx<< "+++++++++++" << std::endl;
    //std::cout << step_data_array_[stp_idx] << std::endl;
  }
  //std::cout << "------------------------------------" << std::endl;
}

void FootStepGenerator::calcYLeftKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const alice_walking_module_msgs::StepData& ref_step_data)
{

  alice_walking_module_msgs::StepData step_data_msg;
  step_data_msg = ref_step_data;

  step_data_array->clear();
  step_data_array_.clear();

  //Start 1 Step Data
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1;//0.1; //kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = kick_dsp_ratio_;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 2 move zmp to left foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_array_.push_back(step_data_msg);


  //StepData 3 kick - 1st : raise foot
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0;

  //std::cout << "kick_height00 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.z += kick_height_m_;
  step_data_msg.position_data.left_foot_pose.pitch = 0.0;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "kick_height11 :::::::: "<< kick_height_m_ << std::endl;

  //StepData 4 kick - 2nd : move right foot back
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = y_kick_front_x_m;
  step_data_msg.position_data.left_foot_pose.y += y_kick_front_y_m;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);


  //StepData 5 kick - 3rd : kick
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += y_kick_time;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.y -= y_kick_front_y_m;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 6 kick - 4th : move right foot to x0
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = 0;
  step_data_msg.position_data.left_foot_pose.pitch = 0;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 7 move right foot to original place
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
  step_data_msg.time_data.dsp_ratio = 0;


  //std::cout << "kick_height22 :::::::: "<< kick_height_m_ << std::endl;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = 0;
  step_data_msg.position_data.left_foot_pose.z -= kick_height_m_;
  step_data_msg.position_data.left_foot_pose.pitch = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "kick_height33 :::::::: "<< kick_height_m_ << std::endl;


  //StepData 8 End
  step_data_msg.time_data.walking_state = alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.body_z_swap = 0;
  step_data_msg.position_data.moving_foot = alice_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  //std::cout << "FOOT STEP PARAM" << std::endl;

  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);
    //ROS_INFO("idx : %d    foot:  %d",stp_idx,step_data_array_[stp_idx].position_data.moving_foot);
    //   ROS_INFO("right %f   %f   %f",step_data_array_[stp_idx].position_data.right_foot_pose.x,step_data_array_[stp_idx].position_data.right_foot_pose.y ,step_data_array_[stp_idx].position_data.right_foot_pose.yaw );
    //   ROS_INFO("right %f   %f   %f",step_data_array_[stp_idx].position_data.left_foot_pose.x,step_data_array_[stp_idx].position_data.left_foot_pose.y ,step_data_array_[stp_idx].position_data.left_foot_pose.yaw );

    //std::cout << "++++++++"<< stp_idx<< "+++++++++++" << std::endl;
    //std::cout << step_data_array_[stp_idx] << std::endl;

  }
  //std::cout << "------------------------------------" << std::endl;
}
