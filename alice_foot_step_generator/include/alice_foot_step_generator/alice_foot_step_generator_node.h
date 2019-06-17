/*
 * alice_foot_step_generator_node.h
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_GENERATOR_INCLUDE_ALICE_FOOT_STEP_GENERATOR_ALICE_FOOT_STEP_GENERATOR_NODE_H_
#define HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_GENERATOR_INCLUDE_ALICE_FOOT_STEP_GENERATOR_ALICE_FOOT_STEP_GENERATOR_NODE_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <yaml-cpp/yaml.h>  // above scilab!!!! or error!!
#include <ros/package.h>

#include "robotis_math/robotis_math.h"

#include "alice_walking_module_msgs/AddStepDataArray.h"
#include "alice_foot_step_generator/Step2DArray.h"

#define STOP_WALKING           (0)
#define FORWARD_WALKING        (1)
#define BACKWARD_WALKING       (2)
#define RIGHTWARD_WALKING      (3)
#define LEFTWARD_WALKING       (4)
#define LEFT_ROTATING_WALKING  (5)
#define RIGHT_ROTATING_WALKING (6)
#define REVOLUTE_LEFT_WALKING  (7)
#define REVOLUTE_RIGHT_WALKING (8)

#define centered (1)
#define expanded (2)

#define MINIMUM_STEP_TIME_SEC  (0.4)

namespace alice
{

class FootStepGenerator
{
public:
  FootStepGenerator();
  ~FootStepGenerator();

  void readFootStep_Yaml();
  double leg_offset_;
  double foot_offset_yaw_;

  void initialize();

  void getStepData(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data,
      int desired_step_type,
      int desired_step_type_num);

  void getStepDataFromStepData2DArray(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data,
      const alice_foot_step_generator::Step2DArray::ConstPtr& request_step_2d);

  void calcRightKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data);
  void calcLeftKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data);

  void calcTurnLeftAndRightKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data);
  void calcTurnRightAndLeftKickStep(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data);


  int    num_of_step_;
  double fb_step_length_m_;
  double rl_step_length_m_;
  double rotate_step_angle_rad_;
  double step_time_sec_;

  double ep_step_length_m_;
  double eps_step_length_m_;
  double ep_step_angle_rad_;
  double ep_step_time_sec_;

  double ct_step_length_m_;
  double cts_step_length_m_;
  double ct_step_angle_rad_;
  double ct_step_time_sec_;

  double dsp_ratio_;
  double foot_z_swap_m_;
  double body_z_swap_m_;

  double y_zmp_convergence_m_;

  double start_end_time_sec_;
  double default_y_feet_offset_m_;
  double defalut_yaw_feet_offset_rad_;

  double kick_height_m_;
  double kick_far_m_;
  double kick_pitch_rad_;
  double kick_time_sec_;

  int previous_step_type_;
  int revolute_type_;

private:
  bool calcStep(const alice_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type, int desired_step_type_num);

  void calcFBStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRLStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRoStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRevRLStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction, int desired_step_type_num);

  void calcStopStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);

  Eigen::MatrixXd getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw);
  void getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform, double *position_x, double *position_y, double *position_z, double *roll, double *pitch, double *yaw);
  alice_walking_module_msgs::PoseXYZRPY getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform);
  Eigen::MatrixXd getInverseTransformation(Eigen::MatrixXd transform);

  alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type step_data_array_;


};


}




#endif /* HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_GENERATOR_INCLUDE_ALICE_FOOT_STEP_GENERATOR_ALICE_FOOT_STEP_GENERATOR_NODE_H_ */
