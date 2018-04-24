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
#include "alice_walking_module_msgs/AddStepDataArray.h"
#include "alice_foot_step_generator/Step2DArray.h"

#define STOP_WALKING           (0)
#define FORWARD_WALKING        (1)
#define BACKWARD_WALKING       (2)
#define RIGHTWARD_WALKING      (3)
#define LEFTWARD_WALKING       (4)
#define LEFT_ROTATING_WALKING  (5)
#define RIGHT_ROTATING_WALKING (6)

#define MINIMUM_STEP_TIME_SEC  (0.4)

namespace alice
{

class FootStepGenerator
{
public:
  FootStepGenerator();
  ~FootStepGenerator();

  void initialize();

  void getStepData(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data,
      int desired_step_type);

  void getStepDataFromStepData2DArray(alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const alice_walking_module_msgs::StepData& ref_step_data,
      const alice_foot_step_generator::Step2DArray::ConstPtr& request_step_2d);

  int    num_of_step_;
  double fb_step_length_m_;
  double rl_step_length_m_;
  double rotate_step_angle_rad_;

  double step_time_sec_;
  double start_end_time_sec_;
  double dsp_ratio_;

  double foot_z_swap_m_;
  double body_z_swap_m_;

  double default_y_feet_offset_m_;

private:
  bool calcStep(const alice_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type);

  void calcFBStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRLStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRoStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcStopStep(const alice_walking_module_msgs::StepData& ref_step_data, int direction);

  Eigen::MatrixXd getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw);
  void getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform, double *position_x, double *position_y, double *position_z, double *roll, double *pitch, double *yaw);
  alice_walking_module_msgs::PoseXYZRPY getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform);
  Eigen::MatrixXd getInverseTransformation(Eigen::MatrixXd transform);

  alice_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type step_data_array_;

  int previous_step_type_;

};


}




#endif /* HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_GENERATOR_INCLUDE_ALICE_FOOT_STEP_GENERATOR_ALICE_FOOT_STEP_GENERATOR_NODE_H_ */
