/*
 * alice_walking_path_planner_test.h
 *
 *  Created on: Apr 27, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ALICE_OPERATION_ALICE_WALKING_PATH_PLANNER_TEST_INCLUDE_ALICE_WALKING_PATH_PLANNER_TEST_ALICE_WALKING_PATH_PLANNER_TEST_H_
#define HEROEHS_ALICE_OPERATION_ALICE_WALKING_PATH_PLANNER_TEST_INCLUDE_ALICE_WALKING_PATH_PLANNER_TEST_ALICE_WALKING_PATH_PLANNER_TEST_H_


#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include "alice_operation_msgs/WalkingPathPlanner.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"
#include <math.h>
#include <cmath>
#include <stdio.h>

double x_data, y_data;
double yaw_degree;
int command;
std::string command_str;

ros::Publisher  global_frame_xy_pub;

#endif /* HEROEHS_ALICE_OPERATION_ALICE_WALKING_PATH_PLANNER_TEST_INCLUDE_ALICE_WALKING_PATH_PLANNER_TEST_ALICE_WALKING_PATH_PLANNER_TEST_H_ */
