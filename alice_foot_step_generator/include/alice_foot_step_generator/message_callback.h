/*
 * message_callback.h
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_GENERATOR_INCLUDE_ALICE_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_
#define HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_GENERATOR_INCLUDE_ALICE_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>


#include "alice_foot_step_generator/FootStepCommand.h"
#include "alice_foot_step_generator/Step2DArray.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "alice_walking_module_msgs/RobotPose.h"
#include "alice_walking_module_msgs/GetReferenceStepData.h"
#include "alice_walking_module_msgs/AddStepDataArray.h"
#include "alice_walking_module_msgs/StartWalking.h"
#include "alice_walking_module_msgs/SetBalanceParam.h"
#include "alice_walking_module_msgs/IsRunning.h"
#include "alice_walking_module_msgs/RemoveExistingStepData.h"


#include "alice_foot_step_generator_node.h"


void initialize(void);

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

void walkingCommandCallback(const alice_foot_step_generator::FootStepCommand::ConstPtr& msg);
void step2DArrayCallback(const alice_foot_step_generator::Step2DArray::ConstPtr& msg);

void dspCallback(const std_msgs::Float64::ConstPtr& msg);
void footZSwapCallback(const std_msgs::Float64::ConstPtr& msg);
void bodyZSwapCallback(const std_msgs::Float64::ConstPtr& msg);
void yZMPConvergenceCallback(const std_msgs::Float64::ConstPtr& msg);

bool isRunning(void);




#endif /* HEROEHS_ALICE_OPERATION_ALICE_FOOT_STEP_GENERATOR_INCLUDE_ALICE_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_ */
