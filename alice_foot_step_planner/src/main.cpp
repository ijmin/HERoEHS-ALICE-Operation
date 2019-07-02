/*
 * main.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: robotemperor
 */

#include "alice_foot_step_planner/alice_foot_step_planner.h"
#include "alice_online_walking_module/online_walking_module.h"

using namespace alice;

FootStepPlanner *foot_step_planner;

int main( int argc , char **argv )
{
  double count = 0;
  double current_stop_time = 0;
  bool command_lock = false;
  foot_step_planner = new alice::FootStepPlanner;
  ros::init( argc , argv , "alice_foot_step_planner" );

  foot_step_planner->initialize();

  while(ros::ok())
  {
    ros::spinOnce();
    if(count > 1000*0.8)
    {
      foot_step_planner->command_interval_check = 1;
      count = 0;
    }
    else
    {
      count += 1;
    }

    if(current_stop_time > 1000*90)
    {
      foot_step_planner-> foot_set_command_msg.command = "stop";
      foot_step_planner->foot_step_command_pub.publish(foot_step_planner->foot_set_command_msg);
      foot_step_planner->command_interval_check = 0;
      current_stop_time = 0;
    }
    else
    {
      //foot_step_planner->command_interval_check = 1;
      current_stop_time += 1;
    }

    usleep(1000);
  }
  //ros::spin();


  return 0;
}


