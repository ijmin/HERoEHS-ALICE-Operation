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
    else count += 1;
    usleep(1000);
  }
  //ros::spin();


  return 0;
}


