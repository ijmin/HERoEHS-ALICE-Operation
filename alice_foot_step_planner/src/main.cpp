/*
 * main.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: robotemperor
 */

#include "alice_foot_step_planner/alice_foot_step_planner.h"

using namespace alice;

FootStepPlanner *foot_step_planner;

int main( int argc , char **argv )
{
	foot_step_planner = new alice::FootStepPlanner;
	ros::init( argc , argv , "alice_foot_step_planner" );

	foot_step_planner->initialize();

	while(ros::ok())
	{
		ros::spinOnce();
		foot_step_planner->on_process_pub.publish( foot_step_planner->on_process_msg);
		usleep(100);
	}


	return 0;
}


