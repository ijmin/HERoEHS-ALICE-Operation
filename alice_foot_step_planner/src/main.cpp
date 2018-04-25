/*
 * main.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: robotemperor
 */

#include "alice_foot_step_planner/alice_foot_step_planner.h"

using namespace alice;


int main( int argc , char **argv )
{
    ros::init( argc , argv , "alice_foot_step_planner" );

    initialize();

    ros::spin();
    return 0;
}


