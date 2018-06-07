/*
 * main.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#include "alice_foot_step_generator/message_callback.h"

int main( int argc , char **argv )
{
    ros::init( argc , argv , "alice_foot_step_generator" );

    ROS_INFO("ALICE FOOT STEP GENERATOR IS EXCUTED");

    initialize();

    ros::spin();
    return 0;
}



