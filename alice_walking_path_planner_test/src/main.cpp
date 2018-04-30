/*
 * main.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: robotemperor
 */

#include "alice_walking_path_planner_test/alice_walking_path_planner_test.h"

int main( int argc , char **argv )
{

	ros::init( argc , argv , "alice_walking_path_planner_test" );

	ros::NodeHandle nh;

	global_frame_xy_pub = nh.advertise<alice_operation_msgs::WalkingPathPlanner>("/heroehs/alice_walking_path_planner_test", 1);


	alice_operation_msgs::WalkingPathPlanner msg;


	while(ros::ok())
	{
		printf("Insert dummy X: \n");
		scanf("%lf", &x_data);
		printf("Insert dummy Y: \n");
		scanf("%lf", &y_data);
		printf("Insert dummy Theta: \n");
		scanf("%lf", &yaw_degree);
		printf("Insert dummy command: \n");
		scanf("%d", &command);


		switch(command)
		{
		case 8:
			command_str = "forward";
			break;
		case 5:
			command_str = "stop";
			break;
		case 4:
			command_str = "left";
			break;
		case 6:
			command_str = "right";
			break;
		case 2:
			command_str = "backward";
			break;
		case 7:
			command_str = "turn left";
			break;
		case 9:
			command_str = "turn right";
			break;
		}

		printf("%s", command_str.c_str());
		msg.position_x = x_data;
		msg.position_y = y_data;
		msg.yaw_degree = yaw_degree;
		msg.command = command_str;

		global_frame_xy_pub.publish(msg);
		ros::spinOnce();
	}



	printf("END: \n");
	return 0;
}


