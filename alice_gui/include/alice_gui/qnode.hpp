/**
 * @file /include/offset_tuner_operation/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef alice_gui_QNODE_HPP_
#define alice_gui_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
/*****************************************************************************
 ** Message
 *****************************************************************************/
#include "offset_tuner_msgs/JointOffsetState.h"
#include "offset_tuner_msgs/JointTorqueOnOffData.h"
#include "offset_tuner_msgs/JointTorqueOnOff.h"
#include "offset_tuner_msgs/JointTorqueOnOffArray.h"

#include "offset_tuner_msgs/PresentJointStateData.h"
#include "offset_tuner_msgs/PresentJointStateArray.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "alice_foot_step_generator/FootStepCommand.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace alice {

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread {
	Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	 ** Logging
	 **********************/
	enum LogLevel {
		Debug,
		Info,
		Warn,
		Error,
		Fatal
	};

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	void MovingStateMsgsCallBack(const std_msgs::Bool::ConstPtr& msg);

	/*****************************************************************************
	 ** dynamixel test
	 *****************************************************************************/
	//publisher
	ros::Publisher joint_offset_state_pub;
	ros::Publisher command_state_pub;
	ros::Publisher enable_module_pub;


	//subscriber
	ros::Subscriber moving_state_sub;

	//service client
	ros::ServiceClient joint_torque_on_off_cl;
	ros::ServiceClient joint_torque_on_off_array_cl;
	ros::ServiceClient present_joint_state_array_cl;

	//message
	offset_tuner_msgs::JointOffsetState      joint_offset_state_msg;
	offset_tuner_msgs::JointTorqueOnOffData  joint_torque_on_off_data;
	offset_tuner_msgs::JointTorqueOnOff      joint_torque_on_off_srv;
	offset_tuner_msgs::JointTorqueOnOffArray joint_torque_on_off_array_srv;

	offset_tuner_msgs::PresentJointStateArray present_joint_state_array_srv;
	offset_tuner_msgs::PresentJointStateData  present_joint_state_data;

	std_msgs::String command_state_msg;
	std_msgs::String enable_module_msg;

	/*****************************************************************************
	 ** walking test
	 *****************************************************************************/
	ros::Publisher foot_step_command_pub;
	//ros::ServiceClient set_balance_param_client;
	//ros::ServiceClient joint_feedback_gain_client;

	ros::Publisher module_on_off; // 모듈 on off


	Q_SIGNALS:
	void loggingUpdated();
	void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	QStringListModel logging_model;
};

}  // namespace offset_tuner_operation

#endif /* alice_gui_QNODE_HPP_ */
