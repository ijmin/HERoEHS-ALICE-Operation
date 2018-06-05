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
#include "alice_foot_step_generator/FootStepCommand.h"
#include "alice_msgs/ForceTorque.h"
#include "alice_msgs/BalanceParam.h"
#include "alice_walking_module_msgs/SetBalanceParam.h"
#include "alice_walking_module_msgs/SetJointFeedBackGain.h"


#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
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

	//test
	ros::Publisher ball_tracking_pub;


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
	ros::ServiceClient set_balance_param_client;
	ros::ServiceClient joint_feedback_gain_client;


	/*****************************************************************************
	 ** module on off
	 *****************************************************************************/
	ros::Publisher module_on_off; // 모듈 on off
	ros::Publisher init_pose_pub;
	std_msgs::String init_pose_msg;
	ros::Publisher alice_ft_init_pub;
	ros::Subscriber alice_ft_init_done_sub;
	void aliceFtInitDoneMsgCallback(const std_msgs::Bool::ConstPtr& msg);
	bool ft_init_done_check;

	/*****************************************************************************
	 **desired pose
	 *****************************************************************************/
	ros::Publisher desired_pose_waist_pub;
	ros::Publisher desired_pose_head_pub;
	ros::Publisher desired_pose_arm_pub;


	/*****************************************************************************
	 ** control
	 *****************************************************************************/
	ros::Publisher alice_balance_parameter_pub;

	/*****************************************************************************
	 ** graph
	 *****************************************************************************/
	//subscriber
	ros::Subscriber joint_goal_state_sub;
	ros::Subscriber joint_present_state_sub;
	ros::Subscriber alice_force_torque_data_sub;
	ros::Subscriber zmp_fz_sub;




	double currentForceX_l_gui, currentForceY_l_gui, currentForceZ_l_gui;
	double currentForceX_r_gui, currentForceY_r_gui, currentForceZ_r_gui;
	double currentTorqueX_l_gui, currentTorqueY_l_gui, currentTorqueZ_l_gui;
	double currentTorqueX_r_gui, currentTorqueY_r_gui, currentTorqueZ_r_gui;

	double current_zmp_fz_x, current_zmp_fz_y;
	double reference_zmp_fz_x, reference_zmp_fz_y;

	std::map<int, std::string>      joint_index_to_name;
	std::map<std::string, double>   joint_name_to_goal;
	std::map<std::string, double>   joint_name_to_present;





	Q_SIGNALS:
	void loggingUpdated();
	void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	QStringListModel logging_model;
	void forceTorqueDataMsgCallback(const alice_msgs::ForceTorque::ConstPtr& msg);
	void goalJointStateMsgCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void presentJointStateMsgCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void zmpFzMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
};

}  // namespace offset_tuner_operation

#endif /* alice_gui_QNODE_HPP_ */
