/**
 * @file /include/offset_tuner_operation/main_window.hpp
 *
 * @brief Qt based gui for offset_tuner_operation.
 *
 * @date November 2010
 **/
#ifndef alice_gui_MAIN_WINDOW_H
#define alice_gui_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <QString>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "alice_foot_step_generator/FootStepCommand.h"
#endif

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace alice{

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void closeEvent(QCloseEvent *event);
	void showNoMasterMessage();

	public Q_SLOTS:
	void updateLoggingView(); // no idea why this can't connect automatically
	void realtimeDataSlot();
	void graph_draw(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count);
	void graph_draw_update(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ);
	void graph_draw_sensor(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count);
	void graph_draw_sensor_update(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ);
	void graph_draw_clean(QCustomPlot *ui_graph);



	// <------------------------------------------------------------------- dynamixel offset -->
	void on_update_button_clicked();
	void on_all_torque_on_button_clicked();
	void on_all_torque_off_button_clicked();

	void on_initial_pose_button_clicked();
	void on_pre_offset_pose_button_clicked();
	void on_save_button_clicked();
	void on_base_module_button_clicked();

	void offset_change_button();
	void offset_goal_value_changed_function();
	void offset_torque_state_changed_function();

	void change_button(QString id_string, std::string type);
	void goal_value_changed_function(QString id_string, double spin_value);
	void torque_state_changed_function(QString id_string, bool check);
	void torque_state(bool state);


	//<------------------------------------------------------------------- walking test -->
	void on_online_walking_module_clicked();
	void on_none_clicked();
	////command
	void on_turn_left_clicked();
	void on_turn_right_clicked();

	void on_left_clicked();
	void on_right_clicked();

	void on_forward_clicked();
	void on_backward_clicked();

	void on_stop_clicked();
	//parameter
	void on_apply_data_clicked();
	void on_joint_feedback_gain_clicked();
	void on_balance_param_apply_clicked();

	//<------------------------------------------------------------------- module on off -->
	//<------------------------------------------------------------------- base_module-->
	void on_base_module_real_button_clicked();
	void on_initial_pose_real_button_clicked();
	void on_initialize_ft_sensor_button_clicked();
	//<------------------------------------------------------------------- leg_module-->
	void on_alice_leg_module_button_clicked();

	//<------------------------------------------------------------------- upper_body_module-->
	void on_upper_body_module_button_clicked();
	void on_waist_change_button_clicked();
	void on_head_change_button_clicked();

	//<------------------------------------------------------------------- arm_body_module-->
	void on_arm_module_button_clicked();
	void on_arm_change_end_effector_button_clicked();

	//<------------------------------------------------------------------- control on off-->
	void on_zmp_on_clicked();
	void on_zmp_off_clicked();

	void on_tracking_on_clicked();
	void on_tracking_off_clicked();

	//<------------------------------------------------------------------- graph -->
	void on_stop_button_clicked();
	void on_start_button_clicked();
	void on_joint_state_init_button_clicked();


	private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	void joint_data_parse(const std::string &path);
	std_msgs::String module_msg; // module on off command msg
	// graph variables
	QTimer *dataTimer;
	QFont legendFont;
	double key;


	// <------------------------------------------------------------------- dynamixel offset -->

	std::map<std::string, int> joint_name_to_id;
	std::map<int, double>      joint_id_to_goal;
	std::map<int, std::string> joint_id_to_name;

	std::map<std::string, bool>   joint_name_to_torque_state;
	std::map<std::string, double> joint_name_to_present_value;
	std::map<std::string, double> joint_name_to_offset_data;

	//<------------------------------------------------------------------- walking test -->

	alice_foot_step_generator::FootStepCommand foot_step_command_msg;
	alice_walking_module_msgs::SetBalanceParam set_balance_param_msg;
	alice_walking_module_msgs::SetJointFeedBackGain joint_feedback_gain_msg;

	int pose_num;




	//<------------------------------------------------------------------- module -->
	//<------------------------------------------------------------------- base_module-->
	std_msgs::Bool ft_init_msg;
	QString temp_check_state;

	//<------------------------------------------------------------------- leg_module-->


	//<------------------------------------------------------------------- desired_pose-->
	std_msgs::Float64MultiArray desired_pose_waist_msg; // desired_pose command msg
	std_msgs::Float64MultiArray desired_pose_head_msg; // desired_pose command msg
	std_msgs::Float64MultiArray desired_pose_arm_msg; // desired_pose command msg

	//<------------------------------------------------------------------- control -->
	void parse_gain_data();
	alice_msgs::BalanceParam alice_balance_parameter_msg;
	double foot_zmpFz_p_gain;
	double foot_zmpFz_d_gain;
	double updating_duration;

	double cob_x_offset_m;
	double cob_y_offset_m;

	double foot_roll_gyro_p_gain;
	double foot_roll_gyro_d_gain;
	double foot_pitch_gyro_p_gain;
	double foot_pitch_gyro_d_gain;

	//ball tracking
	void parse_gain_tracking_data();
	std_msgs::Float64MultiArray tracking_param_msg;

	//<------------------------------------------------------------------- graph -->
	void check_sensor_menu();
	void select_joint_state();

};

}  // namespace offset_tuner_operation

#endif // alice_gui_MAIN_WINDOW_H
