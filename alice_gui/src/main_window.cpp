/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include "../include/alice_gui/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace alice{

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent)
, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	ui.setting_comboBox->clear();
	ui.setting_comboBox->addItem("setting clear");
        ui.setting_comboBox->addItem("setting default");
	ui.setting_comboBox->addItem("setting expanded");
	ui.setting_comboBox->addItem("setting centered");
	qnode.init();
	setWindowIcon(QIcon(":/images/heroes.png"));
	QPixmap pix(":/images/Alice.png");
	alice_scene.addPixmap(pix);
	ui.alice_graphicsView->setScene(&alice_scene);
	ui.alice_graphicsView->show();
	QPixmap pix2(":/images/Alice_leg.png");
	alice_leg_scene.addPixmap(pix2);
	ui.alice_leg_graphicsView->setScene(&alice_leg_scene);
	ui.alice_leg_graphicsView->show();
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	/****************************
	 ** Connect
	 ****************************/
	// offset tab
	QObject::connect(ui.goal_spin_box, SIGNAL(valueChanged(QString)), this, SLOT(offset_goal_value_changed_function()));
	QObject::connect(ui.torque_state_check_box, SIGNAL(clicked()), this, SLOT(offset_torque_state_changed_function()));
	QObject::connect(ui.change_button, SIGNAL(clicked()), this, SLOT(offset_change_button()));
	// ALICE ID	


	/*********************
	 ** Logging
	 **********************/
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

	/*********************
	 ** Graph
	 **********************/

	legendFont = font();  // start out with MainWindow's font..
	graph_draw(ui.state_plot, "Goal / Present", "Rad", -1, 1 , 10);
	graph_draw_sensor(ui.sensor_plot_1, "Left Sensor", "m", -3, 3, 10);
	graph_draw_sensor(ui.sensor_plot_2, "Right Sensor", "m", -3, 3, 10);
	graph_draw_none_line(ui.zmp_graph, "         ZMP FZ", "m", -0.3, 0.3, -0.3, 0.3, 10);

	graph_draw_none_line(ui.ground_graph, "         MAP", "m", -5, 5, -5, 5, 10);
	ui.ground_graph->xAxis->setLabel("X  m");
	ui.ground_graph->yAxis->setLabel("Y  m");
	ground_map = new QCPItemRect(ui.ground_graph);
	draw_ractangle(ui.ground_graph, ground_map, "ground");
	ground_map->topLeft->setCoords(-4.5,-3);
	ground_map->bottomRight->setCoords(4.5,3);  //reverse !!

	//change_ractangle(ground, , qnode.lf_point_y);


	foot_left = new QCPItemRect(ui.zmp_graph);
	foot_right = new QCPItemRect(ui.zmp_graph);
	draw_ractangle(ui.zmp_graph, foot_left, "left");
	draw_ractangle(ui.zmp_graph, foot_right, "right");


	dataTimer = new QTimer(this);
	connect(dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
	dataTimer->start(0.006); // Interval 0 means to refresh as fast as possible

	/****************************
	 ** Initialize  load joint data
	 ****************************/
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	init_pose_path = ros::package::getPath("alice_gui") + "/config/joint_data.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	joint_data_parse(init_pose_path);
	ui.id_line_edit->setText("0");
	pose_num = 0;
	foot_zmpFz_p_gain = 0;
	foot_zmpFz_d_gain = 0;
	updating_duration = 0;
}
MainWindow::~MainWindow() {}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
	close();
}

void MainWindow::on_ALICE_ID_1_Button_clicked(){
	ALICE_ID = 1;
	std::string balance_path_;
	balance_path_ = ros::package::getPath("alice_gui") + "/config/balance_param1_on.yaml";
	parse_balance_param_data(balance_path_);
	ui.balance_cob_x_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.cob_x_offset_m));
	ui.balance_cob_y_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.cob_y_offset_m));
	ui.balance_hip_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.hip_roll_swap_angle_rad));

	ui.balance_duration_value->setText(QString("%1").arg(set_balance_param_msg.request.updating_duration));

	ui.balance_gyro_rp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_gyro_p_gain));
	ui.balance_gyro_rd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_gyro_d_gain));
	ui.balance_gyro_pp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_gyro_p_gain));
	ui.balance_gyro_pd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_gyro_d_gain));
	ui.balance_gyro_rc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.roll_gyro_cut_off_frequency));
	ui.balance_gyro_pc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.pitch_gyro_cut_off_frequency));

	ui.balance_torque_rp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_torque_p_gain));
	ui.balance_torque_rd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_torque_d_gain));
	ui.balance_torque_pp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_torque_p_gain));
	ui.balance_torque_pd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_torque_d_gain));
	ui.balance_torque_rc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_torque_cut_off_frequency));
	ui.balance_torque_pc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_torque_cut_off_frequency));

	ui.balance_imu_rp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_angle_p_gain));
	ui.balance_imu_rd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_angle_d_gain));
	ui.balance_imu_pp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_angle_p_gain));
	ui.balance_imu_pd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_angle_d_gain));
        ui.balance_imu_rc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.roll_angle_cut_off_frequency));
        ui.balance_imu_pc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.pitch_angle_cut_off_frequency));

	ui.balance_force_xp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_x_force_p_gain));
	ui.balance_force_xd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_x_force_d_gain));
	ui.balance_force_yp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_y_force_p_gain));
	ui.balance_force_yd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_y_force_d_gain));
	ui.balance_force_zp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_z_force_p_gain));
	ui.balance_force_zd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_z_force_d_gain));
	ui.balance_force_xc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_x_force_cut_off_frequency));
	ui.balance_force_yc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_y_force_cut_off_frequency));
	ui.balance_force_zc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_z_force_cut_off_frequency));


	std::string joint_path_;
	joint_path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain1_on.yaml";
	parse_joint_feed_back_param_data(joint_path_);
	ui.joint_r_leg_hip_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain));
	ui.joint_r_leg_hip_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain));
	ui.joint_r_leg_hip_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain));
	ui.joint_r_leg_hip_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain));
	ui.joint_r_leg_hip_y_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain));
	ui.joint_r_leg_hip_y_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain));

	ui.joint_r_leg_kn_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_p_gain));
	ui.joint_r_leg_kn_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_d_gain));

	ui.joint_r_leg_an_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain));
	ui.joint_r_leg_an_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain));
	ui.joint_r_leg_an_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain));
	ui.joint_r_leg_an_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain));

	ui.joint_l_leg_hip_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain));
	ui.joint_l_leg_hip_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain));
	ui.joint_l_leg_hip_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain));
	ui.joint_l_leg_hip_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain));
	ui.joint_l_leg_hip_y_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain));
	ui.joint_l_leg_hip_y_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain));

	ui.joint_l_leg_kn_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_p_gain));
	ui.joint_l_leg_kn_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_d_gain));

	ui.joint_l_leg_an_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain));
	ui.joint_l_leg_an_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain));
	ui.joint_l_leg_an_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain));
	ui.joint_l_leg_an_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain));

	ui.joint_updating_duration->setText(QString("%1").arg(joint_feedback_gain_msg.request.updating_duration));
	

	std::string step_path_;
	step_path_ = ros::package::getPath("alice_gui") + "/config/step_parameter1.yaml";
	parse_step_param_data(step_path_);
	ui.default_step_num->setText(QString("%1").arg(default_step_num));
	ui.default_step_length->setText(QString("%1").arg(default_step_length));
	ui.default_side_step_length->setText(QString("%1").arg(default_side_step_length));
	ui.default_step_angle_radian->setText(QString("%1").arg(default_step_angle_radian));
	ui.default_step_time->setText(QString("%1").arg(default_step_time));

	ui.expanded_step_num->setText(QString("%1").arg(expanded_step_num));
	ui.expanded_step_length->setText(QString("%1").arg(expanded_step_length));
	ui.expanded_side_step_length->setText(QString("%1").arg(expanded_side_step_length));
	ui.expanded_step_angle_radian->setText(QString("%1").arg(expanded_step_angle_radian));
	ui.expanded_step_time->setText(QString("%1").arg(expanded_step_time));

	ui.centered_step_num->setText(QString("%1").arg(centered_step_num));
	ui.centered_step_length->setText(QString("%1").arg(centered_step_length));
	ui.centered_side_step_length->setText(QString("%1").arg(centered_side_step_length));
	ui.centered_step_angle_radian->setText(QString("%1").arg(centered_step_angle_radian));
	ui.centered_step_time->setText(QString("%1").arg(centered_step_time));

	ui.foot_step_dsp->setText(QString("%1").arg(foot_step_dsp));
	ui.foot_step_foot_z_swap->setText(QString("%1").arg(foot_step_foot_z_swap));
	ui.foot_step_body_z_swap->setText(QString("%1").arg(foot_step_body_z_swap));
	ui.foot_step_y_zmp_conv->setText(QString("%1").arg(foot_step_y_zmp_conv));

	ui.expanded_left->setEnabled(1);
	ui.expanded_right->setEnabled(1);
	ui.centered_left->setEnabled(1);
	ui.centered_right->setEnabled(1);
	qnode.alice_id_msgs.data = "1";
	qnode.alice_id_pub.publish(qnode.alice_id_msgs);
	
}
void MainWindow::on_ALICE_ID_2_Button_clicked(){
	ALICE_ID = 2;
	std::string balance_path_;
	balance_path_ = ros::package::getPath("alice_gui") + "/config/balance_param2_on.yaml";
	parse_balance_param_data(balance_path_);
	ui.balance_cob_x_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.cob_x_offset_m));
	ui.balance_cob_y_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.cob_y_offset_m));
	ui.balance_hip_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.hip_roll_swap_angle_rad));

	ui.balance_duration_value->setText(QString("%1").arg(set_balance_param_msg.request.updating_duration));

	ui.balance_gyro_rp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_gyro_p_gain));
	ui.balance_gyro_rd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_gyro_d_gain));
	ui.balance_gyro_pp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_gyro_p_gain));
	ui.balance_gyro_pd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_gyro_d_gain));
	ui.balance_gyro_rc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.roll_gyro_cut_off_frequency));
	ui.balance_gyro_pc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.pitch_gyro_cut_off_frequency));

	ui.balance_torque_rp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_torque_p_gain));
	ui.balance_torque_rd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_torque_d_gain));
	ui.balance_torque_pp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_torque_p_gain));
	ui.balance_torque_pd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_torque_d_gain));
	ui.balance_torque_rc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_torque_cut_off_frequency));
	ui.balance_torque_pc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_torque_cut_off_frequency));

	ui.balance_imu_rp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_angle_p_gain));
	ui.balance_imu_rd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_roll_angle_d_gain));
	ui.balance_imu_pp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_angle_p_gain));
	ui.balance_imu_pd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_pitch_angle_d_gain));
        ui.balance_imu_rc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.roll_angle_cut_off_frequency));
        ui.balance_imu_pc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.pitch_angle_cut_off_frequency));

	ui.balance_force_xp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_x_force_p_gain));
	ui.balance_force_xd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_x_force_d_gain));
	ui.balance_force_yp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_y_force_p_gain));
	ui.balance_force_yd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_y_force_d_gain));
	ui.balance_force_zp_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_z_force_p_gain));
	ui.balance_force_zd_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_z_force_d_gain));
	ui.balance_force_xc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_x_force_cut_off_frequency));
	ui.balance_force_yc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_y_force_cut_off_frequency));
	ui.balance_force_zc_value->setText(QString("%1").arg(set_balance_param_msg.request.balance_param.foot_z_force_cut_off_frequency));


	std::string joint_path_;
	joint_path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain2_on.yaml";
	parse_joint_feed_back_param_data(joint_path_);
	ui.joint_r_leg_hip_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain));
	ui.joint_r_leg_hip_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain));
	ui.joint_r_leg_hip_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain));
	ui.joint_r_leg_hip_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain));
	ui.joint_r_leg_hip_y_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain));
	ui.joint_r_leg_hip_y_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain));

	ui.joint_r_leg_kn_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_p_gain));
	ui.joint_r_leg_kn_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_d_gain));

	ui.joint_r_leg_an_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain));
	ui.joint_r_leg_an_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain));
	ui.joint_r_leg_an_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain));
	ui.joint_r_leg_an_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain));

	ui.joint_l_leg_hip_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain));
	ui.joint_l_leg_hip_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain));
	ui.joint_l_leg_hip_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain));
	ui.joint_l_leg_hip_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain));
	ui.joint_l_leg_hip_y_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain));
	ui.joint_l_leg_hip_y_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain));

	ui.joint_l_leg_kn_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_p_gain));
	ui.joint_l_leg_kn_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_d_gain));

	ui.joint_l_leg_an_p_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain));
	ui.joint_l_leg_an_p_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain));
	ui.joint_l_leg_an_r_p_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain));
	ui.joint_l_leg_an_r_d_gain->setText(QString("%1").arg(joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain));

	ui.joint_updating_duration->setText(QString("%1").arg(joint_feedback_gain_msg.request.updating_duration));


	std::string step_path_;
	step_path_ = ros::package::getPath("alice_gui") + "/config/step_parameter2.yaml";
	parse_step_param_data(step_path_);
	ui.default_step_num->setText(QString("%1").arg(default_step_num));
	ui.default_step_length->setText(QString("%1").arg(default_step_length));
	ui.default_side_step_length->setText(QString("%1").arg(default_side_step_length));
	ui.default_step_angle_radian->setText(QString("%1").arg(default_step_angle_radian));
	ui.default_step_time->setText(QString("%1").arg(default_step_time));

	ui.expanded_step_num->setText(QString("%1").arg(expanded_step_num));
	ui.expanded_step_length->setText(QString("%1").arg(expanded_step_length));
	ui.expanded_side_step_length->setText(QString("%1").arg(expanded_side_step_length));
	ui.expanded_step_angle_radian->setText(QString("%1").arg(expanded_step_angle_radian));
	ui.expanded_step_time->setText(QString("%1").arg(expanded_step_time));

	ui.centered_step_num->setText(QString("%1").arg(centered_step_num));
	ui.centered_step_length->setText(QString("%1").arg(centered_step_length));
	ui.centered_side_step_length->setText(QString("%1").arg(centered_side_step_length));
	ui.centered_step_angle_radian->setText(QString("%1").arg(centered_step_angle_radian));
	ui.centered_step_time->setText(QString("%1").arg(centered_step_time));

	ui.foot_step_dsp->setText(QString("%1").arg(foot_step_dsp));
	ui.foot_step_foot_z_swap->setText(QString("%1").arg(foot_step_foot_z_swap));
	ui.foot_step_body_z_swap->setText(QString("%1").arg(foot_step_body_z_swap));
	ui.foot_step_y_zmp_conv->setText(QString("%1").arg(foot_step_y_zmp_conv));

	ui.expanded_left->setEnabled(0);
	ui.expanded_right->setEnabled(0);
	ui.centered_left->setEnabled(0);
	ui.centered_right->setEnabled(0);
	qnode.alice_id_msgs.data = "2";
	qnode.alice_id_pub.publish(qnode.alice_id_msgs);
}
/*****************************************************************************
/*****************************************************************************
 ** dynamixel offset
 *****************************************************************************/
void MainWindow::on_balance_apply_button_clicked()
{
	set_balance_param_msg.request.updating_duration = ui.balance_duration_value->text().toDouble();

	set_balance_param_msg.request.balance_param.cob_x_offset_m = ui.balance_cob_x_value->text().toDouble();
	set_balance_param_msg.request.balance_param.cob_y_offset_m = ui.balance_cob_y_value->text().toDouble();
	set_balance_param_msg.request.balance_param.hip_roll_swap_angle_rad = ui.balance_hip_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_roll_gyro_p_gain = ui.balance_gyro_rp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_roll_gyro_d_gain = ui.balance_gyro_rd_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_p_gain = ui.balance_gyro_pp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_d_gain = ui.balance_gyro_pd_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_roll_angle_p_gain = ui.balance_imu_rp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_roll_angle_d_gain = ui.balance_imu_rd_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_p_gain = ui.balance_imu_pp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_d_gain = ui.balance_imu_pd_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_x_force_p_gain = ui.balance_force_xp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_x_force_d_gain = ui.balance_force_xd_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_y_force_p_gain = ui.balance_force_yp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_y_force_d_gain = ui.balance_force_yd_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_z_force_p_gain = ui.balance_force_zp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_z_force_d_gain = ui.balance_force_zd_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_roll_torque_p_gain = ui.balance_torque_rp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_roll_torque_d_gain = ui.balance_torque_rd_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_pitch_torque_p_gain = ui.balance_torque_pp_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_d_gain = ui.balance_torque_pd_value->text().toDouble();

	set_balance_param_msg.request.balance_param.roll_gyro_cut_off_frequency = ui.balance_gyro_rc_value->text().toDouble();
	set_balance_param_msg.request.balance_param.pitch_gyro_cut_off_frequency = ui.balance_gyro_pc_value->text().toDouble();

	set_balance_param_msg.request.balance_param.roll_angle_cut_off_frequency = ui.balance_imu_rc_value->text().toDouble();
	set_balance_param_msg.request.balance_param.pitch_angle_cut_off_frequency = ui.balance_imu_pc_value->text().toDouble();

	set_balance_param_msg.request.balance_param.foot_x_force_cut_off_frequency = ui.balance_force_xc_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_y_force_cut_off_frequency = ui.balance_force_yc_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_z_force_cut_off_frequency = ui.balance_force_zc_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_roll_torque_cut_off_frequency = ui.balance_torque_rc_value->text().toDouble();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_cut_off_frequency = ui.balance_torque_pc_value->text().toDouble();
}
void MainWindow::on_balance_save_button_clicked()
{
	YAML::Emitter out;
	std::string path_;
	if(ALICE_ID == 1) path_ = ros::package::getPath("alice_gui") + "/config/balance_param1_on.yaml";
	if(ALICE_ID == 2) path_ = ros::package::getPath("alice_gui") + "/config/balance_param2_on.yaml";

	out << YAML::BeginMap;
	out << YAML::Key << "cob_x_offset_m" << YAML::Value << ui.balance_cob_x_value->text().toDouble();
	out << YAML::Key << "cob_y_offset_m" << YAML::Value << ui.balance_cob_y_value->text().toDouble();
	out << YAML::Key << "hip_roll_swap_angle_rad" << YAML::Value << ui.balance_hip_value->text().toDouble();

	out << YAML::Key << "foot_roll_gyro_p_gain" << YAML::Value << ui.balance_gyro_rp_value->text().toDouble();
	out << YAML::Key << "foot_roll_gyro_d_gain" << YAML::Value << ui.balance_gyro_rd_value->text().toDouble();
	out << YAML::Key << "foot_pitch_gyro_p_gain" << YAML::Value << ui.balance_gyro_pp_value->text().toDouble();
	out << YAML::Key << "foot_pitch_gyro_d_gain" << YAML::Value << ui.balance_gyro_pd_value->text().toDouble();

	out << YAML::Key << "foot_roll_angle_p_gain" << YAML::Value << ui.balance_imu_rp_value->text().toDouble();
	out << YAML::Key << "foot_roll_angle_d_gain" << YAML::Value << ui.balance_imu_rd_value->text().toDouble();
	out << YAML::Key << "foot_pitch_angle_p_gain" << YAML::Value << ui.balance_imu_pp_value->text().toDouble();
	out << YAML::Key << "foot_pitch_angle_d_gain" << YAML::Value << ui.balance_imu_pd_value->text().toDouble();

	out << YAML::Key << "foot_x_force_p_gain" << YAML::Value << ui.balance_force_xp_value->text().toDouble();
	out << YAML::Key << "foot_x_force_d_gain" << YAML::Value << ui.balance_force_xd_value->text().toDouble();

	out << YAML::Key << "foot_y_force_p_gain" << YAML::Value << ui.balance_force_yp_value->text().toDouble();
	out << YAML::Key << "foot_y_force_d_gain" << YAML::Value << ui.balance_force_yd_value->text().toDouble();

	out << YAML::Key << "foot_z_force_p_gain" << YAML::Value << ui.balance_force_zp_value->text().toDouble();
	out << YAML::Key << "foot_z_force_d_gain" << YAML::Value << ui.balance_force_zd_value->text().toDouble();

	out << YAML::Key << "foot_roll_torque_p_gain" << YAML::Value << ui.balance_torque_rp_value->text().toDouble();
	out << YAML::Key << "foot_roll_torque_d_gain" << YAML::Value << ui.balance_torque_rd_value->text().toDouble();

	out << YAML::Key << "foot_pitch_torque_p_gain" << YAML::Value << ui.balance_torque_pp_value->text().toDouble();
	out << YAML::Key << "foot_pitch_torque_d_gain" << YAML::Value << ui.balance_torque_pd_value->text().toDouble();

	out << YAML::Key << "roll_gyro_cut_off_frequency" << YAML::Value << ui.balance_gyro_rc_value->text().toDouble();
	out << YAML::Key << "pitch_gyro_cut_off_frequency" << YAML::Value << ui.balance_gyro_pc_value->text().toDouble();

	out << YAML::Key << "roll_angle_cut_off_frequency" << YAML::Value << ui.balance_imu_rc_value->text().toDouble();
	out << YAML::Key << "pitch_angle_cut_off_frequency" << YAML::Value << ui.balance_imu_pc_value->text().toDouble();

	out << YAML::Key << "foot_x_force_cut_off_frequency" << YAML::Value << ui.balance_force_xc_value->text().toDouble();
	out << YAML::Key << "foot_y_force_cut_off_frequency" << YAML::Value << ui.balance_force_yc_value->text().toDouble();
	out << YAML::Key << "foot_z_force_cut_off_frequency" << YAML::Value << ui.balance_force_zc_value->text().toDouble();
	out << YAML::Key << "foot_roll_torque_cut_off_frequency" << YAML::Value << ui.balance_torque_rc_value->text().toDouble();
	out << YAML::Key << "foot_pitch_torque_cut_off_frequency" << YAML::Value << ui.balance_torque_pc_value->text().toDouble();

	out << YAML::Key << "updating_duration" << YAML::Value << ui.balance_duration_value->text().toDouble();
	out << YAML::EndMap;
	std::ofstream fout(path_.c_str());
	fout << out.c_str(); // dump it back into the file
	on_balance_apply_button_clicked();
}

void MainWindow::on_joint_apply_button_clicked()
{
	joint_feedback_gain_msg.request.updating_duration = ui.joint_updating_duration->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain = ui.joint_r_leg_hip_y_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain = ui.joint_r_leg_hip_y_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain = ui.joint_r_leg_hip_r_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain = ui.joint_r_leg_hip_r_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain = ui.joint_r_leg_hip_p_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain = ui.joint_r_leg_hip_p_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain = ui.joint_r_leg_an_p_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain = ui.joint_r_leg_an_p_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_p_gain = ui.joint_r_leg_kn_p_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_d_gain = ui.joint_r_leg_kn_p_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain = ui.joint_r_leg_an_r_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain = ui.joint_r_leg_an_r_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain = ui.joint_l_leg_hip_y_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain = ui.joint_l_leg_hip_y_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain = ui.joint_l_leg_hip_r_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain = ui.joint_l_leg_hip_r_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain = ui.joint_l_leg_hip_p_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain = ui.joint_l_leg_hip_p_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_p_gain = ui.joint_l_leg_kn_p_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_d_gain = ui.joint_l_leg_kn_p_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain = ui.joint_l_leg_an_p_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain = ui.joint_l_leg_an_p_d_gain->text().toDouble();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain = ui.joint_l_leg_an_r_p_gain->text().toDouble();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain = ui.joint_l_leg_an_r_d_gain->text().toDouble();
}

void MainWindow::on_joint_save_button_clicked()
{
	YAML::Emitter out;
	std::string path_;
	if(ALICE_ID == 1) path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain1_on.yaml";
	if(ALICE_ID == 2) path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain2_on.yaml";

	out << YAML::BeginMap;
	out << YAML::Key << "r_leg_hip_y_p_gain" << YAML::Value << ui.joint_r_leg_hip_y_p_gain->text().toDouble();
	out << YAML::Key << "r_leg_hip_y_d_gain" << YAML::Value << ui.joint_r_leg_hip_y_d_gain->text().toDouble();

	out << YAML::Key << "r_leg_hip_r_p_gain" << YAML::Value << ui.joint_r_leg_hip_r_p_gain->text().toDouble();
	out << YAML::Key << "r_leg_hip_r_d_gain" << YAML::Value << ui.joint_r_leg_hip_r_d_gain->text().toDouble();

	out << YAML::Key << "r_leg_hip_p_p_gain" << YAML::Value << ui.joint_r_leg_hip_p_p_gain->text().toDouble();
	out << YAML::Key << "r_leg_hip_p_d_gain" << YAML::Value << ui.joint_r_leg_hip_p_d_gain->text().toDouble();

	out << YAML::Key << "r_leg_an_p_p_gain" << YAML::Value << ui.joint_r_leg_an_p_p_gain->text().toDouble();
	out << YAML::Key << "r_leg_an_p_d_gain" << YAML::Value << ui.joint_r_leg_an_p_d_gain->text().toDouble();

	out << YAML::Key << "r_leg_kn_p_p_gain" << YAML::Value << ui.joint_r_leg_kn_p_p_gain->text().toDouble();
	out << YAML::Key << "r_leg_kn_p_d_gain" << YAML::Value << ui.joint_r_leg_kn_p_d_gain->text().toDouble();

	out << YAML::Key << "r_leg_an_r_p_gain" << YAML::Value << ui.joint_r_leg_an_r_p_gain->text().toDouble();
	out << YAML::Key << "r_leg_an_r_d_gain" << YAML::Value << ui.joint_r_leg_an_r_d_gain->text().toDouble();

	out << YAML::Key << "l_leg_hip_y_p_gain" << YAML::Value << ui.joint_l_leg_hip_y_p_gain->text().toDouble();
	out << YAML::Key << "l_leg_hip_y_d_gain" << YAML::Value << ui.joint_l_leg_hip_y_d_gain->text().toDouble();

	out << YAML::Key << "l_leg_hip_r_p_gain" << YAML::Value << ui.joint_l_leg_hip_r_p_gain->text().toDouble();
	out << YAML::Key << "l_leg_hip_r_d_gain" << YAML::Value << ui.joint_l_leg_hip_r_d_gain->text().toDouble();

	out << YAML::Key << "l_leg_hip_p_p_gain" << YAML::Value << ui.joint_l_leg_hip_p_p_gain->text().toDouble();
	out << YAML::Key << "l_leg_hip_p_d_gain" << YAML::Value << ui.joint_l_leg_hip_p_d_gain->text().toDouble();

	out << YAML::Key << "l_leg_an_p_p_gain" << YAML::Value << ui.joint_l_leg_an_p_p_gain->text().toDouble();
	out << YAML::Key << "l_leg_an_p_d_gain" << YAML::Value << ui.joint_l_leg_an_p_d_gain->text().toDouble();

	out << YAML::Key << "l_leg_kn_p_p_gain" << YAML::Value << ui.joint_l_leg_kn_p_p_gain->text().toDouble();
	out << YAML::Key << "l_leg_kn_p_d_gain" << YAML::Value << ui.joint_l_leg_kn_p_d_gain->text().toDouble();

	out << YAML::Key << "l_leg_an_r_p_gain" << YAML::Value << ui.joint_l_leg_an_r_p_gain->text().toDouble();
	out << YAML::Key << "l_leg_an_r_d_gain" << YAML::Value << ui.joint_l_leg_an_r_d_gain->text().toDouble();

	out << YAML::Key << "updating_duration" << YAML::Value << ui.joint_updating_duration->text().toDouble();
	out << YAML::EndMap;
	std::ofstream fout(path_.c_str());
	fout << out.c_str(); // dump it back into the file
	on_joint_apply_button_clicked();
}

void MainWindow::on_step_apply_button_clicked()
{
	default_step_num = ui.default_step_num->text().toDouble();
	default_step_length = ui.default_step_length->text().toDouble();
	default_side_step_length = ui.default_side_step_length->text().toDouble();
	default_step_angle_radian = ui.default_step_angle_radian->text().toDouble();
	default_step_time = ui.default_step_time->text().toDouble();

	expanded_step_num = ui.expanded_step_num->text().toDouble();
	expanded_step_length = ui.expanded_step_length->text().toDouble();
	expanded_side_step_length = ui.expanded_side_step_length->text().toDouble();
	expanded_step_angle_radian = ui.expanded_step_angle_radian->text().toDouble();
	expanded_step_time = ui.expanded_step_time->text().toDouble();

	centered_step_num = ui.centered_step_num->text().toDouble();
	centered_step_length = ui.centered_step_length->text().toDouble();
	centered_side_step_length = ui.centered_side_step_length->text().toDouble();
	centered_step_angle_radian = ui.centered_step_angle_radian->text().toDouble();
	centered_step_time = ui.centered_step_time->text().toDouble();

	foot_step_dsp = ui.foot_step_dsp->text().toDouble();
	foot_step_foot_z_swap = ui.foot_step_foot_z_swap->text().toDouble();
	foot_step_body_z_swap = ui.foot_step_body_z_swap->text().toDouble();
	foot_step_y_zmp_conv = ui.foot_step_y_zmp_conv->text().toDouble();
}

void MainWindow::on_step_save_button_clicked()
{
	YAML::Emitter out;
	std::string path_;
	if(ALICE_ID == 1) path_ = ros::package::getPath("alice_gui") + "/config/step_parameter1.yaml";
	if(ALICE_ID == 2) path_ = ros::package::getPath("alice_gui") + "/config/step_parameter1.yaml";

	out << YAML::BeginMap;
	out << YAML::Key << "default_step_num" << YAML::Value << ui.default_step_num->text().toDouble();
	out << YAML::Key << "default_step_length" << YAML::Value << ui.default_step_length->text().toDouble();
	out << YAML::Key << "default_side_step_length" << YAML::Value << ui.default_side_step_length->text().toDouble();
	out << YAML::Key << "default_step_angle_radian" << YAML::Value << ui.default_step_angle_radian->text().toDouble();
	out << YAML::Key << "default_step_time" << YAML::Value << ui.default_step_time->text().toDouble();

	out << YAML::Key << "expanded_step_num" << YAML::Value << ui.expanded_step_num->text().toDouble();
	out << YAML::Key << "expanded_step_length" << YAML::Value << ui.expanded_step_length->text().toDouble();
	out << YAML::Key << "expanded_side_step_length" << YAML::Value << ui.expanded_side_step_length->text().toDouble();
	out << YAML::Key << "expanded_step_angle_radian" << YAML::Value << ui.expanded_step_angle_radian->text().toDouble();
	out << YAML::Key << "expanded_step_time" << YAML::Value << ui.expanded_step_time->text().toDouble();

	out << YAML::Key << "centered_step_num" << YAML::Value << ui.centered_step_num->text().toDouble();
	out << YAML::Key << "centered_step_length" << YAML::Value << ui.centered_step_length->text().toDouble();
	out << YAML::Key << "centered_side_step_length" << YAML::Value << ui.centered_side_step_length->text().toDouble();
	out << YAML::Key << "centered_step_angle_radian" << YAML::Value << ui.centered_step_angle_radian->text().toDouble();
	out << YAML::Key << "centered_step_time" << YAML::Value << ui.centered_step_time->text().toDouble();

	out << YAML::Key << "foot_step_dsp" << YAML::Value << ui.foot_step_dsp->text().toDouble();
	out << YAML::Key << "foot_step_foot_z_swap" << YAML::Value << ui.foot_step_foot_z_swap->text().toDouble();
	out << YAML::Key << "foot_step_body_z_swap" << YAML::Value << ui.foot_step_body_z_swap->text().toDouble();
	out << YAML::Key << "foot_step_y_zmp_conv" << YAML::Value << ui.foot_step_y_zmp_conv->text().toDouble();

	out << YAML::EndMap;
	std::ofstream fout(path_.c_str());
	fout << out.c_str(); // dump it back into the file
	on_step_apply_button_clicked();
	if(ALICE_ID == 1)
	{
		qnode.alice_id_msgs.data = "1";
		qnode.alice_id_pub.publish(qnode.alice_id_msgs);
	}
	if(ALICE_ID == 2)
	{
		qnode.alice_id_msgs.data = "2";
		qnode.alice_id_pub.publish(qnode.alice_id_msgs);
	}
}
void MainWindow::on_update_button_clicked()
{
	qnode.present_joint_state_array_srv.request.update_check = true;
	if(qnode.present_joint_state_array_cl.call(qnode.present_joint_state_array_srv))
	{
		for(int num = 0; num < qnode.present_joint_state_array_srv.response.joint_data.size(); num ++)
		{
			for (std::map<int, std::string>::iterator it = joint_id_to_name.begin(); it != joint_id_to_name.end(); ++it) //
			{
				std::string joint_name;
				joint_name = it->second;
				if(!qnode.present_joint_state_array_srv.response.joint_data[num].joint_name.compare(joint_name))
				{
					joint_name_to_torque_state[joint_name]  = qnode.present_joint_state_array_srv.response.joint_data[num].torque_state;
					joint_name_to_present_value[joint_name] = qnode.present_joint_state_array_srv.response.joint_data[num].present_position_value;
					joint_name_to_offset_data[joint_name]   = qnode.present_joint_state_array_srv.response.joint_data[num].offset_data;
					joint_id_to_goal[joint_name_to_id[joint_name]] = qnode.present_joint_state_array_srv.response.joint_data[num].present_position_value;

				}
			}
		}
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
}
void MainWindow::on_setting_comboBox_currentIndexChanged(int index)
{
	if(index == 0)
	{
		ui.edit_step_num->setText("");
		ui.edit_step_length->setText("");
		ui.edit_side_step_length->setText("");
		ui.edit_step_angle_rad->setText("");
		ui.edit_step_time->setText("");
		ui.edit_dsp->setText("");
		ui.edit_foot_z_swap->setText("");
		ui.edit_body_z_swap->setText("");
		ui.edit_y_zmp_conv->setText("");
	}
	if(index == 1)
	{
		ui.edit_step_num->setText(QString("%1").arg(default_step_num));
		ui.edit_step_length->setText(QString("%1").arg(default_step_length));
		ui.edit_side_step_length->setText(QString("%1").arg(default_side_step_length));
		ui.edit_step_angle_rad->setText(QString("%1").arg(default_step_angle_radian));
		ui.edit_step_time->setText(QString("%1").arg(default_step_time));
		ui.edit_dsp->setText(QString("%1").arg(foot_step_dsp));
		ui.edit_foot_z_swap->setText(QString("%1").arg(foot_step_foot_z_swap));
		ui.edit_body_z_swap->setText(QString("%1").arg(foot_step_body_z_swap));
		ui.edit_y_zmp_conv->setText(QString("%1").arg(foot_step_y_zmp_conv));
	}
	if(index == 2)
	{
		ui.edit_step_num->setText(QString("%1").arg(expanded_step_num));
		ui.edit_step_length->setText(QString("%1").arg(expanded_step_length));
		ui.edit_side_step_length->setText(QString("%1").arg(expanded_side_step_length));
		ui.edit_step_angle_rad->setText(QString("%1").arg(expanded_step_angle_radian));
		ui.edit_step_time->setText(QString("%1").arg(expanded_step_time));
		ui.edit_dsp->setText(QString("%1").arg(foot_step_dsp));
		ui.edit_foot_z_swap->setText(QString("%1").arg(foot_step_foot_z_swap));
		ui.edit_body_z_swap->setText(QString("%1").arg(foot_step_body_z_swap));
		ui.edit_y_zmp_conv->setText(QString("%1").arg(foot_step_y_zmp_conv));
	}
	if(index == 3)
	{
		ui.edit_step_num->setText(QString("%1").arg(centered_step_num));
		ui.edit_step_length->setText(QString("%1").arg(centered_step_length));
		ui.edit_side_step_length->setText(QString("%1").arg(centered_side_step_length));
		ui.edit_step_angle_rad->setText(QString("%1").arg(centered_step_angle_radian));
		ui.edit_step_time->setText(QString("%1").arg(centered_step_time));
		ui.edit_dsp->setText(QString("%1").arg(foot_step_dsp));
		ui.edit_foot_z_swap->setText(QString("%1").arg(foot_step_foot_z_swap));
		ui.edit_body_z_swap->setText(QString("%1").arg(foot_step_body_z_swap));
		ui.edit_y_zmp_conv->setText(QString("%1").arg(foot_step_y_zmp_conv));
	}
}
void MainWindow::on_all_torque_on_button_clicked()
{
	torque_state(1);
	on_update_button_clicked();
	usleep(10*1000);
	change_button(ui.id_line_edit->text(), "offset");

}
void MainWindow::on_all_torque_off_button_clicked()
{
	torque_state(0);
	on_update_button_clicked();
	usleep(10*1000);
	change_button(ui.id_line_edit->text(), "offset");
}
void MainWindow::on_initial_pose_button_clicked()
{
	qnode.log(qnode.Info, "zero_offset_pose");
	qnode.command_state_msg.data = "init_offset_pose_zero";
	qnode.command_state_pub.publish(qnode.command_state_msg);

}
void MainWindow::on_pre_offset_pose_button_clicked()
{
	qnode.log(qnode.Info, "pre_offset_pose");
	qnode.command_state_msg.data = "init_offset_pose";
	qnode.command_state_pub.publish(qnode.command_state_msg);

}
void MainWindow::on_save_button_clicked()
{
	qnode.command_state_msg.data = "save";
	qnode.command_state_pub.publish(qnode.command_state_msg);
}
void MainWindow::on_base_module_button_clicked()
{
	qnode.enable_module_msg.data = "base_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::torque_state(bool state)
{
	qnode.joint_torque_on_off_array_srv.request.torque_command.clear();

	for (std::map<int, std::string>::iterator it = joint_id_to_name.begin(); it != joint_id_to_name.end(); ++it) //
	{
		std::string joint_name;
		joint_name = it->second;

		qnode.joint_torque_on_off_data.joint_name = joint_name;
		qnode.joint_torque_on_off_data.joint_torque_on_off = state;
		qnode.joint_torque_on_off_array_srv.request.torque_command.push_back(qnode.joint_torque_on_off_data);
	}
	if(qnode.joint_torque_on_off_array_cl.call(qnode.joint_torque_on_off_array_srv))
	{
		qnode.log(qnode.Info, "Send command to sever complete [All Torque]");
	}
	else
	{
		qnode.log(qnode.Error, "Check communication <SEND FAIL>");
	}
}
void MainWindow::offset_torque_state_changed_function()
{
	torque_state_changed_function(ui.id_line_edit->text(), ui.torque_state_check_box->isChecked());
}
void MainWindow::offset_goal_value_changed_function()
{
	goal_value_changed_function(ui.id_line_edit->text(), ui.goal_spin_box->value());
}
void MainWindow::offset_change_button()
{
	on_update_button_clicked();
	usleep(10*1000);
	change_button(ui.id_line_edit->text(),"offset");
}
void MainWindow::goal_value_changed_function(QString id_string, double spin_value)
{
	int id_int;

	id_int = id_string.toInt();

	if(joint_id_to_name.count(id_int))
	{
		joint_id_to_goal[id_int] =  spin_value;
		qnode.joint_offset_state_msg.joint_name = joint_id_to_name[id_int];
		qnode.joint_offset_state_msg.joint_goal_value =  spin_value;
		qnode.joint_offset_state_pub.publish(qnode.joint_offset_state_msg);
		qnode.log(qnode.Info, "Id confirm and Send Goal value!!");
	}
	else
	{
		qnode.log(qnode.Error, "Check ID number <SEND FAIL>");
	}
}
void MainWindow::torque_state_changed_function(QString id_string, bool check)
{
	int id_int;
	id_int = id_string.toInt();

	if(joint_id_to_name.count(id_int))
	{
		qnode.joint_torque_on_off_srv.request.torque_command.joint_name = joint_id_to_name[id_int];
		qnode.joint_torque_on_off_srv.request.torque_command.joint_torque_on_off = check;
		if(qnode.joint_torque_on_off_cl.call(qnode.joint_torque_on_off_srv))
		{
			qnode.log(qnode.Info, "Send command to sever complete [One Torque]");
		}
		else
		{
			qnode.log(qnode.Error, "Check communication <SEND FAIL>");
		}
	}
	else
	{
		qnode.log(qnode.Error, "Check ID number <SEND FAIL>");
	}

}
void MainWindow::joint_data_parse(const std::string &path)
{
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}

	YAML::Node joint_data = doc["joint_data"];// YAML 에 string "tar_pose"을 읽어온다.
	for (YAML::iterator it = joint_data.begin(); it != joint_data.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int id;
		std::string joint_name;
		// 한 줄에서 int 와 double 을 분리한다.
		id = it->first.as<int>();
		joint_name = it->second.as<std::string>();

		//joint 정보 저장
		joint_name_to_id[joint_name] = id;
		joint_id_to_name[id] = joint_name;
		joint_id_to_goal[id] = 0;
	}
}
void MainWindow::change_button(QString id_string, std::string type)
{
	int id_int;
	id_int = id_string.toInt();
	if(joint_id_to_name.count(id_int))
	{
		//offset
		if(!type.compare("offset"))
		{
			ui.goal_spin_box->setValue(joint_id_to_goal[id_int]);
			ui.link_name_line_edit->setText(QString::fromStdString(joint_id_to_name[id_int]));
			ui.torque_state_check_box->setChecked(joint_name_to_torque_state[joint_id_to_name[id_int]]);
			ui.offset_line_edit->setText(QString::number(joint_name_to_offset_data[joint_id_to_name[id_int]]));
			ui.offset_line_edit_2->setText(QString::number(joint_name_to_offset_data[joint_id_to_name[id_int]]*180/M_PI));
			ui.present_line_edit->setText(QString::number(joint_name_to_present_value[joint_id_to_name[id_int]]));
		}
		else
			return;
	}
	else
	{
		qnode.log(qnode.Error, "Check ID number <SEND FAIL>");
	}
}
/*****************************************************************************
 ** walking test
 *****************************************************************************/
void MainWindow::on_online_walking_module_clicked() {
	module_msg.data = "online_walking_module";
	qnode.module_on_off.publish(module_msg);
}
/*void MainWindow::on_op3_walking_module_clicked()
{
	module_msg.data = "walking_module";
	qnode.module_on_off.publish(module_msg);
}*/
void MainWindow::on_none_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_default_walking_button_clicked() {

	foot_step_command_msg.command = "default walking type";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_y_walking_button_clicked() {

	foot_step_command_msg.command = "y walking type";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
//walking module button
void MainWindow::on_turn_left_clicked() {

	foot_step_command_msg.command = "turn left";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_turn_right_clicked() {

	foot_step_command_msg.command = "turn right";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_left_clicked() {

	foot_step_command_msg.command = "left";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_right_clicked() {
	foot_step_command_msg.command = "right";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}

void MainWindow::on_forward_clicked() {

	foot_step_command_msg.command = "forward";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_backward_clicked() {

	foot_step_command_msg.command = "backward";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_stop_clicked() {
	foot_step_command_msg.command = "stop";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_left_kick_clicked()
{
	foot_step_command_msg.command = "left kick";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);

}
void MainWindow::on_right_kick_clicked()
{
	foot_step_command_msg.command = "right kick";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);

}
void MainWindow::on_expanded_left_clicked() {

	foot_step_command_msg.command = "expanded left";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_expanded_right_clicked() {

	foot_step_command_msg.command = "expanded right";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_centered_left_clicked() {

	foot_step_command_msg.command = "centered left";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_centered_right_clicked() {

	foot_step_command_msg.command = "centered right";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
void MainWindow::on_apply_data_clicked() {

	QString parameter_str;
	double  parameter_double = 0;


	parameter_str = ui.edit_step_num->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_num = parameter_double;

	parameter_str = ui.edit_step_length->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_length = parameter_double;

	parameter_str = ui.edit_side_step_length->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.side_step_length = parameter_double;

	parameter_str = ui.edit_step_angle_rad->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_angle_rad = parameter_double;

	parameter_str = ui.edit_step_time->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	foot_step_command_msg.step_time = parameter_double;

	// send message
	foot_step_command_msg.command = "stop";
	qnode.foot_step_command_pub.publish(foot_step_command_msg);
}
// foot step generater
void MainWindow::on_apply_foot_step_generator_clicked()
{
	QString parameter_str;
	double  parameter_double = 0;


	parameter_str = ui.edit_dsp->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	qnode.dsp_msgs.data = parameter_double;
	qnode.dsp_pub.publish(qnode.dsp_msgs);

	parameter_str = ui.edit_foot_z_swap->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	qnode.foot_z_swap_msgs.data = parameter_double;
	qnode.foot_z_swap_pub.publish(qnode.foot_z_swap_msgs);

	parameter_str = ui.edit_body_z_swap->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	qnode.body_z_swap_msgs.data = 	parameter_double;
	qnode.body_z_swap_pub.publish(qnode.body_z_swap_msgs);

	parameter_str = ui.edit_y_zmp_conv->text();
	parameter_double = parameter_str.toDouble();

	// message variables store
	qnode.y_zmp_convergence_msgs.data = parameter_double;
	qnode.y_zmp_convergence_pub.publish(qnode.y_zmp_convergence_msgs);
}
/*****************************************************************************
 ** module on off
 *****************************************************************************/
//<------------------------------------------------------------------- base_module-->
void MainWindow::on_base_module_real_button_clicked(){
	qnode.enable_module_msg.data = "base_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::on_initial_pose_real_button_clicked()
{
	qnode.init_pose_msg.data = "init_pose";
	qnode.init_pose_pub.publish(qnode.init_pose_msg);
}
//initialize button
void MainWindow::on_initialize_ft_sensor_button_clicked()
{
	ft_init_msg.data = 1;
	qnode.alice_ft_init_pub.publish(ft_init_msg);
}
//<------------------------------------------------------------------- leg_module-->
/*void MainWindow::on_alice_leg_module_button_clicked()
{
	qnode.enable_module_msg.data = "alice_leg_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);

}
void MainWindow::on_base_module_real_clicked()
{
	ROS_INFO("!!");
	qnode.enable_module_msg.data = "base_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::on_initial_pose_real_button_clicked()
{
	ROS_INFO("!!");
	qnode.init_pose_msg.data = "init_pose";
	qnode.init_pose_pub.publish(qnode.init_pose_msg);
}*/
//<------------------------------------------------------------------- upper_body_module-->
void MainWindow::on_upper_body_module_button_clicked() {
	module_msg.data = "upper_body_module";
	qnode.module_on_off.publish(module_msg);
}
//<------------------------------------------------------------------- arm_module-->
/*void MainWindow::on_arm_module_button_clicked() {
	module_msg.data = "arm_module";
	qnode.module_on_off.publish(module_msg);
}*/
//<------------------------------------------------------------------- desired pose -->
void MainWindow::on_waist_change_button_clicked()
{
	QString str_waist_pitch = ui.Edit_waist_10->text();
	double dWaist_pitch = str_waist_pitch.toDouble();

	dWaist_pitch = (dWaist_pitch*M_PI)/180;

	QString str_waist_yaw = ui.Edit_waist_9->text();
	double dWaist_yaw = str_waist_yaw.toDouble();

	dWaist_yaw = (dWaist_yaw*M_PI)/180;

	desired_pose_waist_msg.data.push_back(dWaist_yaw);
	desired_pose_waist_msg.data.push_back(dWaist_pitch);
	desired_pose_waist_msg.data.push_back(4);
	desired_pose_waist_msg.data.push_back(4);

	qnode.desired_pose_waist_pub.publish(desired_pose_waist_msg);

	desired_pose_waist_msg.data.clear();
}
void MainWindow::on_head_change_button_clicked()
{
	QString str_head_yaw = ui.Edit_head_8->text();
	double dHead_yaw = str_head_yaw.toDouble();
	dHead_yaw  = (dHead_yaw*M_PI)/180;

	QString str_head_pitch = ui.Edit_head_7->text();
	double dHead_pitch = str_head_pitch.toDouble();
	dHead_pitch  = (dHead_pitch*M_PI)/180;


	desired_pose_head_msg.data.push_back(dHead_yaw);
	desired_pose_head_msg.data.push_back(dHead_pitch);
	desired_pose_head_msg.data.push_back(4);
	desired_pose_head_msg.data.push_back(4);

	qnode.desired_pose_head_pub.publish(desired_pose_head_msg);

	desired_pose_head_msg.data.clear();
}
/*void MainWindow::on_arm_change_end_effector_button_clicked()
{
	QString str_l_arm_x = ui.Edit_left_x->text();
	double dl_arm_x = str_l_arm_x.toDouble();


	QString str_l_arm_y = ui.Edit_left_y->text();
	double dl_arm_y = str_l_arm_y.toDouble();


	QString str_l_arm_z = ui.Edit_left_z->text();
	double dl_arm_z = str_l_arm_z.toDouble();


	QString str_r_arm_x = ui.Edit_right_x->text();
	double dr_arm_x = str_r_arm_x.toDouble();


	QString str_r_arm_y = ui.Edit_right_y->text();
	double dr_arm_y = str_r_arm_y.toDouble();


	QString str_r_arm_z = ui.Edit_right_z->text();
	double dr_arm_z = str_r_arm_z.toDouble();


	desired_pose_arm_msg.data.push_back(dl_arm_x);
	desired_pose_arm_msg.data.push_back(dl_arm_y);
	desired_pose_arm_msg.data.push_back(dl_arm_z);

	desired_pose_arm_msg.data.push_back(dr_arm_x);
	desired_pose_arm_msg.data.push_back(dr_arm_y);
	desired_pose_arm_msg.data.push_back(dr_arm_z);

	desired_pose_arm_msg.data.push_back(4);

	qnode.desired_pose_arm_pub.publish(desired_pose_arm_msg);

	desired_pose_arm_msg.data.clear();
}*/
//<------------------------------------------------------------------- control-->
void MainWindow::on_zmp_on_clicked() {
	parse_gain_data();
	alice_balance_parameter_msg.updating_duration =  updating_duration;

	alice_balance_parameter_msg.foot_zmpFz_p_gain = foot_zmpFz_p_gain;
	alice_balance_parameter_msg.foot_zmpFz_d_gain = foot_zmpFz_d_gain;

	qnode.alice_balance_parameter_pub.publish(alice_balance_parameter_msg);
}
void MainWindow::on_zmp_off_clicked() {
	alice_balance_parameter_msg.updating_duration =  updating_duration;

	alice_balance_parameter_msg.foot_zmpFz_p_gain = 0;
	alice_balance_parameter_msg.foot_zmpFz_d_gain = 0;

	qnode.alice_balance_parameter_pub.publish(alice_balance_parameter_msg);
}
void MainWindow::on_tracking_on_clicked()
{
	parse_gain_tracking_data();
	qnode.ball_tracking_pub.publish(tracking_param_msg);
	tracking_param_msg.data.clear();

}
void MainWindow::on_tracking_off_clicked()
{
	tracking_param_msg.data.push_back(2);
	tracking_param_msg.data.push_back(0);
	tracking_param_msg.data.push_back(0);
	tracking_param_msg.data.push_back(0);
	tracking_param_msg.data.push_back(0);

	qnode.ball_tracking_pub.publish(tracking_param_msg);
	tracking_param_msg.data.clear();

}
/*****************************************************************************
 ** common
 *****************************************************************************/
void MainWindow::updateLoggingView() {
	ui.view_logging->scrollToBottom();
	ui.view_logging->model()->removeRows( 0, ui.view_logging->model()->rowCount()-2);// 제거 하는 함수
}
void MainWindow::closeEvent(QCloseEvent *event)
{
	//WriteSettings();
	QMainWindow::closeEvent(event);
}
void MainWindow::parse_gain_data()
{
	updating_duration = 0.0;
	cob_x_offset_m = 0.0;
	cob_y_offset_m = 0.0;
	foot_roll_gyro_p_gain = 0.0;
	foot_roll_gyro_d_gain = 0.0;
	foot_pitch_gyro_p_gain = 0.0;
	foot_pitch_gyro_d_gain = 0.0;
	foot_zmpFz_p_gain = 0.0;
	foot_zmpFz_d_gain = 0.0;

	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("alice_gui") + "/config/leg_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	// time load //
	updating_duration = doc["updating_duration"].as<double>();

	// offset load //
	cob_x_offset_m = doc["cob_x_offset_m"].as<double>();
	cob_y_offset_m = doc["cob_y_offset_m"].as<double>();

	//gain load //
	foot_roll_gyro_p_gain = doc["foot_roll_gyro_p_gain"].as<double>();
	foot_roll_gyro_d_gain = doc["foot_roll_gyro_d_gain"].as<double>();

	foot_pitch_gyro_p_gain = doc["foot_pitch_gyro_p_gain"].as<double>();
	foot_pitch_gyro_d_gain = doc["foot_pitch_gyro_d_gain"].as<double>();

	foot_zmpFz_p_gain = doc["foot_copFz_p_gain"].as<double>();
	foot_zmpFz_d_gain = doc["foot_copFz_d_gain"].as<double>();
}
void MainWindow::on_joint_feedback_gain_on_clicked()
{
/*	std::string path_;
	if(ALICE_ID == 1) path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain1_on.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	if(ALICE_ID == 2) path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain2_on.yaml";
	parse_joint_feed_back_param_data(path_);*/
	qnode.joint_feedback_gain_client.call(joint_feedback_gain_msg);
	qnode.joint_feedback_gain_client_save.call(joint_feedback_gain_msg);
}
void MainWindow::on_joint_feedback_gain_off_clicked()
{
	std::string path_;
	if(ALICE_ID == 1) path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain1_off.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	if(ALICE_ID == 2) path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain2_off.yaml";	
	parse_joint_feed_back_param_data(path_);
	qnode.joint_feedback_gain_client.call(joint_feedback_gain_msg);
}
void MainWindow::on_balance_param_on_clicked()
{
	qnode.set_balance_param_client.call(set_balance_param_msg);
	qnode.set_balance_param_client_save.call(set_balance_param_msg);
}
void MainWindow::on_balance_param_off_clicked()
{
	std::string path_;
	if(ALICE_ID == 1) path_ = ros::package::getPath("alice_gui") + "/config/balance_param1_off.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	if(ALICE_ID == 2) path_ = ros::package::getPath("alice_gui") + "/config/balance_param2_off.yaml";
	parse_balance_param_data(path_);
	qnode.set_balance_param_client.call(set_balance_param_msg);
}
void MainWindow::parse_gain_tracking_data()
{
	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("alice_gui") + "/config/ball_tracking_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	// load //
	tracking_param_msg.data.push_back(doc["updating_duration"].as<double>());
	tracking_param_msg.data.push_back(doc["x_p_gain"].as<double>());
	tracking_param_msg.data.push_back(doc["x_d_gain"].as<double>());
	tracking_param_msg.data.push_back(doc["y_p_gain"].as<double>());
	tracking_param_msg.data.push_back(doc["y_d_gain"].as<double>());
}
void MainWindow::parse_balance_param_data(std::string path)
{
	YAML::Node doc; // YAML file class 선언!
	//std::string path_ = ros::package::getPath("alice_gui") + "/config/balance_param.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}

	set_balance_param_msg.request.updating_duration = doc["updating_duration"].as<double>();

	set_balance_param_msg.request.balance_param.cob_x_offset_m = doc["cob_x_offset_m"].as<double>();
	set_balance_param_msg.request.balance_param.cob_y_offset_m = doc["cob_y_offset_m"].as<double>();

	set_balance_param_msg.request.balance_param.hip_roll_swap_angle_rad = doc["hip_roll_swap_angle_rad"].as<double>();

	//gain load //
	set_balance_param_msg.request.balance_param.foot_roll_gyro_p_gain = doc["foot_roll_gyro_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_gyro_d_gain = doc["foot_roll_gyro_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_p_gain = doc["foot_pitch_gyro_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_gyro_d_gain = doc["foot_pitch_gyro_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_roll_angle_p_gain = doc["foot_roll_angle_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_angle_d_gain = doc["foot_roll_angle_d_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_p_gain = doc["foot_pitch_angle_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_angle_d_gain = doc["foot_pitch_angle_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_x_force_p_gain = doc["foot_x_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_x_force_d_gain = doc["foot_x_force_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_y_force_p_gain = doc["foot_y_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_y_force_d_gain = doc["foot_y_force_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_z_force_p_gain = doc["foot_z_force_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_z_force_d_gain = doc["foot_z_force_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_roll_torque_p_gain = doc["foot_roll_torque_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_torque_d_gain = doc["foot_roll_torque_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.foot_pitch_torque_p_gain = doc["foot_pitch_torque_p_gain"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_d_gain = doc["foot_pitch_torque_d_gain"].as<double>();

	set_balance_param_msg.request.balance_param.roll_gyro_cut_off_frequency = doc["roll_gyro_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.pitch_gyro_cut_off_frequency = doc["pitch_gyro_cut_off_frequency"].as<double>();

	set_balance_param_msg.request.balance_param.roll_angle_cut_off_frequency = doc["roll_angle_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.pitch_angle_cut_off_frequency = doc["pitch_angle_cut_off_frequency"].as<double>();

	set_balance_param_msg.request.balance_param.foot_x_force_cut_off_frequency = doc["foot_x_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_y_force_cut_off_frequency = doc["foot_y_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_z_force_cut_off_frequency = doc["foot_z_force_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_roll_torque_cut_off_frequency = doc["foot_roll_torque_cut_off_frequency"].as<double>();
	set_balance_param_msg.request.balance_param.foot_pitch_torque_cut_off_frequency = doc["foot_pitch_torque_cut_off_frequency"].as<double>();

}
void MainWindow::parse_joint_feed_back_param_data(std::string path)
{
	YAML::Node doc; // YAML file class 선언!
	//std::string path_ = ros::package::getPath("alice_gui") + "/config/joint_feedback_gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	joint_feedback_gain_msg.request.updating_duration = doc["updating_duration"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_p_gain = doc["r_leg_hip_y_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_y_d_gain = doc["r_leg_hip_y_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_p_gain = doc["r_leg_hip_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_r_d_gain = doc["r_leg_hip_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_p_gain = doc["r_leg_hip_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_hip_p_d_gain = doc["r_leg_hip_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_p_gain = doc["r_leg_an_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_p_d_gain = doc["r_leg_an_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_p_gain = doc["r_leg_kn_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_kn_p_d_gain = doc["r_leg_kn_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_p_gain = doc["r_leg_an_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.r_leg_an_r_d_gain = doc["r_leg_an_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_p_gain = doc["l_leg_hip_y_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_y_d_gain = doc["l_leg_hip_y_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_p_gain = doc["l_leg_hip_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_r_d_gain = doc["l_leg_hip_r_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_p_gain = doc["l_leg_hip_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_hip_p_d_gain = doc["l_leg_hip_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_p_gain = doc["l_leg_kn_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_kn_p_d_gain = doc["l_leg_kn_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_p_gain = doc["l_leg_an_p_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_p_d_gain = doc["l_leg_an_p_d_gain"].as<double>();

	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_p_gain = doc["l_leg_an_r_p_gain"].as<double>();
	joint_feedback_gain_msg.request.feedback_gain.l_leg_an_r_d_gain = doc["l_leg_an_r_d_gain"].as<double>();

}

void MainWindow::parse_step_param_data(std::string path)
{
	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	default_step_num = doc["default_step_num"].as<double>();
	default_step_length = doc["default_step_length"].as<double>();
	default_side_step_length = doc["default_side_step_length"].as<double>();
	default_step_angle_radian = doc["default_step_angle_radian"].as<double>();
	default_step_time = doc["default_step_time"].as<double>();

	expanded_step_num = doc["expanded_step_num"].as<double>();
	expanded_step_length = doc["expanded_step_length"].as<double>();
	expanded_side_step_length = doc["expanded_side_step_length"].as<double>();
	expanded_step_angle_radian = doc["expanded_step_angle_radian"].as<double>();
	expanded_step_time = doc["expanded_step_time"].as<double>();

	centered_step_num = doc["centered_step_num"].as<double>();
	centered_step_length = doc["centered_step_length"].as<double>();
	centered_side_step_length = doc["centered_side_step_length"].as<double>();
	centered_step_angle_radian = doc["centered_step_angle_radian"].as<double>();
	centered_step_time = doc["centered_step_time"].as<double>();

	foot_step_dsp = doc["foot_step_dsp"].as<double>();
	foot_step_foot_z_swap = doc["foot_step_foot_z_swap"].as<double>();
	foot_step_body_z_swap = doc["foot_step_body_z_swap"].as<double>();
	foot_step_y_zmp_conv = doc["foot_step_y_zmp_conv"].as<double>();

}

}  // namespace offset_tuner_operation

