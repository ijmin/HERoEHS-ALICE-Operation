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
	qnode.init();
	setWindowIcon(QIcon(":/images/icon.png"));

	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/****************************
	 ** Connect
	 ****************************/
	// offset tab
	QObject::connect(ui.goal_spin_box, SIGNAL(valueChanged(QString)), this, SLOT(offset_goal_value_changed_function()));
	QObject::connect(ui.torque_state_check_box, SIGNAL(clicked()), this, SLOT(offset_torque_state_changed_function()));
	QObject::connect(ui.change_button, SIGNAL(clicked()), this, SLOT(offset_change_button()));


	/*********************
	 ** Logging
	 **********************/
	ui.view_logging->setModel(qnode.loggingModel());
	QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

	/*********************
	 ** Graph
	 **********************/

	legendFont = font();  // start out with MainWindow's font..
	graph_draw(ui.state_plot, "Goal / Present Position ", "Rad", -1, 1 , 10);
	graph_draw_sensor(ui.sensor_plot_1, "Left Sensor", "m", -3, 3, 10);
	graph_draw_sensor(ui.sensor_plot_2, "Right Sensor", "m", -3, 3, 10);


	dataTimer = new QTimer(this);
	connect(dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
	dataTimer->start(0.006); // Interval 0 means to refresh as fast as possible

	/****************************
	 ** Initialize  load joint data
	 ****************************/
	std::string init_pose_path;// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	init_pose_path = ros::package::getPath("offset_tuner_operation") + "/config/joint_data.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	joint_data_parse(init_pose_path);
	ui.id_line_edit->setText("0");
	pose_num = 0;
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
/*****************************************************************************
 ** dynamixel offset
 *****************************************************************************/
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
void MainWindow::on_none_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
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
/*****************************************************************************
 ** module on off
 *****************************************************************************/
void MainWindow::on_base_module_real_button_clicked(){
	qnode.enable_module_msg.data = "base_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
/*void MainWindow::on_base_module_real_clicked()
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
void MainWindow::on_initial_pose_real_button_clicked()
{
	qnode.init_pose_msg.data = "init_pose";
	qnode.init_pose_pub.publish(qnode.init_pose_msg);
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

}  // namespace offset_tuner_operation

