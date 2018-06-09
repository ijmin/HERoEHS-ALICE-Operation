/*
 * graph.cpp
 *
 *  Created on: May 23, 2018
 *      Author: robotemperor
 */

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

void MainWindow::realtimeDataSlot()
{
	static QTime time(QTime::currentTime());
	// calculate two new data points:
	key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
	static double lastPointKey = 0;

	change_ractangle(foot_left, qnode.lf_point_x, qnode.lf_point_y);
	change_ractangle(foot_right, qnode.rf_point_x, qnode.rf_point_y);

	if (key-lastPointKey > 0.006) //
	{
		graph_draw_update_none_line(ui.zmp_graph, qnode.current_zmp_fz_x, qnode.current_zmp_fz_y,0,0);

		check_sensor_menu();
		select_joint_state();


		lastPointKey = key;
	}
	// make key axis range scroll with the data (at a constant range size of 8):
	graph_draw_clean(ui.state_plot);
	graph_draw_clean(ui.sensor_plot_1);
	graph_draw_clean(ui.sensor_plot_2);

	if(qnode.ft_init_done_check)
	{
		temp_check_state = "FT Sensor Initialize Completed";
		ui.sensor_state->setText(temp_check_state);
	}
	else
	{
		temp_check_state = "Need to initialize";
		ui.sensor_state->setText(temp_check_state);
	}
}
void MainWindow::graph_draw_sensor(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(true);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));

	ui_graph->addGraph();
	ui_graph->graph(0)->setPen(QPen(QColor(40, 110, 255)));
	ui_graph->graph(0)->setName("X");
	ui_graph->addGraph();
	ui_graph->graph(1)->setPen(QPen(QColor(255, 0, 0)));
	ui_graph->graph(1)->setName("Y");
	ui_graph->addGraph();
	ui_graph->graph(2)->setPen(QPen(QColor(0, 0, 0)));
	ui_graph->graph(2)->setName("Z");
	ui_graph->xAxis->setLabel("Time(s)");
	ui_graph->yAxis->setLabel(unit);

	QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
	timeTicker->setTimeFormat("%s");
	timeTicker->setFieldWidth(timeTicker->tuSeconds,1);
	timeTicker->setTickCount(tick_count);
	ui_graph->xAxis->setTicker(timeTicker);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->yAxis->setRange(min_value, max_value);
}
void MainWindow::graph_draw_sensor_update(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ)
{
	// add data to lines:
	ui_graph->graph(0)->addData(key, valueX);
	ui_graph->graph(1)->addData(key, valueY);
	ui_graph->graph(2)->addData(key, valueZ);

	ui_graph->graph(0)->rescaleValueAxis(true);
	ui_graph->graph(1)->rescaleValueAxis(true);
	ui_graph->graph(2)->rescaleValueAxis(true);
}

void MainWindow::graph_draw(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(true);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));

	ui_graph->addGraph();
	ui_graph->graph(0)->setPen(QPen(QColor(40, 110, 255)));
	ui_graph->graph(0)->setName("Present");
	ui_graph->addGraph();
	ui_graph->graph(1)->setPen(QPen(QColor(255, 0, 0)));
	ui_graph->graph(1)->setName("Goal");
	ui_graph->xAxis->setLabel("Time(s)");
	ui_graph->yAxis->setLabel(unit);

	QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
	timeTicker->setTimeFormat("%s");
	timeTicker->setFieldWidth(timeTicker->tuSeconds,1);
	timeTicker->setTickCount(tick_count);
	ui_graph->xAxis->setTicker(timeTicker);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->yAxis->setRange(min_value, max_value);
}
void MainWindow::graph_draw_update(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ)
{
	// add data to lines:
	ui_graph->graph(0)->addData(key, valueX);
	ui_graph->graph(1)->addData(key, valueY);
	ui_graph->graph(0)->rescaleValueAxis(true);
	ui_graph->graph(1)->rescaleValueAxis(true);
}
void MainWindow::graph_draw_clean(QCustomPlot *ui_graph)
{
	ui_graph->xAxis->setRange(key, 8, Qt::AlignRight);
	ui_graph->replot();
}
void MainWindow::graph_draw_none_line(QCustomPlot *ui_graph, const QString title, const QString unit, double min_value_x, double max_value_x, double min_value_y, double max_value_y, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(false);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));

	ui_graph->addGraph();
	ui_graph->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,7));
	ui_graph->graph(0)->setPen(QPen(QColor(40, 110, 255)));
	ui_graph->graph(0)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(0)->setName("Current_cop_Fz");

	ui_graph->addGraph();
	ui_graph->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,7));
	ui_graph->graph(1)->setPen(QPen(QColor(255, 0, 0)));
	ui_graph->graph(1)->setLineStyle(QCPGraph::lsNone);
	ui_graph->graph(1)->setName("Reference_cop_Fz");


	ui_graph->xAxis->setLabel("Y  "+unit);
	ui_graph->yAxis->setLabel("X  "+unit);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->xAxis->setRange(min_value_x, max_value_x);
	ui_graph->xAxis->setRangeReversed(true);
	ui_graph->yAxis->setRange(min_value_y, max_value_y);
}
void MainWindow::graph_draw_update_none_line(QCustomPlot *ui_graph, double cur_value1, double cur_value2, double ref_value1, double ref_value2)
{
	QVector<double> c_1, c_2, r_11, r_22;

	c_1.append(cur_value1);
	c_2.append(cur_value2);

	r_11.append(ref_value1);
	r_22.append(ref_value2);


	ui_graph->graph(0)->setData(c_2, c_1);
	ui_graph->replot();
	ui_graph->update();

	ui_graph->graph(1)->setData(r_22, r_11);
	ui_graph->replot();
	ui_graph->update();



	c_1.clear();
	c_2.clear();
	r_11.clear();
	r_22.clear();

}
void MainWindow::draw_ractangle(QCustomPlot *ui_graph, QCPItemRect* section, const QString layer_name)
{
	section->topLeft->setType(QCPItemPosition::ptPlotCoords);
	section->topLeft->setAxes(ui_graph->xAxis, ui_graph->yAxis);
	section->bottomRight->setType(QCPItemPosition::ptPlotCoords);
	section->bottomRight->setAxes(ui_graph->xAxis, ui_graph->yAxis);
	section->topLeft->setCoords(0,0);
	section->bottomRight->setCoords(0,0);
	section->setBrush(QBrush(QColor(0,200,0,100)));
	section->setPen(Qt::NoPen);
	ui_graph->addLayer(layer_name, ui_graph->layer("grid"), QCustomPlot::limBelow);
	section->setLayer(layer_name);
}
void MainWindow::change_ractangle(QCPItemRect* section, double valueX, double valueY)
{
	section->topLeft->setCoords(valueY+0.065,valueX+0.125);
	section->bottomRight->setCoords(valueY-0.065,valueX-0.125);  //reverse !!
}
void MainWindow::check_sensor_menu()
{
	if(ui.sensor1_combo_box->currentText() == "Torque")
	{
		ui.sensor_plot_1->yAxis->setLabel("Nm");
		graph_draw_sensor_update(ui.sensor_plot_1, qnode.currentTorqueX_l_gui, qnode.currentTorqueY_l_gui, qnode.currentTorqueZ_l_gui);
		ui.sensor_plot_1->yAxis->setRange(-3, 3);
	}
	else
	{
		ui.sensor_plot_1->yAxis->setLabel("N");
		graph_draw_sensor_update(ui.sensor_plot_1, qnode.currentForceX_l_gui, qnode.currentForceY_l_gui, qnode.currentForceZ_l_gui);
	}
	if(ui.sensor2_combo_box->currentText() == "Torque")
	{
		ui.sensor_plot_2->yAxis->setLabel("Nm");
		graph_draw_sensor_update(ui.sensor_plot_2, qnode.currentTorqueX_r_gui, qnode.currentTorqueY_r_gui, qnode.currentTorqueZ_r_gui);
		ui.sensor_plot_2->yAxis->setRange(-3, 3);
	}
	else
	{
		ui.sensor_plot_2->yAxis->setLabel("N");
		graph_draw_sensor_update(ui.sensor_plot_2, qnode.currentForceX_r_gui, qnode.currentForceY_r_gui, qnode.currentForceZ_r_gui);
	}
}
void MainWindow::select_joint_state()
{
	if(ui.mode_select_combo_box->currentText() == "Joint State")
	{
		ui.state_plot->yAxis->setLabel("Rad");
		graph_draw_update(ui.state_plot, qnode.joint_name_to_present[ui.state_combo_box->currentText().toStdString()], qnode.joint_name_to_goal[ui.state_combo_box->currentText().toStdString()], 0);
	}
	if(ui.mode_select_combo_box->currentText() == "ZMP_Y")
	{
		ui.state_plot->yAxis->setLabel("m");
		graph_draw_update(ui.state_plot, qnode.current_zmp_fz_y, qnode.reference_zmp_fz_y, 0);
	}
	if(ui.mode_select_combo_box->currentText() == "ZMP_X")
	{
		ui.state_plot->yAxis->setLabel("m");
		graph_draw_update(ui.state_plot, qnode.current_zmp_fz_x, qnode.reference_zmp_fz_x, 0);
	}
}

void MainWindow::on_stop_button_clicked()
{
	dataTimer->stop();
}
void MainWindow::on_start_button_clicked()
{
	dataTimer->start(0);
}
void MainWindow::on_joint_state_init_button_clicked()
{
	ui.state_combo_box->clear();
	for(int num=0; num < qnode.joint_index_to_name.size(); num++)
	{

		ui.state_combo_box->addItem(QString::fromStdString(qnode.joint_index_to_name[num]));
		//printf("%s ::  %f \n", joint_index_to_name[num].c_str(), joint_name_to_present[joint_index_to_name[num]]);
	}

}

}
