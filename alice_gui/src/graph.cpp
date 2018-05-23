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

	if (key-lastPointKey > 0.006) //
	{
		check_sensor_menu();
		select_joint_state();


		lastPointKey = key;
	}
	// make key axis range scroll with the data (at a constant range size of 8):
	graph_draw_clean(ui.state_plot);
	graph_draw_clean(ui.sensor_plot_1);
	graph_draw_clean(ui.sensor_plot_2);
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
    graph_draw_update(ui.state_plot, qnode.joint_name_to_present[ui.state_combo_box->currentText().toStdString()], qnode.joint_name_to_goal[ui.state_combo_box->currentText().toStdString()], 0);
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
