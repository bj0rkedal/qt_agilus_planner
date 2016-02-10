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
#include <cmath>
#include "../include/qt_agilus_planner/main_window.hpp"


namespace qt_agilus_planner {

using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this);

	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(this, SIGNAL(send_move_ag1_command(bool,bool,double,double,double,
                                                        bool,double,double,double)), &qnode,
                     SLOT(move_ag1(bool,bool,double,double,double,
                                   bool,double,double,double)));
    QObject::connect(this, SIGNAL(send_move_ag2_command(bool,bool,double,double,double,
                                                        bool,double,double,double)), &qnode,
                     SLOT(move_ag2(bool,bool,double,double,double,
                                   bool,double,double,double)));
    QObject::connect(this, SIGNAL(send_plan_ag1_command(bool,bool,double,double,double,
                                                        bool,double,double,double)), &qnode,
                     SLOT(plan_ag1(bool,bool,double,double,double,
                                   bool,double,double,double)));
    QObject::connect(this, SIGNAL(send_plan_ag2_command(bool,bool,double,double,double,
                                                        bool,double,double,double)), &qnode,
                     SLOT(plan_ag2(bool,bool,double,double,double,
                                   bool,double,double,double)));
    ui.pushButton_move_ag1->setEnabled(true);
    ui.pushButton_move_ag2->setEnabled(true);
    ui.pushButton_plan_ag1->setEnabled(true);
    ui.pushButton_plan_ag2->setEnabled(true);
    qnode.init();
}

MainWindow::~MainWindow() {}

void MainWindow::on_pushButton_move_ag1_clicked(){
    Q_EMIT send_move_ag1_command(ui.checkBox_relative->isChecked(), ui.checkBox_position->isChecked(),
                                 ui.spinBox_pos_x->value(),ui.spinBox_pos_y->value(),ui.spinBox_pos_z->value(),
                                 ui.checkBox_orientation->isChecked(),ui.spinBox_roll->value(),
                                 ui.spinBox_pitch->value(),ui.spinBox_yaw->value());
}

void MainWindow::on_pushButton_move_ag2_clicked(){
    Q_EMIT send_move_ag2_command(ui.checkBox_relative->isChecked(), ui.checkBox_position->isChecked(),
                                 ui.spinBox_pos_x->value(),ui.spinBox_pos_y->value(),ui.spinBox_pos_z->value(),
                                 ui.checkBox_orientation->isChecked(),ui.spinBox_roll->value(),
                                 ui.spinBox_pitch->value(),ui.spinBox_yaw->value());
}

void MainWindow::on_pushButton_plan_ag1_clicked(){
    Q_EMIT send_plan_ag1_command(ui.checkBox_relative->isChecked(), ui.checkBox_position->isChecked(),
                                 ui.spinBox_pos_x->value(),ui.spinBox_pos_y->value(),ui.spinBox_pos_z->value(),
                                 ui.checkBox_orientation->isChecked(),ui.spinBox_roll->value(),
                                 ui.spinBox_pitch->value(),ui.spinBox_yaw->value());
}

void MainWindow::on_pushButton_plan_ag2_clicked(){
    Q_EMIT send_plan_ag2_command(ui.checkBox_relative->isChecked(), ui.checkBox_position->isChecked(),
                                 ui.spinBox_pos_x->value(),ui.spinBox_pos_y->value(),ui.spinBox_pos_z->value(),
                                 ui.checkBox_orientation->isChecked(),ui.spinBox_roll->value(),
                                 ui.spinBox_pitch->value(),ui.spinBox_yaw->value());
}

void MainWindow::on_horizontalSlider_pos_x_valueChanged(int i)
{
    double tmp = i/100.0;
    ui.spinBox_pos_x->setValue(ui.spinBox_pos_x->minimum()+(tmp*(ui.spinBox_pos_x->maximum()*2)));
}

void MainWindow::on_horizontalSlider_pos_y_valueChanged(int i)
{
    double tmp = i/100.0;
    ui.spinBox_pos_y->setValue(ui.spinBox_pos_y->minimum()+(tmp*(ui.spinBox_pos_y->maximum()*2)));
}

void MainWindow::on_horizontalSlider_pos_z_valueChanged(int i)
{
    double tmp = i/100.0;
    ui.spinBox_pos_z->setValue(ui.spinBox_pos_z->minimum()+(tmp*(ui.spinBox_pos_z->maximum()*2)));
}

void MainWindow::on_horizontalSlider_roll_valueChanged(int i)
{
    double tmp = i/100.0;
    ui.spinBox_roll->setValue(ui.spinBox_roll->minimum()+(tmp*(ui.spinBox_roll->maximum()*2)));
}

void MainWindow::on_horizontalSlider_pitch_valueChanged(int i)
{
    double tmp = i/100.0;
    ui.spinBox_pitch->setValue(ui.spinBox_pitch->minimum()+(tmp*(ui.spinBox_pitch->maximum()*2)));
}

void MainWindow::on_horizontalSlider_yaw_valueChanged(int i)
{
    double tmp = i/100.0;
    ui.spinBox_yaw->setValue(ui.spinBox_yaw->minimum()+(tmp*(ui.spinBox_yaw->maximum()*2)));
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace qt_agilus_planner

