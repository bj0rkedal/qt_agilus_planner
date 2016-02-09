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
                                 ui.checkBox_orientation->isChecked(),ui.doubleSpinBox_roll->value(),
                                 ui.doubleSpinBox_pitch->value(),ui.doubleSpinBox_yaw->value());
}

void MainWindow::on_pushButton_move_ag2_clicked(){
    Q_EMIT send_move_ag2_command(ui.checkBox_relative->isChecked(), ui.checkBox_position->isChecked(),
                                 ui.spinBox_pos_x->value(),ui.spinBox_pos_y->value(),ui.spinBox_pos_z->value(),
                                 ui.checkBox_orientation->isChecked(),ui.doubleSpinBox_roll->value(),
                                 ui.doubleSpinBox_pitch->value(),ui.doubleSpinBox_yaw->value());
}

void MainWindow::on_pushButton_plan_ag1_clicked(){
    Q_EMIT send_plan_ag1_command(ui.checkBox_relative->isChecked(), ui.checkBox_position->isChecked(),
                                 ui.spinBox_pos_x->value(),ui.spinBox_pos_y->value(),ui.spinBox_pos_z->value(),
                                 ui.checkBox_orientation->isChecked(),ui.doubleSpinBox_roll->value(),
                                 ui.doubleSpinBox_pitch->value(),ui.doubleSpinBox_yaw->value());
}

void MainWindow::on_pushButton_plan_ag2_clicked(){
    Q_EMIT send_plan_ag2_command(ui.checkBox_relative->isChecked(), ui.checkBox_position->isChecked(),
                                 ui.spinBox_pos_x->value(),ui.spinBox_pos_y->value(),ui.spinBox_pos_z->value(),
                                 ui.checkBox_orientation->isChecked(),ui.doubleSpinBox_roll->value(),
                                 ui.doubleSpinBox_pitch->value(),ui.doubleSpinBox_yaw->value());
}

void MainWindow::on_horizontalSlider_valueChanged(int i)
{
    std::cout << "slider 1 value: " << i << std::endl;
}

void MainWindow::on_horizontalSlider_2_valueChanged(int i)
{
    std::cout << "slider 2 value: " << i << std::endl;
}

void MainWindow::on_horizontalSlider_3_valueChanged(int i)
{
    std::cout << "slider 3 value: " << i << std::endl;
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace qt_agilus_planner

