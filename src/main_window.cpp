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
    QObject::connect(this, SIGNAL(send_set_gimbal_angles_command(double,double,double)), &qnode,
                     SLOT(set_gimbal_angles(double,double,double)));
    QObject::connect(this, SIGNAL(send_set_gimbal_point_command(double,double,double)), &qnode,
                     SLOT(set_gimbal_point(double,double,double)));
    QObject::connect(this, SIGNAL(send_gimbal_control_mode_command(bool)), &qnode,
                     SLOT(set_control_mode(bool)));
    QObject::connect(this, SIGNAL(send_image_processor_command(bool,bool,bool,bool,std::string,std::string)),
                     &qnode, SLOT(set_image_processor_mode(bool,bool,bool,bool,std::string,std::string)));
    QObject::connect(this, SIGNAL(send_plan_test(ih::RobotPlanningExecution*,bool,double,double,double,double,double,double)), &qnode,
                     SLOT(plan_test(ih::RobotPlanningExecution*,bool,double,double,double,double,double,double)));
    QObject::connect(this, SIGNAL(send_move_test(ih::RobotPlanningExecution*,bool,double,double,double,double,double,double)), &qnode,
                     SLOT(move_test(ih::RobotPlanningExecution*,bool,double,double,double,double,double,double)));
    QObject::connect(this, SIGNAL(send_home_test(ih::RobotPlanningExecution*)), &qnode, SLOT(home_test(ih::RobotPlanningExecution*)));
    ui.pushButton_move_ag1->setEnabled(true);
    ui.pushButton_move_ag2->setEnabled(true);
    ui.pushButton_plan_ag1->setEnabled(true);
    ui.pushButton_plan_ag2->setEnabled(true);

    init_ui_elements();

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

void MainWindow::on_pushButton_get_offset1_clicked()
{
    std::vector<double> temp;
    temp = qnode.getobjectPose(ui.spinBox_lambda->value());
    ui.spinBox_pos_x->setValue(-temp.at(1));
    ui.spinBox_pos_y->setValue(-temp.at(0));
    ui.spinBox_pos_z->setValue(0.0);

}

void MainWindow::on_pushButton_set_gimbal_clicked()
{
    Q_EMIT send_set_gimbal_angles_command(ui.spinBox_roll_gimbal->value(),
                                          ui.spinBox_pitch_gimbal->value(),
                                          ui.spinBox_yaw_gimbal->value());
}

void MainWindow::on_pushButton_set_point_clicked()
{
    Q_EMIT send_set_gimbal_point_command(ui.spinBox_gimbal_point_x->value(),
                                         ui.spinBox_gimbal_point_y->value(),
                                         ui.spinBox_gimbal_point_z->value());
}

void MainWindow::on_pushButton_reset_position_clicked()
{
    ui.horizontalSlider_pos_x->setValue(50);
    ui.horizontalSlider_pos_y->setValue(50);
    ui.horizontalSlider_pos_z->setValue(50);
}

void MainWindow::on_pushButton_reset_orientation_clicked()
{
    ui.horizontalSlider_roll->setValue(50);
    ui.horizontalSlider_pitch->setValue(50);
    ui.horizontalSlider_yaw->setValue(50);
}

void MainWindow::on_pushButton_point_lock_clicked()
{
    ui.spinBox_gimbal_point_x->setEnabled(true);
    ui.spinBox_gimbal_point_y->setEnabled(true);
    ui.spinBox_gimbal_point_z->setEnabled(true);

    ui.spinBox_roll_gimbal->setEnabled(false);
    ui.spinBox_pitch_gimbal->setEnabled(false);
    ui.spinBox_yaw_gimbal->setEnabled(false);

    ui.pushButton_set_gimbal->setEnabled(false);
    ui.pushButton_set_point->setEnabled(true);

    Q_EMIT send_gimbal_control_mode_command(true);
}

void MainWindow::on_pushButton_angle_lock_clicked()
{
    ui.spinBox_gimbal_point_x->setEnabled(false);
    ui.spinBox_gimbal_point_y->setEnabled(false);
    ui.spinBox_gimbal_point_z->setEnabled(false);

    ui.spinBox_roll_gimbal->setEnabled(true);
    ui.spinBox_pitch_gimbal->setEnabled(true);
    ui.spinBox_yaw_gimbal->setEnabled(true);

    ui.pushButton_set_gimbal->setEnabled(true);
    ui.pushButton_set_point->setEnabled(false);

    Q_EMIT send_gimbal_control_mode_command(false);
}

void MainWindow::on_pushButton_set_detection_clicked()
{
    Q_EMIT send_image_processor_command(ui.checkBox_running->isChecked(),
                                        ui.checkBox_color->isChecked(),
                                        ui.checkBox_bruteforce->isChecked(),
                                        ui.checkBox_undistort->isChecked(),
                                        ui.comboBox_keypoint->currentText().toStdString(),
                                        ui.comboBox_descriptor->currentText().toStdString());
}

void MainWindow::on_pushButton_home_ag1_clicked()
{
    ui.spinBox_pos_x->setValue(0.445);
    ui.spinBox_pos_y->setValue(-0.6025);
    ui.spinBox_pos_z->setValue(1.66);
    ui.spinBox_roll->setValue(0);
    ui.spinBox_pitch->setValue(3.1415);
    ui.spinBox_yaw->setValue(0);
    ui.checkBox_relative->setChecked(false);
    ui.checkBox_position->setChecked(true);
    ui.checkBox_orientation->setChecked(true);
}

void MainWindow::on_pushButton_home_ag2_clicked()
{
    ui.spinBox_pos_x->setValue(0.445);
    ui.spinBox_pos_y->setValue(0.6025);
    ui.spinBox_pos_z->setValue(1.66);
    ui.spinBox_roll->setValue(0);
    ui.spinBox_pitch->setValue(3.1415);
    ui.spinBox_yaw->setValue(0);
    ui.checkBox_relative->setChecked(false);
    ui.checkBox_position->setChecked(true);
    ui.checkBox_orientation->setChecked(true);
}

void MainWindow::on_pushButton_testPlan_clicked()
{
    if(ui.robotComboBox->currentIndex() == 0) {
        Q_EMIT send_plan_test(qnode.getRobot1(),
                              ui.checkBox_relative->isChecked(),
                              ui.spinBox_pos_x->value(),
                              ui.spinBox_pos_y->value(),
                              ui.spinBox_pos_z->value(),
                              ui.spinBox_roll->value(),
                              ui.spinBox_pitch->value(),
                              ui.spinBox_yaw->value());
    } else {
        Q_EMIT send_plan_test(qnode.getRobot2(),
                              ui.checkBox_relative->isChecked(),
                              ui.spinBox_pos_x->value(),
                              ui.spinBox_pos_y->value(),
                              ui.spinBox_pos_z->value(),
                              ui.spinBox_roll->value(),
                              ui.spinBox_pitch->value(),
                              ui.spinBox_yaw->value());
    }
}

void MainWindow::on_pushButton_testHome_clicked()
{
    if(ui.robotComboBox->currentIndex() == 0) {
        Q_EMIT send_home_test(qnode.getRobot1());
    } else {
        Q_EMIT send_home_test(qnode.getRobot2());
    }
}

void MainWindow::on_pushButton_testMove_clicked()
{
    if(ui.robotComboBox->currentIndex() == 0) {
        Q_EMIT send_move_test(qnode.getRobot1(),
                              ui.checkBox_relative->isChecked(),
                              ui.spinBox_pos_x->value(),
                              ui.spinBox_pos_y->value(),
                              ui.spinBox_pos_z->value(),
                              ui.spinBox_roll->value(),
                              ui.spinBox_pitch->value(),
                              ui.spinBox_yaw->value());
    } else {
        Q_EMIT send_move_test(qnode.getRobot2(),
                              ui.checkBox_relative->isChecked(),
                              ui.spinBox_pos_x->value(),
                              ui.spinBox_pos_y->value(),
                              ui.spinBox_pos_z->value(),
                              ui.spinBox_roll->value(),
                              ui.spinBox_pitch->value(),
                              ui.spinBox_yaw->value());
    }
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

void MainWindow::init_ui_elements()
{
    QStringList manipulators;
    manipulators.append("KUKA Agilus 1");
    manipulators.append("KUKA Agilus 2");
    ui.robotComboBox->addItems(manipulators);

    ui.comboBox_keypoint->clear();
    ui.comboBox_keypoint->addItem("SIFT");
    ui.comboBox_keypoint->addItem("SURF");
    ui.comboBox_keypoint->addItem("FAST");
    ui.comboBox_keypoint->addItem("STAR");
    ui.comboBox_keypoint->addItem("BRISK");
    ui.comboBox_keypoint->addItem("ORB");
    ui.comboBox_keypoint->addItem("AKAZE");
    ui.comboBox_descriptor->clear();
    ui.comboBox_descriptor->addItem("SIFT");
    ui.comboBox_descriptor->addItem("SURF");
    ui.comboBox_descriptor->addItem("BRIEF");
    ui.comboBox_descriptor->addItem("BRISK");
    ui.comboBox_descriptor->addItem("FREAK");
    ui.comboBox_descriptor->addItem("ORB");
    ui.comboBox_descriptor->addItem("AKAZE");
}

}  // namespace qt_agilus_planner

