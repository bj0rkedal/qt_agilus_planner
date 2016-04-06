/**
 * @file /include/qt_agilus_planner/main_window.hpp
 *
 * @brief Qt based gui for QT_agilus_planner.
 *
 * @date November 2010
 **/
#ifndef qt_agilus_planner_MAIN_WINDOW_H
#define qt_agilus_planner_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_agilus_planner {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

    void init_ui_elements();

public Q_SLOTS:

    void on_pushButton_move_ag1_clicked();
    void on_pushButton_move_ag2_clicked();
    void on_pushButton_plan_ag1_clicked();
    void on_pushButton_plan_ag2_clicked();
    void on_pushButton_get_offset1_clicked();
    void on_pushButton_set_gimbal_clicked();
    void on_pushButton_set_point_clicked();
    void on_pushButton_reset_position_clicked();
    void on_pushButton_reset_orientation_clicked();
    void on_pushButton_point_lock_clicked();
    void on_pushButton_angle_lock_clicked();
    void on_pushButton_set_detection_clicked();
    void on_pushButton_home_ag1_clicked();
    void on_pushButton_home_ag2_clicked();
    void on_horizontalSlider_pos_x_valueChanged(int i);
    void on_horizontalSlider_pos_y_valueChanged(int i);
    void on_horizontalSlider_pos_z_valueChanged(int i);
    void on_horizontalSlider_roll_valueChanged(int i);
    void on_horizontalSlider_pitch_valueChanged(int i);
    void on_horizontalSlider_yaw_valueChanged(int i);

Q_SIGNALS:
    void send_move_ag1_command(bool rel, bool pos, double x, double y, double z,
                               bool orient, double roll, double pitch, double yaw);
    void send_move_ag2_command(bool rel, bool pos, double x, double y, double z,
                               bool orient, double roll, double pitch, double yaw);
    void send_plan_ag1_command(bool rel, bool pos, double x, double y, double z,
                               bool orient, double roll, double pitch, double yaw);
    void send_plan_ag2_command(bool rel, bool pos, double x, double y, double z,
                               bool orient, double roll, double pitch, double yaw);
    void send_set_gimbal_angles_command(double roll, double pitch, double yaw);
    void send_set_gimbal_point_command(double x, double y, double z);
    void send_gimbal_control_mode_command(bool control);
    void send_image_processor_command(bool running, bool color, bool bruteforce,
                                      bool undistort, std::string keypoint,
                                      std::string descriptor);
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace qt_agilus_planner

#endif // qt_agilus_planner_MAIN_WINDOW_H
