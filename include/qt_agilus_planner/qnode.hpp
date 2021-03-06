/**
 * @file /include/qt_agilus_planner/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_agilus_planner_QNODE_HPP_
#define qt_agilus_planner_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <qt_agilus_planner/Pose.h>
#include <steadycam/getControlMode.h>
#include <steadycam/setControlMode.h>
#include <steadycam/getEulerAngles.h>
#include <steadycam/setEulerAngles.h>
#include <steadycam/getPoint.h>
#include <steadycam/setPoint.h>

#include <image_processor/getBruteforceMatching.h>
#include <image_processor/setBruteforceMatching.h>
#include <image_processor/getProcessRunning.h>
#include <image_processor/setProcessRunning.h>
#include <image_processor/getVideoColor.h>
#include <image_processor/setVideoColor.h>
#include <image_processor/getVideoUndistortion.h>
#include <image_processor/setVideoUndistortion.h>
#include <image_processor/getKeypointDetectorType.h>
#include <image_processor/setKeypointDetectorType.h>
#include <image_processor/getDescriptorType.h>
#include <image_processor/setDescriptorType.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose2D.h>

#include <eigen3/Eigen/Dense>

#include "opencv2/core.hpp"

#include "robot_planning_execution.hpp"

const std::string CAMERA_PARAMS = "/home/minions/Documents/calibration_reserve_camera.yml";

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_agilus_planner {

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

Q_SIGNALS:
    void rosShutdown();

public:
    void object1PoseCallback(const geometry_msgs::Pose2DConstPtr &msg);
    cv::Mat getCameraMatrix(const std::string path);
    Eigen::Vector3d getNormImageCoords(double x, double y, double lambda, cv::Mat camera_matrix);
    std::vector<double> getobjectPose(double lambda);
    moveit::planning_interface::MoveGroup* getAgilus1();
    moveit::planning_interface::MoveGroup* getAgilus2();
    ih::RobotPlanningExecution* getRobot1();
    ih::RobotPlanningExecution* getRobot2();

public Q_SLOTS:
    void setPoseRequest(bool relative, bool position,
                        double x, double y, double z,
                        bool orientation, double roll, double pitch,
                        double yaw);
    void move_ag1(bool relative, bool position, double x, double y, double z,
                  bool orientation, double roll, double pitch, double yaw);
    void move_ag2(bool relative, bool position, double x, double y, double z,
                  bool orientation, double roll, double pitch, double yaw);
    void plan_ag1(bool relative, bool position, double x, double y, double z,
                  bool orientation, double roll, double pitch, double yaw);
    void plan_ag2(bool relative, bool position, double x, double y, double z,
                  bool orientation, double roll, double pitch, double yaw);
    void set_gimbal_angles(double roll, double pitch, double yaw);
    void set_gimbal_point(double x, double y, double z);
    void set_control_mode(bool control);
    void set_image_processor_mode(bool running, bool color, bool bruteforce,
                                  bool undistort, std::string keypoint,
                                  std::string descriptor);
    //moveit::planning_interface::MoveGroup* manipulator
    void plan_test(ih::RobotPlanningExecution* robot, bool relative,
                   double x, double y, double z, double roll, double pitch, double yaw);
    void move_test(ih::RobotPlanningExecution* robot, bool relative,
                   double x, double y, double z, double roll, double pitch, double yaw);
    void home_test(ih::RobotPlanningExecution* robot);


private:
	int init_argc;
	char** init_argv;
    // Robot control
    ros::ServiceClient goToClient_ag1;
    ros::ServiceClient goToClient_ag2;
    ros::ServiceClient planClient_ag1;
    ros::ServiceClient planClient_ag2;

    qt_agilus_planner::Pose pose_service;

    ros::Subscriber object2Dpose1;

    // Steadycam
    ros::ServiceClient getSteadycamControlMode;
    ros::ServiceClient setSteadycamControlMode;
    ros::ServiceClient getSteadycamEulerAngles;
    ros::ServiceClient setSteadycamEulerAngles;
    ros::ServiceClient getSteadycamPoint;
    ros::ServiceClient setSteadycamPoint;

    steadycam::setEulerAngles gimbal_euler;
    steadycam::setControlMode controlMode;
    steadycam::setPoint gimbal_point;

    // Image processor
    ros::ServiceClient getImageprocessorRunning;
    ros::ServiceClient setImageprocessorRunning;
    ros::ServiceClient getImageprocessorColor;
    ros::ServiceClient setImageprocessorColor;
    ros::ServiceClient getImageprocessorBruteforce;
    ros::ServiceClient setImageprocessorBruteforce;
    ros::ServiceClient getImageprocessorUndistort;
    ros::ServiceClient setImageprocessorUndistort;
    ros::ServiceClient getImageprocessorKeypoint;
    ros::ServiceClient setImageprocessorKeypoint;
    ros::ServiceClient getImageprocessorDescriptor;
    ros::ServiceClient setImageprocessorDescriptor;

    image_processor::getProcessRunning getImproRunning;
    image_processor::setProcessRunning setImproRunning;
    image_processor::getVideoColor getVideoColor;
    image_processor::setVideoColor setVideoColor;
    image_processor::getVideoUndistortion getVideoUndist;
    image_processor::setVideoUndistortion setVideoUndist;
    image_processor::getBruteforceMatching getBF;
    image_processor::setBruteforceMatching setBF;
    image_processor::getKeypointDetectorType getKeypoint;
    image_processor::setKeypointDetectorType setKeypoint;
    image_processor::getDescriptorType getDecriptor;
    image_processor::setDescriptorType setDecriptor;

    double x_object1;
    double y_object1;
    cv::Mat camera_matrix;

    moveit::planning_interface::MoveGroup *ag1; //!< Move group used to acquire manipulator pose for agilus 1.
    moveit::planning_interface::MoveGroup *ag2; //!< Move group used to acquire manipulator pose for agilus 2.

    ih::RobotPlanningExecution *robot1;
    ih::RobotPlanningExecution *robot2;
};

}  // namespace qt_agilus_planner

#endif /* qt_agilus_planner_QNODE_HPP_ */
