/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qt_agilus_planner/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_agilus_planner {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qt_agilus_planner");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.

    goToClient_ag1 = n.serviceClient<qt_agilus_planner::Pose>("/robot_service_ag1/go_to_pose");
    goToClient_ag2 = n.serviceClient<qt_agilus_planner::Pose>("/robot_service_ag2/go_to_pose");
    planClient_ag1 = n.serviceClient<qt_agilus_planner::Pose>("/robot_service_ag1/plan_pose");
    planClient_ag2 = n.serviceClient<qt_agilus_planner::Pose>("/robot_service_ag2/plan_pose");

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"QT_agilus_planner");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::setPoseRequest(bool relative, bool position, double x, double y, double z, bool orientation, double roll, double pitch, double yaw)
{
    pose_service.request.header.frame_id = "/world";
    pose_service.request.relative = relative;
    pose_service.request.set_position = position;
    pose_service.request.position_x = x;
    pose_service.request.position_y = y;
    pose_service.request.position_z = z;
    pose_service.request.set_orientation = orientation;
    pose_service.request.orientation_r = roll;
    pose_service.request.orientation_p = pitch;
    pose_service.request.orientation_y = yaw;
}

void QNode::move_ag1(bool relative, bool position, double x, double y, double z, bool orientation, double roll, double pitch, double yaw)
{
    // Set service request parameters
    setPoseRequest(relative,position,x,y,z,orientation,roll,pitch,yaw);
    goToClient_ag1.call(pose_service);
}

void QNode::move_ag2(bool relative, bool position, double x, double y, double z, bool orientation, double roll, double pitch, double yaw)
{
    // Set service request parameters
    setPoseRequest(relative,position,x,y,z,orientation,roll,pitch,yaw);
    goToClient_ag2.call(pose_service);
}

void QNode::plan_ag1(bool relative, bool position, double x, double y, double z, bool orientation, double roll, double pitch, double yaw)
{
    // Set service request parameters
    setPoseRequest(relative,position,x,y,z,orientation,roll,pitch,yaw);
    planClient_ag1.call(pose_service);
}

void QNode::plan_ag2(bool relative, bool position, double x, double y, double z, bool orientation, double roll, double pitch, double yaw)
{
    // Set service request parameters
    setPoseRequest(relative,position,x,y,z,orientation,roll,pitch,yaw);
    planClient_ag2.call(pose_service);
}

}
