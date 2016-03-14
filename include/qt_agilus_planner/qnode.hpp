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
#include <string>
#include <QThread>
#include <QStringListModel>
#include <qt_agilus_planner/Pose.h>
#include <geometry_msgs/Pose2D.h>


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
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

Q_SIGNALS:
    void rosShutdown();

public:
    void object1PoseCallback(const geometry_msgs::Pose2DConstPtr &msg);
    void object2PoseCallback(const geometry_msgs::Pose2DConstPtr &msg);

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


private:
	int init_argc;
	char** init_argv;
    ros::ServiceClient goToClient_ag1;
    ros::ServiceClient goToClient_ag2;
    ros::ServiceClient planClient_ag1;
    ros::ServiceClient planClient_ag2;
    qt_agilus_planner::Pose pose_service;
    ros::Subscriber object2Dpose1;
    ros::Subscriber object2Dpose2;

};

}  // namespace qt_agilus_planner

#endif /* qt_agilus_planner_QNODE_HPP_ */
