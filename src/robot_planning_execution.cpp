//
// Original author: Adam Leon Kleppe
// Modified by: Asgeir Bjørkedal
//
// Used in the Master's thesis for planning and execution of trajectories in MoveIt! for ROS.
//
#include "../include/qt_agilus_planner/robot_planning_execution.hpp"

namespace ih {
//*********************************************************************************************************************
// PUBLIC FUNCTIONS
//*********************************************************************************************************************
    RobotPlanningExecution::RobotPlanningExecution(
            const std::string groupname, const double max_vel_scale_factor,
            const int num_planning_attempts, const int planning_time,
            const RobotOptionFlag options)
            : move_group_(groupname), options_(options) {
        assert(max_vel_scale_factor >= 0.0 && max_vel_scale_factor <= 1.0);
        assert(num_planning_attempts > 0);
        assert(planning_time > 0);

        ros::NodeHandle nodeHandler;
        ros::Publisher display_publisher = nodeHandler.advertise<moveit_msgs::DisplayTrajectory>(
                "/MoveGroup/display_planned_path", 1, true);

        this->move_group_.setMaxVelocityScalingFactor(max_vel_scale_factor);
        this->move_group_.setNumPlanningAttempts(num_planning_attempts);
        this->move_group_.setPlanningTime(planning_time);
        this->move_group_.setPlannerId("RRTConnectkConfigDefault");

        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Initialized robot");
            ROS_INFO("\tname: %s", groupname.c_str());
            ROS_INFO("\tmax_vel_scale_factor: %f", max_vel_scale_factor);
            ROS_INFO("\tnum_planning_attempts: %d", num_planning_attempts);
            ROS_INFO("\tplanning_time: %d", planning_time);
            ROS_INFO("\toptions: %s", RobotOptionFlagToString(options).c_str());
        }
    }

    const geometry_msgs::Pose    RobotPlanningExecution::getCurrentPose() const {
        return this->move_group_.getCurrentPose().pose;
    }

//*********************************************************************************************************************
// LIN
//*********************************************************************************************************************
// All below plan functions will plan a linear trajectory to the given pose and output the resulting plan
// in the trajectory_plan variable.
// The returned value is the fraction of the plan which is feasible. 
    const double    RobotPlanningExecution::planPoseByPose(
            const geometry_msgs::Pose &target_pose,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        std::vector<geometry_msgs::Pose> waypoints;

        waypoints.push_back(this->getCurrentPose());
        waypoints.push_back(target_pose);
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = this->move_group_.computeCartesianPath(
                waypoints,
                0.01,    // eef_step
                0.0,    // jump_threshold
                trajectory);

        if (trajectory_plan != NULL) {
            trajectory_plan->trajectory_ = trajectory;
        }

        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO(
                    "Visualizing LIN plan (%.2f%% acheived)\n\t Pos: x=%f, y=%f, z=%f,\n\t Rot: x=%f, y=%f, z=%f, w=%f",
                    fraction * 100.0,
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z,
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w);
            if (fraction < 1.0) {
                ROS_WARN("Plan not fully feasible");
            }
        }

        return fraction;
    }

    const double    RobotPlanningExecution::planPoseByXYZRPY(
            const double pos_x, const double pos_y, const double pos_z,
            const double rot_r, const double rot_p, const double rot_y,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = pos_x;
        target_pose.position.y = pos_y;
        target_pose.position.z = pos_z;
        target_pose.orientation = this->QuaternionFromRPY(rot_r, rot_p, rot_y);
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

    const double    RobotPlanningExecution::planRelativePoseByXYZRPY(
            const double pos_x, const double pos_y, const double pos_z,
            const double rot_r, const double rot_p, const double rot_y,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose = this->getCurrentPose();
        target_pose.position.x += pos_x;
        target_pose.position.y += pos_y;
        target_pose.position.z += pos_z;
        target_pose.orientation = this->AddRPYToQuaternion(rot_r, rot_p, rot_y, target_pose.orientation);
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

    const double    RobotPlanningExecution::planPoseByXYZ(
            const double pos_x, const double pos_y, const double pos_z,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = pos_x;
        target_pose.position.y = pos_y;
        target_pose.position.z = pos_z;
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

    const double    RobotPlanningExecution::planRelativePoseByXYZ(
            const double pos_x, const double pos_y, const double pos_z,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose = this->getCurrentPose();
        target_pose.position.x += pos_x;
        target_pose.position.y += pos_y;
        target_pose.position.z += pos_z;
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

    const double    RobotPlanningExecution::planPoseByRPY(
            const double rot_r, const double rot_p, const double rot_y,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose;
        target_pose.orientation = this->QuaternionFromRPY(rot_r, rot_p, rot_y);
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

    const double RobotPlanningExecution::planRelativePoseByRPY(
            const double rot_r, const double rot_p, const double rot_y,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose = this->getCurrentPose();
        target_pose.orientation = this->AddRPYToQuaternion(rot_r, rot_p, rot_y, target_pose.orientation);
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

    const double    RobotPlanningExecution::planPoseByQuaternion(
            const double rot_x, const double rot_y, const double rot_z, const double rot_w,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose = this->getCurrentPose();
        target_pose.orientation.x = rot_x;
        target_pose.orientation.y = rot_y;
        target_pose.orientation.z = rot_z;
        target_pose.orientation.w = rot_w;
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

    const double RobotPlanningExecution::planRelativePoseByQuaternion(
            const double rot_x, const double rot_y, const double rot_z, const double rot_w,
            moveit::planning_interface::MoveGroup::Plan *trajectory_plan) const {
        geometry_msgs::Pose target_pose = this->getCurrentPose();
        target_pose.orientation.x *= rot_x;
        target_pose.orientation.y *= rot_y;
        target_pose.orientation.z *= rot_z;
        target_pose.orientation.w *= rot_w;
        return this->planPoseByPose(target_pose, trajectory_plan);
    }

// All goTo functions will plan a linear trajectory to the given pose and execute the plan
// The returned value is the fraction of plan which is feasible.
    const double    RobotPlanningExecution::goToPoseByPose(const geometry_msgs::Pose &target_pose) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planPoseByPose(target_pose, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToPoseByXYZRPY(
            const double pos_x, const double pos_y, const double pos_z,
            const double rot_r, const double rot_p, const double rot_y) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planPoseByXYZRPY(pos_x, pos_y, pos_z, rot_r, rot_p, rot_y, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToPoseByXYZ(
            const double pos_x, const double pos_y, const double pos_z) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planPoseByXYZ(pos_x, pos_y, pos_z, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToPoseByRPY(
            const double rot_r, const double rot_p, const double rot_y) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planPoseByRPY(rot_r, rot_p, rot_y, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToPoseByQuaternion(
            const double rot_x, const double rot_y, const double rot_z, const double rot_w) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planPoseByQuaternion(rot_x, rot_y, rot_z, rot_w, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToRelativePoseByXYZRPY(
            const double pos_x, const double pos_y, const double pos_z,
            const double rot_r, const double rot_p, const double rot_y) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planRelativePoseByXYZRPY(pos_x, pos_y, pos_z, rot_r, rot_p, rot_y, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToRelativePoseByXYZ(
            const double pos_x, const double pos_y, const double pos_z) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planRelativePoseByXYZ(pos_x, pos_y, pos_z, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToRelativePoseByRPY(
            const double rot_r, const double rot_p, const double rot_y) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planRelativePoseByRPY(rot_r, rot_p, rot_y, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

    const double    RobotPlanningExecution::goToRelativePoseByQuaternion(
            const double rot_x, const double rot_y, const double rot_z, const double rot_w) const {
        moveit::planning_interface::MoveGroup::Plan trajectory_plan;
        double fraction = this->planRelativePoseByQuaternion(rot_x, rot_y, rot_z, rot_w, &trajectory_plan);
        if (this->isOption(ROBOT_OPTION_VERBOSE_INFO)) {
            ROS_INFO("Executing plan");
        }
        if (fraction < 1.0) {
            ROS_ERROR("Plan not fully feasible");
        }
        if (!this->isOption(ROBOT_OPTION_DUMMY_ROBOT)) {
            this->move_group_.execute(trajectory_plan);
        }
        ros::Duration(0.5).sleep();
        return fraction;
    }

//*********************************************************************************************************************
// PTP
//*********************************************************************************************************************
    const bool RobotPlanningExecution::planPosePTP(double x, double y, double z, double roll, double pitch,
                                                   double yaw) const {
        geometry_msgs::Pose target_pose;
        target_pose.orientation = QuaternionFromRPY(roll, pitch, yaw);
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroup::Plan pose_plan;
        bool success = move_group_.plan(pose_plan);

        ROS_INFO("Visualizing PTP plan (pose goal) %s \n\t Pos: x=%f, y=%f, z=%f,\n\t Rot: x=%f, y=%f, z=%f, w=%f",
                 success ? "" : "FAILED",
                 target_pose.position.x,
                 target_pose.position.y,
                 target_pose.position.z,
                 target_pose.orientation.x,
                 target_pose.orientation.y,
                 target_pose.orientation.z,
                 target_pose.orientation.w);
        return success;
    }

    const bool RobotPlanningExecution::planRelativePosePTP(double x, double y, double z, double roll, double pitch,
                                                           double yaw) const {
        geometry_msgs::Pose target_pose = this->getCurrentPose();
        target_pose.orientation = this->AddRPYToQuaternion(roll, pitch, yaw, target_pose.orientation);
        target_pose.position.x += x;
        target_pose.position.y += y;
        target_pose.position.z += z;
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroup::Plan pose_plan;
        bool success = move_group_.plan(pose_plan);

        ROS_INFO("Visualizing PTP plan (pose goal) %s \n\t Pos: x=%f, y=%f, z=%f,\n\t Rot: x=%f, y=%f, z=%f, w=%f",
                 success ? "" : "FAILED",
                 target_pose.position.x,
                 target_pose.position.y,
                 target_pose.position.z,
                 target_pose.orientation.x,
                 target_pose.orientation.y,
                 target_pose.orientation.z,
                 target_pose.orientation.w);
        return success;
    }

    const bool RobotPlanningExecution::goToPosePTP(double x, double y, double z, double roll, double pitch,
                                                   double yaw) const {
        geometry_msgs::Pose target_pose;
        target_pose.orientation = QuaternionFromRPY(roll, pitch, yaw);
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_.setPoseTarget(target_pose);
        bool success = move_group_.move();

        ROS_INFO("Executing PTP plan (pose goal) %s \n\t Pos: x=%f, y=%f, z=%f,\n\t Rot: x=%f, y=%f, z=%f, w=%f",
                 success ? "" : "FAILED",
                 target_pose.position.x,
                 target_pose.position.y,
                 target_pose.position.z,
                 target_pose.orientation.x,
                 target_pose.orientation.y,
                 target_pose.orientation.z,
                 target_pose.orientation.w);
        return success;
    }

    const bool RobotPlanningExecution::goToRelativePosePTP(double x, double y, double z, double roll, double pitch,
                                                           double yaw) const {
        geometry_msgs::Pose target_pose = this->getCurrentPose();
        target_pose.orientation = this->AddRPYToQuaternion(roll, pitch, yaw, target_pose.orientation);
        target_pose.position.x += x;
        target_pose.position.y += y;
        target_pose.position.z += z;
        move_group_.setPoseTarget(target_pose);
        bool success = move_group_.move();

        ROS_INFO("Executing PTP plan (pose goal) %s \n\t Pos: x=%f, y=%f, z=%f,\n\t Rot: x=%f, y=%f, z=%f, w=%f",
                 success ? "" : "FAILED",
                 target_pose.position.x,
                 target_pose.position.y,
                 target_pose.position.z,
                 target_pose.orientation.x,
                 target_pose.orientation.y,
                 target_pose.orientation.z,
                 target_pose.orientation.w);
        return success;
    }

    const bool RobotPlanningExecution::homeRobot() const {
        std::vector<double> group_variable_values;
        move_group_.getCurrentState()->copyJointGroupPositions(
                move_group_.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_.getName()),
                group_variable_values);
        group_variable_values[0] = 0.0;
        group_variable_values[1] = -M_PI / 2;
        group_variable_values[2] = M_PI / 2;
        group_variable_values[3] = 0.0;
        group_variable_values[4] = M_PI / 2;
        group_variable_values[5] = 0.0;
        move_group_.setJointValueTarget(group_variable_values);
        bool success = move_group_.move();

        ROS_INFO(
                "Executing homing plan (joint space home goal) %s \n\t Joints: A1=%f, A2=%f, A3=%f,\n\t\t A4=%f, A5=%f, A6=%f",
                success ? "" : "FAILED",
                group_variable_values[0],
                group_variable_values[1],
                group_variable_values[2],
                group_variable_values[3],
                group_variable_values[4],
                group_variable_values[5]);
        return success;
    }

    const void RobotPlanningExecution::moveRobot(double max_velocity_scale_factor) const {
        move_group_.setMaxVelocityScalingFactor(max_velocity_scale_factor);
        ROS_INFO("Executing plan");
        move_group_.move();
    }

//*********************************************************************************************************************
// PRIVATE FUNCTIONS
//*********************************************************************************************************************
    const bool    RobotPlanningExecution::isOption(const RobotOptionFlag option) const {
        return this->options_ & option;
    }

    const geometry_msgs::Quaternion    RobotPlanningExecution::QuaternionFromRPY(
            const double roll, const double pitch, const double yaw) const {
        return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    }

    const geometry_msgs::Quaternion    RobotPlanningExecution::AddRPYToQuaternion(
            const double roll, const double pitch, const double yaw, const geometry_msgs::Quaternion &q) const {
        tf::Quaternion add_tf = tf::createQuaternionFromRPY(roll, pitch, yaw);
        tf::Quaternion q_tf;
        tf::quaternionMsgToTF(q, q_tf);
        q_tf *= add_tf;
        geometry_msgs::Quaternion result;
        tf::quaternionTFToMsg(q_tf, result);
        return result;
    }
}

