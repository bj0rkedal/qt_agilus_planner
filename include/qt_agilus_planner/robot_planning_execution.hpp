//
// Original author: Adam Leon Kleppe
// Modified by: Asgeir Bjørkedal
//
// Used in the Master's thesis for planning and execution of trajectories in MoveIt! for ROS.
//
#ifndef __IRONHIDE__ROBOT_PLANNING_EXECUTION_HPP__
#define __IRONHIDE__ROBOT_PLANNING_EXECUTION_HPP__

#include <assert.h>
#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "robot_option_flag.hpp"

namespace ih {

//! A class for planning and executing trajectories for a given robot
/*! 
 * The RobotPlanningExecution class can plan and execute trajectories for a given robot. 
 * It currently can only perform linear movement. 
 */
    class RobotPlanningExecution {
    public:
        //! Initializes the class for a specific robot
        /*!
         * \param groupname The name of the MoveIt! group. Default set to manipulator
         * \param max_vel_scale_factor The scale factor for the speed. Default is 0.5, which is 50%
         * \param num_planning_attempts The number of attempts before accepting a plan. Default is 5
         * \param planning_time The maximum time for generating a plan. Default is 10.
         * \param options Option flags for the robot. Default none.
         */
        RobotPlanningExecution(
                const std::string groupname = "manipulator",
                const double max_vel_scale_factor = 0.5,
                const int num_planning_attempts = 5,
                const int planning_time = 10,
                const RobotOptionFlag options = ROBOT_OPTION_NONE);

        //! Gets the current pose of the robot
        /*!
         * \return The current pose of the robot
         */
        const geometry_msgs::Pose getCurrentPose() const;

        //! Makes the robot go to the target pose
        /*!
         * \param target_pose The target pose.
         * \return The fraction of the plan which is feasible.
         */
        const double goToPoseByPose(const geometry_msgs::Pose &target_pose) const;

        //! Makes the robot go to the target pose
        /*!
         * \param pos_x The target x position.
         * \param pos_y The target y position.
         * \param pos_z The target z position.
         * \param rot_r The target roll.
         * \param rot_p The target pitch.
         * \param rot_y The target yaw.
         * \return The fraction of the plan which is feasible.
         */
        const double goToPoseByXYZRPY(
                const double pos_x, const double pos_y, const double pos_z,
                const double rot_r, const double rot_p, const double rot_y) const;

        //! Makes the robot go to the target pose
        /*!
         * \param pos_x The target x position.
         * \param pos_y The target y position.
         * \param pos_z The target z position.
         * \return The fraction of the plan which is feasible.
         */
        const double goToPoseByXYZ(const double pos_x, const double pos_y, const double pos_z) const;

        //! Makes the robot go to the target pose
        /*!
         * \param rot_r The target roll.
         * \param rot_p The target pitch.
         * \param rot_y The target yaw.
         * \return The fraction of the plan which is feasible.
         */
        const double goToPoseByRPY(const double rot_r, const double rot_p, const double rot_y) const;

        //! Makes the robot go to the target pose
        /*!
         * \param rot_x The target x.
         * \param rot_y The target y.
         * \param rot_z The target z.
         * \param rot_w The target w.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double goToPoseByQuaternion(
                const double rot_x, const double rot_y, const double rot_z, const double rot_w) const;

        //! Makes the robot go to the relative pose
        /*!
         * \param pos_x The relative x position.
         * \param pos_y The relative y position.
         * \param pos_z The relative z position.
         * \param rot_r The relative roll.
         * \param rot_p The relative pitch.
         * \param rot_y The relative yaw.
         * \return The fraction of the plan which is feasible.
         */
        const double goToRelativePoseByXYZRPY(
                const double pos_x, const double pos_y, const double pos_z,
                const double rot_r, const double rot_p, const double rot_y) const;

        //! Makes the robot go to the relative pose
        /*!
         * \param pos_x The relative x position.
         * \param pos_y The relative y position.
         * \param pos_z The relative z position.
         * \return The fraction of the plan which is feasible.
         */
        const double goToRelativePoseByXYZ(const double pos_x, const double pos_y, const double pos_z) const;

        //! Makes the robot go to the relative pose
        /*!
         * \param rot_r The relative roll.
         * \param rot_p The relative pitch.
         * \param rot_y The relative yaw.
         * \return The fraction of the plan which is feasible.
         */
        const double goToRelativePoseByRPY(const double rot_r, const double rot_p, const double rot_y) const;

        //! Makes the robot go to the relative pose
        /*!
         * \param rot_x The relative x.
         * \param rot_y The relative y.
         * \param rot_z The relative z.
         * \param rot_w The relative w.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double goToRelativePoseByQuaternion(
                const double rot_x, const double rot_y, const double rot_z, const double rot_w) const;

        //! Plans the trajectory to the target pose
        /*!
         * \param target_pose The target pose
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible
         */
        const double planPoseByPose(
                const geometry_msgs::Pose &target_pose,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the target pose
        /*!
         * \param pos_x The target x position.
         * \param pos_y The target y position.
         * \param pos_z The target z position.
         * \param rot_r The target roll.
         * \param rot_p The target pitch.
         * \param rot_y The target yaw.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planPoseByXYZRPY(
                const double pos_x, const double pos_y, const double pos_z,
                const double rot_r, const double rot_p, const double rot_y,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the target pose
        /*!
         * \param pos_x The target x position.
         * \param pos_y The target y position.
         * \param pos_z The target z position.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planPoseByXYZ(
                const double pos_x, const double pos_y, const double pos_z,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the target pose
        /*!
         * \param rot_r The target roll.
         * \param rot_p The target pitch.
         * \param rot_y The target yaw.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planPoseByRPY(
                const double rot_r, const double rot_p, const double rot_y,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the relative pose
        /*!
         * \param rot_x The target x.
         * \param rot_y The target y.
         * \param rot_z The target z.
         * \param rot_w The target w.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planPoseByQuaternion(
                const double rot_x, const double rot_y, const double rot_z, const double rot_w,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the relative pose
        /*!
         * \param pos_x The relative x position.
         * \param pos_y The relative y position.
         * \param pos_z The relative z position.
         * \param rot_r The relative roll.
         * \param rot_p The relative pitch.
         * \param rot_y The relative yaw.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planRelativePoseByXYZRPY(
                const double pos_x, const double pos_y, const double pos_z,
                const double rot_r, const double rot_p, const double rot_y,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the relative pose
        /*!
         * \param pos_x The relative x position.
         * \param pos_y The relative y position.
         * \param pos_z The relative z position.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planRelativePoseByXYZ(
                const double pos_x, const double pos_y, const double pos_z,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the relative pose
        /*!
         * \param rot_r The relative roll.
         * \param rot_p The relative pitch.
         * \param rot_y The relative yaw.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planRelativePoseByRPY(
                const double rot_r, const double rot_p, const double rot_y,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans the trajectory to the relative pose
        /*!
         * \param rot_x The relative x.
         * \param rot_y The relative y.
         * \param rot_z The relative z.
         * \param rot_w The relative w.
         * \param trajectory_plan The planned trajectory
         * \return The fraction of the plan which is feasible.
         */
        const double planRelativePoseByQuaternion(
                const double rot_x, const double rot_y, const double rot_z, const double rot_w,
                moveit::planning_interface::MoveGroup::Plan *trajectory_plan = NULL) const;

        //! Plans a PTP trajectory in world coordinates. The trajectory is visualized in MoveIt!
        /*!
         * \param x The target world x coordinate.
         * \param y The target world y coordinate.
         * \param z The target world z coordinate.
         * \param roll The target roll in radians.
         * \param pitch The target pitch in radians.
         * \param yaw The target yaw in radians.
         * \return True if the plan is successful and feasible. False if the plan is not successful.
         */
        const bool planPosePTP(double x, double y, double z, double roll, double pitch, double yaw) const;

        //! Plans a relative PTP trajectory from current pose. The trajectory is visualized in MoveIt!
        /*!
         * \param x The relative x to be added.
         * \param y The relative y to be added.
         * \param z The relative z to be added.
         * \param roll The relative roll in radians to be added.
         * \param pitch The relative pitch in radians to be added.
         * \param yaw The relative yaw in radians to be added.
         * \return True if the plan is successful and feasible. False if the plan is not successful.
         */
        const bool planRelativePosePTP(double x, double y, double z, double roll, double pitch, double yaw) const;

        //! Plans a PTP trajectory in world coordinates and commands the robot to move.
        /*!
         * \param x The target world x coordinate.
         * \param y The target world y coordinate.
         * \param z The target world z coordinate.
         * \param roll The target roll in radians.
         * \param pitch The target pitch in radians.
         * \param yaw The target yaw in radians.
         * \return True if the plan is successful and feasible. False if the plan is not successful.
         */
        const bool goToPosePTP(double x, double y, double z, double roll, double pitch, double yaw) const;

        //! Plans a relative PTP trajectory from current pose and commands the robot to move.
        /*!
         * \param x The relative x to be added.
         * \param y The relative y to be added.
         * \param z The relative z to be added.
         * \param roll The relative roll in radians to be added.
         * \param pitch The relative pitch in radians to be added.
         * \param yaw The relative yaw in radians to be added.
         * \return True if the plan is successful and feasible. False if the plan is not successful.
         */
        const bool goToRelativePosePTP(double x, double y, double z, double roll, double pitch, double yaw) const;

        //! Plans a trajectory from current pose to home pose and executes it.
        /*!
         * \return True if the plan is successful and feasible. False if the plan is not successful.
         */
        const bool homeRobot() const;

        //! Command the robot to execute the current plan of the move group.
        /*!
         * \param max_velocity_scale_factor The movement velocity of the robot. 0.1 is equal to 10%.
         */
        const void moveRobot(double max_velocity_scale_factor) const;

    private:
        //! Checks if the instance has flagged an option
        /*!
         * \param option The option checked for
         * \return Returns true if the option is flagged, else false
         */
        const bool isOption(const RobotOptionFlag option) const;

        //! Creates a quaternion from roll, pitch, yaw
        /*!
         * \param roll The wanted roll
         * \param pitch The wanted pitch
         * \param yaw The wanted yaw
         * \return The resulting quaternion
         */
        const geometry_msgs::Quaternion QuaternionFromRPY(
                const double roll, const double pitch, const double yaw) const;


        //! Adds roll, pitch, yaw to add to a quaternion
        /*!
         * \param roll The added roll
         * \param pitch The added pitch
         * \param yaw The added yaw
         * \param q The input quaternion
         * \return The resulting quaternion
         */
        const geometry_msgs::Quaternion AddRPYToQuaternion(
                const double roll, const double pitch, const double yaw, const
        geometry_msgs::Quaternion &q) const;

        //! The move group used by MoveIt!
        mutable moveit::planning_interface::MoveGroup move_group_;

        //! The flag option for the instance
        const RobotOptionFlag options_;
    };
}
#endif //__IRONHIDE__ROBOT_PLANNING_EXECUTION_HPP__

