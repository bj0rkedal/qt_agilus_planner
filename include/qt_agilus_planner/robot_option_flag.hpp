//
// Original author: Adam Leon Kleppe
//
// Used in the Master's thesis for planning and execution of trajectories in MoveIt! for ROS.
//
#ifndef __IRONHIDE__ROBOT_OPTION_FLAG_HPP__
#define __IRONHIDE__ROBOT_OPTION_FLAG_HPP__
namespace ih
{
//! Options for the robot
/*!
 * The options are used for configuring a RobotPlanningExecution and RobotGripper instance.
 * \sa RobotPlanningExecution, RobotGripper
 */
enum RobotOptionFlag {
	ROBOT_OPTION_NONE = 		1 << 0, /*!< No option configured */
	ROBOT_OPTION_VERBOSE_INFO = 	1 << 1, /*!< Outputs verbose information of the instance's actions */
	ROBOT_OPTION_DUMMY_ROBOT = 	1 << 2  /*!< Makes the robot a dummy, so that no movement is executed */
};

//! Operator overload for applying multiple flags
inline RobotOptionFlag operator| ( RobotOptionFlag a, RobotOptionFlag b ) {
	return static_cast<RobotOptionFlag>( static_cast<int>(a) | static_cast<int>(b) );
};

inline RobotOptionFlag RobotOptionFlagFromInt(int flag_num){
	return static_cast<RobotOptionFlag>( flag_num );
};

inline std::string RobotOptionFlagToString(RobotOptionFlag flag){
	std::string flagString = "";
	if(flag & ROBOT_OPTION_VERBOSE_INFO)
	{
		return "NONE";
	}
	if(flag & ROBOT_OPTION_VERBOSE_INFO)
	{
		flagString += "VERBOSE_INFO ";
	}
	if(flag & ROBOT_OPTION_DUMMY_ROBOT)
	{
		flagString += "DUMMY_ROBOT ";
	}
	return flagString;
};
}

#endif // __IRONHIDE__ROBOT_OPTION_FLAG_HPP__

