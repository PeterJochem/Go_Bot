#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "moveit_msgs/MoveGroupFeedback.h"
#include <iostream>

/** @brief Provides high level abstraction and motion planning services for the GoBot */
class go_motion_planner {
	public:
		
		ros::NodeHandle node_handle;
		go_motion_planner();
        

		// Utilities
		/** @brief 
		*/
		geometry_msgs::Pose create_pose(float x, float y, float z, float roll, float pitch, float yaw);
	
		/** @brief Create a pose goal relative to the robot's end effector's current pose
	 	*/
		geometry_msgs::Pose create_relative_pose(float d_x, float d_y, float d_z, float d_roll, float d_pitch, float d_yaw);
		
		/** @brief 
		 */	
		geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw);
		std::tuple<float, float, float> rpy_from_quaternion(geometry_msgs::Quaternion q);
		
		void add_orientation_constraints(float roll, float pitch, float yaw);	
			
		void setup_subscribers();

		void process_move_group_feedback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

		// Services
		/** @brief 
		 */
		void setup_services();
		
		ros::ServiceServer home_position_service;
		ros::ServiceServer pickup_piece_service;

		bool move_to_home_position(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);	
		bool pickup_piece(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
			
		// Temporary Stuff for Testing 
		bool move_to_position();
	
	private:

		/** @brief  
		 */
		void initialize_moveit();
		
		// MoveIt data structures 	
		std::string PLANNING_GROUP;
	  	moveit::planning_interface::MoveGroupInterface* move_group;
  		moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
  		const moveit::core::JointModelGroup* joint_model_group;
				

	

};
