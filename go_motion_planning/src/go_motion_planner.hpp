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
#include <interbotix_sdk/arm_obj.h>
#include <std_msgs/Float64.h>
#include "go_motion_planning/home_position.h"
#include "go_motion_planning/pickup_piece.h"
#include "tf2_ros/transform_listener.h"

/** @brief Provides high level abstraction and motion planning services for the GoBot */
class go_motion_planner {
	public:
		
		ros::NodeHandle node_handle;
		tf2_ros::Buffer* tfBuffer; 
		tf2_ros::TransformListener* tfListener;

		go_motion_planner();
       	 
		// There may be a better way to do this?
		RobotArm* a;


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
			
		bool move_to_pose(geometry_msgs::Pose pose);
		bool cartesian_sequence(geometry_msgs::Pose);
		geometry_msgs::Pose stance_pose(int row, int column);	
		geometry_msgs::Point board_location(int row, int column);
		geometry_msgs::Quaternion grasp_orientation(geometry_msgs::Point); 	

		void setup_transform_listeners();

		void load_param_values();
		void setup_subscribers();
		void setup_publishers();
		ros::Publisher gripper_position_pub;
		ros::Publisher gripper_pwm_pub;
			
		
		bool open_gripper_simulation();
		bool close_gripper_simulation(void);
		void process_gripper_data(const std_msgs::Float64::ConstPtr& msg);

		// Services
		/** @brief 
		 */
		void setup_services();

		ros::ServiceServer home_position_service;
		ros::ServiceServer pickup_piece_service;

		bool move_to_home_position(go_motion_planning::home_position::Request &req, go_motion_planning::home_position::Response &res);	
		bool pickup_piece(go_motion_planning::pickup_piece::Request &req, go_motion_planning::pickup_piece::Response &res);
			
		// Temporary Stuff for Testing 
		bool move_to_position();
	


	private:

		/** @brief  
		 */
		void initialize_moveit();
		void wait_for_params();

		// MoveIt data structures 	
		std::string PLANNING_GROUP;
	  	moveit::planning_interface::MoveGroupInterface* move_group;
		moveit::planning_interface::MoveGroupInterface* gripper_move_group;
  		moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
  		const moveit::core::JointModelGroup* joint_model_group;
		
		geometry_msgs::Pose home_pose;	
		
	

};
