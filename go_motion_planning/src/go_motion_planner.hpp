/** @file
 *  @brief go_motion_planner header declares a set of ROS services for picking and placing go_pieces
 *
 * Parameters: /home_position_x - Gripper frame's pose x value in the home position 
 *             /home_position_y - Gripper frame's pose y value in the home position
 *             /home_position_z - Gripper frame's pose z value in the home position
 *             /home_position_roll - Gripper frame's pose roll value (radians) in the home position
 *             /home_position_pitch - Gripper frame's pose pitch (radians) value in the home position
 *             /home_position_yaw - Gripper frame's pose yaw (radians) value in the home position
 * 		
 *	       /row_width - The width of the go board's square 
 *             /row_height - The height of the go board's square
 *
 *             /z_board_plane - The z-axis value of the top of the board's plane
 *             /piece_height - Height of the go piece
 *             /z_stance_offset - The distance in the z-axis to be offset by in the stance pose
 *             /finger_length - The length of the gripper's fingers
 *		
 *
 *
 * Publishes: /rx200/gripper_controller/command - Position controller of the robot's gripper 
 *            /rx200/gripper/command - PWM control of the gripper
 * 
 * Subscribes: /rx200/gripper/command - PWM control feedback from the gripper 
 *
 * Services: /home_position - Move robot into a home pose 
 *             /pickup_piece - Pickup a piece at the given (row, column) pair
 *             /place_piece_in_unused - Place a piece in the used pile 
 *             /remove_piece - Pickup and remove a piece at the given (row, column) from the board
 *             /pickup_unused_piece - Pickup an unused piece from the empty pile
 *             /place_piece - Place a piece at the given (row, column) on the board
 *             /play_piece - Pickup an used piece and play it on the board at (row, column) 
 *             /pickup_set_of_pieces - Pickup a set of pieces and remove them from the board   
 *   
 */

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
#include "go_motion_planning/place_piece_in_unused.h"
#include "go_motion_planning/remove_piece.h"
#include "go_motion_planning/pickup_unused_piece.h"
#include "go_motion_planning/place_piece.h"
#include "go_motion_planning/play_piece.h"
#include "go_motion_planning/pickup_set_of_pieces.h"
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
		geometry_msgs::Point next_empty_unused_piece_location();
		geometry_msgs::Point next_unused_piece_location();
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
		ros::ServiceServer place_piece_in_unused_service;
		ros::ServiceServer remove_piece_service;
		ros::ServiceServer pickup_unused_piece_service;
		ros::ServiceServer place_piece_service;
		ros::ServiceServer play_piece_service;
		ros::ServiceServer pickup_set_of_pieces_service;

		// Service Implementations
		bool pickup_piece(int row, int column);
		bool remove_piece(int row, int column);
		bool place_in_unused_pile();
		bool place_piece(int row, int column);
		bool pickup_unused_piece();
		bool move_to_home_position();
		bool play_piece(int row, int column);
		//bool pickup_set_of_pieces();
	

		// Service Bindings
		bool move_to_home_position_service_binding(go_motion_planning::home_position::Request &req, go_motion_planning::home_position::Response &res);	
		bool pickup_piece_service_binding(go_motion_planning::pickup_piece::Request &req, go_motion_planning::pickup_piece::Response &res);
		bool place_piece_in_unused_service_binding(go_motion_planning::place_piece_in_unused::Request &req, go_motion_planning::place_piece_in_unused::Response &res);	
		bool remove_piece_service_binding(go_motion_planning::remove_piece::Request &req, go_motion_planning::remove_piece::Response &res);
		bool pickup_unused_piece_service_binding(go_motion_planning::pickup_unused_piece::Request &req, go_motion_planning::pickup_unused_piece::Response &res);
		bool place_piece_service_binding(go_motion_planning::place_piece::Request &req, go_motion_planning::place_piece::Response &res);
		bool play_piece_service_binding(go_motion_planning::play_piece::Request &req, go_motion_planning::play_piece::Response &res);
		bool pickup_set_of_pieces_service_binding(go_motion_planning::pickup_set_of_pieces::Request &req, go_motion_planning::pickup_set_of_pieces::Response &res); 

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
		double row_width, row_height, z_board_plane, finger_length, piece_height, z_stance_offset;
		
		int num_unused_pieces = 0;
		geometry_msgs::Point convert_board_frame_to_world(geometry_msgs::Point board_point);	
						

};
