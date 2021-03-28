/** @file 
 *  @brief Service node to use MoveIt to pick and place go pieces
 *
 *  Parameters: /home_position_x - Gripper frame's pose x value in the home position 
 *       	/home_position_y - Gripper frame's pose y value in the home position
 *       	/home_position_z - Gripper frame's pose z value in the home position
 *       	/home_position_roll - Gripper frame's pose roll value (radians) in the home position
 *       	/home_position_pitch - Gripper frame's pose pitch (radians) value in the home position
 *       	/home_position_yaw - Gripper frame's pose yaw (radians) value in the home position
 *
 *       	/row_width - The width of the go board's square 
 *       	/row_height - The height of the go board's square
 *
 *       	/z_board_plane - The z-axis value of the top of the board's plane
 *       	/piece_height - Height of the go piece
 *       	/z_stance_offset - The distance in the z-axis to be offset by in the stance pose
 *       	/finger_length - The length of the gripper's fingers
 *
 *  Publishers: /rx200/gripper_controller/command - Position controller of the robot's gripper 
 *		/rx200/gripper/command - PWM control of the gripper
 *  	
 *	
 *  Subscribers: /rx200/gripper/command - PWM control feedback from the gripper 
 *	
 *  Services: /home_position - Move robot into a home pose 
 *            /pickup_piece - Pickup a piece at the given (row, column) pair
 *            /place_piece_in_unused - Place a piece in the used pile 
 *            /remove_piece - Pickup and remove a piece at the given (row, column) from the board
 *            /pickup_unused_piece - Pickup an unused piece from the empty pile
 *            /place_piece - Place a piece at the given (row, column) on the board
 *            /play_piece - Pickup an used piece and play it on the board at (row, column) 
 *            /pickup_set_of_pieces - Pickup a set of pieces and remove them from the board   
 */


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "go_motion_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "moveit_msgs/MoveGroupFeedback.h"
#include "actionlib_msgs/GoalStatus.h"
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include "go_motion_planning/home_position.h"
#include "go_motion_planning/pickup_piece.h"
#include <math.h>

// TO Do:
// 1) Create a service to remove a set of pieces?
// 2) Add the pieces to Gazebo programmaticly
// 3) Change the node names and the file names 
// 4) Make things in the class private/public
// 5) Write units tests!!


/** @brief Construct go_motion_planner object */
go_motion_planner::go_motion_planner() {
	
	node_handle = ros::NodeHandle();
	initialize_moveit();
	load_param_values();
	setup_services();			
	setup_publishers();
	setup_subscribers();
	setup_transform_listeners();
	setup_transform_listeners();
	
}

/** @brief Wait until required parameters are on the server and then return */
void go_motion_planner::wait_for_params() {

	std::cout << "Waiting for parameters to be available on the param server" << std::endl;
        while (!node_handle.hasParam("/home_position_x") && !node_handle.hasParam("/z_board_plane")) {
                ;
        }
	std::cout << "Parameters are available on the param server" << std::endl;
}

/** @brief Load parameters from the ROS parameter server */ 
void go_motion_planner::load_param_values() { 
	
	double x, y, z, roll, pitch, yaw;
	
	wait_for_params();
	node_handle.getParam("/home_position_x", x);  
	node_handle.getParam("/home_position_y", y);
	node_handle.getParam("/home_position_z", z);
	node_handle.getParam("/home_position_roll", roll);
	node_handle.getParam("/home_position_pitch", pitch);
	node_handle.getParam("/home_position_yaw", yaw);

	node_handle.getParam("/row_width", row_width);
        node_handle.getParam("/row_height", row_height);
        
	node_handle.getParam("/z_board_plane", z_board_plane);
	node_handle.getParam("/piece_height", piece_height);
	node_handle.getParam("/z_stance_offset", z_stance_offset);
	node_handle.getParam("/finger_length", finger_length);
		
	home_pose = create_pose(x, y, z, roll, pitch, yaw); 
}

/** @brief Advertise all the services provided by the ROS node */
void go_motion_planner::setup_services() {
	
	home_position_service = node_handle.advertiseService("/home_position", &go_motion_planner::move_to_home_position_service_binding, this);
	pickup_piece_service = node_handle.advertiseService("/pickup_piece", &go_motion_planner::pickup_piece_service_binding, this);
	place_piece_in_unused_service = node_handle.advertiseService("/place_piece_in_unused", &go_motion_planner::place_piece_in_unused_service_binding, this);
	remove_piece_service = node_handle.advertiseService("/remove_piece", &go_motion_planner::remove_piece_service_binding, this);
	pickup_unused_piece_service = node_handle.advertiseService("/pickup_unused_piece", &go_motion_planner::pickup_unused_piece_service_binding, this);
	place_piece_service = node_handle.advertiseService("/place_piece", &go_motion_planner::place_piece_service_binding, this);	
	play_piece_service = node_handle.advertiseService("/play_piece", &go_motion_planner::play_piece_service_binding, this);  
	pickup_set_of_pieces_service = node_handle.advertiseService("/pickup_set_of_pieces", &go_motion_planner::pickup_set_of_pieces_service_binding, this);
}

/** @brief Setup ROs subscribers */
void go_motion_planner::setup_subscribers() {

	node_handle.subscribe("/rx200/gripper/command", 1000, &go_motion_planner::process_gripper_data, this);
}

/** @brief Setup the ROS subscriptions */
void go_motion_planner::setup_publishers() {
	
	// This controls the gripper's position in Gazebo only?
	gripper_position_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/rx200/gripper_controller/command", 1);	
	
	// This allows us to do pwm control. The line below is from interbotix_ros_arms/interbotix_sdk/src/interbotix_sdk/robot_manipulation.py
        //self.pub_gripper_command = rospy.Publisher(robot_name + "/gripper/command", Float64, queue_size=100)
	// The arm_node from the interbotix_sdk package will subscribe to this topic and use it to do pwm control
	gripper_pwm_pub = node_handle.advertise<std_msgs::Float64>("/rx200/gripper/command", 100);
}

/** @brief Setup a ROS transform listener for the class */
void go_motion_planner::setup_transform_listeners() { 

	tfBuffer = new tf2_ros::Buffer();
        tfListener = new tf2_ros::TransformListener(*tfBuffer);	
}

/** @brief Not-implemented yet. Requires hardware. Will process the pwm data from the gripper */
void go_motion_planner::process_gripper_data(const std_msgs::Float64::ConstPtr& msg) {
	// Convert what will be a pwm command for the real robot to the gazebo simulation
	// closing or opening the gripper
	// use the gripper_position_pub to make the gripper open/close		
}

/** @brief Setup datastructures and objects in MoveIt */
void go_motion_planner::initialize_moveit() {
	
	PLANNING_GROUP = "interbotix_arm";
  	move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  	planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	
	move_group->setPlanningTime(10.0);	
	move_group->setMaxAccelerationScalingFactor(0.5); // Slower speeds have prevent failure at runtime
        move_group->setMaxVelocityScalingFactor(0.5);	
	
	ROS_INFO_NAMED("MoveItSetupInfo", "Planning frame: %s", move_group->getPlanningFrame().c_str());
	ROS_INFO_NAMED("MoveItSetupInfo", "End effector link: %s", move_group->getEndEffectorLink().c_str());

	// Setup the gripper planning group?
	gripper_move_group = new moveit::planning_interface::MoveGroupInterface("interbotix_gripper");
	gripper_move_group->setGoalTolerance(0.00001);		
}

/** @brief ROS service binding which simply calls move_to_pose with the home_pose as the argument
 */
bool go_motion_planner::move_to_home_position_service_binding(go_motion_planning::home_position::Request &req, go_motion_planning::home_position::Response &res) {
	
	res.success = move_to_pose(home_pose);
	return res.success;
}

/** @brief ROS service binding which simply calls pickup_piece with passed res.row, res.column */
bool go_motion_planner::pickup_piece_service_binding(go_motion_planning::pickup_piece::Request &req, go_motion_planning::pickup_piece::Response &res) {
       
	res.success = pickup_piece(req.row, req.column);
	return res.success;
}

/** @brief ROS service binding which simply calls place_in_unused_pile */
bool go_motion_planner::place_piece_in_unused_service_binding(go_motion_planning::place_piece_in_unused::Request &req, go_motion_planning::place_piece_in_unused::Response &res) {
	
	res.success = place_in_unused_pile();
        return res.success;
}

/** @brief ROS service binding which simply calls remove_piece with the passed req.row and req.column */
bool go_motion_planner::remove_piece_service_binding(go_motion_planning::remove_piece::Request &req, go_motion_planning::remove_piece::Response &res) {

        res.success = remove_piece(req.row, req.column);
        return res.success;
}

/** @brief ROS service binding which simply calls pickup_unused piece*/
bool go_motion_planner::pickup_unused_piece_service_binding(go_motion_planning::pickup_unused_piece::Request &req, go_motion_planning::pickup_unused_piece::Response &res) {
	
	res.success = pickup_unused_piece();
	return res.success;
}

/** @brief ROS service binding which simply calls play_piece with the passed req.row and req.column */
bool go_motion_planner::play_piece_service_binding(go_motion_planning::play_piece::Request &req, go_motion_planning::play_piece::Response &res) {
	
	res.success = play_piece(req.row, req.column);
        return res.success;
}

/** @brief ROS service binding which simply calls place_piece with the passed req.row and req.column */
bool go_motion_planner::place_piece_service_binding(go_motion_planning::place_piece::Request &req, go_motion_planning::place_piece::Response &res) {

        res.success = place_piece(req.row, req.column);
        return res.success;
}

/** @brief Pick up the piece at the given (row, column) on the board. Assumes the robot is not holding a
 *  piece when it is called
 *  @param row - The row of the go board
 *  @param column - The column of the go board */
bool go_motion_planner::pickup_piece(int row, int column) {	

	open_gripper_simulation(); 
	
	// put this in a try block?
	bool sucess = move_to_pose(stance_pose(row, column));
	if (sucess) {
		// Compute the relative z remaining?
		sucess = cartesian_sequence(create_relative_pose(0.0, 0.0, -z_stance_offset, 0.0, 0.0, 0.0));
	}
	
	if (sucess) {
		sucess = close_gripper_simulation();
	}

	if (sucess) {
		sucess = cartesian_sequence(create_relative_pose(0.0, 0.0, z_stance_offset, 0.0, 0.0, 0.0));
	}
		
	return sucess;
}

/** @brief ROS service binding. Pick up the set of pieces in the request message.
 *  @param - req - ROS type of (row, column) pairs of pieces to pickup  
 *  @param res - ROS type which is just a boolean indicating success or failure */
bool go_motion_planner::pickup_set_of_pieces_service_binding(go_motion_planning::pickup_set_of_pieces::Request &req, go_motion_planning::pickup_set_of_pieces::Response &res) { 

	bool success = true;
	if (req.row_coords.size() != req.column_coords.size()) {
		std::cout << "Illegal input to the pickup_set_of_pieces_service" << std::endl;	
		return false;
	}
	
	while (success && req.row_coords.size() > 0 && req.column_coords.size() > 0) {
		success = remove_piece(req.row_coords.back(), req.column_coords.back());
		req.row_coords.pop_back();
		req.column_coords.pop_back();
	}
	
	if (!success) {
        	std::cout << "Motion planning/execution failure while trying to remove a set of pieces" << std::endl;
        }

	return success;
}

/** @brief Place the piece specefied in the request message.
 *  @param - req - ROS type of (row, column) pair of piece to pickup  
 *  @param res - ROS type which is just a boolean indicating success or failure */
bool go_motion_planner::place_piece(int row, int column) {

        // put this in a try block?
        bool sucess = move_to_pose(stance_pose(row, column));

        if (sucess) {
                sucess = cartesian_sequence(create_relative_pose(0.0, 0.0, -z_stance_offset, 0.0, 0.0, 0.0));
        }

        if (sucess) {
                sucess = open_gripper_simulation();
        }

        if (sucess) {
                sucess = cartesian_sequence(create_relative_pose(0.0, 0.0, z_stance_offset, 0.0, 0.0, 0.0));
        }

        return sucess;
}

/** @brief Remove the piece specefied in the request message.
 *  @param - req - ROS type of (row, column) pair of piece to remove from the board 
 *  @param res - ROS type which is just a boolean indicating success or failure */
bool go_motion_planner::remove_piece(int row, int column) {
	
	bool success = pickup_piece(row, column);	
	if (success) { 
		place_in_unused_pile();
	}
				
	return true;
}

/** @brief Get an unused piece at play at the given board location
 *  @param - req - ROS type of (row, column) pair of piece to pickup
 *  @param res - ROS type which is just a boolean indicating success or failure */
bool go_motion_planner::play_piece(int row, int column) {

        bool success = pickup_unused_piece();
        if (success) {
        	place_piece(row, column);
        }

        return success;
}


/** @brief Place the piece the robot is holding into the unused pile */
bool go_motion_planner::place_in_unused_pile() {

	// Change this so that we check the datastuctures to see what the array/line of unused pieces is like
	// and let this determine where we put the piece
	geometry_msgs::Point drop_point = next_empty_unused_piece_location();
	geometry_msgs::Quaternion q = grasp_orientation(drop_point);
	geometry_msgs::Pose p;
	p.orientation = q;
	p.position = drop_point; 
		
	bool success = move_to_pose(p);
	if (success) { 
		success = cartesian_sequence(create_relative_pose(0.0, 0.0, -z_stance_offset, 0.0, 0.0, 0.0));
	}
	
	if (success) {
		success = open_gripper_simulation();
	}
	
	if (success) { 
		success = cartesian_sequence(create_relative_pose(0.0, 0.0, z_stance_offset, 0.0, 0.0, 0.0));
	}
	
	// Increment the data structure which records how many unused pieces there are
	// is the dropping of the piece was successful
	return success;	
}

/** @brief Pickup an unused piece */
bool go_motion_planner::pickup_unused_piece() { 
	
	// Change this so that we check the datastuctures to see what the array/line of unused pieces is like
        // and let this determine where we put the piece
	geometry_msgs::Point pickup_point = next_unused_piece_location();
	geometry_msgs::Quaternion q = grasp_orientation(pickup_point);
        geometry_msgs::Pose p;
        p.orientation = q;
        p.position = pickup_point;
        
	bool success = move_to_pose(p);
	if (success) {
		success = open_gripper_simulation();
	}

	if (success) {
        	success = cartesian_sequence(create_relative_pose(0.0, 0.0, -z_stance_offset, 0.0, 0.0, 0.0));
	}

        if (success) {
                success = close_gripper_simulation();
        }

        if (success) {
        	success = cartesian_sequence(create_relative_pose(0.0, 0.0, z_stance_offset, 0.0, 0.0, 0.0));
	}

        // Increment the data structure which records how many unused pieces there are
        // is the dropping of the piece was successful
        return success;
}


/** @brief Query datastructures to find the place to put the next unused piece */
geometry_msgs::Point go_motion_planner::next_empty_unused_piece_location() { 
	
	// This should be computed by looking at the datastructures to see where the unused pieces are
	// Query data structure to see how many pieces already exist    
        // num_unused_pieces
	geometry_msgs::Point next_free_location;
	next_free_location.x = 0.0;
	next_free_location.y = 0.0;
	next_free_location.z = finger_length + z_board_plane + piece_height/2 + z_stance_offset;

	return convert_board_frame_to_world(next_free_location);
}

/** @brief Query datastructures to find the place where the next unused piece is currently */
geometry_msgs::Point go_motion_planner::next_unused_piece_location() {

        // This should be computed by looking at the datastructures to see where the unused pieces are
        // Query data structure to see how many pieces already exist
        // num_unused_pieces
        geometry_msgs::Point next_free_location;
        next_free_location.x = 0.0;
        next_free_location.y = 0.0;
        next_free_location.z = finger_length + z_board_plane + piece_height/2 + z_stance_offset;

        return convert_board_frame_to_world(next_free_location);
}

/** @brief Convert a point from the board frame to the world frame 
 *  @param point_go_board_frame - The point in the go frame */
geometry_msgs::Point go_motion_planner::convert_board_frame_to_world(geometry_msgs::Point point_go_board_frame) { 

	geometry_msgs::PointStamped point_stamped_world;
        geometry_msgs::PointStamped point_stamped_go_board_frame;
	
	//go_board_point.header.seq = This is filled automatically
        point_stamped_go_board_frame.header.stamp = ros::Time::now();
	point_stamped_go_board_frame.header.frame_id = "go_board";
	point_stamped_go_board_frame.point = point_go_board_frame;

	try {
                tfBuffer->transform(point_stamped_go_board_frame, point_stamped_world, "world");
        }
        catch (tf2::TransformException &ex) {
                // FIX ME This error should propagate up instead
                ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        }

        return point_stamped_world.point;
}

/** @brief Compute the stance pose for a given (row,column) pair on the go board 
 *  @param row - The row value on the go_board
 *  @param column - The column value on the go board */
geometry_msgs::Pose go_motion_planner::stance_pose(int row, int column) { 
	
	geometry_msgs::Pose desired_pose;
	geometry_msgs::Point desired_point = board_location(row, column);
	geometry_msgs::Quaternion desired_orientation = grasp_orientation(desired_point);	
	
	desired_pose.position = desired_point;
	//desired_pose.position.z = 0.083 +  z_board_plane + piece_height/2 + z_stance_offset; // z_stance_height; //desired_point.z + 0.10;
	desired_pose.position.z = finger_length +  z_board_plane + piece_height/2 + z_stance_offset;
	desired_pose.orientation = desired_orientation;
	
	return desired_pose; 
}

/** @brief Compute the (x, y) value for a given (row, column) pair on the go board
 *  @param row - The row value on the go_board
 *  @param column - The column value on the go board */
geometry_msgs::Point go_motion_planner::board_location(int row, int column) {

        geometry_msgs::Point go_board_frame_point;

        //go_board_point.header.seq = This is filled automatically
        go_board_frame_point.x = row_width * row;
        go_board_frame_point.y = row_height * column;
        go_board_frame_point.z = z_board_plane;

	return convert_board_frame_to_world(go_board_frame_point);
}

/** @brief Compute the grasp pose for a given (row, column) pair on the go board 
 *  @param stance_point - The point in the world frame of the desired board location */
geometry_msgs::Quaternion go_motion_planner::grasp_orientation(geometry_msgs::Point stance_point) {	
	
	// Compute the number of radians we need to rotate about the z-axis	
	double roll = atan2(stance_point.y, stance_point.x);
	return quaternion_from_rpy(0.029, 1.522, roll); 
}


/** @brief Compute the stance pose for a given (row,column) pair on the go board
 *  @param row - The row vlaue on the go_board
 *  @param column - The column value on the go board */
bool go_motion_planner::cartesian_sequence(geometry_msgs::Pose pose) { 
	
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(pose);
	
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.90) {
        	move_group->execute(trajectory); // This blocks until the plan is done executing
        	return true;
	}
			
	return false;
}	

/* self.pub_gripper_command = rospy.Publisher(robot_name + "/gripper/command", Float64, queue_size=100)
 * Do PWM
 * bool open_gripper(void) {
 * } */


/** @brief - Subscribe to the same topic as the arm_node topic
 * but instead of doing pwm control like the arm_node, do a simple 
 * approximation by opening the gripper */
bool go_motion_planner::open_gripper_simulation() {	
	
	const robot_state::JointModelGroup* joint_model_group =
        gripper_move_group->getCurrentState()->getJointModelGroup("interbotix_gripper");

        moveit::core::RobotStatePtr current_state = gripper_move_group->getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	// [-0.037, 0.037] is the interval of positions the gripper's two fingers can be in
	joint_group_positions[0] = 0.037/1.75; // Open
        joint_group_positions[1] = -0.037/1.75;

        gripper_move_group->setJointValueTarget(joint_group_positions);
        bool success = (gripper_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
                gripper_move_group->execute(my_plan); // This blocks until the plan is done executing
        }

        return true;	
}

/** @brief - Close the gripper in Gazebo with the position controller */
bool go_motion_planner::close_gripper_simulation(void) {
        
	const robot_state::JointModelGroup* joint_model_group =
        gripper_move_group->getCurrentState()->getJointModelGroup("interbotix_gripper");

	moveit::core::RobotStatePtr current_state = gripper_move_group->getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	joint_group_positions[0] = 0.015; // Closed
        joint_group_positions[1] = -0.015;  

	gripper_move_group->setJointValueTarget(joint_group_positions);
	bool success = (gripper_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (success) {
        	gripper_move_group->execute(my_plan); // This blocks until the plan is done executing
        }
		
	return true;
}

/** @brief - Move the robot's end effector frame to the given pose
 *  @brief pose - The desired pose of the end effector frame */
bool go_motion_planner::move_to_pose(geometry_msgs::Pose pose) { 
	
	const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	move_group->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
        	move_group->execute(my_plan); // This blocks until the plan is done executing
	}

	return success;
}

/** @brief - Create the pose object specefied by the arguments
 *  @param x, y, z - The (x, y, z) values of the desired pose
 *  @param roll, pitch, yaw - The desired roll, pitch, and yaw values (rads) of the given pose */
geometry_msgs::Pose go_motion_planner::create_pose(float x, float y, float z, float roll, float pitch, float yaw) { 
	
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	pose.orientation = quaternion_from_rpy(roll, pitch, yaw);	
	return pose;
}

/** @brief - Create the pose object relative to the robot's end effector frame's
 *  @param d_x, _d_y, d_z - The (d_x, d_y, d_z) values from the current pose
 *  @param d_roll, d_pitch, d_yaw - The desired d_roll, d_pitch, and d_yaw values (rads) from the current pose*/
geometry_msgs::Pose go_motion_planner::create_relative_pose(float d_x, float d_y, float d_z, float d_roll, float d_pitch, float d_yaw) {
		
	geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;	
	geometry_msgs::Pose relative_pose;
	relative_pose.position.x = current_pose.position.x + d_x;
	relative_pose.position.y = current_pose.position.y + d_y;	
	relative_pose.position.z = current_pose.position.z + d_z;
	
	auto[roll, pitch, yaw] = rpy_from_quaternion(current_pose.orientation);
	roll += d_roll;
	pitch += d_pitch;
	yaw += d_yaw;	
	relative_pose.orientation = quaternion_from_rpy(roll, pitch, yaw);

	return relative_pose;
}	


/** @brief - Create a quaternion from the given Euler angles
 *  @param roll, pitch, yaw - The roll, pitch, and yaw values (rads) */
geometry_msgs::Quaternion go_motion_planner::quaternion_from_rpy(double roll, double pitch, double yaw) { 
	tf2::Quaternion quaternion;
	quaternion.setRPY(roll, pitch, yaw);
	return tf2::toMsg(quaternion);
}

/** @brief - Compute the Euler angles given the geom_msgs::quaternion object
 *  @param geom_msgs_quat - The quaternion */
std::tuple<float, float, float> go_motion_planner::rpy_from_quaternion(geometry_msgs::Quaternion geom_msgs_quat) {

	double roll, pitch, yaw;
	tf::Quaternion tf_quat;
    	tf::quaternionMsgToTF(geom_msgs_quat, tf_quat);
    	tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	return std::make_tuple(roll, pitch, yaw);
}


/** @brief - Add orientation constraints to the next motion planning request
 *  @param roll_tolerance - Interval from the current roll value that plan shoud allow 
 *  @param - pitch_tolerance - Interval from the current pitch value that plan shoud allow 
 *  @param - yaw_tolerance - Interval from the current yaw value that plan shoud allow  */
void go_motion_planner::add_orientation_constraints(float roll_tolerance, float pitch_tolerance, float yaw_tolerance) {
	
	moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = move_group->getEndEffectorLink().c_str();
        ocm.header.frame_id = move_group->getPlanningFrame().c_str();
        ocm.orientation = move_group->getCurrentPose().pose.orientation;
        ocm.absolute_x_axis_tolerance = roll_tolerance;
        ocm.absolute_y_axis_tolerance = pitch_tolerance;
        ocm.absolute_z_axis_tolerance = yaw_tolerance;
        ocm.weight = 1.0;

        moveit_msgs::Constraints test_constraints;
        test_constraints.orientation_constraints.push_back(ocm);
        move_group->setPathConstraints(test_constraints);
}

int main(int argc, char** argv) {
   
  ros::init(argc, argv, "pick_and_place_node");
  ros::AsyncSpinner spinner(0.1); 
  spinner.start();
  go_motion_planner my_planner = go_motion_planner();   
  
  while (true) {
	;
  }

  ros::shutdown();
  return 0;
}

