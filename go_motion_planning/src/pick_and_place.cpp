#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "go_motion_planner.hpp"
#include <std_srvs/SetBool.h> // Delete this
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "moveit_msgs/MoveGroupFeedback.h"
#include "actionlib_msgs/GoalStatus.h"
#include <tf/tf.h>
#include <unistd.h> // For sleep - delete this
#include <std_msgs/Float64.h>
#include "go_motion_planning/home_position.h"
#include "go_motion_planning/pickup_piece.h"

// TO Do:
// 1) Create a service to move to a column
// 2) Change the node names and the file names 
// 3) Setup ROS Test!!

go_motion_planner::go_motion_planner() {
	
	node_handle = ros::NodeHandle();
	initialize_moveit();
	setup_services();			
	setup_publishers();
	setup_subscribers();
	setup_transform_listeners();
	setup_transform_listeners();
	
	

}

void go_motion_planner::setup_services() {
	home_position_service = node_handle.advertiseService("/home_position", &go_motion_planner::move_to_home_position, this);
	pickup_piece_service = node_handle.advertiseService("/pickup_piece", &go_motion_planner::pickup_piece, this);
}

void go_motion_planner::setup_subscribers() {

	node_handle.subscribe("/rx200/gripper/command", 1000, &go_motion_planner::process_gripper_data, this);
}



void go_motion_planner::setup_publishers() {
	
	
	// This controls the gripper's position in Gazebo only?
	gripper_position_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/rx200/gripper_controller/command", 1);	
	
	// This allows us to do pwm control. The line below is from interbotix_ros_arms/interbotix_sdk/src/interbotix_sdk/robot_manipulation.py
        //self.pub_gripper_command = rospy.Publisher(robot_name + "/gripper/command", Float64, queue_size=100)
	// The arm_node from the interbotix_sdk package will subscribe to this topic and use it to do pwm control
	gripper_pwm_pub = node_handle.advertise<std_msgs::Float64>("/rx200/gripper/command", 100);

}

void go_motion_planner::setup_transform_listeners() { 

	tfBuffer = new tf2_ros::Buffer();
        tfListener = new tf2_ros::TransformListener(*tfBuffer);	
}

void go_motion_planner::process_gripper_data(const std_msgs::Float64::ConstPtr& msg) {
	// Convert what will be a pwm command for the real robot to the gazebo simulation
	// closing or opening the gripper
	// use the gripper_position_pub to make the gripper open/close		
}


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


bool go_motion_planner::move_to_home_position(go_motion_planning::home_position::Request &req, go_motion_planning::home_position::Response &res) {
	return true;
}


bool go_motion_planner::pickup_piece(go_motion_planning::pickup_piece::Request &req, go_motion_planning::pickup_piece::Response &res) {
        
	// put this in a try block?

	// Move to the stance position
	geometry_msgs::Pose my_stance_pose = stance_pose(req.row, req.column); 
	
	std::cout << "The stance pose is " << my_stance_pose << std::endl;
	return move_to_pose(my_stance_pose);



	// Do a cartesian path down 
	// close the gripper
	// Do a cartesian path up 			
	// move to home?
}

geometry_msgs::Pose go_motion_planner::stance_pose(int row, int column) { 
	
	geometry_msgs::Pose desired_pose;
	geometry_msgs::Point desired_point = board_location(row, column);
	geometry_msgs::Quaternion desired_orientation = grasp_orientation();	
	
	desired_pose.position = desired_point;
	desired_pose.position.z = desired_point.z + 0.10;
	desired_pose.orientation = desired_orientation;
	
	return desired_pose; 
}

geometry_msgs::Point go_motion_planner::board_location(int row, int column) {
	
	double row_width = 0.02; // Move to the param server with a yaml file
        double row_height = 0.002;
	double z_stance_height = 0.05; 

        geometry_msgs::PointStamped world_point;
        geometry_msgs::PointStamped go_board_point;

        //go_board_point.header.seq = This is filled automatically
        go_board_point.header.stamp = ros::Time::now();
        go_board_point.header.frame_id = "go_board";

        go_board_point.point.x = row_width * row;
        go_board_point.point.y = row_height * column;
        go_board_point.point.z = 0.0;

        try {
        	tfBuffer->transform(go_board_point, world_point, "world");
        }
        catch (tf2::TransformException &ex) {
                // FIX ME This error should propagate up instead 
		ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        }

	return world_point.point;
}

geometry_msgs::Quaternion go_motion_planner::grasp_orientation() {	
	return quaternion_from_rpy(0.029, 1.522, -0.006); 
	//return quaternion_from_rpy(-0.618, -1.511, 0.618);
}

/* self.pub_gripper_command = rospy.Publisher(robot_name + "/gripper/command", Float64, queue_size=100)
 * Do PWM
 * bool open_gripper(void) {
 *
 * }
 */


/** @brief - Subscribe to the same topic as the arm_node topic
 * but instead of doing pwm control like the arm_node, do a simple 
 * approximation by opening the gripper  
 */
bool go_motion_planner::open_gripper_simulation() {	
	
	const robot_state::JointModelGroup* joint_model_group =
        gripper_move_group->getCurrentState()->getJointModelGroup("interbotix_gripper");

        moveit::core::RobotStatePtr current_state = gripper_move_group->getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	joint_group_positions[0] = 0.037; // Open
        joint_group_positions[1] = -0.037;

        gripper_move_group->setJointValueTarget(joint_group_positions);
        bool success = (gripper_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
                gripper_move_group->execute(my_plan); // This blocks until the plan is done executing
        }

        return true;	
}

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

geometry_msgs::Pose go_motion_planner::create_pose(float x, float y, float z, float roll, float pitch, float yaw) { 
	
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	pose.orientation = quaternion_from_rpy(roll, pitch, yaw);	
	return pose;
}

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

geometry_msgs::Quaternion go_motion_planner::quaternion_from_rpy(double roll, double pitch, double yaw) { 
	tf2::Quaternion quaternion;
	quaternion.setRPY(roll, pitch, yaw);
	return tf2::toMsg(quaternion);
}

std::tuple<float, float, float> go_motion_planner::rpy_from_quaternion(geometry_msgs::Quaternion geom_msgs_quat) {

	double roll, pitch, yaw;
	tf::Quaternion tf_quat;
    	tf::quaternionMsgToTF(geom_msgs_quat, tf_quat);
    	tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	return std::make_tuple(roll, pitch, yaw);
}

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

// Temporary function for testing 
bool go_motion_planner::move_to_position() { 
	
	//const robot_state::JointModelGroup* joint_model_group =
	//move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	for (int i= 0; i < 5; i++) {
	
		const robot_state::JointModelGroup* joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		auto new_pose = create_relative_pose(0, 0, 0.01, 0.0, 0.0, 0.0);
		if ((i % 2) == 0) {
			new_pose = create_relative_pose(0, 0, -0.01, 0.0, 0.0, 0.0); 
		}
		
		add_orientation_constraints(3.14 / 3.0, 3.14 / 3.0, 3.14 / 3.0);
		move_group->setPoseTarget(new_pose);	
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (success) {
			//move_group->move();
			move_group->execute(my_plan); // This blocks until the plan is done executing
			std::cout << "Executed plan : " << i << std::endl;
		}
	}

	return true;
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

