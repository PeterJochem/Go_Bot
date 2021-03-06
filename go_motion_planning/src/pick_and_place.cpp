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

// TO Do: 
// 1) Create Utility Functions
// 2) Create a service to move to a column
// 3) Change the node names and the file names 
// 4) Setup ROS Test!!

go_motion_planner::go_motion_planner() {
	
	node_handle = ros::NodeHandle();
	initialize_moveit();
	setup_services();			
	setup_subscribers();
}

void go_motion_planner::setup_services() {
	home_position_service = node_handle.advertiseService("/home_position", &go_motion_planner::move_to_home_position, this);
	pickup_piece_service = node_handle.advertiseService("/pickup_piece", &go_motion_planner::pickup_piece, this);
}

void go_motion_planner::setup_subscribers() {
	node_handle.subscribe("/rx200/execute_trajectory/status", 1000, &go_motion_planner::process_move_group_feedback, this);
}


void go_motion_planner::process_move_group_feedback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
	std::cout << "Processing a feedback message" << std::endl;

	
	std::cout << "The messages's status is " << msg->status_list.size() << std::endl;  

}

void go_motion_planner::initialize_moveit() {
	
	PLANNING_GROUP = "interbotix_arm";
  	move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  	planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	
	ROS_INFO_NAMED("MoveItSetupInfo", "Planning frame: %s", move_group->getPlanningFrame().c_str());
	ROS_INFO_NAMED("MoveItSetupInfo", "End effector link: %s", move_group->getEndEffectorLink().c_str());
}


bool go_motion_planner::move_to_home_position(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	return true;
}


bool go_motion_planner::pickup_piece(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        return true;
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

		auto new_pose = create_relative_pose(0, 0, -0.01, 0.0, 0.0, 0.0); 

		//add_orientation_constraints(3.14 / 3.0, 3.14 / 3.0, 3.14 / 3.0);
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
  ros::AsyncSpinner spinner(0.001); 
  spinner.start();
  
  go_motion_planner my_planner = go_motion_planner();   
  bool x = my_planner.move_to_position();
  x = my_planner.move_to_position();
  
  
  ros::shutdown();
  return 0;
}

