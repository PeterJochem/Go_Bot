/*** @file Tests for the motion_planning package
 * @author Peter Jochem <peterjochem2020@u.northwestern.edu>
 * @section Tests for the motion_planning package */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdlib.h>
#include <sstream>


TEST(TestSuite, A_Test_Description) {
	
	ASSERT_NEAR(10, 10.0000001, 1e-3);	
}

int main(int argc, char **argv){

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "go_motion_planning_test");
  ros::NodeHandle n;


  return RUN_ALL_TESTS();
}
