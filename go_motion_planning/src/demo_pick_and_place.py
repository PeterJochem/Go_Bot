#!/usr/bin/env python
"""! @brief Simple script for demo-ing the pick and place-ing of the robot"""
##
# Parameters: None
# 
# Publishers: None
#       
# Subscribers: None
#
# Services: Uses /
#           Uses /


import rospy, tf2_ros
import tf
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from go_motion_planning.srv import spawn_piece, remove_pieces_from_gazebo, pickup_piece, place_piece 
import time

""" @brief Provides services for spawning and removing objects in Gazebo"""
class demo_pick_and_place:

    def __init__(self):
        
        #rospy.wait_for_service("/")
        #rospy.wait_for_service("/")
        
        self.delete_pieces = rospy.ServiceProxy("/remove_pieces_from_gazebo", remove_pieces_from_gazebo)
        self.spawn_piece = rospy.ServiceProxy("/spawn_piece", spawn_piece)
        
        self.pickup_piece = rospy.ServiceProxy("/pickup_piece", pickup_piece)
        self.place_piece = rospy.ServiceProxy("/place_piece", place_piece)
        
        
    def demo(self):
    
        x_black_start = [0, 2, 4]
        y_black_start = [1, 3, 5]
        
        x_white_start = [5, 3, 1] 
        y_white_start = [6, 4, 2]

        x_black_finished = [1, 3, 5]
        y_black_finished = [5, 3, 1] 

         
        for i in range(len(x_black_start)):    
            self.spawn_piece(x_black_start[i], y_black_start[i], True)
        
        for i in range(len(y_white_start)):
            self.spawn_piece(y_white_start[i], y_white_start[i], False)
        

        # Move the pieces
        for i in range(len(x_black_finished)):
            
            self.pickup_piece(x_black_start[i], y_black_start[i])
            #time.sleep(1.0)
            self.place_piece(x_black_finished[i], y_black_finished[i])
            #time.sleep(1.0)





if __name__ == '__main__':

    rospy.init_node("demo_pick_and_place_node")
    myDemo = demo_pick_and_place()
    myDemo.demo()
    rospy.spin()

