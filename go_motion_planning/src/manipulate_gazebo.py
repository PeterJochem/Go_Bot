#!/usr/bin/env python
"""! @brief Services for spawning and removing objects in Gazebo"""
##
# Parameters:   /row_width: Name of the base frame 
#  		/row_height: the side lengths of pentagon 
#		/piece_height: robot's starting_x
#		/z_board_plane: robot's starting position in world frame
# 
# Publishers: None
#	
# Subscribers: None
#
# Services: /spawn_piece_service - Add a piece at a grid position on the board
#           /remove_pieces_from_gazebo - Remove all the pieces from the board in Gazebo


import rospy, tf2_ros
import tf
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from geometry_msgs.msg import PointStamped, Pose, Quaternion
from go_motion_planning.srv import spawn_piece, remove_pieces_from_gazebo
import random

""" @brief Provides services for spawning and removing objects in Gazebo"""
class manipulate_gazebo:

    def __init__(self):
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        
        print("Gazebo services to add pieces are ready")
        
        # Use RosPack to get the relative path
        self.black_piece_sdf = open('/home/peterjochem/Desktop/Go_Bot/catkin_ws/src/go_motion_planning/models/go_piece_black/model.sdf', 'r').read()
        self.white_piece_sdf = open('/home/peterjochem/Desktop/Go_Bot/catkin_ws/src/go_motion_planning/models/go_piece_white/model.sdf', 'r').read()
    
        self.listener = tf.TransformListener()
        self.get_params()
    
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        
        self.spawned_pieces = []
        self.spawn_piece_service = rospy.Service("spawn_piece", spawn_piece, self.add_piece)
        self.remove_piece_service = rospy.Service("remove_pieces_from_gazebo", remove_pieces_from_gazebo, self.remove_pieces)
       
    """ @brief ROS Service - Adds a piece to Gazebo at the (req.row, req.colum) location"""
    def add_piece(self, req):
        x, y = self.convert_from_grid_cords_to_world(req.row, req.column)
        z = 0.1 # Use param server value

        name = self.next_piece_name(req.isBlack)
        if (req.isBlack):
            sdf = self.black_piece_sdf
        else:
            sdf = self.white_piece_sdf

        # Spawn the piece
        initial_pose = Pose(Point(x, y, z), Quaternion(*(tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0))))
        self.spawn_model(name, sdf, "/", initial_pose, "world")
        self.spawned_pieces.append(name)
        return True

    """ @brief ROS Service - Remove all the pieces from Gazebo"""
    def remove_pieces(self, req):
        for name in self.spawned_pieces:
            self.delete_model(name)
        return True
    
    """ @brief Convert a board (row, column) value to a point in the world frame"""
    def convert_from_grid_cords_to_world(self, grid_x, grid_y):
        self.listener.waitForTransform("/go_board", "/world", rospy.Time(0), rospy.Duration(0.1))
        
        point_board = PointStamped()
        point_board.header.frame_id = "go_board"
        point_board.header.stamp = rospy.Time(0)
        
        point_board.point.x = (grid_x * self.row_width)
        point_board.point.y = (grid_y * self.row_height) 
        point_board.point.z = 0.0
        
        point_world = self.listener.transformPoint("world", point_board)
        return point_world.point.x, point_world.point.y

    """ @brief Create a new name for the next piece"""
    def next_piece_name(self, black=True):
        
        value = random.randint(0, 10000)
        if (black):
            name = "black_piece_" + str(value)
        else:
            name = "white_piece_" + str(value)
        return name

    """ @brief Load the parameters from the ROS parameter server"""
    def get_params(self):
    
        self.row_width = rospy.get_param("/row_width")
        self.row_height = rospy.get_param("/row_height")
        self.piece_height = rospy.get_param("/piece_height")
        self.z_board_plane = rospy.get_param("/z_board_plane")

if __name__ == '__main__':
    rospy.init_node("set_initial_board")    
    my_utilities = manipulate_gazebo()
    rospy.spin()
