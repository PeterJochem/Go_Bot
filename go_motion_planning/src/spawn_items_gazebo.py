#!/usr/bin/env python
import rospy, tf2_ros
import tf
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from geometry_msgs.msg import PointStamped, Pose, Quaternion
from go_motion_planning.srv import spawn_piece
import random

class spawn_items:

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
        
        spawn_piece_service = rospy.Service("spawn_piece", spawn_piece, self.add_piece)

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
        
        return True
     
    """Generate a new piece name"""
    def next_piece_name(self, black=True):
        
        value = random.randint(0, 10000)
        if (black):
            name = "black_piece_" + str(value)
        else:
            name = "white_piece_" + str(value)
    
        return name


    def get_params(self):
    
        self.row_width = rospy.get_param("/row_width")
        self.row_height = rospy.get_param("/row_height")
        self.piece_height = rospy.get_param("/piece_height")
        self.z_board_plane = rospy.get_param("/z_board_plane")




if __name__ == '__main__':
    
    rospy.init_node("set_initial_board")
    
    mySpawner = spawn_items()
    

    rospy.spin()


