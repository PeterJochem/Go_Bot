
from urdfpy import *

#/rx200/joint_states

robot = URDF.load('../../interbotix_ros_arms/interbotix_descriptions/urdf/rx200.urdf')

robot.animate()

