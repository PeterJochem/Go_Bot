#!/usr/bin/env python
"""! @brief Class for coordinating a game"""
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
from go_motion_planning.srv import pickup_set_of_pieces
from deep_learning_go.game_state import GameState
from go_browser.msg import move, gamestate
import time

""" @brief Class for coordinating a game """
class go_commander:

    def __init__(self):
        
        self.game_state = GameState(4, 4) 
         
        self.delete_pieces = rospy.ServiceProxy("/remove_pieces_from_gazebo", remove_pieces_from_gazebo)
        self.spawn_piece = rospy.ServiceProxy("/spawn_piece", spawn_piece)
        
        self.pickup_piece = rospy.ServiceProxy("/pickup_piece", pickup_piece)
        self.place_piece = rospy.ServiceProxy("/place_piece", place_piece)
        self.pickup_set_of_pieces = rospy.ServiceProxy("/pickup_set_of_pieces", pickup_set_of_pieces)
        
        self.get_online_move = rospy.Subscriber("/online_player/next_move", move, self.game_state.black_player.process_move_data)
        self.game_state_publisher = rospy.Publisher('/current_game_state', gamestate, queue_size=1)
         
        rate = rospy.Rate(10)
        rate.sleep()

     
    def play_game(self):
        
        while (not self.game_state.isOver()):

            board_location = self.game_state.get_next_players_move()
            if (board_location is None):
                # The agent passed
                self.game_state.execute_move(board_location)
                continue
                
            rate = rospy.Rate(10)
            rate.sleep()

            # Add the new piece to the real board
            self.update_real_board(board_location, self.game_state.blacksTurn)

            # Update data structures tracking the game
            captured_pieces = self.game_state.execute_move(board_location)
            self.game_state.display_board()
            self.remove_captured_groups(captured_pieces)
            

            # You could also call a method here to let objects lower in tree spin once
            self.game_state_publisher.publish(self.game_state.toMsg())   
            
            # rospy.spinOnce()
            rate = rospy.Rate(10)
            rate.sleep()


    def update_real_board(self, board_location, isBlack):
        
        try:
            # FIX ME - For now, spawn a piece at a hardcoded location
            self.spawn_piece(5, 5, isBlack)
            time.sleep(0.1)

            self.pickup_piece(5, 5)
            print("Board Location (row, column) = " + str(board_location.row) + ", " + str(board_location.column))
            self.place_piece(board_location.row, board_location.column)
        
        except:
            print("Caught an error when trying to update the real board in play_game.py")
            print("Board Location (row, column) = " + str(board_location.row) + ", " + str(board_location.column))

    def remove_captured_groups(self, captured_pieces):
        
        if (captured_pieces is None or len(captured_pieces) == 0):
            return
        
        row_cords = []
        column_cords = []
        for piece in captured_pieces:
            row_cords.append(piece.row)
            column_cords.append(piece.column)

        self.pickup_set_of_pieces(row_cords, column_cords)



if __name__ == '__main__':
    
    rospy.init_node("go_commander_node")
    myCommander = go_commander()
    myCommander.play_game()
    rospy.spin()
