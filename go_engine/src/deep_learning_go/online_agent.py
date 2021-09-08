from agent import Agent
from go_types import BoardLocation
from go_bot_server.srv import move
import random
import rospy
import copy
import time

class OnlineAgent(Agent):

    def __init__(self, isBlack, name="OnlineHumanName"):
        
        self.name = name
        self.isBlack = isBlack
        self.reqMove = None
        if (self.isBlack):
            topic_name = "/enter_online_player_move/black_player"
        else:
            topic_name = "/enter_online_player_move/white_player"
        self.move_listening_service = rospy.Service(topic_name, move, self.process_move_data)
    
    def process_move_data(self, req):
        
        print("Test, the online player requested a move to the position " + str(req.row) + str(" and ") + str(req.column))
        try:
            self.reqMove = BoardLocation(req.row, req.column)
            return True
        except:
            return False

    def get_move(self, game_board):
        
        print("The online player has not input a move yet. Waiting for their move")
        is_move_valid = False
        while (not is_move_valid):
            time.sleep(1.0)
            
            if (self.reqMove is not None):
                board_location = copy.deepcopy(self.reqMove)
                if (not game_board.isMoveLegal(board_location, self.isBlack)):
                    #raise Exception("Online player requested an illegal move")
                    self.reqMove = None
                else:
                    self.reqMove = board_location
                    is_move_valid = True

        self.reqMove = None # Reset for the player's next move 
        return board_location

    def _create_random_move(self, game_board):
        all_legal_board_locations = []

        for row in range(game_board.num_rows):
            for column in range(game_board.num_columns):
                
                board_location = BoardLocation(row, column)

                if (game_board.isMoveLegal(board_location, self.isBlack)):
                    all_legal_board_locations.append(board_location)
        
        if (len(all_legal_board_locations) == 0):
            return None # We must do a pass
        
        random_index = random.randrange(len(all_legal_board_locations))
        return all_legal_board_locations[random_index]
