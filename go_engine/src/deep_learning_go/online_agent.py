from agent import Agent
from go_types import BoardLocation
from go_browser.srv import move
import random
import rospy
import copy

class OnlineAgent(Agent):

    def __init__(self, isBlack):
        
        self.name = "OnlineHumanName"
        self.isBlack = isBlack
        self.currentMove = None
    
    def process_move_data(self, req):
        
        # buffer the online player's move
        # convert the message to a gameboard location
        # check if the move is legal
        self.currentMove = BoardLocation(8, 8)


    def get_move(self, game_board):
        
        if (self.currentMove == None):
            print('The online player has not input a move yet. Waiting for their move')
            return

        board_location = copy.deepcopy(self.currentMove)

        if (not game_board.isMoveLegal(board_location, self.isBlack)):
            # FIX ME - alert the user that the move is illegal
            pass
        
        self.currentMove = None # Reset for the player's next move 
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
    
    


