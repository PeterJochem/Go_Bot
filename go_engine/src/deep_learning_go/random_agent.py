from agent import Agent
from go_types import BoardLocation
import random

class RandomAgent(Agent):

    def __init__(self, isBlack):
        
        self.name = "RandomBotName"
        self.isBlack = isBlack

    def get_move(self, game_board):
        
        return self._create_random_move(game_board)


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
