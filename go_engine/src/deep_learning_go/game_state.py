from go_board import GoBoard
from random_agent import RandomAgent
from go_types import BoardLocation
import numpy as np

class GameState():

    def __init__(self, num_rows, num_columns):


        self.blacksTurn = True
        self.go_board =  GoBoard(num_rows, num_columns)
        self.white_player = RandomAgent(False)
        self.black_player = RandomAgent(True)

        self.num_moves = 0

    
    def isOver(self):
        if (self.num_moves < 10):
            return False
        return True

    def play_game(self):

        while (self.isOver() == False):
            
            if (self.blacksTurn):
                board_location = self.black_player.get_move(self.go_board) 
            else:
                board_location = self.white_player.get_move(self.go_board)
            
            self.execute_move(board_location)
            self.display_board()

        self.display_winner()
    
    def execute_move(self, board_location):
        
        # Check if the move is a pass
        if (board_location is not None):
            self.go_board.place_stone(board_location, self.blacksTurn)
        
        self.blacksTurn = not self.blacksTurn
        self.num_moves = self.num_moves + 1

    
    def display_board(self):
        
        board_array_representation = np.zeros((self.go_board.num_rows, self.go_board.num_columns))

        for row in range(self.go_board.num_rows):
            for column in range(self.go_board.num_columns):
                
                if (self.go_board.does_piece_exist_at_location(BoardLocation(row, column), False)):  
                    board_array_representation[row][column] = '1'
                elif (self.go_board.does_piece_exist_at_location(BoardLocation(row, column), True)):
                    board_array_representation[row][column] = '2'
        
        print("-----------------")
        print(board_array_representation)

               


    def display_winner(self):
        pass

        """
        if ():
            print("The black player won the game")
        else:
            print("The white player won the game")
        """
            
myGame_State = GameState(3,3)
myGame_State.play_game()

