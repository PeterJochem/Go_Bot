#!/usr/bin/env python3
from go_board import GoBoard, PydanticGoBoard
from random_agent import RandomAgent
from go_types import BoardLocation
import numpy as np
#from deep_q_agent import DeepQLearningAgent
from online_agent import OnlineAgent
from go_browser.msg import gamestate

class GameState():

    def __init__(self, num_rows, num_columns, whitePlayerType=RandomAgent, blackPlayerType=OnlineAgent):

        self.blacksTurn = True
        self.go_board = GoBoard(num_rows, num_columns)
        self.white_player = whitePlayerType(False)
        self.black_player = blackPlayerType(True)
        #self.black_player = DeepQLearningAgent(True, num_rows, num_columns)
        #self.black_player = RandomAgent(True)
        
        self.did_white_just_pass = False
        self.did_black_just_pass = False
        self.num_moves = 0

    def toMsg(self):
        
        gstate_msg = gamestate
        gstate_msg.isBlackPlayersTurn = self.blacksTurn
        gstate_msg.isOnlinePlayersTurn = self.blacksTurn
        gstate_msg.num_rows = self.go_board.num_rows
        gstate_msg.num_columns = self.go_board.num_columns
        
        # 0 = empty, 1 = white, 2 = black
        gstate_msg.board = [1, 0, 0, 2, 1, 0] 
        return gstate_msg

    def isOver(self):
        """
        if (self.num_moves < 10):
            return False
        return True
        """
        if (self.did_white_just_pass and self.did_black_just_pass):
            return True
        return False

    def get_next_players_move(self):

        if (self.blacksTurn):
            board_location = self.black_player.get_move(self.go_board)
        else:
            board_location = self.white_player.get_move(self.go_board)

        return board_location

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
        
        captured_locations = []
        # Check if the move is a pass
        if (board_location is not None):
            captured_locations = self.go_board.place_stone(board_location, self.blacksTurn)
        
        if (board_location is None and self.blacksTurn):
            self.did_black_just_pass = True
            
        elif (board_location is None and self.blacksTurn == False):
            self.did_white_just_pass = True
            
        
        self.blacksTurn = not self.blacksTurn
        self.num_moves = self.num_moves + 1
        return captured_locations

    def add_scores_for_pieces_on_board(self):
        
        # Look for empty points on the board that your stones surround.
        # Add the number of pieces still currently on the board
        pass 

    def count_eyes(self, isBlack):
        
        for row in range(self.go_board.num_rows):
            for column in range(self.go_board.num_columns):

                if (self.go_board.does_piece_exist_at_location(BoardLocation(row, column), False)):
                    board_array_representation[row][column] = '1'
                elif (self.go_board.does_piece_exist_at_location(BoardLocation(row, column), True)):
                    board_array_representation[row][column] = '2'
        
    """Check if a board location is an eye. isBlack indicates if 
    the eye BELONGS to the black player"""
    def is_board_location_an_eye(self, board_location, isBlack):
        
        if (self.isEmpty(board_location) == False):
            return False
        
        if (len(board_location.neighors()) == 0):
            return False

        for neighbor in board_location.neighors():
            if (self.isOnBoard(neighbor)):
                if (self.go_board.does_piece_exist_at_location(neighbor, isBlack)):
                    continue
                else:
                    return False
        return True

    def display_board(self):
        board_array_representation = self.create_array_representation()
        print("-----------------")
        print(board_array_representation)
        
    def create_array_representation(self):
        board_array_representation = np.zeros((self.go_board.num_rows, self.go_board.num_columns))
        for row in range(self.go_board.num_rows):
            for column in range(self.go_board.num_columns):
                if (self.go_board.does_piece_exist_at_location(BoardLocation(row, column), False)):  
                    board_array_representation[row][column] = '1'
                elif (self.go_board.does_piece_exist_at_location(BoardLocation(row, column), True)):
                    board_array_representation[row][column] = '2'
        return board_array_representation
    
    def create_list_representation(self):
        
        board_as_array = self.create_array_representation()
        board_as_lists = []
        for row in board_as_array:
            board_as_lists.append(list(row))
        return board_as_lists 

    def display_winner(self):
        pass

        """
        if ():
            print("The black player won the game")
        else:
            print("The white player won the game")
        """
