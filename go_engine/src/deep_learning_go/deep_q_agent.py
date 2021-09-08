from agent import Agent
from go_types import BoardLocation
import random
import signal, os
#from replayBuffer import *
#import gym
#import tensorflow as tf
#from tensorflow.keras import layers
import numpy as np
import matplotlib.pyplot as plt
#from tensorflow.keras import regularizers
#from tensorflow import keras
#from deep_q import Agent

class DeepQLearningAgent(Agent):

    def __init__(self, isBlack, num_rows, num_columns):
        
        self.name = "DeepQLearningAgent"
        self.isBlack = isBlack
        self.board_width = num_rows
        self.board_height = num_columns

    def get_move(self, game_board):
        return self._create_random_move(game_board)

    def train_agent(self, num_games):
        pass

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
