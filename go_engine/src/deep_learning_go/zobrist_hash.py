from go_types import BoardLocation
import random

"""Describe me"""
class ZobristHash():

    def __init__(self, num_rows, num_columns):
        self.num_rows = num_rows
        self.num_columns = num_columns
        
        """One value for each possible move in the game - 2 x (num_rows) x (num_columns)"""
        self.move_values = {}
        self.fill_table()
        self.current_hash = 0 # The empty board has a hash value of 0
    

    def fill_table(self):
        
        max63 = 0x7fffffffffffffff
        for row in range(self.num_rows):
            for column in range(self.num_columns):
                
                self.move_values[BoardLocation(row, column), True] = random.randint(0, max63)
                self.move_values[BoardLocation(row, column), False] = random.randint(0, max63)
                
    def lookup_table_value(self, board_location, isBlack):
        return self.move_values[board_location, isBlack]

    def get_hash_value(self, board_location, isBlack):
        return self.current_hash ^ self.lookup_table_value(board_location, isBlack)
        
    def apply_move(self, board_location, isBlack):
        self.current_hash = self.current_hash ^ self.lookup_table_value(board_location, isBlack)
        return self.current_hash

    def undo_move(self, board_location, isBlack):
        self.current_hash = self.current_hash ^ self.lookup_table_value(board_location, isBlack)
        return self.current_hash
